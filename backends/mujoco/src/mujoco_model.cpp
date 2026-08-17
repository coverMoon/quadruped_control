/**
 * @file mujoco_model.cpp
 * @brief 实现 MuJoCo 场景加载和 RobotModel 到关节、执行器、IMU 的名称映射。
 */

#include "quadruped/backends/mujoco/mujoco_model.hpp"

#include "quadruped/core/validation.hpp"

#include <filesystem>
#include <string>
#include <system_error>
#include <utility>

namespace quadruped::backends::mujoco
{
namespace
{

// MuJoCo 错误缓冲区的固定长度，足以容纳常见的 XML 解析错误文本。
constexpr std::size_t kMujocoErrorBufferSize = 1024;

// 映射辅助函数的约定：返回空字符串表示成功，否则返回包含具体名称或编号的
// 失败原因。所有检查只在加载时执行一次，物理步进路径不会重复构造映射。

// 按名称为每个逻辑关节建立映射，拒绝缺失或非单自由度关节。
std::string map_joints(
    const mjModel& model,
    const core::RobotModel& robot_model,
    std::array<JointMapping, core::kMaxJoints>& mappings)
{
    for (std::size_t i = 0; i < robot_model.joint_count; ++i)
    {
        const std::string& name = robot_model.joints[i].name;
        const int joint_id = mj_name2id(&model, mjOBJ_JOINT, name.c_str());
        if (joint_id < 0)
        {
            return "joint \"" + name + "\" does not exist in the MuJoCo model";
        }

        // 只有 hinge 和 slide 是单自由度关节，qpos/qvel 才各占用一个标量地址。
        const int joint_type = model.jnt_type[joint_id];
        if (joint_type != mjJNT_HINGE && joint_type != mjJNT_SLIDE)
        {
            return "joint \"" + name + "\" is not a single-DOF hinge or slide joint";
        }

        const int qpos_address = model.jnt_qposadr[joint_id];
        const int qvel_address = model.jnt_dofadr[joint_id];
        if (qpos_address < 0 || qpos_address >= model.nq || qvel_address < 0 ||
            qvel_address >= model.nv)
        {
            return "joint \"" + name + "\" has an out-of-range qpos or qvel address";
        }

        mappings[i] = JointMapping{i, joint_id, qpos_address, qvel_address, -1};
    }
    return {};
}

// 通过关节传动把执行器关联到逻辑关节。
// 固定 XML 中的 motor 没有 name 属性，不能按执行器名查询，只能从传动关系反查。
std::string map_actuators(
    const mjModel& model,
    const core::RobotModel& robot_model,
    std::array<JointMapping, core::kMaxJoints>& mappings)
{
    for (int actuator_id = 0; actuator_id < model.nu; ++actuator_id)
    {
        // 只接受直接驱动单个关节的传动；肌腱等复合传动没有等价的标量控制语义。
        if (model.actuator_trntype[actuator_id] != mjTRN_JOINT)
        {
            return "actuator " + std::to_string(actuator_id) +
                " is not a single-joint transmission";
        }

        // actuator_trnid 是 (nu, 2) 数组，关节传动用第一个元素保存 joint ID。
        const int joint_id = model.actuator_trnid[actuator_id * 2];

        JointMapping* owner = nullptr;
        for (std::size_t i = 0; i < robot_model.joint_count; ++i)
        {
            if (mappings[i].joint_id == joint_id)
            {
                owner = &mappings[i];
                break;
            }
        }
        if (owner == nullptr)
        {
            return "actuator " + std::to_string(actuator_id) +
                " drives a joint outside the RobotModel joint set";
        }
        if (owner->actuator_id >= 0)
        {
            return "joint \"" + robot_model.joints[owner->logical_index].name +
                "\" is driven by more than one actuator";
        }
        owner->actuator_id = actuator_id;
    }

    for (std::size_t i = 0; i < robot_model.joint_count; ++i)
    {
        if (mappings[i].actuator_id < 0)
        {
            return "joint \"" + robot_model.joints[i].name + "\" has no actuator";
        }
    }
    return {};
}

// IMU 传感器的期望：固定名称、传感器类型和输出维度。
struct ImuSensorExpectation
{
    const char* name;
    int type;
    int dimension;
};

// IMU 必需的传感器数量：四元数、角速度和线加速度。
constexpr std::size_t kImuSensorCount = 3;

// 按名称检查 IMU 传感器，并记录 sensordata 地址。
std::string map_imu_sensors(const mjModel& model, ImuMapping& imu_mapping)
{
    const ImuSensorExpectation expectations[kImuSensorCount] = {
        {"imu_quat", mjSENS_FRAMEQUAT, 4},
        {"imu_gyro", mjSENS_GYRO, 3},
        {"imu_acc", mjSENS_ACCELEROMETER, 3},
    };
    ImuSensorEntry* entries[kImuSensorCount] = {
        &imu_mapping.quat,
        &imu_mapping.gyro,
        &imu_mapping.acc,
    };

    for (std::size_t i = 0; i < kImuSensorCount; ++i)
    {
        const auto& expectation = expectations[i];
        const std::string name = expectation.name;
        const int sensor_id = mj_name2id(&model, mjOBJ_SENSOR, expectation.name);
        if (sensor_id < 0)
        {
            return "sensor \"" + name + "\" does not exist in the MuJoCo model";
        }
        if (model.sensor_type[sensor_id] != expectation.type)
        {
            return "sensor \"" + name + "\" has an unexpected sensor type";
        }
        if (model.sensor_dim[sensor_id] != expectation.dimension)
        {
            return "sensor \"" + name + "\" has an unexpected output dimension";
        }

        const int data_address = model.sensor_adr[sensor_id];
        if (data_address < 0 || data_address + expectation.dimension > model.nsensordata)
        {
            return "sensor \"" + name + "\" has an out-of-range sensordata address";
        }
        *entries[i] = ImuSensorEntry{sensor_id, data_address};
    }
    return {};
}

}  // 匿名命名空间

LoadResult MujocoModel::load(const std::string& scene_path, const core::RobotModel& robot_model)
{
    LoadResult result;

    // 先检查路径存在，把文件缺失和 XML 解析错误分开报告。
    // 使用带 std::error_code 的重载，避免底层文件系统错误抛出异常。
    std::error_code path_error;
    const bool path_exists = std::filesystem::exists(scene_path, path_error);
    if (path_error)
    {
        result.error_message = "cannot check scene file path: " + path_error.message();
        return result;
    }
    if (!path_exists)
    {
        result.error_message = "scene file does not exist: " + scene_path;
        return result;
    }

    // 复用核心校验，不在加载器里复制另一套 RobotModel 规则。
    if (const auto validation = core::validate(robot_model); !validation)
    {
        result.error_message = "RobotModel is invalid: " + validation.message;
        return result;
    }

    // MuJoCo 把解析警告和错误写进缓冲区；加载失败时必须保留完整文本。
    char mujoco_error[kMujocoErrorBufferSize] = {};
    MjModelPtr model{
        mj_loadXML(scene_path.c_str(), nullptr, mujoco_error, sizeof(mujoco_error))};
    if (model == nullptr)
    {
        result.error_message = "MuJoCo failed to load " + scene_path + ": " + mujoco_error;
        return result;
    }

    // mjData 创建失败时 model 由 RAII 自动释放，不继续后续检查。
    MjDataPtr data{mj_makeData(model.get())};
    if (data == nullptr)
    {
        result.error_message = "MuJoCo failed to allocate mjData for " + scene_path;
        return result;
    }

    MujocoModel instance;
    instance.joint_count_ = robot_model.joint_count;
    if (const auto error = map_joints(*model, robot_model, instance.joint_mappings_);
        !error.empty())
    {
        result.error_message = error;
        return result;
    }
    if (const auto error = map_actuators(*model, robot_model, instance.joint_mappings_);
        !error.empty())
    {
        result.error_message = error;
        return result;
    }
    if (const auto error = map_imu_sensors(*model, instance.imu_mapping_); !error.empty())
    {
        result.error_message = error;
        return result;
    }

    instance.info_ = ModelInfo{model->nq, model->nv, model->nu, model->opt.timestep};
    instance.model_ = std::move(model);
    instance.data_ = std::move(data);
    result.model.reset(new MujocoModel(std::move(instance)));
    return result;
}

}  // 命名空间 quadruped::backends::mujoco
