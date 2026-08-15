/**
 * @file mujoco_model.hpp
 * @brief 定义 MuJoCo 模型的加载结果、名称映射和 RAII 生命周期管理。
 */

#pragma once

#include "quadruped/core/constants.hpp"
#include "quadruped/core/robot_model.hpp"

#include <array>
#include <cstddef>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>

namespace quadruped::backends::mujoco
{

// mjModel 只能由 mj_deleteModel 释放；unique_ptr 只在指针非空时调用删除器。
struct MjModelDeleter
{
    void operator()(mjModel* model) const noexcept
    {
        mj_deleteModel(model);
    }
};

// mjData 只能由 mj_deleteData 释放。
struct MjDataDeleter
{
    void operator()(mjData* data) const noexcept
    {
        mj_deleteData(data);
    }
};

using MjModelPtr = std::unique_ptr<mjModel, MjModelDeleter>;
using MjDataPtr = std::unique_ptr<mjData, MjDataDeleter>;

// 加载时一次性记录的模型维度和物理步长。
struct ModelInfo
{
    mjtSize nq{0};        // 广义位置维度，即 qpos 数组长度
    mjtSize nv{0};        // 速度自由度维度，即 qvel 数组长度
    mjtSize nu{0};        // 执行器数量，即 ctrl 数组长度
    double timestep{0.0}; // 物理步长，单位为秒
};

// 一个逻辑关节到 MuJoCo 对象的只读映射，加载时按名称一次性建立，之后不再修改。
// 后续读取关节位置和速度时直接使用 qpos_address 和 qvel_address，
// 不使用 XML 中的 jointpos/jointvel 传感器，保证关节状态只有这一个来源。
struct JointMapping
{
    std::size_t logical_index{0}; // 在 RobotModel 统一关节顺序中的下标
    int joint_id{-1};             // mjModel 中的关节编号，按 mjOBJ_JOINT 名称查询得到
    int qpos_address{-1};         // 该关节在 qpos 数组中的起始地址
    int qvel_address{-1};         // 该关节在 qvel 数组中的起始地址
    int actuator_id{-1};          // 通过关节传动关联到的执行器编号
};

// 一个 IMU 传感器在 mjModel 中的编号和 sensordata 输出地址。
struct ImuSensorEntry
{
    int sensor_id{-1};    // mjModel 中的传感器编号，按 mjOBJ_SENSOR 名称查询得到
    int data_address{-1}; // 该传感器在 sensordata 数组中的起始地址
};

// IMU 必需的三个传感器映射；输出维度分别为 4、3、3。
struct ImuMapping
{
    ImuSensorEntry quat; // imu_quat：机体姿态四元数
    ImuSensorEntry gyro; // imu_gyro：机体角速度
    ImuSensorEntry acc;  // imu_acc：机体线加速度
};

class MujocoModel;

// load() 的返回结果：成功时 model 非空，失败时 error_message 含有可读原因。
struct LoadResult
{
    std::unique_ptr<MujocoModel> model{};
    std::string error_message{};

    [[nodiscard]] bool ok() const noexcept
    {
        return model != nullptr;
    }
};

// MuJoCo 模型及其名称映射的拥有者；只能移动，不能复制。
class MujocoModel
{
public:
    MujocoModel(MujocoModel&&) noexcept = default;
    MujocoModel& operator=(MujocoModel&&) noexcept = default;
    MujocoModel(const MujocoModel&) = delete;
    MujocoModel& operator=(const MujocoModel&) = delete;
    ~MujocoModel() = default;

    // 加载场景 XML 并按 robot_model 的名称和顺序建立显式映射。
    // 逻辑关节顺序只来自 robot_model，不使用 XML 中 body 或 joint 的书写顺序。
    // 任何检查失败都返回空 model 和包含具体名称的可读原因，不抛出异常。
    static LoadResult load(const std::string& scene_path, const core::RobotModel& robot_model);

    // 模型维度和物理步长快照。
    const ModelInfo& info() const noexcept
    {
        return info_;
    }

    // 全部关节映射；只有前 joint_count() 项有效。
    const std::array<JointMapping, core::kMaxJoints>& joint_mappings() const noexcept
    {
        return joint_mappings_;
    }

    // 有效逻辑关节数量，与传入的 RobotModel 一致。
    std::size_t joint_count() const noexcept
    {
        return joint_count_;
    }

    // IMU 传感器映射。
    const ImuMapping& imu_mapping() const noexcept
    {
        return imu_mapping_;
    }

    // 非拥有的 MuJoCo 对象访问，指针生命周期与本对象一致。
    const mjModel* raw_model() const noexcept
    {
        return model_.get();
    }

    mjData* raw_data() noexcept
    {
        return data_.get();
    }

    const mjData* raw_data() const noexcept
    {
        return data_.get();
    }

private:
    // 只能通过 load() 完成全部检查后构造。
    MujocoModel() = default;

    MjModelPtr model_{};
    MjDataPtr data_{};
    ModelInfo info_{};
    std::array<JointMapping, core::kMaxJoints> joint_mappings_{};
    std::size_t joint_count_{0};
    ImuMapping imu_mapping_{};
};

}  // 命名空间 quadruped::backends::mujoco
