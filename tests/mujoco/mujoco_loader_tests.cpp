/**
 * @file mujoco_loader_tests.cpp
 * @brief 测试 MuJoCo 模型加载、名称映射和全部启动失败路径。
 */

#include "quadruped/backends/mujoco/mujoco_model.hpp"
#include "quadruped/core/core.hpp"

#include <cstddef>
#include <iostream>
#include <set>
#include <string>

namespace qc = quadruped::core;
namespace qm = quadruped::backends::mujoco;

namespace
{

// 与 core 测试相同的简单断言风格：累计失败，全部用例运行后统一返回非零退出码。
int failures = 0;

void expect(bool condition, const std::string& description)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << description << '\n';
        ++failures;
    }
}

// 正式模型和 fixture 的路径由 CMake 在构建期传入，不依赖运行时当前目录。
const std::string kBlackScenePath = QUADRUPED_BLACK_SCENE_PATH;
const std::string kFixtureDir = QUADRUPED_MUJOCO_FIXTURE_DIR;

// 正式 black 模型的逻辑关节顺序，必须与 configs/robots/black.yaml 一致。
qc::RobotModel make_black_model()
{
    qc::RobotModel model;
    model.name = "black";
    model.model_id = 0x008A1E56CD69E8F4;
    model.joint_count = 12;

    constexpr const char* names[12] = {
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    };
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        model.joints[i].name = names[i];
        model.joints[i].role = qc::JointRole::Leg;
        model.joints[i].limits = {true, -3.0, 3.0, 20.0, 40.0, 100.0, 10.0};
    }
    return model;
}

// fixture 模型只有两个逻辑关节，与 fixtures/ 中的小型模型对应。
qc::RobotModel make_fixture_model()
{
    qc::RobotModel model;
    model.name = "fixture";
    model.model_id = 0x1;
    model.joint_count = 2;

    constexpr const char* names[2] = {"j1", "j2"};
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        model.joints[i].name = names[i];
        model.joints[i].role = qc::JointRole::Leg;
        model.joints[i].limits = {true, -3.0, 3.0, 20.0, 40.0, 100.0, 10.0};
    }
    return model;
}

bool expect_loaded(const qm::LoadResult& result, const std::string& description)
{
    if (!result.ok())
    {
        std::cerr << "FAIL: " << description << "，错误：" << result.error_message << '\n';
        ++failures;
        return false;
    }
    return true;
}

// 预期加载失败时，同时要求错误信息非空并包含关键字，防止错误原因张冠李戴。
void expect_rejected(
    const qm::LoadResult& result,
    const std::string& description,
    const std::string& keyword)
{
    if (result.ok())
    {
        std::cerr << "FAIL: " << description << "（预期被拒绝，但加载成功）\n";
        ++failures;
        return;
    }
    expect(!result.error_message.empty(), description + "：错误信息非空");
    expect(
        result.error_message.find(keyword) != std::string::npos,
        description + "：错误信息应包含 \"" + keyword + "\"，实际为 " + result.error_message);
}

void test_black_model_loads()
{
    const auto result = qm::MujocoModel::load(kBlackScenePath, make_black_model());
    if (!expect_loaded(result, "正式 black 场景加载成功"))
    {
        return;
    }
    const auto& info = result.model->info();
    expect(info.nq == 19, "black 模型 nq 为 19");
    expect(info.nv == 18, "black 模型 nv 为 18");
    expect(info.nu == 12, "black 模型 nu 为 12");
    expect(info.timestep == 0.002, "black 模型物理步长为 2 ms");
}

void test_black_joint_mappings()
{
    const auto robot = make_black_model();
    const auto result = qm::MujocoModel::load(kBlackScenePath, robot);
    if (!expect_loaded(result, "正式 black 场景加载成功"))
    {
        return;
    }
    const auto& model = *result.model;
    expect(model.joint_count() == robot.joint_count, "逻辑关节数量与 RobotModel 一致");

    // 用集合检查四个编号空间各自没有重复映射。
    std::set<int> joint_ids;
    std::set<int> qpos_addresses;
    std::set<int> qvel_addresses;
    std::set<int> actuator_ids;
    for (std::size_t i = 0; i < model.joint_count(); ++i)
    {
        const auto& mapping = model.joint_mappings()[i];
        expect(mapping.logical_index == i, "映射顺序与 RobotModel 下标一致");

        // 用名称独立查询 joint ID，验证映射来自名称而不是 XML 书写顺序。
        const int expected_id =
            mj_name2id(model.raw_model(), mjOBJ_JOINT, robot.joints[i].name.c_str());
        expect(mapping.joint_id == expected_id, robot.joints[i].name + " 的 joint ID 正确");
        expect(mapping.actuator_id >= 0, robot.joints[i].name + " 关联到执行器");

        joint_ids.insert(mapping.joint_id);
        qpos_addresses.insert(mapping.qpos_address);
        qvel_addresses.insert(mapping.qvel_address);
        actuator_ids.insert(mapping.actuator_id);
    }
    expect(joint_ids.size() == model.joint_count(), "joint ID 互不重复");
    expect(qpos_addresses.size() == model.joint_count(), "qpos 地址互不重复");
    expect(qvel_addresses.size() == model.joint_count(), "qvel 地址互不重复");
    expect(actuator_ids.size() == model.joint_count(), "actuator ID 互不重复");
}

void test_black_imu_mapping()
{
    const auto result = qm::MujocoModel::load(kBlackScenePath, make_black_model());
    if (!expect_loaded(result, "正式 black 场景加载成功"))
    {
        return;
    }
    const auto& model = *result.model;
    const mjModel* raw = model.raw_model();
    const auto& imu = model.imu_mapping();

    expect(
        imu.quat.sensor_id >= 0 && raw->sensor_type[imu.quat.sensor_id] == mjSENS_FRAMEQUAT &&
            raw->sensor_dim[imu.quat.sensor_id] == 4,
        "imu_quat 存在且为 4 维四元数传感器");
    expect(
        imu.gyro.sensor_id >= 0 && raw->sensor_type[imu.gyro.sensor_id] == mjSENS_GYRO &&
            raw->sensor_dim[imu.gyro.sensor_id] == 3,
        "imu_gyro 存在且为 3 维角速度传感器");
    expect(
        imu.acc.sensor_id >= 0 && raw->sensor_type[imu.acc.sensor_id] == mjSENS_ACCELEROMETER &&
            raw->sensor_dim[imu.acc.sensor_id] == 3,
        "imu_acc 存在且为 3 维加速度传感器");
}

void test_missing_path()
{
    const std::string missing = kFixtureDir + "/does_not_exist.xml";
    const auto result = qm::MujocoModel::load(missing, make_fixture_model());
    expect_rejected(result, "不存在的场景路径被拒绝", missing);
}

void test_invalid_xml()
{
    const auto result = qm::MujocoModel::load(kFixtureDir + "/invalid.xml", make_fixture_model());
    expect_rejected(result, "非法 XML 被拒绝", "failed to load");
}

void test_missing_joint()
{
    const auto result =
        qm::MujocoModel::load(kFixtureDir + "/missing_joint.xml", make_fixture_model());
    expect_rejected(result, "缺少关节的模型被拒绝", "j2");
}

void test_wrong_joint_type()
{
    const auto result =
        qm::MujocoModel::load(kFixtureDir + "/wrong_joint_type.xml", make_fixture_model());
    expect_rejected(result, "非单自由度关节被拒绝", "j2");
}

void test_missing_actuator()
{
    const auto result =
        qm::MujocoModel::load(kFixtureDir + "/missing_actuator.xml", make_fixture_model());
    expect_rejected(result, "缺少执行器的模型被拒绝", "j2");
}

void test_duplicate_actuator()
{
    const auto result =
        qm::MujocoModel::load(kFixtureDir + "/duplicate_actuator.xml", make_fixture_model());
    expect_rejected(result, "重复执行器关联被拒绝", "j1");
}

void test_extra_actuator()
{
    const auto result =
        qm::MujocoModel::load(kFixtureDir + "/extra_actuator.xml", make_fixture_model());
    expect_rejected(result, "多余执行器被拒绝", "outside");
}

void test_joint_count_mismatch()
{
    // RobotModel 只有 11 个关节时，正式模型中多余的执行器必须导致失败。
    auto robot = make_black_model();
    robot.joint_count = 11;
    const auto result = qm::MujocoModel::load(kBlackScenePath, robot);
    expect_rejected(result, "关节数量与模型不符被拒绝", "outside");
}

void test_missing_imu_sensor()
{
    const auto result =
        qm::MujocoModel::load(kFixtureDir + "/missing_imu_sensor.xml", make_fixture_model());
    expect_rejected(result, "缺少 IMU 传感器的模型被拒绝", "imu_gyro");
}

void test_wrong_imu_sensor()
{
    const auto result =
        qm::MujocoModel::load(kFixtureDir + "/wrong_imu_sensor.xml", make_fixture_model());
    expect_rejected(result, "IMU 传感器类型或维度错误被拒绝", "imu_quat");
}

}  // 匿名命名空间

int main()
{
    test_black_model_loads();
    test_black_joint_mappings();
    test_black_imu_mapping();
    test_missing_path();
    test_invalid_xml();
    test_missing_joint();
    test_wrong_joint_type();
    test_missing_actuator();
    test_duplicate_actuator();
    test_extra_actuator();
    test_joint_count_mismatch();
    test_missing_imu_sensor();
    test_wrong_imu_sensor();

    if (failures != 0)
    {
        std::cerr << failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All quadruped_mujoco tests passed\n";
    return 0;
}
