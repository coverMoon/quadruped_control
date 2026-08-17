/**
 * @file config_tests.cpp
 * @brief 测试 RobotModel YAML 加载的合法路径、标量字段和结构拒绝路径。
 */

#include "config_test_helpers.hpp"

#include "quadruped/core/core.hpp"

namespace
{

using config_test::expect;

void test_load_real_configs()
{
    const auto robot = quadruped::config::load_robot_model(QUADRUPED_ROBOT_CONFIG_PATH);
    expect(robot.ok(), "仓库 black 机器人配置应加载成功");
    if (!robot.ok())
    {
        return;
    }
    expect(robot.model.name == "black", "black 配置名称应为 black");
    expect(robot.model.joint_count == 12, "black 应有 12 个关节");
    expect(robot.model.joints[0].name == "FL_hip_joint", "第一个关节应为 FL_hip_joint");
    expect(robot.model.joints[11].name == "RR_calf_joint", "最后一个关节应为 RR_calf_joint");
    expect(robot.model.joints[0].role == qc::JointRole::Leg, "black 关节角色应为 leg");
    expect(robot.model.joints[0].limits.position_limited, "black 关节应有位置限制");
    expect(robot.model.joints[0].limits.min_position == -10.0,
        "black 位置限制应与 MuJoCo 模型一致");
    expect(robot.model.joints[0].limits.max_kp == 100.0, "black KP 上限应为 100");
}

// 缺字段和未知角色等标量字段错误应拒绝加载。
void test_robot_rejections_scalars()
{
    const auto missing = quadruped::config::load_robot_model("qc_no_such_file.yaml");
    expect(!missing.ok(), "不存在的文件应拒绝加载");

    config_test::expect_robot_rejected(
        {"qc_robot_missing_kp.yaml", ", max_kp: 100.0", "", "缺少 max_kp 字段应拒绝"});
    config_test::expect_robot_rejected(
        {"qc_robot_bad_role.yaml", "role: leg", "role: wing", "未知关节角色应拒绝"});
}

// 关节名称、有限值和范围等结构错误应拒绝加载。
void test_robot_rejections_structure()
{
    config_test::expect_robot_rejected(
        {"qc_robot_duplicate.yaml", "joint_b", "joint_a", "重复关节名称应拒绝"});
    config_test::expect_robot_rejected(
        {"qc_robot_nan.yaml", "max_velocity: 20.0", "max_velocity: .nan",
            "非有限限制值应拒绝"});
    config_test::expect_robot_rejected(
        {"qc_robot_range.yaml", "min_position: -3.0", "min_position: 4.0",
            "位置下限大于上限应拒绝"});
    config_test::expect_robot_rejected(
        {"qc_robot_scalar_joint.yaml", "  - {name: joint_a", "  - 5\n  - {name: joint_a",
            "关节条目为标量应拒绝"});
}

// 根节点不是映射时加载接口应拒绝并返回错误，而不是抛出 YAML 异常。
void test_root_node_rejections()
{
    config_test::expect_robot_root_rejected(
        {"qc_robot_scalar_root.yaml", "5", "标量根节点的机器人配置应拒绝"});
    config_test::expect_robot_root_rejected(
        {"qc_robot_seq_root.yaml", "[1, 2]", "序列根节点的机器人配置应拒绝"});
}

}  // namespace

int main()
{
    test_load_real_configs();
    test_robot_rejections_scalars();
    test_robot_rejections_structure();
    test_root_node_rejections();

    if (config_test::failures > 0)
    {
        std::cerr << config_test::failures << " 个机器人配置测试失败\n";
        return 1;
    }
    std::cout << "全部机器人配置测试通过\n";
    return 0;
}
