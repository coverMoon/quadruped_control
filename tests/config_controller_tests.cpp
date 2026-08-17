/**
 * @file config_controller_tests.cpp
 * @brief 测试 ControllerConfig YAML 加载的合法路径、拒绝路径和根节点异常契约。
 */

#include "config_test_helpers.hpp"

#include "quadruped/core/core.hpp"

namespace
{

using config_test::expect;

void test_load_real_controller_config()
{
    const auto robot = quadruped::config::load_robot_model(QUADRUPED_ROBOT_CONFIG_PATH);
    expect(robot.ok(), "仓库 black 机器人配置应加载成功");
    if (!robot.ok())
    {
        return;
    }
    const auto controller = quadruped::config::load_controller_config(
        QUADRUPED_CONTROLLER_CONFIG_PATH, robot.model);
    expect(controller.ok(), "仓库 black 控制器配置应加载成功");
    if (!controller.ok())
    {
        return;
    }
    expect(controller.config.control_period_ns == 5'000'000, "控制周期应为 5 ms");
    expect(controller.config.command_validity_ns == 10'000'000, "命令有效期应为 10 ms");
    expect(controller.config.getup_pre_cycles == 100, "起立第一段应为 100 周期");
    expect(controller.config.getup_cycles == 100, "起立第二段应为 100 周期");
    expect(controller.config.getdown_cycles == 500, "趴下应为 500 周期");
    expect(controller.config.pre_getup_position[1] == 1.4, "预起立大腿姿态应为 1.4");
    expect(controller.config.stand_position[2] == -1.5, "站姿小腿姿态应为 -1.5");
    expect(controller.config.fixed_kp[0] == 80.0, "固定 KP 应为 80");
    expect(controller.config.fixed_kd[11] == 3.0, "固定 KD 应为 3");
}

void test_controller_rejections()
{
    const qc::RobotModel model = config_test::load_test_model();
    if (model.joint_count == 0)
    {
        return;
    }

    const auto missing =
        quadruped::config::load_controller_config("qc_no_such_file.yaml", model);
    expect(!missing.ok(), "不存在的控制器文件应拒绝加载");

    config_test::expect_controller_rejected(
        model, {"qc_ctrl_name.yaml", "name: test_quadruped", "name: other_robot",
            "控制器名称与 RobotModel 不一致应拒绝"});
    config_test::expect_controller_rejected(
        model, {"qc_ctrl_order.yaml", "[joint_a, joint_b]", "[joint_b, joint_a]",
            "关节顺序错误应拒绝"});
    config_test::expect_controller_rejected(
        model, {"qc_ctrl_length.yaml", "fixed_kp: [80.0, 80.0]", "fixed_kp: [80.0]",
            "增益数组长度错误应拒绝"});
    config_test::expect_controller_rejected(
        model, {"qc_ctrl_inf.yaml", "stand_position: [0.0, 0.82]",
            "stand_position: [0.0, .inf]", "非有限姿态值应拒绝"});
    config_test::expect_controller_rejected(
        model, {"qc_ctrl_pose.yaml", "pre_getup_position: [0.0, 1.4]",
            "pre_getup_position: [0.0, 3.5]", "姿态越过关节位置限制应拒绝"});
    config_test::expect_controller_rejected(
        model, {"qc_ctrl_kp.yaml", "fixed_kp: [80.0, 80.0]", "fixed_kp: [120.0, 80.0]",
            "KP 越过模型上限应拒绝"});
    config_test::expect_controller_rejected(
        model, {"qc_ctrl_cycles.yaml", "getup_cycles: 100", "getup_cycles: 0",
            "插值周期数为 0 应拒绝"});
    config_test::expect_controller_rejected(
        model, {"qc_ctrl_period.yaml", "control_period_ms: 5", "control_period_ms: 0",
            "控制周期为 0 应拒绝"});
    config_test::expect_controller_rejected(
        model, {"qc_ctrl_period_huge.yaml", "control_period_ms: 5",
            "control_period_ms: 9223372036854775807", "换算溢出的控制周期应拒绝"});
    config_test::expect_controller_rejected(
        model, {"qc_ctrl_validity.yaml", "command_validity_ms: 10", "command_validity_ms: 5",
            "命令有效期不超过一个控制周期应拒绝"});
    config_test::expect_controller_rejected(
        model, {"qc_ctrl_names_type.yaml", "joint_names: [joint_a, joint_b]",
            "joint_names: [joint_a, {}]", "joint_names 非字符串条目应拒绝且不抛异常"});
}

// 根节点不是映射时加载接口应拒绝并返回错误，而不是抛出 YAML 异常。
void test_root_node_rejections()
{
    const qc::RobotModel model = config_test::load_test_model();
    if (model.joint_count == 0)
    {
        return;
    }
    config_test::expect_controller_root_rejected(
        model, {"qc_ctrl_scalar_root.yaml", "5", "标量根节点的控制器配置应拒绝"});
    config_test::expect_controller_root_rejected(
        model, {"qc_ctrl_seq_root.yaml", "[1, 2]", "序列根节点的控制器配置应拒绝"});
}

}  // namespace

int main()
{
    test_load_real_controller_config();
    test_controller_rejections();
    test_root_node_rejections();

    if (config_test::failures > 0)
    {
        std::cerr << config_test::failures << " 个控制器配置测试失败\n";
        return 1;
    }
    std::cout << "全部控制器配置测试通过\n";
    return 0;
}
