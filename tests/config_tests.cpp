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
    expect(robot.model.model_id == 0x008A1E56CD69E8F4, "black model_id 应保持稳定");
    expect(robot.model.joint_count == 12, "black 应有 12 个关节");
    expect(robot.model.joints[0].name == "FL_hip_joint", "第一个关节应为 FL_hip_joint");
    expect(robot.model.joints[11].name == "RR_calf_joint", "最后一个关节应为 RR_calf_joint");
    expect(robot.model.joints[0].role == qc::JointRole::Leg, "black 关节角色应为 leg");
    expect(robot.model.joints[0].limits.position_limited, "black 关节应有位置限制");
    expect(robot.model.joints[0].limits.min_position == -10.0,
        "black 位置限制应与 MuJoCo 模型一致");
    expect(robot.model.joints[0].limits.max_kp == 100.0, "black KP 上限应为 100");
}

// 加载 model_id 为指定字面量的最小配置，返回解析结果；失败时记录断言并返回 0。
std::uint64_t load_model_id_literal(const std::string& literal, const std::string& description)
{
    std::string yaml = config_test::make_robot_yaml();
    const auto pos = yaml.find("model_id: 0x1234");
    yaml.replace(pos, std::strlen("model_id: 0x1234"), "model_id: " + literal);
    const auto result = quadruped::config::load_robot_model(
        config_test::write_temp_config("qc_id_literal.yaml", yaml));
    expect(result.ok(), description + "（应加载成功）");
    return result.ok() ? result.model.model_id : 0;
}

// 严格无符号整数字面量：前导零按十进制，仅 0x/0X 前缀按十六进制。
void test_model_id_literals()
{
    expect(load_model_id_literal("010", "前导零的十进制") == 10,
        "model_id: 010 应解析为十进制 10");
    expect(load_model_id_literal("08", "前导零且含 8 的十进制") == 8,
        "model_id: 08 应解析为十进制 8");
    expect(load_model_id_literal("0x10", "0x 前缀十六进制") == 16,
        "model_id: 0x10 应解析为十六进制 16");
    expect(load_model_id_literal("0X1A", "0X 前缀十六进制") == 26,
        "model_id: 0X1A 应解析为十六进制 26");

    config_test::expect_robot_rejected(
        {"qc_robot_space_neg.yaml", "model_id: 0x1234", "model_id: \" -1\"",
            "带前导空白的负数 model_id 应拒绝"});
    config_test::expect_robot_rejected(
        {"qc_robot_hex_space.yaml", "model_id: 0x1234", "model_id: \"0x 10\"",
            "十六进制数字部分带空格应拒绝"});
    config_test::expect_robot_rejected(
        {"qc_robot_hex_tab.yaml", "model_id: 0x1234", "model_id: \"0x\\t10\"",
            "十六进制数字部分带 Tab 应拒绝"});
}

// 缺字段、未知角色、非法和负数 model_id 等标量字段错误应拒绝加载。
void test_robot_rejections_scalars()
{
    const auto missing = quadruped::config::load_robot_model("qc_no_such_file.yaml");
    expect(!missing.ok(), "不存在的文件应拒绝加载");

    config_test::expect_robot_rejected(
        {"qc_robot_missing_kp.yaml", ", max_kp: 100.0", "", "缺少 max_kp 字段应拒绝"});
    config_test::expect_robot_rejected(
        {"qc_robot_bad_role.yaml", "role: leg", "role: wing", "未知关节角色应拒绝"});
    config_test::expect_robot_rejected(
        {"qc_robot_bad_id.yaml", "model_id: 0x1234", "model_id: black",
            "非数值 model_id 应拒绝"});
    config_test::expect_robot_rejected(
        {"qc_robot_neg_id.yaml", "model_id: 0x1234", "model_id: -1",
            "负数 model_id 应拒绝"});
    config_test::expect_robot_rejected(
        {"qc_robot_neg_calib.yaml", "calibration_id: 0", "calibration_id: -1",
            "负数 calibration_id 应拒绝"});
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
    test_model_id_literals();
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
