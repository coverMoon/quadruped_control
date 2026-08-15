/**
 * @file state_frame_fill.cpp
 * @brief 实现从当前 mjData 填充 StateFrame 关节和 IMU 字段的辅助函数。
 */

#include "state_frame_fill.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <string>

namespace quadruped::backends::mujoco
{
namespace
{

// 四元数模长与 1 的最大允许偏差。framequat 由有效旋转矩阵转换而来，理论上模长为 1；
// 这里只用于捕获明显的数值异常，避免用过大容差掩盖错误。
constexpr double kQuaternionNormTolerance = 1.0e-6;

// 检查四元数模长是否接近 1，用于捕获 NaN、Inf 或严重失真的姿态。
bool quaternion_is_unit(const std::array<double, 4>& quat)
{
    const double norm_sq =
        quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3];
    return std::abs(std::sqrt(norm_sq) - 1.0) <= kQuaternionNormTolerance;
}

}  // 匿名命名空间

std::string fill_joint_states(
    core::StateFrame& frame,
    const MujocoModel& model,
    const core::RobotModel& robot_model)
{
    const mjModel* raw = model.raw_model();
    const mjData* data = model.raw_data();

    for (std::size_t i = 0; i < robot_model.joint_count; ++i)
    {
        const auto& mapping = model.joint_mappings()[i];
        if (mapping.qpos_address < 0 || mapping.qpos_address >= raw->nq ||
            mapping.qvel_address < 0 || mapping.qvel_address >= raw->nv)
        {
            return "joint address is out of range for joint \"" +
                robot_model.joints[i].name + "\"";
        }

        // qfrc_actuator 是传动作用到关节自由度后的广义力，对 hinge 关节即关节力矩；
        // 不直接把 ctrl 当成实际力矩，也不无条件使用 actuator_force。
        const double position = data->qpos[mapping.qpos_address];
        const double velocity = data->qvel[mapping.qvel_address];
        const double effort = data->qfrc_actuator[mapping.qvel_address];
        if (!std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(effort))
        {
            return "joint state is not finite for joint \"" +
                robot_model.joints[i].name + "\"";
        }

        frame.joints[i].position = position;
        frame.joints[i].velocity = velocity;
        frame.joints[i].effort = effort;
        // 仿真当前没有温度、通信时延和硬件故障模型，这里以 0/true 表示无故障默认值。
        frame.joints[i].temperature_c = 0.0;
        frame.joints[i].error_code = 0;
        frame.joints[i].age_ns = 0;
        frame.joints[i].online = true;
        frame.joints[i].valid = true;
    }
    return {};
}

std::string fill_imu_state(core::StateFrame& frame, const MujocoModel& model)
{
    const mjModel* raw = model.raw_model();
    const mjData* data = model.raw_data();
    const auto& imu = model.imu_mapping();

    if (imu.quat.data_address < 0 || imu.quat.data_address + 4 > raw->nsensordata ||
        imu.gyro.data_address < 0 || imu.gyro.data_address + 3 > raw->nsensordata ||
        imu.acc.data_address < 0 || imu.acc.data_address + 3 > raw->nsensordata)
    {
        return "IMU sensor address is out of range";
    }

    // MuJoCo framequat 输出顺序固定为 w、x、y、z，与 StateFrame 一致，无需重排。
    for (std::size_t i = 0; i < 4; ++i)
    {
        frame.imu.orientation[i] = data->sensordata[imu.quat.data_address + i];
    }
    // gyro 和 accelerometer 都位于 imu site 的局部坐标系。
    // 当前 imu site 相对机体的姿态是单位四元数，因此局部坐标系与机体坐标系一致；
    // 以后 site 安装姿态改变时，不能再默认二者相同。
    for (std::size_t i = 0; i < 3; ++i)
    {
        frame.imu.angular_velocity[i] = data->sensordata[imu.gyro.data_address + i];
        frame.imu.linear_acceleration[i] = data->sensordata[imu.acc.data_address + i];
    }

    for (const double value : frame.imu.orientation)
    {
        if (!std::isfinite(value))
        {
            return "IMU orientation is not finite";
        }
    }
    for (const double value : frame.imu.angular_velocity)
    {
        if (!std::isfinite(value))
        {
            return "IMU angular velocity is not finite";
        }
    }
    for (const double value : frame.imu.linear_acceleration)
    {
        if (!std::isfinite(value))
        {
            return "IMU linear acceleration is not finite";
        }
    }
    if (!quaternion_is_unit(frame.imu.orientation))
    {
        return "IMU orientation quaternion is not close to unit norm";
    }

    frame.imu.age_ns = 0;
    frame.imu.valid = true;
    return {};
}

}  // 命名空间 quadruped::backends::mujoco
