#include "swerve_imu.hpp"

using namespace ctre::phoenix6;

namespace robo {
swerve_imu::swerve_imu() : imu_id(IMU_ID++) { name = BASE_NAME + std::to_string(imu_id); }

swerve_imu::swerve_imu(const std::string& name_) : imu_id(IMU_ID++) { name = BASE_NAME + name_; }

swerve_imu::~swerve_imu() = default;

swerve_pigeon2::swerve_pigeon2()
    : swerve_imu(),
      imu(std::make_shared<hardware::Pigeon2>(imu_id)),
      configurator(imu->GetConfigurator()),
      offset(0.0, 0.0),
      is_inverted(false) {}

swerve_pigeon2::swerve_pigeon2(const std::string& name_)
    : swerve_imu(name_),
      imu(std::make_shared<hardware::Pigeon2>(imu_id)),
      configurator(imu->GetConfigurator()),
      offset(0.0, 0.0),
      is_inverted(false) {}

swerve_pigeon2::swerve_pigeon2(const std::string& name_, std::shared_ptr<hardware::Pigeon2> imu_)
    : swerve_imu(name_),
      imu(std::move(imu_)),
      configurator(imu->GetConfigurator()),
      offset(0.0, 0.0),
      is_inverted(false) {}

auto swerve_pigeon2::factory_reset() -> void {
  config = configs::Pigeon2Configuration{};
  configurator.Apply(config);
}

auto swerve_pigeon2::clear_faults() -> void { imu->ClearStickyFaults(); }

auto swerve_pigeon2::set_offset(const swerve::Rotation2d& offset_) -> void { this->offset = offset_; }

auto swerve_pigeon2::set_inverted(bool inverted) noexcept -> void { this->is_inverted = inverted; }

auto swerve_pigeon2::get_rotation() const -> swerve::Rotation2d { return get_rotation_no_offset().RotateBy(offset); }

auto swerve_pigeon2::get_rotation_no_offset() const -> swerve::Rotation2d {
  auto rotation = swerve::Rotation2d{units::angular_velocity::turns_per_second_t{imu->GetYaw().GetValue()}};
  return is_inverted ? -rotation : rotation;
}

auto swerve_pigeon2::get_acceleration() const -> swerve::Translation2d {
  auto x_accel = units::acceleration::meters_per_second_squared_t{imu->GetAccelerationX().GetValueAsDouble()};
  auto y_accel = units::acceleration::meters_per_second_squared_t{imu->GetAccelerationY().GetValueAsDouble()};

  return swerve::Translation2d{x_accel, y_accel};  // units in m/s^2
}

auto swerve_pigeon2::get_yaw_velocity() const -> units::angular_velocity::turns_per_second_t {
  return units::angular_velocity::turns_per_second_t{imu->GetYaw().GetValue()};
}

}  // namespace robo