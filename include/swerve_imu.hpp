#pragma once

#include <ctre/phoenix/StatusCodes.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>

#include <cstddef>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/swerve/utility/Geometry.hpp>
#include <functional>

using namespace ctre::phoenix6;

namespace robo {
class swerve_imu {
 private:
  static size_t IMU_ID;
  static constexpr auto BASE_NAME = "swerve_imu_";

 protected:
  std::string name;
  size_t imu_id;

 public:
  swerve_imu();
  swerve_imu(const std::string& name_);
  virtual ~swerve_imu() = 0;

  virtual auto factory_reset() -> void = 0;  // Reset the IMU to factory settings
  virtual auto clear_faults() -> void = 0;   // Clear any faults in the IMU
};

class swerve_pigeon2 : public swerve_imu {
 private:
  static constexpr auto STATUS_TIMEOUT = 0.04;  // in seconds

  std::shared_ptr<hardware::Pigeon2> imu;

  configs::Pigeon2Configuration config;        // Configuration for the Pigeon2 IMU
  configs::Pigeon2Configurator& configurator;  // Configurator for the Pigeon

  swerve::Rotation2d offset;                                 // Rotation status of the IMU

  bool is_inverted;  // Whether the IMU is inverted

 public:
  swerve_pigeon2();
  swerve_pigeon2(const std::string& name_);
  swerve_pigeon2(const std::string& name_, std::shared_ptr<hardware::Pigeon2> imu_);

  auto factory_reset() -> void override;
  auto clear_faults() -> void override;
  auto set_offset(const swerve::Rotation2d& offset_) -> void;  // Set the offset for the IMU
  auto set_inverted(bool inverted) noexcept -> void;           // Set whether the IMU is inverted
  auto get_rotation() const -> swerve::Rotation2d;             // Get the current rotation of the IMU
  auto get_rotation_no_offset() const -> swerve::Rotation2d;   // Get the rotation without offset
  auto get_acceleration() const -> swerve::Translation2d;      // Get the acceleration of the IMU
  auto get_yaw_velocity() const noexcept
      -> units::angular_velocity::turns_per_second_t;  // Get the angular velocity of the IMU
};

}  // namespace robo