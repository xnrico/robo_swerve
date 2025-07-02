#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>
#include <memory>
#include <string>

#include "swerve_common.hpp"

using namespace ctre::phoenix6;

namespace robo {
class swerve_motor {
 private:
  static size_t MOTOR_ID;
  static constexpr auto BASE_NAME = "swerve_motor_";

 protected:
  std::string name;
  size_t motor_id;

 public:
  swerve_motor();
  swerve_motor(const std::string& name_);
  virtual ~swerve_motor() = 0;

  virtual auto factory_reset() -> void = 0;
  virtual auto clear_faults() -> void = 0;
  virtual auto configure_encoder(double conversion_factor) -> void = 0;
  virtual auto configure_pidf(pidf_config config) -> void = 0;
  virtual auto configure_pid_wrap(double min_input, double max_input) -> void = 0;
  virtual auto disable_pid_wrap() -> void = 0;
  virtual auto set_motor_brake() -> void = 0;
  virtual auto set_motor_coast() -> void = 0;
  virtual auto set_motor_inverted(bool inverted) -> void = 0;
  virtual auto get_output() -> double = 0;
  virtual auto set_output(double percentage) -> void = 0;
  virtual auto set_reference(double setpoint, double kV) -> void = 0;  // Feedforward kV in volt-meter/s
  virtual auto get_voltage() -> double = 0;
  virtual auto set_voltage(double voltage) -> void = 0;
  virtual auto get_velocity() -> double = 0;
  virtual auto get_position() -> double = 0;
  virtual auto set_position(double position) -> void = 0;
  virtual auto set_current_limit(int max_amps) -> void = 0;
  virtual auto set_ramp_rate(double ramp_rate) -> void = 0;
};

class swerve_talon : public swerve_motor {
 private:
  static constexpr double STATUS_TIMEOUT = 0.02;  // in seconds

  std::shared_ptr<hardware::TalonFX> motor;

  configs::TalonFXConfiguration config;
  configs::TalonFXConfigurator& configurator;

  controls::MotionMagicVoltage angle_voltage;  // angle_voltage setter
  controls::VelocityVoltage velocity_voltage;  // velocity_voltage setter

  double conversion_factor;
  bool is_drive_motor;
  bool is_factory_reset;

 private:
  swerve_talon();
  swerve_talon(const std::string& name_);
  swerve_talon(const std::string& name_, bool is_drive_motor_);
  swerve_talon(const std::string& name_, std::shared_ptr<hardware::TalonFX> motor_, bool is_drive_motor_);

  auto factory_reset() -> void override;
  auto clear_faults() -> void override;
  auto configure_encoder(double pos_conversion_factor) -> void override;
  auto configure_pidf(pidf_config cfg) -> void override;
  auto configure_pid_wrap(double min_input, double max_input) -> void override;
  auto disable_pid_wrap() -> void override;
  auto set_motor_brake() -> void override;
  auto set_motor_coast() -> void override;
  auto set_motor_inverted(bool inverted) -> void override;
  auto get_output() -> double override = 0;
  auto set_output(double percentage) -> void override;
  auto set_reference(double setpoint, double kV) -> void override;
  auto get_voltage() -> double override;
  auto set_voltage(double voltage) -> void override;
  auto get_velocity() -> double override;
  auto get_position() -> double override;
  auto set_position(double position) -> void override;
  auto set_current_limit(int max_amps) -> void override;
  auto set_ramp_rate(double ramp_rate) -> void override;
  auto get_motor() -> std::shared_ptr<hardware::TalonFX>;
};

};  // namespace robo
