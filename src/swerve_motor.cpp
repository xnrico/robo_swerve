#include "swerve_motor.hpp"

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_jerk.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/frequency.h>
#include <units/time.h>
#include <units/voltage.h>

#include <ctre/phoenix6/signals/SpnEnums.hpp>

namespace robo {
size_t swerve_motor::MOTOR_ID = 0;  // Initialize static member variable

swerve_motor::swerve_motor() : name(BASE_NAME + std::to_string(MOTOR_ID)), motor_id(MOTOR_ID) { MOTOR_ID++; }

swerve_motor::swerve_motor(const std::string& name_) : name(name_), motor_id(MOTOR_ID) { MOTOR_ID++; }

swerve_motor::~swerve_motor() = default;  // Default destructor

swerve_talon::swerve_talon()
    : swerve_motor(),
      motor(std::make_shared<hardware::TalonFX>(this->motor_id)),
      configurator(motor->GetConfigurator()),
      angle_voltage(units::angle::turn_t{0.0}),
      velocity_voltage(units::angular_velocity::turns_per_second_t{0.0}),
      is_drive_motor(true),
      is_factory_reset(false) {
  this->factory_reset();
  this->clear_faults();
}

swerve_talon::swerve_talon(const std::string& name_)
    : swerve_motor(name_),
      motor(std::make_shared<hardware::TalonFX>(this->motor_id)),
      configurator(motor->GetConfigurator()),
      angle_voltage(units::angle::turn_t{0.0}),
      velocity_voltage(units::angular_velocity::turns_per_second_t{0.0}),
      is_drive_motor(true),
      is_factory_reset(false) {
  this->factory_reset();
  this->clear_faults();
}

swerve_talon::swerve_talon(const std::string& name_, bool is_drive_motor_)
    : swerve_motor(name_),
      motor(std::make_shared<hardware::TalonFX>(this->motor_id)),
      configurator(motor->GetConfigurator()),
      angle_voltage(units::angle::turn_t{0.0}),
      velocity_voltage(units::angular_velocity::turns_per_second_t{0.0}),
      is_drive_motor(is_drive_motor_),
      is_factory_reset(false) {
  this->factory_reset();
  this->clear_faults();
}

swerve_talon::swerve_talon(const std::string& name_, std::shared_ptr<hardware::TalonFX> motor_, bool is_drive_motor_)
    : swerve_motor(name_),
      motor(std::move(motor_)),
      configurator(motor->GetConfigurator()),
      angle_voltage(units::angle::turn_t{0.0}),
      velocity_voltage(units::angular_velocity::turns_per_second_t{0.0}),
      is_drive_motor(is_drive_motor_),
      is_factory_reset(false) {
  this->factory_reset();
  this->clear_faults();
}

auto swerve_talon::factory_reset() -> void {
  if (is_factory_reset) return;  // Avoid multiple resets

  config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
  config.ClosedLoopGeneral.ContinuousWrap = true;
  configurator.Apply(config);

  angle_voltage.UpdateFreqHz = 0_Hz;     // Disable periodic updastes
  velocity_voltage.UpdateFreqHz = 0_Hz;  // Disable periodic updates
}

auto swerve_talon::clear_faults() -> void {
  if (motor) motor->ClearStickyFaults();
}

// Degrees factor: 360 / (angle_gear_ratio * 1 rotation of encoder)
// Meters factor: (PI * wheel_diameter_in_meters) / (drive_gear_ratio * 1 rotation of encoder)
auto swerve_talon::configure_encoder(double pos_conversion_factor) -> void {
  configurator.Refresh(this->config);

  // set the conversion factor to ticks / (degrees or meters)
  pos_conversion_factor =
      1.0 / pos_conversion_factor;  // (drive_gear_ratio * 1 rotation of encoder) / (wheel circumference)
  if (!is_drive_motor) pos_conversion_factor *= 360.0;  // angle_gear_ratio * 1 rotation of encoder

  this->conversion_factor = pos_conversion_factor;  // in rotations per degree or 1 rotation of encoder per meter

  config.MotionMagic =
      config.MotionMagic
          .WithMotionMagicCruiseVelocity(units::angular_velocity::turns_per_second_t{100.0} /
                                         pos_conversion_factor)  // Max 6000 RPM
          .WithMotionMagicAcceleration(
              (units::angular_acceleration::turns_per_second_squared_t{100.0} / pos_conversion_factor) /
              0.100)  // Max 1000 RPM /s
          .WithMotionMagicExpo_kV(ctre::unit::volts_per_turn_per_second_t{0.12} *
                                  pos_conversion_factor)  // Default 0.12 V/(rps)
          .WithMotionMagicExpo_kA(ctre::unit::volts_per_turn_per_second_squared_t{0.1});

  config.Feedback.WithFeedbackSensorSource(ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor)
      .WithSensorToMechanismRatio(pos_conversion_factor);

  configurator.Apply(this->config);
}

auto swerve_talon::configure_pidf(pidf_config cfg) -> void {
  configurator.Refresh(this->config.Slot0);  // only configure Slot0 for now
  config.Slot0 = config.Slot0.WithKP(cfg.kP).WithKI(cfg.kI).WithKD(cfg.kD).WithKS(cfg.kF);
  configurator.Apply(this->config.Slot0);
}

auto swerve_talon::configure_pid_wrap(double min_input, double max_input) -> void {
  // custom ranges are not supported by TalonFX, so we use the default range
  configurator.Refresh(this->config.ClosedLoopGeneral);
  config.ClosedLoopGeneral.ContinuousWrap = true;
  configurator.Apply(this->config.ClosedLoopGeneral);
}

auto swerve_talon::disable_pid_wrap() -> void {
  configurator.Refresh(this->config.ClosedLoopGeneral);
  config.ClosedLoopGeneral.ContinuousWrap = false;
  configurator.Apply(this->config.ClosedLoopGeneral);
}

auto swerve_talon::set_motor_brake() -> void {
  configurator.Refresh(this->config.MotorOutput);
  config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  configurator.Apply(this->config.MotorOutput);
}

auto swerve_talon::set_motor_coast() -> void {
  configurator.Refresh(this->config.MotorOutput);
  config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
  configurator.Apply(this->config.MotorOutput);
}

auto swerve_talon::set_motor_inverted(bool inverted) -> void {
  configurator.Refresh(this->config.MotorOutput);
  config.MotorOutput.Inverted = inverted ? ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive
                                         : ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
  configurator.Apply(this->config.MotorOutput);
}

auto swerve_talon::get_output() -> double {
  if (motor) return motor->GetDutyCycle().WaitForUpdate(units::time::second_t{STATUS_TIMEOUT}).GetValueAsDouble();
  return double{};  // Return 0.0 if motor is not initialized
}

auto swerve_talon::set_output(double percentage) -> void {  // this API is for non-FRC applications
  if (motor) motor->SetControl(controls::DutyCycleOut{percentage});
}

// setpoint in meters per second for drive motors, degrees for angle motors
auto swerve_talon::set_reference(double setpoint, double kV) -> void {
  if (motor) {
    if (is_drive_motor) {
      setpoint *= conversion_factor;  // Convert setpoint to the correct units (rotations per second)
      motor->SetControl(velocity_voltage.WithVelocity(units::angular_velocity::turns_per_second_t{setpoint})
                            .WithFeedForward(units::voltage::volt_t{kV}));
    } else {
      setpoint /= 360.0;  // Convert setpoint to turns (1 rotation = 360 degrees)
      motor->SetControl(angle_voltage.WithPosition(units::angle::turn_t{setpoint}));
    }
  }
}

auto swerve_talon::get_voltage() -> double {
  if (motor) return motor->GetMotorVoltage().WaitForUpdate(units::time::second_t{STATUS_TIMEOUT}).GetValueAsDouble();
  return double{};  // Return 0.0 if motor is not initialized
}

auto swerve_talon::set_voltage(double voltage) -> void {
  if (motor) motor->SetControl(controls::VoltageOut{units::voltage::volt_t{voltage}});
}

auto swerve_talon::get_velocity() -> double {
  if (motor) return motor->GetVelocity().GetValueAsDouble();
  return double{};  // Return 0.0 if motor is not initialized
}

auto swerve_talon::get_position() -> double {
  if (motor) return motor->GetPosition().GetValueAsDouble();
  return double{};  // Return 0.0 if motor is not initialized
}

auto swerve_talon::set_position(double position) -> void {           // Position in degrees
  configurator.SetPosition(units::angle::turn_t{position / 360.0});  // Convert to turns
}

auto swerve_talon::set_current_limit(int max_amps) -> void {
  configurator.Refresh(this->config.CurrentLimits);
  config.CurrentLimits = config.CurrentLimits.WithSupplyCurrentLimit(units::current::ampere_t{max_amps})
                             .WithSupplyCurrentLimitEnable(true);
  configurator.Apply(this->config.CurrentLimits);
}

// Ramp rate in seconds for voltage to ramp from 0 to full voltage
auto swerve_talon::set_ramp_rate(double ramp_rate) -> void {
  configurator.Refresh(this->config.ClosedLoopRamps);
  config.ClosedLoopRamps = config.ClosedLoopRamps.WithVoltageClosedLoopRampPeriod(units::time::second_t{ramp_rate});
  configurator.Apply(this->config.ClosedLoopRamps);
}

}  // namespace robo
