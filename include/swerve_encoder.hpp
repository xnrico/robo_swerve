#pragma once

#include <ctre/phoenix/StatusCodes.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <memory>
#include <string>
#include <cstddef>

using namespace ctre::phoenix6;
namespace robo {

class swerve_encoder {
 private:
  static size_t ENCODER_ID;
  static constexpr auto BASE_NAME = "swerve_encoder_";

 protected:
  static constexpr auto MAX_RETRIES = 5U;

  std::string name;
  size_t encoder_id;
  bool read_error;

 public:
  swerve_encoder();
  swerve_encoder(const std::string& name_);
  virtual ~swerve_encoder() = 0;

  virtual auto factory_reset() -> void = 0;
  virtual auto clear_faults() -> void = 0;
  virtual auto configure(bool inverted) -> void = 0;
  virtual auto get_position() -> double = 0;           // absolute position in degrees
  virtual auto get_velocity() -> double = 0;           // velocity in degrees per second
  virtual auto set_offset(double offset) -> bool = 0;  // set the offset in degrees
};

class swerve_cancoder : public swerve_encoder {
 private:
  static constexpr auto STATUS_TIMEOUT = 0.001;  // in seconds

  std::shared_ptr<hardware::CANcoder> encoder;

  StatusSignal<signals::MagnetHealthValue> magnet_health;              // Magnet health status
  StatusSignal<units::angle::turn_t> angle;                            // Angle status
  StatusSignal<units::angular_velocity::turns_per_second_t> velocity;  // Velocity status

  configs::CANcoderConfiguration config;
  configs::CANcoderConfigurator& configurator;

 public:
  swerve_cancoder();
  swerve_cancoder(const std::string& name_);
  swerve_cancoder(const std::string& name_, std::shared_ptr<hardware::CANcoder> encoder_);

  auto factory_reset() -> void override;
  auto clear_faults() -> void override;
  auto configure(bool inverted) -> void override;
  auto get_position() -> double override;           // absolute position in degrees
  auto get_velocity() -> double override;           // velocity in degrees per second
  auto set_offset(double offset) -> bool override;  // set the offset in degrees
};
}  // namespace robo