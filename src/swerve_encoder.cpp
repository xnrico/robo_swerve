#include "swerve_encoder.hpp"

namespace robo {
swerve_encoder::swerve_encoder() : name(BASE_NAME + std::to_string(ENCODER_ID)), read_error(false) {
  ENCODER_ID++;
}  // Default constructor

swerve_encoder::swerve_encoder(const std::string& name_)
    : name(name_), encoder_id(ENCODER_ID++), read_error(false) {}  // Constructor with custom name

swerve_encoder::~swerve_encoder() = default;  // Default destructor

swerve_cancoder::swerve_cancoder()
    : swerve_encoder(),
      encoder(std::make_shared<hardware::CANcoder>(this->encoder_id)),
      magnet_health(encoder->GetMagnetHealth()),
      angle(encoder->GetAbsolutePosition()),
      velocity(encoder->GetVelocity()),
      configurator(encoder->GetConfigurator()) {
  this->factory_reset();
  this->clear_faults();
}

swerve_cancoder::swerve_cancoder(const std::string& name_)
    : swerve_encoder(name_),
      encoder(std::make_shared<hardware::CANcoder>(this->encoder_id)),
      magnet_health(encoder->GetMagnetHealth()),
      angle(encoder->GetAbsolutePosition()),
      velocity(encoder->GetVelocity()),
      configurator(encoder->GetConfigurator()) {
  this->factory_reset();
  this->clear_faults();
}  // Constructor with custom name

swerve_cancoder::swerve_cancoder(const std::string& name_, std::shared_ptr<hardware::CANcoder> encoder_)
    : swerve_encoder(name_),
      encoder(std::move(encoder_)),
      magnet_health(encoder->GetMagnetHealth()),
      angle(encoder->GetAbsolutePosition()),
      velocity(encoder->GetVelocity()),
      configurator(encoder->GetConfigurator()) {
  this->factory_reset();
  this->clear_faults();
}  // Constructor with custom name and encoder

auto swerve_cancoder::factory_reset() -> void {
  config = configs::CANcoderConfiguration{};
  configurator.Apply(config);  // Apply the default configuration
}  // Factory reset the encoder

auto swerve_cancoder::clear_faults() -> void {
  if (encoder) encoder->ClearStickyFaults();  // Clear any faults in the encoder
}  // Clear faults in the encoder

auto swerve_cancoder::configure(bool inverted) -> void {
  configurator.Refresh(config);  // Refresh the current configuration
  config.MagnetSensor = config.MagnetSensor.WithAbsoluteSensorDiscontinuityPoint(units::angle::turn_t{1})
                            .WithSensorDirection(inverted ? signals::SensorDirectionValue::Clockwise_Positive
                                                          : signals::SensorDirectionValue::CounterClockwise_Positive);
  configurator.Apply(config);  // Apply the configuration with the specified direction
}  // Configure the encoder

auto swerve_cancoder::get_position() -> double {
  read_error = false;  // Reset read error flag
  signals::MagnetHealthValue health = magnet_health.Refresh().GetValue();
  angle.Refresh();  // Refresh the angle status

  if (health == signals::MagnetHealthValue::Magnet_Invalid || health == signals::MagnetHealthValue::Magnet_Red) {
    read_error = true;  // Set read error if magnet health is invalid or red
    return 0.0;         // Return 0.0 if there is a read error
  }

  for (auto i = 0; i < MAX_RETRIES; ++i) {
    if (angle.GetStatus() == ctre::phoenix::StatusCode::OK) break;
    angle.WaitForUpdate(units::time::second_t{STATUS_TIMEOUT});  // Wait for the angle to update
  }

  if (angle.GetStatus() != ctre::phoenix::StatusCode::OK) {
    read_error = true;  // Set read error if angle status is not OK
    return 0.0;         // Return 0.0 if there is a read error
  }

  return angle.GetValueAsDouble() * 360.0;
}  // Get the absolute position in degrees

auto swerve_cancoder::get_velocity() -> double {
  return velocity.Refresh().GetValueAsDouble() * 360.0;  // Return velocity in degrees per second
}  // Get the velocity in degrees per second

auto swerve_cancoder::set_offset(double offset) -> bool {        // offset in degrees
  ctre::phoenix::StatusCode err = configurator.Refresh(config);  // Refresh the configuration
  if (err != ctre::phoenix::StatusCode::OK) return false;

  err = configurator.Apply(
      config.MagnetSensor.WithMagnetOffset(units::angle::turn_t{offset / 360.0}));  // Set the magnet offset
  return err == ctre::phoenix::StatusCode::OK ? true : false;                       // Return true indicating success
}
}  // namespace robo