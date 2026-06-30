#include "jetracer_control/jetracer_hardware.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace jetracer_control
{

// ---- PCA9685 register map ----
static constexpr uint8_t PCA9685_MODE1 = 0x00;
static constexpr uint8_t PCA9685_PRESCALE = 0xFE;
static constexpr uint8_t LED0_ON_L = 0x06;
static constexpr uint8_t MODE1_SLEEP = 0x10;
static constexpr uint8_t MODE1_AI = 0x20;     // auto-increment
static constexpr uint8_t MODE1_RESTART = 0x80;

PCA9685::~PCA9685()
{
  if (fd_ >= 0) {
    all_off();
    ::close(fd_);
  }
}

bool PCA9685::open_device(int bus, int addr)
{
  char path[32];
  std::snprintf(path, sizeof(path), "/dev/i2c-%d", bus);
  fd_ = ::open(path, O_RDWR);
  if (fd_ < 0) {
    return false;
  }
  if (::ioctl(fd_, I2C_SLAVE, addr) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  // Reset MODE1, enable auto-increment
  write8(PCA9685_MODE1, MODE1_AI);
  return true;
}

void PCA9685::write8(uint8_t reg, uint8_t value)
{
  ::i2c_smbus_write_byte_data(fd_, reg, value);
}

uint8_t PCA9685::read8(uint8_t reg)
{
  return static_cast<uint8_t>(::i2c_smbus_read_byte_data(fd_, reg));
}

void PCA9685::set_pwm_freq(double freq_hz)
{
  freq_hz_ = freq_hz;
  // datasheet: prescale = round(25MHz / (4096 * freq)) - 1
  double prescaleval = 25000000.0 / (4096.0 * freq_hz) - 1.0;
  uint8_t prescale = static_cast<uint8_t>(std::lround(prescaleval));

  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t sleepmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;
  write8(PCA9685_MODE1, sleepmode);          // go to sleep to set prescale
  write8(PCA9685_PRESCALE, prescale);
  write8(PCA9685_MODE1, oldmode);
  usleep(5000);
  write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}

void PCA9685::set_pwm(int channel, int on, int off)
{
  uint8_t reg = LED0_ON_L + 4 * channel;
  uint8_t data[4] = {
    static_cast<uint8_t>(on & 0xFF),
    static_cast<uint8_t>((on >> 8) & 0x0F),
    static_cast<uint8_t>(off & 0xFF),
    static_cast<uint8_t>((off >> 8) & 0x0F)};
  ::i2c_smbus_write_i2c_block_data(fd_, reg, 4, data);
}

void PCA9685::set_pulse_us(int channel, double pulse_us)
{
  double period_us = 1000000.0 / freq_hz_;
  int ticks = static_cast<int>(std::lround(pulse_us / period_us * 4096.0));
  if (ticks < 0) ticks = 0;
  if (ticks > 4095) ticks = 4095;
  set_pwm(channel, 0, ticks);
}

void PCA9685::all_off()
{
  if (fd_ < 0) return;
  for (int ch = 0; ch < 16; ++ch) {
    set_pwm(ch, 0, 0);
  }
}

// ---------------- JetracerHardware ----------------

namespace
{
double get_param(
  const hardware_interface::HardwareInfo & info, const std::string & key, double def)
{
  auto it = info.hardware_parameters.find(key);
  if (it != info.hardware_parameters.end()) {
    return std::stod(it->second);
  }
  return def;
}
}  // namespace

hardware_interface::CallbackReturn JetracerHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto logger = rclcpp::get_logger("JetracerHardware");

  // Expect exactly two joints: one steering (position), one traction (velocity)
  if (info_.joints.size() != 2) {
    RCLCPP_FATAL(logger, "Expected 2 joints (steering + traction), got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Identify joints by their declared command interface
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(logger, "Joint '%s' must have exactly 1 command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto & cmd = joint.command_interfaces[0].name;
    if (cmd == hardware_interface::HW_IF_POSITION) {
      steering_joint_ = joint.name;
    } else if (cmd == hardware_interface::HW_IF_VELOCITY) {
      traction_joint_ = joint.name;
    } else {
      RCLCPP_FATAL(logger, "Joint '%s' has unsupported command interface '%s'",
        joint.name.c_str(), cmd.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (steering_joint_.empty() || traction_joint_.empty()) {
    RCLCPP_FATAL(logger, "Need one position joint (steering) and one velocity joint (traction)");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read hardware params from URDF
  i2c_bus_ = static_cast<int>(get_param(info_, "i2c_bus", 1));
  i2c_address_ = static_cast<int>(get_param(info_, "i2c_address", 0x40));
  steering_channel_ = static_cast<int>(get_param(info_, "steering_channel", 0));
  throttle_channel_ = static_cast<int>(get_param(info_, "throttle_channel", 1));
  pwm_freq_ = get_param(info_, "pwm_frequency", 50.0);

  steer_center_us_ = get_param(info_, "steer_center_us", 1600.0);
  steer_us_per_rad_ = get_param(info_, "steer_us_per_rad", 500.0);
  steer_min_us_ = get_param(info_, "steer_min_us", 1100.0);
  steer_max_us_ = get_param(info_, "steer_max_us", 2100.0);

  throttle_neutral_us_ = get_param(info_, "throttle_neutral_us", 1600.0);
  throttle_us_per_rad_per_s_ = get_param(info_, "throttle_us_per_rad_per_s", 20.0);
  throttle_min_us_ = get_param(info_, "throttle_min_us", 1100.0);
  throttle_max_us_ = get_param(info_, "throttle_max_us", 2100.0);

  RCLCPP_INFO(logger, "JetracerHardware initialized: steering='%s', traction='%s'",
    steering_joint_.c_str(), traction_joint_.c_str());
  RCLCPP_INFO(logger, "I2C bus=%d addr=0x%02X steer_ch=%d throttle_ch=%d",
    i2c_bus_, i2c_address_, steering_channel_, throttle_channel_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JetracerHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("JetracerHardware");
  pca_ = std::make_unique<PCA9685>();
  if (!pca_->open_device(i2c_bus_, i2c_address_)) {
    RCLCPP_FATAL(logger, "Failed to open I2C bus %d addr 0x%02X", i2c_bus_, i2c_address_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  pca_->set_pwm_freq(pwm_freq_);

  // Center steering, neutral throttle on configure
  pca_->set_pulse_us(steering_channel_, steer_center_us_);
  pca_->set_pulse_us(throttle_channel_, throttle_neutral_us_);

  steering_position_command_ = 0.0;
  traction_velocity_command_ = 0.0;
  steering_position_state_ = 0.0;
  traction_velocity_state_ = 0.0;
  traction_position_state_ = 0.0;

  RCLCPP_INFO(logger, "JetracerHardware configured (I2C open).");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JetracerHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(
    steering_joint_, hardware_interface::HW_IF_POSITION, &steering_position_state_);
  state_interfaces.emplace_back(
    traction_joint_, hardware_interface::HW_IF_VELOCITY, &traction_velocity_state_);
  state_interfaces.emplace_back(
    traction_joint_, hardware_interface::HW_IF_POSITION, &traction_position_state_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JetracerHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(
    steering_joint_, hardware_interface::HW_IF_POSITION, &steering_position_command_);
  command_interfaces.emplace_back(
    traction_joint_, hardware_interface::HW_IF_VELOCITY, &traction_velocity_command_);
  return command_interfaces;
}

hardware_interface::CallbackReturn JetracerHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Ensure safe neutral state at activation
  steering_position_command_ = 0.0;
  traction_velocity_command_ = 0.0;
  if (pca_) {
    pca_->set_pulse_us(steering_channel_, steer_center_us_);
    pca_->set_pulse_us(throttle_channel_, throttle_neutral_us_);
  }
  RCLCPP_INFO(rclcpp::get_logger("JetracerHardware"), "Activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JetracerHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop the motor, center steering
  if (pca_) {
    pca_->set_pulse_us(throttle_channel_, throttle_neutral_us_);
    pca_->set_pulse_us(steering_channel_, steer_center_us_);
  }
  RCLCPP_INFO(rclcpp::get_logger("JetracerHardware"), "Deactivated (motor neutral).");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type JetracerHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // No encoders on stock JetRacer: echo commands as state (open loop).
  steering_position_state_ = steering_position_command_;
  traction_velocity_state_ = traction_velocity_command_;
  traction_position_state_ += traction_velocity_state_ * period.seconds();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JetracerHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!pca_) {
    return hardware_interface::return_type::ERROR;
  }

  // --- Steering: angle (rad) -> pulse (us) ---
  double steer_us = steer_center_us_ + steering_position_command_ * steer_us_per_rad_;
  if (steer_us < steer_min_us_) steer_us = steer_min_us_;
  if (steer_us > steer_max_us_) steer_us = steer_max_us_;
  pca_->set_pulse_us(steering_channel_, steer_us);

  // --- Throttle: wheel velocity (rad/s) -> pulse (us) ---
  double throttle_us =
    throttle_neutral_us_ + traction_velocity_command_ * throttle_us_per_rad_per_s_;
  if (throttle_us < throttle_min_us_) throttle_us = throttle_min_us_;
  if (throttle_us > throttle_max_us_) throttle_us = throttle_max_us_;
  pca_->set_pulse_us(throttle_channel_, throttle_us);

  return hardware_interface::return_type::OK;
}

}  // namespace jetracer_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(jetracer_control::JetracerHardware, hardware_interface::SystemInterface)
