#ifndef JETRACER_CONTROL__JETRACER_HARDWARE_HPP_
#define JETRACER_CONTROL__JETRACER_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace jetracer_control
{

// Minimal userland PCA9685 driver over /dev/i2c-*
class PCA9685
{
public:
  PCA9685() = default;
  ~PCA9685();

  // bus e.g. 1 -> /dev/i2c-1, addr default 0x40
  bool open_device(int bus, int addr);
  void set_pwm_freq(double freq_hz);
  // Set channel on/off tick (0..4095). Convenience: set_pulse_us writes microseconds.
  void set_pwm(int channel, int on, int off);
  void set_pulse_us(int channel, double pulse_us);
  void all_off();

private:
  void write8(uint8_t reg, uint8_t value);
  uint8_t read8(uint8_t reg);

  int fd_{-1};
  double freq_hz_{50.0};
};

class JetracerHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(JetracerHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Joint names (resolved from URDF)
  std::string steering_joint_;
  std::string traction_joint_;

  // Command buffers
  double steering_position_command_{0.0};   // radians
  double traction_velocity_command_{0.0};   // rad/s

  // State buffers (open-loop echo of commands; JetRacer has no encoders)
  double steering_position_state_{0.0};
  double traction_velocity_state_{0.0};
  double traction_position_state_{0.0};

  // Hardware config (from <param> tags in the URDF)
  int i2c_bus_{1};
  int i2c_address_{0x40};
  int steering_channel_{0};
  int throttle_channel_{1};
  double pwm_freq_{50.0};

  // Steering calibration: maps steering angle (rad) -> servo pulse (us)
  double steer_center_us_{1500.0};
  double steer_us_per_rad_{500.0};   // pulse change per radian
  double steer_min_us_{1000.0};
  double steer_max_us_{2000.0};

  // Throttle calibration: maps wheel velocity (rad/s) -> ESC pulse (us)
  double throttle_neutral_us_{1500.0};
  double throttle_us_per_rad_per_s_{20.0};
  double throttle_min_us_{1000.0};
  double throttle_max_us_{2000.0};

  std::unique_ptr<PCA9685> pca_;
};

}  // namespace jetracer_control

#endif  // JETRACER_CONTROL__JETRACER_HARDWARE_HPP_
