#pragma once

#include <string>
#include <vector>
#include <cstring>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// SocketCAN
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

namespace robot_hardware
{

class RobStrideHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobStrideHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // CAN bus
  std::string can_interface_;
  int can_socket_ = -1;
  std::vector<uint8_t> motor_ids_;

  // Joint state storage
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Joint command storage
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_commands_velocity_;

  // CAN helpers
  bool open_can_socket();
  void close_can_socket();
  bool send_can_frame(uint32_t id, const uint8_t* data, uint8_t len);
  bool recv_can_frame(struct can_frame& frame, int timeout_ms = 5);

  // RobStride protocol
  bool robstride_read_state(uint8_t motor_id, double& pos, double& vel, double& effort);
  bool robstride_set_position(uint8_t motor_id, double position);
  bool robstride_enable(uint8_t motor_id);
  bool robstride_disable(uint8_t motor_id);
};

}  // namespace robot_hardware
