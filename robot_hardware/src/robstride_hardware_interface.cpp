// RobStride CAN bus hardware interface for ros2_control
//
// RobStride CAN protocol (simplified):
//   TX: [CMD_BYTE] [MOTOR_ID] [DATA...]
//   RX: [MOTOR_ID] [POS_H] [POS_L] [VEL_H] [VEL_L] [TORQUE_H] [TORQUE_L] [TEMP]
//
// FONTOS: Ez egy SABLON implementacio!
// A tenyleges CAN protokollt a RobStride dokumentacioja alapjan kell kitolteni.
// A jelenlegi kod a SocketCAN infrastrukturat epiti fel es
// placeholder encoder/decoder fuggvenyeket tartalmaz.

#include "robot_hardware/robstride_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <unistd.h>
#include <poll.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_hardware
{

// === LIFECYCLE ===

hardware_interface::CallbackReturn RobStrideHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read parameters from URDF <ros2_control> tag
  can_interface_ = info_.hardware_parameters.at("can_interface");

  // Parse motor IDs (comma-separated string)
  std::string motor_ids_str = info_.hardware_parameters.at("motor_ids");
  std::stringstream ss(motor_ids_str);
  std::string item;
  while (std::getline(ss, item, ',')) {
    motor_ids_.push_back(static_cast<uint8_t>(std::stoi(item)));
  }

  size_t num_joints = info_.joints.size();
  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  hw_efforts_.resize(num_joints, 0.0);
  hw_commands_position_.resize(num_joints, 0.0);
  hw_commands_velocity_.resize(num_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("RobStrideHW"),
    "Initialized: %s with %zu joints, motor_ids: %s",
    can_interface_.c_str(), num_joints, motor_ids_str.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!open_can_socket()) {
    RCLCPP_ERROR(rclcpp::get_logger("RobStrideHW"),
      "Failed to open CAN socket: %s", can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("RobStrideHW"),
    "CAN socket opened: %s", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Enable all motors
  for (size_t i = 0; i < motor_ids_.size() && i < info_.joints.size(); i++) {
    if (!robstride_enable(motor_ids_[i])) {
      RCLCPP_WARN(rclcpp::get_logger("RobStrideHW"),
        "Failed to enable motor %d on %s", motor_ids_[i], can_interface_.c_str());
    }
  }

  // Read initial positions
  for (size_t i = 0; i < motor_ids_.size() && i < info_.joints.size(); i++) {
    robstride_read_state(motor_ids_[i],
      hw_positions_[i], hw_velocities_[i], hw_efforts_[i]);
    hw_commands_position_[i] = hw_positions_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("RobStrideHW"), "Activated: %s", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Disable all motors
  for (size_t i = 0; i < motor_ids_.size() && i < info_.joints.size(); i++) {
    robstride_disable(motor_ids_[i]);
  }

  RCLCPP_INFO(rclcpp::get_logger("RobStrideHW"), "Deactivated: %s", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_can_socket();
  RCLCPP_INFO(rclcpp::get_logger("RobStrideHW"), "Cleaned up: %s", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// === INTERFACES ===

std::vector<hardware_interface::StateInterface>
RobStrideHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RobStrideHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]);
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]);
  }
  return command_interfaces;
}

// === READ / WRITE ===

hardware_interface::return_type RobStrideHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < motor_ids_.size() && i < info_.joints.size(); i++) {
    if (!robstride_read_state(motor_ids_[i],
      hw_positions_[i], hw_velocities_[i], hw_efforts_[i]))
    {
      // On read failure, keep last known state
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobStrideHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < motor_ids_.size() && i < info_.joints.size(); i++) {
    robstride_set_position(motor_ids_[i], hw_commands_position_[i]);
  }
  return hardware_interface::return_type::OK;
}

// === CAN BUS HELPERS ===

bool RobStrideHardwareInterface::open_can_socket()
{
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    close(can_socket_);
    can_socket_ = -1;
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    close(can_socket_);
    can_socket_ = -1;
    return false;
  }

  return true;
}

void RobStrideHardwareInterface::close_can_socket()
{
  if (can_socket_ >= 0) {
    close(can_socket_);
    can_socket_ = -1;
  }
}

bool RobStrideHardwareInterface::send_can_frame(
  uint32_t id, const uint8_t* data, uint8_t len)
{
  if (can_socket_ < 0) return false;

  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = id;
  frame.can_dlc = std::min(len, static_cast<uint8_t>(8));
  std::memcpy(frame.data, data, frame.can_dlc);

  return ::write(can_socket_, &frame, sizeof(frame)) == sizeof(frame);
}

bool RobStrideHardwareInterface::recv_can_frame(
  struct can_frame& frame, int timeout_ms)
{
  if (can_socket_ < 0) return false;

  struct pollfd pfd;
  pfd.fd = can_socket_;
  pfd.events = POLLIN;

  int ret = poll(&pfd, 1, timeout_ms);
  if (ret <= 0) return false;

  return ::read(can_socket_, &frame, sizeof(frame)) == sizeof(frame);
}

// === ROBSTRIDE PROTOCOL ===
// TODO: Implementald a RobStride CAN protokollt a dokumentacio alapjan!
// Az alabbi fuggvenyek PLACEHOLDER-ek.

bool RobStrideHardwareInterface::robstride_read_state(
  uint8_t motor_id, double& pos, double& vel, double& effort)
{
  // TODO: RobStride status request CAN frame kuldes
  // Pelda: send CMD_READ_STATE to motor_id
  // Majd recv a valasz frame-t es decode-olni

  uint8_t data[8] = {0};
  data[0] = 0x01;  // CMD: read state (PLACEHOLDER - tenyleges CMD a doksiban!)
  data[1] = motor_id;

  if (!send_can_frame(motor_id, data, 8)) {
    return false;
  }

  struct can_frame rx_frame;
  if (!recv_can_frame(rx_frame, 5)) {
    return false;
  }

  // TODO: Decode RobStride response frame
  // A tenyleges dekodolas a RobStride protokolltol fugg:
  // - pozicio: raw_value * scale_factor -> radians
  // - sebesseg: raw_value * scale_factor -> rad/s
  // - nyomatek: raw_value * scale_factor -> Nm

  // PLACEHOLDER: tartjuk az aktualis ertekeket
  (void)pos;
  (void)vel;
  (void)effort;

  return true;
}

bool RobStrideHardwareInterface::robstride_set_position(
  uint8_t motor_id, double position)
{
  // TODO: RobStride position command CAN frame
  // Encode position (radians) to raw CAN data

  uint8_t data[8] = {0};
  data[0] = 0x02;  // CMD: set position (PLACEHOLDER!)
  data[1] = motor_id;

  // TODO: Encode position to bytes
  // int16_t raw_pos = static_cast<int16_t>(position / scale_factor);
  // data[2] = (raw_pos >> 8) & 0xFF;
  // data[3] = raw_pos & 0xFF;

  (void)position;

  return send_can_frame(motor_id, data, 8);
}

bool RobStrideHardwareInterface::robstride_enable(uint8_t motor_id)
{
  // TODO: RobStride motor enable command
  uint8_t data[8] = {0};
  data[0] = 0x03;  // CMD: enable motor (PLACEHOLDER!)
  data[1] = motor_id;

  RCLCPP_INFO(rclcpp::get_logger("RobStrideHW"),
    "Enabling motor %d on %s", motor_id, can_interface_.c_str());

  return send_can_frame(motor_id, data, 8);
}

bool RobStrideHardwareInterface::robstride_disable(uint8_t motor_id)
{
  // TODO: RobStride motor disable command
  uint8_t data[8] = {0};
  data[0] = 0x04;  // CMD: disable motor (PLACEHOLDER!)
  data[1] = motor_id;

  RCLCPP_INFO(rclcpp::get_logger("RobStrideHW"),
    "Disabling motor %d on %s", motor_id, can_interface_.c_str());

  return send_can_frame(motor_id, data, 8);
}

}  // namespace robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robot_hardware::RobStrideHardwareInterface,
  hardware_interface::SystemInterface)
