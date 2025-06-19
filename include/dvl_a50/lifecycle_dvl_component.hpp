#ifndef COMPOSITION__TALKER_COMPONENT_HPP_
#define COMPOSITION__TALKER_COMPONENT_HPP_

#include <chrono>
#include "dvl_a50/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/string.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "dvl_a50/tcpsocket.hpp"

#include <string>
#include "dvl_msgs/msg/dvl.hpp"
#include "dvl_msgs/msg/dvl_beam.hpp"
#include "dvl_msgs/msg/dvldr.hpp"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2/LinearMath/Quaternion.hpp>

//Json Library
#include "dvl_a50/json/single_include/nlohmann/json.hpp"
#include <iomanip>

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace composition
{

class LifecycleDVL : public rclcpp_lifecycle::LifecycleNode
{
public:
  COMPOSITION_PUBLIC
  explicit LifecycleDVL(const rclcpp::NodeOptions & options);
  ~LifecycleDVL();
  
  LNI::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  LNI::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  LNI::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  LNI::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  LNI::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

protected:
  void on_timer();
  void publish_vel_trans_report();
  void publish_dead_reckoning_report();

private:
  size_t count_;
  int fault = 1; 
  double current_altitude;
  double old_altitude;
  std::string ip_address;
  std::string velocity_frame_id;
  std::string position_frame_id;
  std::string altitude_frame_id;
  TCPSocket *tcpSocket;
    
  nlohmann::json json_data;
    
  std::chrono::steady_clock::time_point first_time_loss;
  std::chrono::steady_clock::time_point first_time_error;
    
  // DVL message struct
  dvl_msgs::msg::DVLBeam beam0;
  dvl_msgs::msg::DVLBeam beam1;
  dvl_msgs::msg::DVLBeam beam2;
  dvl_msgs::msg::DVLBeam beam3;
    
  dvl_msgs::msg::DVLDR DVLDeadReckoning;
  dvl_msgs::msg::DVL dvl;
    
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<dvl_msgs::msg::DVL>> dvl_pub_report;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<dvl_msgs::msg::DVLDR>> dvl_pub_pos;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>> dvl_pub_altitude;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistWithCovarianceStamped>> dvl_pub_twist_cov;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>> dvl_pub_dr_pose_cov;
  
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

}  // namespace composition

#endif  // COMPOSITION__TALKER_COMPONENT_HPP_