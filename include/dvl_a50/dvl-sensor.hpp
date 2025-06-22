#ifndef DVL_A50_HPP
#define DVL_A50_HPP

/**
 * dvl-sensor.hpp
 *
 * @author     Pablo Gutiérrez
 * @date       24/11/2021
 */

// ROS 2 Headers
#include <chrono>
#include <memory>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "dvl_a50/tcpsocket.hpp"

// Add message_filters includes here
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <string>
#include "dvl_msgs/msg/dvl.hpp"
#include "dvl_msgs/msg/dvl_beam.hpp"
#include "dvl_msgs/msg/dvldr.hpp"
#include "dvl_msgs/msg/config_command.hpp"
#include "dvl_msgs/msg/command_response.hpp"
#include "dvl_msgs/msg/config_status.hpp"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>

#include <tf2/LinearMath/Quaternion.hpp>

#include "dvl_a50/json/single_include/nlohmann/json.hpp"
#include <iomanip>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::string;
using nlohmann::json;


namespace dvl_sensor {

enum DVL_Parameters {
    speed_of_sound,
    acoustic_enabled,
    dark_mode_enabled,
    mountig_rotation_offset,
    range_mode,
    invalid_param
};

class DVL_A50: public rclcpp::Node
{
public:

    uint8_t ready = 0;
    uint8_t error = 0;

    DVL_A50();
    ~DVL_A50();

private:
    int fault = 1; 
    string delimiter = ",";
    double old_altitude;
    std::string ip_address;
    std::string velocity_frame_id;
    std::string position_frame_id;
    std::string altitude_frame_id;
    TCPSocket *tcpSocket;
    json json_data;

    DVL_Parameters resolveParameter(std::string param);
    
    std::chrono::steady_clock::time_point first_time;
    std::chrono::steady_clock::time_point first_time_error;
    
    rclcpp::TimerBase::SharedPtr timer_receive;
    rclcpp::TimerBase::SharedPtr timer_send;
    rclcpp::Publisher<dvl_msgs::msg::DVL>::SharedPtr dvl_pub_report;
    rclcpp::Publisher<dvl_msgs::msg::DVLDR>::SharedPtr dvl_pub_pos;
    rclcpp::Publisher<dvl_msgs::msg::CommandResponse>::SharedPtr dvl_pub_command_response;
    rclcpp::Publisher<dvl_msgs::msg::ConfigStatus>::SharedPtr dvl_pub_config_status;

    rclcpp::Subscription<dvl_msgs::msg::ConfigCommand>::SharedPtr dvl_sub_config_command;
    
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_pub_twist_cov;  // DVL Velocity
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr dvl_pub_dr_pose_cov; // dead reckoning post
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr dvl_pub_altitude;    // altitude (distance from seafloor)
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr dvl_pub_odometry;
    
    void handle_receive();
    //Publish velocity and transducer report
    void publish_vel_trans_report();
    void publish_dead_reckoning_report();
    void publish_config_status();
    void publish_command_response();

    void command_subscriber(const dvl_msgs::msg::ConfigCommand::SharedPtr msg);
    void set_json_parameter(const std::string name, const std::string value);
    void send_parameter_to_sensor(const json &message);

    // Synchronizer typedef
    typedef message_filters::sync_policies::ApproximateTime<
        geometry_msgs::msg::TwistWithCovarianceStamped,
        geometry_msgs::msg::PoseWithCovarianceStamped
    > SyncPolicy;

    // Message filter subscribers
    message_filters::Subscriber<geometry_msgs::msg::TwistWithCovarianceStamped> twist_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> pose_sub_;
    
    // Synchronizer
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // Internal publishers for synchronization
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr internal_twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr internal_pose_pub_;
    
    // Synchronization callback
    void syncCallback(
        const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr& twist_msg,
        const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& pose_msg);
};

} // namespace
#endif //DVL_A50_HPP
