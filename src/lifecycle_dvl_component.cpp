/**
 * @file   lifecycle_dvl_component.cpp
 *
 * @author Pablo Gutiérrez
 * @date   24/11/2021
 *
 * Contact: pgutierrez@marum.de
 */

#include "dvl_a50/lifecycle_dvl_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using namespace std::chrono_literals;
using nlohmann::json;

namespace composition
{

LifecycleDVL::LifecycleDVL(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("dvl_a50_node", options),
current_altitude(0.0),
old_altitude(0.0)
{
    this->declare_parameter<std::string>("dvl_ip_address", "192.168.194.95");
    this->declare_parameter<std::string>("velocity_frame_id", "dvl_A50/velocity_link");
    this->declare_parameter<std::string>("position_frame_id", "dvl_A50/position_link");
    this->declare_parameter<std::string>("altitude_frame_id", "pool_bottom");
    
    velocity_frame_id = this->get_parameter("velocity_frame_id").as_string();
    position_frame_id = this->get_parameter("position_frame_id").as_string();
    altitude_frame_id = this->get_parameter("altitude_frame_id").as_string();
    ip_address = this->get_parameter("dvl_ip_address").as_string();
    RCLCPP_INFO(get_logger(), "IP_ADDRESS: '%s'", ip_address.c_str());
}

LifecycleDVL::~LifecycleDVL() {
  tcpSocket->Close();
  delete tcpSocket;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
LifecycleDVL::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "on_configure() is called.");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&LifecycleDVL::on_timer, this));
    
    // Create lifecycle publishers
    dvl_pub_report = this->create_publisher<dvl_msgs::msg::DVL>("dvl/data", 10);
    dvl_pub_pos = this->create_publisher<dvl_msgs::msg::DVLDR>("dvl/position", 10);
    dvl_pub_altitude = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("dvl/altitude", 10);
    dvl_pub_twist_cov = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("dvl/twist_cov", 10);
    dvl_pub_dr_pose_cov = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("dvl/deadreckon_pose_cov", 10);

    //--- TCP/IP SOCKET ---- 
    tcpSocket = new TCPSocket((char*)ip_address.c_str() , 16171);
    
    if(tcpSocket->Create() < 0)
    {
        RCLCPP_INFO(get_logger(), "Socket creation error");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
   
    tcpSocket->SetRcvTimeout(300);
    std::string error;
    
    int error_code = 0;
    
    first_time_error = std::chrono::steady_clock::now();
    while(fault != 0)
    {
        fault = tcpSocket->Connect(5000, error, error_code);
        if(error_code == 114)
        {
            RCLCPP_INFO(get_logger(), "TCP Error: [%s]", error.c_str());
            RCLCPP_INFO(get_logger(), "Is the sensor on?");
            std::this_thread::sleep_for(2s);
            std::chrono::steady_clock::time_point current_time_error = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(current_time_error - first_time_error).count();
            if(dt >= 78.5) //Max time to set up
            {
                RCLCPP_INFO(get_logger(), "Error time: [%6.2f]", dt);
                fault = -10;
                break;
            }
        }
        else if(error_code == 103)
        {
            RCLCPP_INFO(get_logger(), "TCP Error: [%s]", error.c_str());
            RCLCPP_INFO(get_logger(), "No route to host, DVL might be booting?");
            std::this_thread::sleep_for(2s);
        }
    }  
    
    if(fault == -10)
    {
        tcpSocket->Close();
        RCLCPP_INFO(get_logger(), "Turn the sensor on and try again!");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    else
        RCLCPP_INFO(get_logger(), "DVL-A50 connected!");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
LifecycleDVL::on_activate(const rclcpp_lifecycle::State &)
{
    dvl_pub_report->on_activate();
    dvl_pub_pos->on_activate();
    dvl_pub_altitude->on_activate();
    dvl_pub_twist_cov->on_activate();
    dvl_pub_dr_pose_cov->on_activate();
    first_time_loss = std::chrono::steady_clock::now();
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
LifecycleDVL::on_deactivate(const rclcpp_lifecycle::State &)
{
    dvl_pub_report->on_deactivate();
    dvl_pub_pos->on_deactivate();
    dvl_pub_altitude->on_deactivate();
    dvl_pub_twist_cov->on_deactivate();
    dvl_pub_dr_pose_cov->on_deactivate();
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleDVL::on_cleanup(const rclcpp_lifecycle::State &)
{
    fault = 1;
    tcpSocket->Close();
 
    timer_.reset();
    dvl_pub_report.reset();
    dvl_pub_pos.reset();
    dvl_pub_altitude.reset();
    dvl_pub_twist_cov.reset();
    dvl_pub_dr_pose_cov.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;  
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleDVL::on_shutdown(const rclcpp_lifecycle::State & state)
{
    timer_.reset();
    dvl_pub_report.reset();
    dvl_pub_pos.reset();
    dvl_pub_altitude.reset();
    dvl_pub_twist_cov.reset();
    dvl_pub_dr_pose_cov.reset();

    RCUTILS_LOG_INFO_NAMED(
      get_name(),
      "on shutdown is called from state %s.",
      state.label().c_str());
      
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void LifecycleDVL::on_timer()
{
    if(!dvl_pub_report->is_activated() || !dvl_pub_pos->is_activated())
    {
        //RCLCPP_INFO(get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
    }else {
        std::flush(std::cout);
        char *tempBuffer = new char[1];
        std::string str; 
    
        while(tempBuffer[0] != '\n')
        {
            tcpSocket->Receive(tempBuffer);
            str = str + tempBuffer[0];         
        }
        
        try
        {
            json_data = json::parse(str);
            
            if (json_data.contains("altitude")) {
                this->publish_vel_trans_report();
            }
            else if (json_data.contains("pitch")) {
                this->publish_dead_reckoning_report();
            }
        }
        catch(std::exception& e)
        {
             UNUSED(e);
        } 
    } 
}

void LifecycleDVL::publish_vel_trans_report()
{
    dvl.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
    dvl.header.frame_id = velocity_frame_id;
    
    dvl.time = double(json_data["time"]);

    // Populate DVL velocity field (Vector3)
    dvl.velocity.x = double(json_data["vx"]);    
    dvl.velocity.y = double(json_data["vy"]);
    dvl.velocity.z = double(json_data["vz"]);
    
    // Create TwistWithCovariance for separate message
    geometry_msgs::msg::TwistWithCovariance twist_with_cov;
    twist_with_cov.twist.linear.x = double(json_data["vx"]);
    twist_with_cov.twist.linear.y = double(json_data["vy"]);
    twist_with_cov.twist.linear.z = double(json_data["vz"]);
    
    // DVL doesn't measure angular velocity, set to zero
    twist_with_cov.twist.angular.x = 0.0;
    twist_with_cov.twist.angular.y = 0.0;
    twist_with_cov.twist.angular.z = 0.0;
    
    // Extract and process covariance from JSON
    std::array<double, 36> twist_covariance = {0};
    
    if (json_data.contains("covariance")) {
        auto cov_json = json_data["covariance"];
        
        // Extract 3x3 covariance matrix for [vx, vy, vz]
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                // Map 3x3 covariance to upper-left of 6x6 matrix
                twist_covariance[i * 6 + j] = double(cov_json[i][j]);
            }
        }
    } else {
        // Default covariance if not provided
        twist_covariance[0] = 0.01;   // vx variance
        twist_covariance[7] = 0.01;   // vy variance  
        twist_covariance[14] = 0.01;  // vz variance
    }
    
    // Set high uncertainty for angular velocities (not measured by DVL)
    twist_covariance[21] = 1e6;  // wx variance
    twist_covariance[28] = 1e6;  // wy variance
    twist_covariance[35] = 1e6;  // wz variance
    
    twist_with_cov.covariance = twist_covariance;
    
    dvl.fom = double(json_data["fom"]);
    
    current_altitude = double(json_data["altitude"]);
    dvl.velocity_valid = json_data["velocity_valid"];
            
    if(current_altitude >= 0.0 && dvl.velocity_valid)
    {
        dvl.altitude = current_altitude;
        old_altitude = current_altitude;
    }
    else
        dvl.altitude = old_altitude;

    // Publish altitude as a PoseWithCovarianceStamped message
    geometry_msgs::msg::PoseWithCovarianceStamped altitude;
    geometry_msgs::msg::PoseWithCovariance altitude_pose;
    altitude_pose.pose.position.x = 0.0;
    altitude_pose.pose.position.y = 0.0;
    altitude_pose.pose.position.z = dvl.altitude;
    altitude_pose.pose.orientation.x = 0.0;
    altitude_pose.pose.orientation.y = 0.0;
    altitude_pose.pose.orientation.z = 0.0;
    altitude_pose.pose.orientation.w = 1.0;
    altitude_pose.covariance[0] = 1e6;
    altitude_pose.covariance[7] = 1e6;
    altitude_pose.covariance[14] = dvl.fom * dvl.fom;
    altitude_pose.covariance[21] = 1e6;
    altitude_pose.covariance[28] = 1e6;
    altitude_pose.covariance[35] = 1e6;

    altitude.header.stamp = this->now();
    altitude.header.frame_id = altitude_frame_id;
    altitude.pose = altitude_pose;
    dvl_pub_altitude->publish(altitude);

    dvl.status = json_data["status"];
    dvl.form = json_data["format"];
            
    beam0.id = json_data["transducers"][0]["id"];
    beam0.velocity = double(json_data["transducers"][0]["velocity"]);
    beam0.distance = double(json_data["transducers"][0]["distance"]);
    beam0.rssi = double(json_data["transducers"][0]["rssi"]);
    beam0.nsd = double(json_data["transducers"][0]["nsd"]);
    beam0.valid = json_data["transducers"][0]["beam_valid"];
            
    beam1.id = json_data["transducers"][1]["id"];
    beam1.velocity = double(json_data["transducers"][1]["velocity"]);
    beam1.distance = double(json_data["transducers"][1]["distance"]);
    beam1.rssi = double(json_data["transducers"][1]["rssi"]);
    beam1.nsd = double(json_data["transducers"][1]["nsd"]);
    beam1.valid = json_data["transducers"][1]["beam_valid"];
            
    beam2.id = json_data["transducers"][2]["id"];
    beam2.velocity = double(json_data["transducers"][2]["velocity"]);
    beam2.distance = double(json_data["transducers"][2]["distance"]);
    beam2.rssi = double(json_data["transducers"][2]["rssi"]);
    beam2.nsd = double(json_data["transducers"][2]["nsd"]);
    beam2.valid = json_data["transducers"][2]["beam_valid"];
            
    beam3.id = json_data["transducers"][3]["id"];
    beam3.velocity = double(json_data["transducers"][3]["velocity"]);
    beam3.distance = double(json_data["transducers"][3]["distance"]);
    beam3.rssi = double(json_data["transducers"][3]["rssi"]);
    beam3.nsd = double(json_data["transducers"][3]["nsd"]);
    beam3.valid = json_data["transducers"][3]["beam_valid"];
            
    dvl.beams = {beam0, beam1, beam2, beam3};
    
    // Publish original DVL message
    dvl_pub_report->publish(dvl);

    // Publish TwistWithCovarianceStamped message
    geometry_msgs::msg::TwistWithCovarianceStamped twist_cov_msg;
    twist_cov_msg.header = dvl.header;
    twist_cov_msg.twist = twist_with_cov;
    dvl_pub_twist_cov->publish(twist_cov_msg);
}

void LifecycleDVL::publish_dead_reckoning_report()
{
    DVLDeadReckoning.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
    DVLDeadReckoning.header.frame_id = position_frame_id;
    DVLDeadReckoning.time = double(json_data["ts"]);
    DVLDeadReckoning.position.x = double(json_data["x"]);
    DVLDeadReckoning.position.y = double(json_data["y"]);
    DVLDeadReckoning.position.z = double(json_data["z"]);
    DVLDeadReckoning.pos_std = double(json_data["std"]);
    DVLDeadReckoning.roll = double(json_data["roll"]);
    DVLDeadReckoning.pitch = double(json_data["pitch"]);
    DVLDeadReckoning.yaw = double(json_data["yaw"]);
    DVLDeadReckoning.type = json_data["type"];
    DVLDeadReckoning.status = json_data["status"];
    DVLDeadReckoning.format = json_data["format"];
    dvl_pub_pos->publish(DVLDeadReckoning);

    // Publish PoseWithCovarianceStamped
    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
    pose_cov_msg.header = DVLDeadReckoning.header;
    
    // Set position
    pose_cov_msg.pose.pose.position.x = DVLDeadReckoning.position.x;
    pose_cov_msg.pose.pose.position.y = DVLDeadReckoning.position.y;
    pose_cov_msg.pose.pose.position.z = DVLDeadReckoning.position.z;
    
    // Convert RPY to quaternion
    tf2::Quaternion q;
    q.setRPY(DVLDeadReckoning.roll, DVLDeadReckoning.pitch, DVLDeadReckoning.yaw);
    pose_cov_msg.pose.pose.orientation.x = q.x();
    pose_cov_msg.pose.pose.orientation.y = q.y();
    pose_cov_msg.pose.pose.orientation.z = q.z();
    pose_cov_msg.pose.pose.orientation.w = q.w();
    
    // Set covariance matrix (6x6 for pose: x, y, z, roll, pitch, yaw)
    std::array<double, 36> pose_covariance = {0};
    double pos_variance = DVLDeadReckoning.pos_std * DVLDeadReckoning.pos_std;
    
    // Position covariance (assuming isotropic uncertainty)
    pose_covariance[0] = pos_variance;   // x variance
    pose_covariance[7] = pos_variance;   // y variance
    pose_covariance[14] = pos_variance;  // z variance
    
    // Orientation covariance (set reasonable defaults)
    pose_covariance[21] = 0.01;  // roll variance (rad^2)
    pose_covariance[28] = 0.01;  // pitch variance (rad^2)
    pose_covariance[35] = 0.01;  // yaw variance (rad^2)
    
    pose_cov_msg.pose.covariance = pose_covariance;
    dvl_pub_dr_pose_cov->publish(pose_cov_msg);
}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::LifecycleDVL)
