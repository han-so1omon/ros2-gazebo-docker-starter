#include <thread>
#include <chrono>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

using namespace std::chrono_literals;

class VelodyneTest : public rclcpp::Node
{
  public:
    VelodyneTest()
      : Node("velodyne_test")
    {
      // Create our gazebo node for communication
      this->gazeboNode = boost::make_shared<gazebo::transport::Node>();
      this->gazeboNode->Init();

      RCLCPP_INFO(this->get_logger(), "GAZEBO NODE INITIALIZED");

      // Publish to the velodyne topic
      this->gazeboPub = this->gazeboNode->Advertise<gazebo::msgs::Vector3d>("~/my_velodyne/vel_cmd");

      // Wait for a subscriber to connect to this publisher
      this->gazeboPub->WaitForConnection();

      RCLCPP_INFO(this->get_logger(), "GAZEBO PUBLISHER CONNECTED");

      this->rosTimer = this->create_wall_timer(
          1s, std::bind(&VelodyneTest::rosTimerCallback, this));
    }

  private:
    void rosTimerCallback()
    {
      RCLCPP_INFO(this->get_logger(), "SETTING VELODYNE VELOCITY");

      this->velodyneVelocity = (this->velodyneVelocity % 10) + 1; 
      // Set the velocity in the x-component
#if GAZEBO_MAJOR_VERSION < 6
      gazebo::msgs::Set(&this->velodyneMsg, gazebo::math::Vector3((float)this->velodyneVelocity, 0, 0));
#else
      gazebo::msgs::Set(&this->velodyneMsg, ignition::math::Vector3d((float)this->velodyneVelocity, 0, 0));
#endif

      // Send the message
      gazeboPub->Publish(this->velodyneMsg);
    }

    // Gazebo elements
    gazebo::transport::NodePtr gazeboNode = nullptr;
    gazebo::transport::PublisherPtr gazeboPub = nullptr;
    gazebo::msgs::Vector3d velodyneMsg;

    // ROS elements
    //rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rosPub;
    rclcpp::TimerBase::SharedPtr rosTimer;
    int velodyneVelocity = 0;
};

int main(int _argc, char** _argv)
{
  // Initialize ROS runtime
  rclcpp::init(_argc, _argv); 

  RCLCPP_INFO(rclcpp::get_logger(""), "ROS INITIALIZED");

  // Load gazebo as a client
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::setupClient(0, NULL);
#else
  gazebo::client::setup(0, NULL);
#endif

  RCLCPP_INFO(rclcpp::get_logger(""), "GAZEBO INITIALIZED");

  rclcpp::executors::MultiThreadedExecutor rosExecutor;

  std::shared_ptr<VelodyneTest> rosNode = std::make_shared<VelodyneTest>();

  rosExecutor.add_node(rosNode);

  rosExecutor.spin();

  // Shutdown ROS
  rclcpp::shutdown();

  // Shutdown Gazebo
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::shutdown();
#else
  gazebo::client::shutdown();
#endif
}
