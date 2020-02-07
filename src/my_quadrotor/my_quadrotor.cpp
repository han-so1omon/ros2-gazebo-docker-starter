//TODO
//  Gazebo publishers:
//    - control
//  Gazebo subscribers:
//    - groundtruth
//    - imu
//    - camera
//  ROS 

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

class MyQuadrotorTransport : public rclcpp::Node
{
  public:
    MyQuadrotorTransport()
      : Node("my_quadrotor_transport")
    {
      // Create our gazebo node for communication
      this->gazeboNode = boost::make_shared<gazebo::transport::Node>();
      this->gazeboNode->Init();

      RCLCPP_INFO(this->get_logger(), "Gazebo node initialized");

      // Publish to the control topic
      this->gazeboControlPub = this->gazeboNode->Advertise<gazebo::msgs::Quaternion>(
          "~/my_quadrotor/control");

      RCLCPP_INFO(this->get_logger(), "Gazebo publishers setup");

      // Setup subscribers
      this->gazeboImuSub = this->gazeboNode->Subscribe(
          "~/my_quadrotor/imu_link/imu_sensor",
          &MyQuadrotorTransport::onImuMsg, this);
      this->gazeboGtSub = this->gazeboNode->Subscribe(
          "~/my_quadrotor/pose",
          &MyQuadrotorTransport::onGtMsg, this);

      RCLCPP_INFO(this->get_logger(), "Gazebo subscribers setup");

      this->rosTimer = this->create_wall_timer(
          10ms, std::bind(&MyQuadrotorTransport::quadrotorControlCallback, this));
    }

  private:
    void onImuMsg(ConstIMUPtr& imuMsg)
    {
    }

    void onGtMsg(ConstPoseStampedPtr& gtMsg)
    {
    }

    void quadrotorControlCallback()
    {
      gazeboControlPub->Publish(controlMsg);
    }

    void rosTimerCallback()
    {
      RCLCPP_INFO(this->get_logger(), "Setting control message");

      // Set the velocity in the x-component
      gazebo::msgs::Set(&this->controlMsg, ignition::math::Quaterniond(ctrl, 0, 0, 0));
      ctrl++;
      if (ctrl > 10000) ctrl = 0;

      // Send the message
      gazeboControlPub->Publish(this->controlMsg);
    }

    // Gazebo elements
    gazebo::transport::NodePtr gazeboNode = nullptr;
    gazebo::transport::PublisherPtr gazeboControlPub = nullptr;
    gazebo::transport::SubscriberPtr gazeboImuSub = nullptr;
    gazebo::transport::SubscriberPtr gazeboGtSub = nullptr;
    // Just use quaternion for convenience because we need a 4d vector message
    gazebo::msgs::Quaternion controlMsg;

    // ROS elements
    //rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rosPub;
    rclcpp::TimerBase::SharedPtr rosTimer;
    float ctrl = 0;
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

  std::shared_ptr<MyQuadrotorTransport> rosNode = std::make_shared<MyQuadrotorTransport>();

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
