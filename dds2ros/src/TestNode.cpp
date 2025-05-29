#include<rclcpp/rclcpp.hpp>
#include<custom_msgs/msg/command_request.hpp>
#include<custom_msgs/msg/object_attack_designate.hpp>
#include<custom_msgs/msg/simple_vehicle.hpp>
#include<custom_msgs/msg/task_stage.hpp>
#include<custom_msgs/srv/command_bool.hpp>
#include<log/Logger.hpp>
#include <chrono>
#include <memory>

using std::placeholders::_1;
using namespace std::chrono_literals;
using command = custom_msgs::srv::CommandBool;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber(): Node("test_subscriber")
    {
      subscription_ = this->create_subscription<custom_msgs::msg::CommandRequest>(
      "outer/command/request", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      txtLog().info(THISMODULE  "sub init: '%s'", "dds2ros/comm_and/request");
    
      pSubAttack = this->create_subscription<custom_msgs::msg::ObjectAttackDesignate>(
      "outer/set/attack_object_designate",10,std::bind(&MinimalSubscriber::topic_attack_callback,this,_1));
       txtLog().info(THISMODULE  "sub init: '%s'", "outer/set/attack_object_designate");

      //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      pPubSimVel = this->create_publisher<custom_msgs::msg::SimpleVehicle>("inner/information/simple_vehicle", 10);    // CHANGE
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalSubscriber::timer_SimVel_callback, this));

      pSubSimVel_1 = this->create_subscription<custom_msgs::msg::SimpleVehicle>(
      "outer/information/simple_vehicle", 10, std::bind(&MinimalSubscriber::topic_Vehicle1_callback, this, _1));
      txtLog().info(THISMODULE  "sub init: '%s'", "outer/information/simple_vehicle1");

    }

  private:
    void topic_callback(const custom_msgs::msg::CommandRequest::SharedPtr msg) const
    {
      txtLog().info(THISMODULE  "CommandRequest type: '%d'", int(msg->type));
    }
    void topic_attack_callback(const custom_msgs::msg::ObjectAttackDesignate::SharedPtr msg) const
    {
      txtLog().info(THISMODULE  "ObjectAttackDesignate grpid: '%d'", int(msg->grpid));
    }

    void topic_Vehicle1_callback(const custom_msgs::msg::SimpleVehicle::SharedPtr msg) const
    {
      txtLog().info(THISMODULE  "SimpleVehicle id: '%d'", int(msg->id));
    }

    void timer_SimVel_callback()
    {
      auto message = custom_msgs::msg::SimpleVehicle();
      //txtLog().info(THISMODULE "Publishing sate: '%d'",int(message.sate));
      message.sate = 100;
      message.id = 1;
      pPubSimVel->publish(message);
    }
    rclcpp::Subscription<custom_msgs::msg::CommandRequest>::SharedPtr subscription_;
    rclcpp::Subscription<custom_msgs::msg::ObjectAttackDesignate>::SharedPtr pSubAttack;
    rclcpp::Subscription<custom_msgs::msg::SimpleVehicle>::SharedPtr pSubSimVel_1;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_msgs::msg::SimpleVehicle>::SharedPtr pPubSimVel;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
