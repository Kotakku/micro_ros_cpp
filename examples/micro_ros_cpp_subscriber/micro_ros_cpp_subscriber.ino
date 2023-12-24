#include <micro_ros_arduino.h>
#include <micro_ros_cpp.hpp>
#include <std_msgs/msg/int32.h>

// define C++ types for the messages used in this example
MICRO_ROS_REGISTER_MSG_TYPE(std_msgs, msg, Int32)

micro_ros::Node::SharedPtr node;
micro_ros::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber;
micro_ros::Timer::SharedPtr timer;

int cnt = 0;

#define LED_PIN 13

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);

  delay(1000);

  micro_ros::init();
  node = std::make_shared<micro_ros::Node>("arduino_node");
  subscriber = node->create_subscription<std_msgs::msg::Int32>("micro_ros_subscriber", 
    [](std_msgs::msg::Int32::ConstPtr msg) {
      digitalWrite(LED_PIN, msg->data);
    }
  );   
}

void loop() {
  delay(10);
  micro_ros::spin_some_ms(10);
}
