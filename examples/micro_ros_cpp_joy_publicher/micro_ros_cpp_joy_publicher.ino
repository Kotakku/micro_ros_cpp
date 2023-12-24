#include <micro_ros_arduino.h>
#include <micro_ros_cpp.hpp>
#include <sensor_msgs/msg/joy.h>

// define C++ types for the messages used in this example
MICRO_ROS_REGISTER_MSG_TYPE(sensor_msgs, msg, Joy)

micro_ros::Node::SharedPtr node;
micro_ros::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher;
sensor_msgs::msg::Joy msg;
micro_ros::Timer::SharedPtr timer;

int cnt = 0;

#define LED_PIN 13

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  
  delay(1000);

  micro_ros::init();
  node = std::make_shared<micro_ros::Node>("arduino_node");
  publisher = node->create_publisher<sensor_msgs::msg::Joy>("micro_ros_joy_publisher");

  micro_ros::set_string(msg.header.frame_id, "joy");
  micro_ros::reseize_array(msg.axes, 2);
  micro_ros::reseize_array(msg.buttons, 3);

  timer = node->create_wall_timer(100, [&]() {
    msg.header.stamp.sec = 0;
    msg.header.stamp.nanosec = cnt * 100000;

    msg.axes.data[0] = 0.0f;
    msg.axes.data[1] = 1.0f;

    msg.buttons.data[0] = 0;
    msg.buttons.data[1] = 1;
    msg.buttons.data[2] = 2;

    cnt++;
    publisher->publish(msg);
    
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  });
}

void loop() {
  delay(10);
  micro_ros::spin_some_ms(10);
}
