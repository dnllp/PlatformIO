#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

#define LED_PIN 2

void setup() {
  Serial.begin(115200);
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_platformio_node", "", &support);
  rclc_publisher_init_default(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "datos_esp32");

  msg.data = 0;
}

void loop() {
  rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
  // ret = rcl_publish(&publisher, &msg, NULL);
  msg.data++;
  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Parpadeo de estado
  delay(500);
}
