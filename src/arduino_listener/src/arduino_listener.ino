#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

// Node, Executor, and Subscriber
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 received_msg;

// Pin definitions
#define LED_PIN_COUNT 6
int ledPins[LED_PIN_COUNT] = {13, 12, 11, 10, 9, 8};

// Error loop (flashes LED_BUILTIN if initialization fails)
void error_loop() {
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Callback function for the subscriber
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;

  // Turn off all LEDs
  for (int i = 0; i < LED_PIN_COUNT; i++) {
    digitalWrite(ledPins[i], LOW);
  }

  // Check if the received pin number is valid
  int pinToFlash = msg->data;
  bool isValidPin = false;
  for (int i = 0; i < LED_PIN_COUNT; i++) {
    if (pinToFlash == ledPins[i]) {
      isValidPin = true;
      digitalWrite(pinToFlash, HIGH); // Flash the specified pin
      break;
    }
  }

  if (!isValidPin) {
    // If the pin is invalid, blink the LED_BUILTIN to indicate an error
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void setup() {
  // Set up pins
  for (int i = 0; i < LED_PIN_COUNT; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize micro-ROS communication
  set_microros_transports();

  delay(2000); // Wait for micro-ROS agent to be ready

  // Allocate memory
  rcl_allocator_t allocator = rcl_get_default_allocator();

  // Initialize support and create the node
  rclc_support_t support;
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "led_controller_node", "", &support);

  // Initialize the subscriber
  rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "led_control");

  // Initialize the executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &received_msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  // Process incoming messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(100); 
}
