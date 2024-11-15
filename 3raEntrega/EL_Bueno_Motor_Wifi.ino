#include <WiFi.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

// WiFi credentials
char ssid[] = "Gomez Saldivar";
char password[] = "Bebito&Coca9921";

// ROS 2 node configuration
rcl_publisher_t velocity_publisher;
rcl_subscription_t velocity_subscriber;
std_msgs__msg__Float32 velocity_msg;
std_msgs__msg__Float32 target_velocity_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Pin definitions
#define LED_PIN 13
#define ENA 5
#define IN1 18
#define IN2 19
#define ENCODER_A 15
#define ENCODER_B 4

// Motor and encoder variables
volatile long encoderValue = 0;
long lastEncoderValue = 0;
unsigned long lastTime = 0;
int ppr = 2100;  // Pulses per revolution of the encoder
float targetSpeed = 0;  // Target speed in degrees/second
int currentPWM = 0;  // Current PWM value

// Calibration parameters
const float MAX_SPEED = 420;  
const int MAX_PWM = 255;  

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void updateEncoder() {
  if (digitalRead(ENCODER_B) == HIGH) {
    encoderValue++;
  } else {
    encoderValue--;
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Calculate and publish encoder velocity
    unsigned long currentTime = millis();
    long currentEncoderValue = encoderValue;
    
    float deltaTime = (currentTime - lastTime) / 1000.0;  // Time in seconds
    float deltaPosition = (float)(currentEncoderValue - lastEncoderValue) / ppr * 360.0;  // Change in degrees
    
    float current_velocity = deltaPosition / deltaTime;  // Degrees per second

    velocity_msg.data = current_velocity;
    RCSOFTCHECK(rcl_publish(&velocity_publisher, &velocity_msg, NULL));

    // Update values for next iteration
    lastTime = currentTime;
    lastEncoderValue = currentEncoderValue;
  }
}

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  targetSpeed = msg->data;
  // Adjust motor speed based on target velocity
  currentPWM = map(targetSpeed, 0, MAX_SPEED, 0, MAX_PWM);
  currentPWM = constrain(currentPWM, 0, 255);
  analogWrite(ENA, currentPWM);
}

void setup() {
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  // Set up micro-ROS
  IPAddress agent_ip(192, 168, 0, 104); // Change this to your ROS2 agent IP
  size_t agent_port = 8888;
  char agent_ip_str[16];
  sprintf(agent_ip_str, "%d.%d.%d.%d", agent_ip[0], agent_ip[1], agent_ip[2], agent_ip[3]);
  set_microros_wifi_transports(ssid, password, agent_ip_str, agent_port);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, RISING);
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);  // Start with speed 0

  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32_motor_control_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &velocity_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "encoder_velocity"));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &velocity_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "target_velocity"));

  // Create timer
  const unsigned int timer_timeout = 1000;  // 10 Hz
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &velocity_subscriber, &target_velocity_msg, &subscription_callback, ON_NEW_DATA));

  velocity_msg.data = 0;
  target_velocity_msg.data = 0;
  lastEncoderValue = encoderValue;
  lastTime = millis();
}

void loop() {
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}