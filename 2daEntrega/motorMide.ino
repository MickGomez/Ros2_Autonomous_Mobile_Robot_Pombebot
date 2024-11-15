#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Float32 pub_msg;
std_msgs__msg__Float32 sub_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13
#define ENA 5
#define IN1 18
#define IN2 19
#define ENCODER_A 15
#define ENCODER_B 4

volatile long encoderValue = 0;
unsigned long lastTime = 0;
int interval =1000;  // Intervalo de actualización en ms
int ppr = 2100;  // Pulsos por revolución del encoder
float targetSpeed = 0;  // Velocidad objetivo en grados/segundo
int currentPWM = 0;  // Valor actual de PWM

volatile long encoder_count = 0;
long last_encoder_count = 0;
unsigned long last_time = 0;
float target_velocity = 0.0;

// Parámetros de calibración
const float MAX_SPEED = 420;  
const int MAX_PWM = 255;  

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void encoder_isr() {
  if (digitalRead(ENCODER_B) == HIGH) {
    encoderValue++;
  } else {
    encoderValue--;
  }
}

// ... (El resto del código permanece igual)

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    unsigned long currentTime = millis();
    long currentValue = encoderValue;
    float deltaTime = (currentTime - last_time) / 1000.0;  // Tiempo en segundos
    
    // Calcular velocidad en grados por segundo
    float deltaPosition = (float)(currentValue - last_encoder_count) / ppr * 360.0;  // Cambio en grados
    float current_velocity = deltaPosition / deltaTime;  // Grados por segundo

    pub_msg.data = current_velocity;
    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));

    // Actualizar valores para la próxima iteración
    last_time = currentTime;
    last_encoder_count = currentValue;
  }
}

// ... (El resto del código permanece igual)

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  target_velocity = msg->data;
  // Aquí puedes agregar código para ajustar la velocidad del motor basado en target_velocity
    currentPWM = map(target_velocity, 0, MAX_SPEED, 0, MAX_PWM);
    currentPWM = constrain(currentPWM, 0, 255);
    analogWrite(ENA, currentPWM);
}

void updateEncoder() {
  if (digitalRead(ENCODER_B) == HIGH) {
    encoderValue++;
  } else {
    encoderValue--;
  }
}

void setup() {
  set_microros_transports();
  
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
  analogWrite(ENA, 0);  // Iniciar con velocidad 0

  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "encoder_velocity"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "target_velocity"));

  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA));

  pub_msg.data = 0;
  sub_msg.data = 0;
  last_time = millis();
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
