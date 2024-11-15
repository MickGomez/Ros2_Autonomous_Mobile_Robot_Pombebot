#include <WiFi.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>  // Agregado para el comando de intervalo
#include <std_msgs/msg/float32.h>

// Primero, agregar los headers necesarios
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>

// WiFi credentials
char ssid[] = "Gomez Saldivar";
char password[] = "Bebito&Coca9921";


// Agregar estas variables globales
rcl_publisher_t odom_publisher;
rcl_publisher_t tf_publisher;
nav_msgs__msg__Odometry odom_msg;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__TransformStamped transform_msg;

// Variables para odometría
float x = 0.0;
float y = 0.0;
float theta = 0.0;
float wheel_radius = 0.033;  // Radio de la rueda en metros
float wheel_separation = 0.17;  // Distancia entre ruedas en metros

// Timer Management
unsigned long last_ultrasonic_time = 0;
unsigned long last_encoder_time = 0;
const unsigned long ENCODER_INTERVAL = 500;    // 500ms fijo
unsigned long ultrasonic_interval = 1000;      // 1000ms por defecto

// Variables PID
float Kp = 0.1;  // Ajusta estos valores según sea necesario
float Ki = 0.01;
float Kd = 0.05;
float integral = 0;
float prevError = 0;
unsigned long lastPIDTime = 0;
const unsigned long PID_INTERVAL = 50;  // Intervalo de actualización del PID en ms

// ROS 2 node configuration
rcl_publisher_t velocity_publisher_left, velocity_publisher_right;
rcl_publisher_t ultrasonic_publisher_front, ultrasonic_publisher_left, ultrasonic_publisher_right;
rcl_subscription_t velocity_subscriber;
rcl_subscription_t ultrasonic_interval_subscriber;
std_msgs__msg__Float32 velocity_msg_left, velocity_msg_right;
std_msgs__msg__Float32 ultrasonic_msg_front, ultrasonic_msg_left, ultrasonic_msg_right;
std_msgs__msg__Float32 target_velocity_msg;
std_msgs__msg__Int32 ultrasonic_interval_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


// Pin definitions
#define ENA_LEFT 5
#define IN1_LEFT 18
#define IN2_LEFT 19
#define ENA_RIGHT 21  // Nuevo pin para el motor derecho
#define IN1_RIGHT 22  // Nuevo pin para el motor derecho
#define IN2_RIGHT 23   // Nuevo pin para el motor derecho
#define ENCODER_A_LEFT 15 // Blanco (Naranja)
#define ENCODER_B_LEFT 4 // Violeta (Amarillo)
#define ENCODER_A_RIGHT 35  // Rojo (Amarillo) (Amarillo)
#define ENCODER_B_RIGHT 34  // Negro (Verde) (Naranja)
#define TRIG_FRONT 14  // Pin para el trigger del sensor ultrasónico frontal
#define ECHO_FRONT 27  // Pin para el echo del sensor ultrasónico frontal
#define TRIG_LEFT 26   // Pin para el trigger del sensor ultrasónico izquierdo
#define ECHO_LEFT 25   // Pin para el echo del sensor ultrasónico izquierdo
#define TRIG_RIGHT 13  // Pin para el trigger del sensor ultrasónico derecho
#define ECHO_RIGHT 12  // Pin para el echo del sensor ultrasónico derecho



// Motor and encoder variables
volatile long encoderValueLeft = 0, encoderValueRight = 0;
long lastEncoderValueLeft = 0, lastEncoderValueRight = 0;
unsigned long lastTime = 0;
float lastVelocityLeft = 0, lastVelocityRight = 0;
int ppr = 3000;  // Pulses per revolution of the encoder
float targetSpeed = 0;  // Target speed in degrees/second
int currentPWMLeft = 0, currentPWMRight = 0;  // Current PWM values

// Calibration parameters
const float MAX_SPEED = 420;  
const int MAX_PWM = 255;  

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    delay(100);
  }
}

void updateEncoderLeft() {
  if (digitalRead(ENCODER_B_LEFT) == HIGH) {
    encoderValueLeft++;
  } else {
    encoderValueLeft--;
  }
}

void updateEncoderRight() {
  if (digitalRead(ENCODER_B_RIGHT) == HIGH) {
    encoderValueRight++;
  } else {
    encoderValueRight--;
  }
}

float calculatePID(float error, float deltaTime) {
    integral += error * deltaTime;
    float derivative = (error - prevError) / deltaTime;
    float output = Kp * error + Ki * integral + Kd * derivative;
    prevError = error;
    return output;
}

// Función mejorada para el cálculo de velocidad
float calculateVelocity(long currentCount, long lastCount, float deltaTimeSeconds) {
    // Calcular el cambio en pulsos
    long deltaPulses = currentCount - lastCount;
    
    // Convertir pulsos a grados
    float deltaAngle = (float)deltaPulses * 360.0 / ppr;
    
    // Calcular velocidad en grados por segundo
    float velocity = deltaAngle / deltaTimeSeconds;
    
    return velocity;
}

// Función para suavizar la velocidad usando un filtro de media móvil simple
float smoothVelocity(float newVelocity, float lastVelocity) {
    const float alpha = 0.7; // Factor de suavizado (ajustar según necesidad)
    return alpha * newVelocity + (1 - alpha) * lastVelocity;
}

// Función para leer sensores ultrasónicos con timeout
float readUltrasonic(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Usar timeout para evitar bloqueos
    unsigned long timeout = 23530;  // Aproximadamente 4 metros de rango máximo
    unsigned long duration = pulseIn(echoPin, HIGH, timeout);
    
    if (duration == 0) {
        return -1.0;  // Indica error de lectura
    }
    
    return duration * 0.034 / 2.0;  // Convertir a centímetros
}

// Callback para cambiar el intervalo de ultrasonidos
void ultrasonic_interval_callback(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    if (msg->data >= 100 && msg->data <= 5000) {
        ultrasonic_interval = msg->data;
    }
}

void executeTasks() {
    unsigned long current_time = millis();
    
    // Procesar encoders cada 500ms
    if (current_time - last_encoder_time >= ENCODER_INTERVAL) {
        // Calcular tiempo transcurrido en segundos
        float deltaTime = (float)(current_time - last_encoder_time) / 1000.0;
        
        // Calcular velocidades
        float velocityLeft = calculateVelocity(encoderValueLeft, lastEncoderValueLeft, deltaTime);
        float velocityRight = calculateVelocity(encoderValueRight, lastEncoderValueRight, deltaTime);
        
        // Actualizar odometría
        updateOdometry(velocityLeft, velocityRight, deltaTime);
        publishOdometry();

        // Aplicar suavizado
        velocityLeft = smoothVelocity(velocityLeft, lastVelocityLeft);
        velocityRight = smoothVelocity(velocityRight, lastVelocityRight);
        
        // Publicar velocidades
        velocity_msg_left.data = velocityLeft;
        velocity_msg_right.data = velocityRight;
        RCSOFTCHECK(rcl_publish(&velocity_publisher_left, &velocity_msg_left, NULL));
        RCSOFTCHECK(rcl_publish(&velocity_publisher_right, &velocity_msg_right, NULL));
        
        // Actualizar valores para la próxima iteración
        lastEncoderValueLeft = encoderValueLeft;
        lastEncoderValueRight = encoderValueRight;
        lastVelocityLeft = velocityLeft;
        lastVelocityRight = velocityRight;
        last_encoder_time = current_time;
    }
    
    // Procesar ultrasonidos según intervalo configurable
    if (current_time - last_ultrasonic_time >= ultrasonic_interval) {
        // Leer y publicar datos de sensores ultrasónicos
        float front_distance = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
        float left_distance = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
        float right_distance = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
        
        // Solo publicar si las lecturas son válidas
        if (front_distance >= 0) {
            ultrasonic_msg_front.data = front_distance;
            RCSOFTCHECK(rcl_publish(&ultrasonic_publisher_front, &ultrasonic_msg_front, NULL));
        }
        if (left_distance >= 0) {
            ultrasonic_msg_left.data = left_distance;
            RCSOFTCHECK(rcl_publish(&ultrasonic_publisher_left, &ultrasonic_msg_left, NULL));
        }
        if (right_distance >= 0) {
            ultrasonic_msg_right.data = right_distance;
            RCSOFTCHECK(rcl_publish(&ultrasonic_publisher_right, &ultrasonic_msg_right, NULL));
        }
        
        last_ultrasonic_time = current_time;
    }
}


// Callback para el control de velocidad
void velocity_callback(const void * msgin) {
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    targetSpeed = msg->data;
    // El PID se aplicará en el bucle principal
}

void applyPIDControl() {
    unsigned long currentTime = millis();
    if (currentTime - lastPIDTime >= PID_INTERVAL) {
        float deltaTime = (float)(currentTime - lastPIDTime) / 1000.0;  // Convertir a segundos
        
        // Calcular el error (diferencia entre las velocidades de las ruedas)
        float error = lastVelocityLeft - lastVelocityRight;
        
        // Calcular la salida del PID
        float pidOutput = calculatePID(error, deltaTime);
        
        // Ajustar los PWM basados en la salida del PID
        int basePWM = map(targetSpeed, 0, MAX_SPEED, 0, MAX_PWM);
        currentPWMLeft = basePWM - pidOutput;
        currentPWMRight = basePWM + pidOutput;
        
        // Asegurar que los valores de PWM estén dentro del rango válido
        currentPWMLeft = constrain(currentPWMLeft, 0, 255);
        currentPWMRight = constrain(currentPWMRight, 0, 255);
        
        // Aplicar los nuevos valores de PWM
        analogWrite(ENA_LEFT, currentPWMLeft);
        analogWrite(ENA_RIGHT, currentPWMRight);
        
        lastPIDTime = currentTime;
    }
}




// Función para inicializar los publishers de odometría
void initializeOdometryPublishers() {
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"));
        
    RCCHECK(rclc_publisher_init_default(
        &tf_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
        "/tf"));
}

// Función para calcular la odometría
void updateOdometry(float delta_left, float delta_right, float dt) {
    // Convertir velocidades angulares a velocidades lineales
    float v_left = (delta_left * PI / 180.0) * wheel_radius;  // m/s
    float v_right = (delta_right * PI / 180.0) * wheel_radius; // m/s
    
    // Calcular velocidad linear y angular del robot
    float v = (v_right + v_left) / 2.0;
    float omega = (v_right - v_left) / wheel_separation;
    
    // Actualizar posición y orientación
    theta += omega * dt;
    x += v * cos(theta) * dt;
    y += v * sin(theta) * dt;
    
    // Normalizar theta entre -π y π
    if(theta > PI) theta -= 2*PI;
    if(theta < -PI) theta += 2*PI;
}

// Función para publicar la odometría
void publishOdometry() {
    static uint32_t seq = 0;
    
    // Configurar el mensaje de odometría
    odom_msg.header.stamp.sec = millis() / 1000;
    odom_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    odom_msg.header.frame_id.data = (char*)"odom";
    odom_msg.child_frame_id.data = (char*)"base_link";
    
    // Establecer la posición
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    
    // Convertir theta a quaternion (simplificado para rotación en Z)
    odom_msg.pose.pose.orientation.w = cos(theta/2);
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(theta/2);
    
    // Publicar el mensaje
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    
    // Publicar transformación
    transform_msg.header = odom_msg.header;
    transform_msg.child_frame_id = odom_msg.child_frame_id;
    transform_msg.transform.translation.x = x;
    transform_msg.transform.translation.y = y;
    transform_msg.transform.translation.z = 0.0;
    transform_msg.transform.rotation = odom_msg.pose.pose.orientation;
    
    tf_msg.transforms.data = &transform_msg;
    tf_msg.transforms.size = 1;
    
    RCSOFTCHECK(rcl_publish(&tf_publisher, &tf_msg, NULL));
    
    seq++;
}



void setup() {
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  // Configuración de pines
    setupPins();

  // Set up micro-ROS
  IPAddress agent_ip(192, 168, 0, 105); // Change this to your ROS2 agent IP
  size_t agent_port = 8888;
  char agent_ip_str[16];
  sprintf(agent_ip_str, "%d.%d.%d.%d", agent_ip[0], agent_ip[1], agent_ip[2], agent_ip[3]);
  set_microros_wifi_transports(ssid, password, agent_ip_str, agent_port);
  


  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32_robot_node", "", &support));


    // Inicializar publishers
    initializePublishers();
    initializeOdometryPublishers(); 

  

  // Inicializar subscribers
    RCCHECK(rclc_subscription_init_default(
        &velocity_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "target_velocity"));

    RCCHECK(rclc_subscription_init_default(
        &ultrasonic_interval_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "ultrasonic_interval"));

  // Inicializar executor solo para subscribers
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &velocity_subscriber, 
        &target_velocity_msg, 
        &velocity_callback, 
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &ultrasonic_interval_subscriber, 
        &ultrasonic_interval_msg, 
        &ultrasonic_interval_callback, 
        ON_NEW_DATA));
  // Inicializar valores
    initializeVariables();


}

void loop() {
    // Handle WiFi reconnection
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.reconnect();
        while (WiFi.status() != WL_CONNECTED) {
            delay(1000);
        }
    }

  // Ejecutar tareas temporizadas
    executeTasks();

    // Aplicar control PID
    applyPIDControl();

    // Procesar callbacks de micro-ROS
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
    
    delay(1);  // Pequeña pausa para evitar sobrecarga del CPU
}

// Funciones auxiliares
void setupPins() {
    // [Pin setup code]
  pinMode(ENA_LEFT, OUTPUT);
  pinMode(IN1_LEFT, OUTPUT);
  pinMode(IN2_LEFT, OUTPUT);
  pinMode(ENA_RIGHT, OUTPUT);
  pinMode(IN1_RIGHT, OUTPUT);
  pinMode(IN2_RIGHT, OUTPUT);
  pinMode(ENCODER_A_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_B_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_A_RIGHT, INPUT_PULLUP);
  pinMode(ENCODER_B_RIGHT, INPUT_PULLUP);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_LEFT), updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_RIGHT), updateEncoderRight, RISING);
  
  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, LOW);
  digitalWrite(IN1_RIGHT, HIGH);
  digitalWrite(IN2_RIGHT, LOW);
  analogWrite(ENA_LEFT, 0);
  analogWrite(ENA_RIGHT, 0);
}

void initializePublishers() {
    // [Publishers initialization code]
    // Create publishers
  RCCHECK(rclc_publisher_init_default(
    &velocity_publisher_left,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "encoder_velocity_left"));
  RCCHECK(rclc_publisher_init_default(
    &velocity_publisher_right,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "encoder_velocity_right"));
  RCCHECK(rclc_publisher_init_default(
    &ultrasonic_publisher_front,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "ultrasonic_front"));
  RCCHECK(rclc_publisher_init_default(
    &ultrasonic_publisher_left,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "ultrasonic_left"));
  RCCHECK(rclc_publisher_init_default(
    &ultrasonic_publisher_right,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "ultrasonic_right"));
}

void initializeVariables() {
    // [Variables initialization code]
      velocity_msg_left.data = 0;
  velocity_msg_right.data = 0;
  ultrasonic_msg_front.data = 0;
  ultrasonic_msg_left.data = 0;
  ultrasonic_msg_right.data = 0;
  target_velocity_msg.data = 0;
  lastEncoderValueLeft = encoderValueLeft;
  lastEncoderValueRight = encoderValueRight;
  lastTime = millis();
   // Inicializar variables de tiempo
    last_encoder_time = millis();
    last_ultrasonic_time = millis();
    
    // Inicializar variables de velocidad
    lastVelocityLeft = 0;
    lastVelocityRight = 0;
}