#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>

#include <Wire.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>

// ========================================
// CONFIGURAZIONE WiFi e Agent
// ========================================
const char* ssid = "xxxxxxxx";
const char* password = "xxxxxxxxx";
const char* agent_ip = "192.168.1.31";
const int agent_port = 8888;

// Pin I2C ESP32
#define I2C_SDA 21
#define I2C_SCL 22

// Indirizzo MPU6050
#define DEVICE_ADDRESS 0x68

// Registri MPU6050
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

struct TiltData {
  float pitch;
  float roll;
};

// ========================================
// Pin per DRV8833 - Driver Motori
// ========================================
#define MOTOR_A_IN1 25
#define MOTOR_A_IN2 26
#define MOTOR_B_IN1 27
#define MOTOR_B_IN2 14
#define MOTOR_C_IN1 32
#define MOTOR_C_IN2 33
#define MOTOR_D_IN1 12
#define MOTOR_D_IN2 13

#define LED_PIN 2

// Impostazioni PWM
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

// Canali PWM per ESP32
#define PWM_CHANNEL_A1 0
#define PWM_CHANNEL_A2 1
#define PWM_CHANNEL_B1 2
#define PWM_CHANNEL_B2 3
#define PWM_CHANNEL_C1 4
#define PWM_CHANNEL_C2 5
#define PWM_CHANNEL_D1 6
#define PWM_CHANNEL_D2 7

// ========================================
// Classe Motor
// ========================================
class Motor {
private:
  int pin1;
  int pin2;
  int pwmChannel1;
  int pwmChannel2;

public:
  Motor(int p1, int p2, int ch1, int ch2) {
    pin1 = p1;
    pin2 = p2;
    pwmChannel1 = ch1;
    pwmChannel2 = ch2;
  }

  void init() {
    ledcSetup(pwmChannel1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(pwmChannel2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(pin1, pwmChannel1);
    ledcAttachPin(pin2, pwmChannel2);
    stop();
  }

  void forward(int speed) {
    speed = constrain(speed, 0, 255);
    ledcWrite(pwmChannel1, speed);
    ledcWrite(pwmChannel2, 0);
  }

  void backward(int speed) {
    speed = constrain(speed, 0, 255);
    ledcWrite(pwmChannel1, 0);
    ledcWrite(pwmChannel2, speed);
  }

  void stop() {
    ledcWrite(pwmChannel1, 0);
    ledcWrite(pwmChannel2, 0);
  }

  void setSpeed(int speed) {
    if (speed > 0) {
      forward(speed);
    } else if (speed < 0) {
      backward(-speed);
    } else {
      stop();
    }
  }
};

// Crea gli oggetti motore
Motor motorA(MOTOR_A_IN1, MOTOR_A_IN2, PWM_CHANNEL_A1, PWM_CHANNEL_A2);
Motor motorB(MOTOR_B_IN1, MOTOR_B_IN2, PWM_CHANNEL_B1, PWM_CHANNEL_B2);
Motor motorC(MOTOR_C_IN1, MOTOR_C_IN2, PWM_CHANNEL_C1, PWM_CHANNEL_C2);
Motor motorD(MOTOR_D_IN1, MOTOR_D_IN2, PWM_CHANNEL_D1, PWM_CHANNEL_D2);

// ========================================
// micro-ROS entities
// ========================================
rcl_subscription_t sub_motor1;
rcl_subscription_t sub_motor2;
rcl_subscription_t sub_motor3;
rcl_subscription_t sub_motor4;
rcl_publisher_t publisher_imu;
std_msgs__msg__String msg_motor1;
std_msgs__msg__String msg_motor2;
std_msgs__msg__String msg_motor3;
std_msgs__msg__String msg_motor4;
std_msgs__msg__String pub_msg_imu;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_imu;

// ========================================
// Callback motori
// ========================================
void motor1_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  int speed = atoi(msg->data.data);
  speed = constrain(speed, -1000, 1000);
  speed = map(speed, -1000, 1000, -255, 255);
  motorA.setSpeed(speed);
}

void motor2_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  int speed = atoi(msg->data.data);
  speed = constrain(speed, -1000, 1000);
  speed = map(speed, -1000, 1000, -255, 255);
  motorB.setSpeed(speed);
}

void motor3_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  int speed = atoi(msg->data.data);
  speed = constrain(speed, -1000, 1000);
  speed = map(speed, -1000, 1000, -255, 255);
  motorC.setSpeed(speed);
}

void motor4_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  int speed = atoi(msg->data.data);
  speed = constrain(speed, -1000, 1000);
  speed = map(speed, -1000, 1000, -255, 255);
  motorD.setSpeed(speed);
}

TiltData readPitchRoll() {
  TiltData t = {0, 0};

  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(DEVICE_ADDRESS, (uint8_t)14);

  if (Wire.available() < 14) return t;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  // Ignora temperatura e giroscopio
  for (int i = 0; i < 8; i++) Wire.read();

  float ax_g = ax / 4096.0;
  float ay_g = ay / 4096.0;
  float az_g = az / 4096.0;

  t.pitch = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0 / PI;
  t.roll  = atan2(-ax_g, az_g) * 180.0 / PI;

  return t;
}

// ========================================
// Timer callback - Pubblica pitch=0 roll=0
// ========================================
void timer_imu_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  (void) timer;

  TiltData tilt = readPitchRoll();

  char buffer[50];
  snprintf(buffer, sizeof(buffer),
         "{\"pitch\":%.1f,\"roll\":%.1f}",
         tilt.pitch, tilt.roll);
  pub_msg_imu.data.data = buffer;
  pub_msg_imu.data.size = strlen(buffer);
  pub_msg_imu.data.capacity = sizeof(buffer);

  rcl_publish(&publisher_imu, &pub_msg_imu, NULL);
}

// ========================================
// Error loop
// ========================================
void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ========================================
// Macro per gestire errori
// ========================================
#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){ \
    error_loop(); \
  } \
}

#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){ \
  } \
}



// ========================================
// SETUP
// ========================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  // Risveglio sensore
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

  // Config accelerometro ±8g
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(MPU6050_ACCEL_CONFIG);
  Wire.write(0x10);
  Wire.endTransmission();

  // Config giroscopio ±500°/s
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(MPU6050_GYRO_CONFIG);
  Wire.write(0x08);
  Wire.endTransmission();

  // Inizializzazione Motori
  motorA.init();
  motorB.init();
  motorC.init();
  motorD.init();

  // Connessione WiFi
  WiFi.begin(ssid, password);
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 30) {
    delay(500);
    wifi_attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    error_loop();
  }

  // Configurazione micro-ROS Transport
  set_microros_wifi_transports(
    (char*)ssid,
    (char*)password,
    (char*)agent_ip,
    agent_port
  );

  delay(1000);

  // Inizializzazione micro-ROS
  allocator = rcl_get_default_allocator();

  rcl_ret_t ret;
  const int max_attempts = 20;

  for (int attempt = 1; attempt <= max_attempts; attempt++) {
    ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret == RCL_RET_OK) {
      break;
    }
    delay(1000);
  }

  if (ret != RCL_RET_OK) {
    error_loop();
  }

  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32_robot", "", &support));

  // Create subscribers motori
  RCCHECK(rclc_subscription_init_default(
    &sub_motor1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "m1ctrl"
  ));

  RCCHECK(rclc_subscription_init_default(
    &sub_motor2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "m2ctrl"
  ));

  RCCHECK(rclc_subscription_init_default(
    &sub_motor3,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "m3ctrl"
  ));

  RCCHECK(rclc_subscription_init_default(
    &sub_motor4,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "m4ctrl"
  ));

  // Create publisher IMU
  RCCHECK(rclc_publisher_init_default(
    &publisher_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "imu"
  ));

  // Create timer IMU (1000ms = 1Hz)
  RCCHECK(rclc_timer_init_default(
    &timer_imu,
    &support,
    RCL_MS_TO_NS(1000),
    timer_imu_callback
  ));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));

  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor1, &msg_motor1, &motor1_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor2, &msg_motor2, &motor2_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor3, &msg_motor3, &motor3_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor4, &msg_motor4, &motor4_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));

  // Alloca memoria per messaggi
  msg_motor1.data.data = (char*)malloc(10);
  msg_motor1.data.capacity = 10;

  msg_motor2.data.data = (char*)malloc(10);
  msg_motor2.data.capacity = 10;

  msg_motor3.data.data = (char*)malloc(10);
  msg_motor3.data.capacity = 10;

  msg_motor4.data.data = (char*)malloc(10);
  msg_motor4.data.capacity = 10;

  pub_msg_imu.data.data = (char*)malloc(50);
  pub_msg_imu.data.capacity = 50;

  // Blink di conferma
  for(int i = 0; i < 6; i++) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  digitalWrite(LED_PIN, LOW);
}

// ========================================
// LOOP
// ========================================
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}
