#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <micro_ros_arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>

// ========================================
// CONFIGURAZIONE WiFi e Agent
// ========================================
const char* ssid = "XXXX";
const char* password = "xxxx";
const char* agent_ip = "192.168.1.31";
const int agent_port = 8888;

// ========================================
// CONFIGURAZIONE MPU6050
// ========================================
#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_MPU6050 mpu6050;
bool mpu6050Available = false;

// ========================================
// Pin per DRV8833 - Driver Motori (TESTATI)
// ========================================
// Primo DRV8833 - Motor A e Motor B
#define MOTOR_A_IN1 25
#define MOTOR_A_IN2 26
#define MOTOR_B_IN1 27
#define MOTOR_B_IN2 14

// Secondo DRV8833 - Motor C e Motor D
#define MOTOR_C_IN1 32
#define MOTOR_C_IN2 33
#define MOTOR_D_IN1 12
#define MOTOR_D_IN2 13

// LED integrato
#define LED_PIN 2

// Impostazioni PWM
#define PWM_FREQ 1000      // Frequenza PWM in Hz
#define PWM_RESOLUTION 8   // Risoluzione PWM (8 bit = 0-255)

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
// Classe Motor (TESTATA)
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

  void brake() {
    ledcWrite(pwmChannel1, 255);
    ledcWrite(pwmChannel2, 255);
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
  // Converti da -1000/+1000 a -255/+255
  speed = constrain(speed, -1000, 1000);
  speed = map(speed, -1000, 1000, -255, 255);
  motorA.setSpeed(speed);
  Serial.print("Motor A: ");
  Serial.println(speed);
}

void motor2_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  int speed = atoi(msg->data.data);
  speed = constrain(speed, -1000, 1000);
  speed = map(speed, -1000, 1000, -255, 255);
  motorB.setSpeed(speed);
  Serial.print("Motor B: ");
  Serial.println(speed);
}

void motor3_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  int speed = atoi(msg->data.data);
  speed = constrain(speed, -1000, 1000);
  speed = map(speed, -1000, 1000, -255, 255);
  motorC.setSpeed(speed);
  Serial.print("Motor C: ");
  Serial.println(speed);
}

void motor4_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  int speed = atoi(msg->data.data);
  speed = constrain(speed, -1000, 1000);
  speed = map(speed, -1000, 1000, -255, 255);
  motorD.setSpeed(speed);
  Serial.print("Motor D: ");
  Serial.println(speed);
}

// ========================================
// Timer callback per pubblicare dati IMU
// ========================================
void timer_imu_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  (void) timer;

  if (!mpu6050Available) {
    return;
  }

  sensors_event_t a, g, temp;

  if (mpu6050.getEvent(&a, &g, &temp)) {
    // Converti accelerazione da m/s² a g
    float accelX = a.acceleration.x / 9.81;
    float accelY = a.acceleration.y / 9.81;
    float accelZ = a.acceleration.z / 9.81;

    // Converti giroscopio da rad/s a deg/s
    float gyroX = g.gyro.x * 180.0 / PI;
    float gyroY = g.gyro.y * 180.0 / PI;
    float gyroZ = g.gyro.z * 180.0 / PI;

    // Calcola angoli di inclinazione
    float pitch = atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ)) * 180.0 / PI;
    float roll = atan2(-accelX, accelZ) * 180.0 / PI;

    // Formato JSON per facile parsing
    char buffer[250];
    snprintf(buffer, sizeof(buffer),
             "{\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
             "\"gyro\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
             "\"temp\":%.1f,\"pitch\":%.1f,\"roll\":%.1f}",
             accelX, accelY, accelZ,
             gyroX, gyroY, gyroZ,
             temp.temperature, pitch, roll);

    pub_msg_imu.data.data = buffer;
    pub_msg_imu.data.size = strlen(buffer);
    pub_msg_imu.data.capacity = sizeof(buffer);

    rcl_ret_t ret = rcl_publish(&publisher_imu, &pub_msg_imu, NULL);

    if (ret == RCL_RET_OK) {
      Serial.print("IMU: ");
      Serial.println(buffer);
    }
  } else {
    static unsigned long lastRetry = 0;
    if (millis() - lastRetry > 10000) {
      Serial.println("Errore lettura MPU6050 - tentativo reinizializzazione...");
      if (mpu6050.begin()) {
        Serial.println("MPU6050 reinizializzato con successo");
        mpu6050Available = true;
      } else {
        Serial.println("Reinizializzazione fallita");
        mpu6050Available = false;
      }
      lastRetry = millis();
    }
  }
}

// ========================================
// Error loop
// ========================================
void error_loop() {
  Serial.println("\nERRORE FATALE - Sistema bloccato");
  Serial.println("Premi RESET per riavviare");
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
    Serial.print("ERRORE in: "); \
    Serial.println(#fn); \
    Serial.print("   Codice errore: "); \
    Serial.println(temp_rc); \
    error_loop(); \
  } \
}

#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){ \
    Serial.print("Warning: "); \
    Serial.print(#fn); \
    Serial.print(" -> "); \
    Serial.println(temp_rc); \
  } \
}

// ========================================
// Test UDP verso l'agent
// ========================================
bool testUDPConnection() {
  Serial.println("\nTEST CONNETTIVITA' AGENT");
  Serial.println("========================================");

  IPAddress agentIPAddr;
  if (!agentIPAddr.fromString(agent_ip)) {
    Serial.println("IP non valido!");
    return false;
  }

  Serial.print("Test invio pacchetto UDP a ");
  Serial.print(agent_ip);
  Serial.print(":");
  Serial.print(agent_port);
  Serial.print("... ");

  WiFiUDP udp;
  udp.begin(0);
  udp.beginPacket(agentIPAddr, agent_port);
  udp.write((const uint8_t*)"TEST", 4);
  bool sent = udp.endPacket();
  udp.stop();

  if (sent) {
    Serial.println("OK");
  } else {
    Serial.println("FALLITO");
    return false;
  }

  Serial.println("========================================\n");
  return true;
}

// ========================================
// Scanner I2C
// ========================================
void scanI2C() {
  Serial.println("\nScansione bus I2C...");
  byte error, address;
  int nDevices = 0;

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo I2C trovato all'indirizzo 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;

      if (address == 0x68) {
        Serial.println("  -> MPU6050 rilevato!");
      }
      if (address == 0x69) {
        Serial.println("  -> MPU6050 (addr alternativo) rilevato!");
      }
    }
  }

  if (nDevices == 0) {
    Serial.println("\nATTENZIONE: Nessun dispositivo I2C trovato!");
    Serial.println("\nVerifica collegamenti:");
    Serial.println("  MPU6050 VCC  -> ESP32 3.3V");
    Serial.println("  MPU6050 GND  -> ESP32 GND");
    Serial.println("  MPU6050 SDA  -> ESP32 GPIO21");
    Serial.println("  MPU6050 SCL  -> ESP32 GPIO22");
  } else {
    Serial.print("\nTrovati ");
    Serial.print(nDevices);
    Serial.println(" dispositivi I2C");
  }
}

// ========================================
// SETUP
// ========================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("\n\n========================================");
  Serial.println("micro-ROS ESP32 Robot Controller");
  Serial.println("MPU6050 + 4x DRV8833 Motors");
  Serial.println("========================================");
  Serial.print("Firmware: ");
  Serial.println(__DATE__ " " __TIME__);
  Serial.println();

  // ========================================
  // Inizializzazione I2C e MPU6050
  // ========================================
  Serial.println("Inizializzazione I2C...");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(500);

  scanI2C();

  Serial.println("\nInizializzazione MPU6050...");
  if (mpu6050.begin()) {
    Serial.println("MPU6050 inizializzato con successo!");
    mpu6050Available = true;

    mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("Configurazione MPU6050:");
    Serial.println("  - Range accelerometro: ±8g");
    Serial.println("  - Range giroscopio: ±500°/s");
    Serial.println("  - Filtro passa-basso: 21Hz");
  } else {
    Serial.println("MPU6050 non rilevato!");
    Serial.println("Il sistema funzionera' solo con i motori.\n");
    mpu6050Available = false;
  }

  // ========================================
  // Inizializzazione Motori
  // ========================================
  Serial.println("\n=== Inizializzazione Motori ===");
  motorA.init();
  motorB.init();
  motorC.init();
  motorD.init();
  Serial.println("Tutti i 4 motori pronti!");

  Serial.println("\n=== Collegamenti Hardware ===");
  Serial.println("MPU6050:");
  Serial.println("  SDA -> GPIO21");
  Serial.println("  SCL -> GPIO22");
  Serial.println("  VCC -> 3.3V");
  Serial.println("  GND -> GND");

  Serial.println("\nPrimo DRV8833:");
  Serial.println("  Motor A: IN1->GPIO25, IN2->GPIO26");
  Serial.println("  Motor B: IN1->GPIO27, IN2->GPIO14");

  Serial.println("\nSecondo DRV8833:");
  Serial.println("  Motor C: IN1->GPIO32, IN2->GPIO33");
  Serial.println("  Motor D: IN1->GPIO12, IN2->GPIO13");

  Serial.println("\nIMPORTANTE:");
  Serial.println("  - Alimenta i DRV8833 con batteria (VM e GND)");
  Serial.println("  - GND COMUNE tra ESP32, MPU6050 e DRV8833!");
  Serial.println("  - MPU6050 solo a 3.3V (NON 5V!)");
  Serial.println("=====================================\n");

  // ========================================
  // Connessione WiFi
  // ========================================
  Serial.print("Connessione WiFi a: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 30) {
    delay(500);
    Serial.print(".");
    wifi_attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nImpossibile connettersi al WiFi!");
    error_loop();
  }

  Serial.println("\nWiFi connesso!");
  Serial.print("IP ESP32: ");
  Serial.println(WiFi.localIP());
  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.println();

  // ========================================
  // Test connettivita' agent
  // ========================================
  if (!testUDPConnection()) {
    Serial.println("ATTENZIONE: Problemi di connettivita' rilevati");
    Serial.println("Continuo comunque...");
  }

  // ========================================
  // Configurazione micro-ROS Transport
  // ========================================
  Serial.println("Configurazione trasporto micro-ROS...");
  Serial.print("   Agent: ");
  Serial.print(agent_ip);
  Serial.print(":");
  Serial.println(agent_port);

  set_microros_wifi_transports(
    (char*)ssid,
    (char*)password,
    (char*)agent_ip,
    agent_port
  );

  Serial.println("   Trasporto configurato");
  delay(1000);

  // ========================================
  // Inizializzazione micro-ROS
  // ========================================
  Serial.println("\nInizializzazione micro-ROS...");
  Serial.println("========================================");

  allocator = rcl_get_default_allocator();
  Serial.println("Allocator creato");

  Serial.println("\nConnessione all'agent...");
  Serial.println("IMPORTANTE: Osserva i log del container Docker!");
  Serial.println();

  rcl_ret_t ret;
  const int max_attempts = 20;

  for (int attempt = 1; attempt <= max_attempts; attempt++) {
    Serial.print("[");
    Serial.print(attempt);
    Serial.print("/");
    Serial.print(max_attempts);
    Serial.print("] ");

    ret = rclc_support_init(&support, 0, NULL, &allocator);

    if (ret == RCL_RET_OK) {
      Serial.println("SUCCESS!");
      break;
    }

    Serial.print("FAIL (");
    Serial.print(ret);
    Serial.print(") ");

    if (attempt % 5 == 0) {
      Serial.println();
    }

    delay(1000);
  }

  if (ret != RCL_RET_OK) {
    Serial.println("\n\n========================================");
    Serial.println("ERRORE: Connessione fallita!");
    Serial.println("========================================");
    Serial.println("Agent: " + String(agent_ip) + ":" + String(agent_port));
    Serial.println();
    Serial.println("Verifica:");
    Serial.println("1. Container Docker attivo:");
    Serial.println("   docker ps | grep micro-ros");
    Serial.println();
    Serial.println("2. Mapping porta corretta:");
    Serial.println("   -p 8888:8888/udp");
    Serial.println();
    Serial.println("3. Log container:");
    Serial.println("   docker logs <container_id>");
    Serial.println("========================================");
    error_loop();
  }

  Serial.println("Support inizializzato");

  // Create node
  Serial.print("Creazione nodo 'esp32_robot'... ");
  RCCHECK(rclc_node_init_default(&node, "esp32_robot", "", &support));
  Serial.println("OK");

  // Create subscribers motori
  Serial.print("Creazione subscriber '/m1ctrl'... ");
  RCCHECK(rclc_subscription_init_default(
    &sub_motor1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "m1ctrl"
  ));
  Serial.println("OK");

  Serial.print("Creazione subscriber '/m2ctrl'... ");
  RCCHECK(rclc_subscription_init_default(
    &sub_motor2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "m2ctrl"
  ));
  Serial.println("OK");

  Serial.print("Creazione subscriber '/m3ctrl'... ");
  RCCHECK(rclc_subscription_init_default(
    &sub_motor3,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "m3ctrl"
  ));
  Serial.println("OK");

  Serial.print("Creazione subscriber '/m4ctrl'... ");
  RCCHECK(rclc_subscription_init_default(
    &sub_motor4,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "m4ctrl"
  ));
  Serial.println("OK");

  // Create publisher IMU
  Serial.print("Creazione publisher '/imu'... ");
  RCCHECK(rclc_publisher_init_default(
    &publisher_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "imu"
  ));
  Serial.println("OK");

  // Create timer IMU (pubblica ogni 100ms = 10Hz)
  Serial.print("Creazione timer IMU (100ms)... ");
  RCCHECK(rclc_timer_init_default(
    &timer_imu,
    &support,
    RCL_MS_TO_NS(100),
    timer_imu_callback
  ));
  Serial.println("OK");

  // Create executor
  // Handles: 4 subscriptions (motori) + 1 timer (IMU) = 5 handles
  Serial.print("Creazione executor (5 handles)... ");
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  Serial.println("OK");

  Serial.print("Aggiunta subscription '/m1ctrl'... ");
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor1, &msg_motor1, &motor1_callback, ON_NEW_DATA));
  Serial.println("OK");

  Serial.print("Aggiunta subscription '/m2ctrl'... ");
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor2, &msg_motor2, &motor2_callback, ON_NEW_DATA));
  Serial.println("OK");

  Serial.print("Aggiunta subscription '/m3ctrl'... ");
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor3, &msg_motor3, &motor3_callback, ON_NEW_DATA));
  Serial.println("OK");

  Serial.print("Aggiunta subscription '/m4ctrl'... ");
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor4, &msg_motor4, &motor4_callback, ON_NEW_DATA));
  Serial.println("OK");

  Serial.print("Aggiunta timer IMU... ");
  RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
  Serial.println("OK");

  // Alloca memoria per messaggi motori
  msg_motor1.data.data = (char*)malloc(10 * sizeof(char));
  msg_motor1.data.size = 0;
  msg_motor1.data.capacity = 10;

  msg_motor2.data.data = (char*)malloc(10 * sizeof(char));
  msg_motor2.data.size = 0;
  msg_motor2.data.capacity = 10;

  msg_motor3.data.data = (char*)malloc(10 * sizeof(char));
  msg_motor3.data.size = 0;
  msg_motor3.data.capacity = 10;

  msg_motor4.data.data = (char*)malloc(10 * sizeof(char));
  msg_motor4.data.size = 0;
  msg_motor4.data.capacity = 10;

  pub_msg_imu.data.data = (char*)malloc(250 * sizeof(char));
  pub_msg_imu.data.size = 0;
  pub_msg_imu.data.capacity = 250;

  Serial.println("========================================");
  Serial.println("SISTEMA PRONTO!");
  Serial.println("========================================");
  Serial.println("Subscribers:");
  Serial.println("   /m1ctrl  (std_msgs/String) - Motor A");
  Serial.println("   /m2ctrl  (std_msgs/String) - Motor B");
  Serial.println("   /m3ctrl  (std_msgs/String) - Motor C");
  Serial.println("   /m4ctrl  (std_msgs/String) - Motor D");
  Serial.println();
  Serial.println("Publishers:");
  Serial.println("   /imu (std_msgs/String) @ 10 Hz");
  Serial.println();
  Serial.println("Controllo Motori:");
  Serial.println("   Valori: -1000 (max indietro) a +1000 (max avanti)");
  Serial.println("   Valore 0 = fermo");
  Serial.println();
  Serial.println("Pin Mapping:");
  Serial.println("   Motor A (M1): GPIO25, GPIO26");
  Serial.println("   Motor B (M2): GPIO27, GPIO14");
  Serial.println("   Motor C (M3): GPIO32, GPIO33");
  Serial.println("   Motor D (M4): GPIO12, GPIO13");
  Serial.println();
  Serial.println("MPU6050:");
  Serial.println("   SDA: GPIO21, SCL: GPIO22");
  Serial.println("========================================\n");

  // Blink di conferma (3 volte veloce)
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
