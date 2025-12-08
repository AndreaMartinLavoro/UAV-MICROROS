#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <micro_ros_arduino.h>
#include <DFRobot_BMI160.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>

// ========================================
// CONFIGURAZIONE WiFi e Agent
// ========================================
const char* ssid = "FIBRA-M4A";
const char* password = "7zQIxOAtXV5cV9";
const char* agent_ip = "192.168.1.31";  // Modifica se necessario
const int agent_port = 8888;

// ========================================
// Pin digitali da monitorare
// ========================================
#define PIN_1 25  // GPIO 25
#define PIN_2 26  // GPIO 26
#define LED_PIN 2 // LED integrato

// Pin I2C per BMI160 (default ESP32)
#define I2C_SDA 21
#define I2C_SCL 22

// ========================================
// Pin per DRV8833 - Driver Motori
// ========================================
// DRV8833 #1 - Motori 1 e 2
#define MOTOR1_IN1  13  // GPIO 13
#define MOTOR1_IN2  12  // GPIO 12
#define MOTOR2_IN1  14  // GPIO 14
#define MOTOR2_IN2  27  // GPIO 27

// DRV8833 #2 - Motori 3 e 4
#define MOTOR3_IN1  26  // GPIO 26
#define MOTOR3_IN2  25  // GPIO 25
#define MOTOR4_IN1  33  // GPIO 33
#define MOTOR4_IN2  32  // GPIO 32

// Canali PWM (ESP32 ha 16 canali PWM)
#define PWM_FREQ 20000     // 20 kHz
#define PWM_RESOLUTION 10  // 10 bit (0-1023)

#define PWM_MOTOR1_IN1 0
#define PWM_MOTOR1_IN2 1
#define PWM_MOTOR2_IN1 2
#define PWM_MOTOR2_IN2 3
#define PWM_MOTOR3_IN1 4
#define PWM_MOTOR3_IN2 5
#define PWM_MOTOR4_IN1 6
#define PWM_MOTOR4_IN2 7

// ========================================
// Sensore BMI160
// ========================================
DFRobot_BMI160 bmi160;

// ========================================
// micro-ROS entities
// ========================================
rcl_subscription_t subscriber;
rcl_subscription_t sub_motor1;
rcl_subscription_t sub_motor2;
rcl_subscription_t sub_motor3;
rcl_subscription_t sub_motor4;
rcl_publisher_t publisher_pins;
rcl_publisher_t publisher_imu;
std_msgs__msg__String sub_msg;
std_msgs__msg__String msg_motor1;
std_msgs__msg__String msg_motor2;
std_msgs__msg__String msg_motor3;
std_msgs__msg__String msg_motor4;
std_msgs__msg__String pub_msg_pins;
std_msgs__msg__String pub_msg_imu;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_pins;
rcl_timer_t timer_imu;

// ========================================
// Funzione per controllare un motore
// ========================================
// speed: -1000 a +1000
// -1000 = massima velocità indietro
//     0 = fermo
// +1000 = massima velocità avanti
void setMotorSpeed(int motorNum, int16_t speed) {
  // Limita il valore
  speed = constrain(speed, -1000, 1000);

  // Converti da -1000/+1000 a 0-1023 (10 bit PWM)
  uint16_t pwm_value = abs(speed) * 1023 / 1000;

  uint8_t pin_in1, pin_in2, pwm_ch1, pwm_ch2;

  // Seleziona i pin in base al motore
  switch(motorNum) {
    case 1:
      pin_in1 = MOTOR1_IN1; pin_in2 = MOTOR1_IN2;
      pwm_ch1 = PWM_MOTOR1_IN1; pwm_ch2 = PWM_MOTOR1_IN2;
      break;
    case 2:
      pin_in1 = MOTOR2_IN1; pin_in2 = MOTOR2_IN2;
      pwm_ch1 = PWM_MOTOR2_IN1; pwm_ch2 = PWM_MOTOR2_IN2;
      break;
    case 3:
      pin_in1 = MOTOR3_IN1; pin_in2 = MOTOR3_IN2;
      pwm_ch1 = PWM_MOTOR3_IN1; pwm_ch2 = PWM_MOTOR3_IN2;
      break;
    case 4:
      pin_in1 = MOTOR4_IN1; pin_in2 = MOTOR4_IN2;
      pwm_ch1 = PWM_MOTOR4_IN1; pwm_ch2 = PWM_MOTOR4_IN2;
      break;
    default:
      return;
  }

  if (speed > 0) {
    // Avanti
    ledcWrite(pwm_ch1, pwm_value);
    ledcWrite(pwm_ch2, 0);
  } else if (speed < 0) {
    // Indietro
    ledcWrite(pwm_ch1, 0);
    ledcWrite(pwm_ch2, pwm_value);
  } else {
    // Fermo
    ledcWrite(pwm_ch1, 0);
    ledcWrite(pwm_ch2, 0);
  }
}

// ========================================
// Callback motori
// ========================================
void motor1_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  int16_t speed = atoi(msg->data.data);
  setMotorSpeed(1, speed);
  Serial.print("🔧 M1: ");
  Serial.println(speed);
}

void motor2_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  int16_t speed = atoi(msg->data.data);
  setMotorSpeed(2, speed);
  Serial.print("🔧 M2: ");
  Serial.println(speed);
}

void motor3_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  int16_t speed = atoi(msg->data.data);
  setMotorSpeed(3, speed);
  Serial.print("🔧 M3: ");
  Serial.println(speed);
}

void motor4_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  int16_t speed = atoi(msg->data.data);
  setMotorSpeed(4, speed);
  Serial.print("🔧 M4: ");
  Serial.println(speed);
}

// ========================================
// Callback quando arriva un messaggio sul subscriber
// ========================================
void subscription_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;

  Serial.print("📥 Ricevuto: ");
  Serial.println(msg->data.data);

  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
}

// ========================================
// Timer callback per pubblicare lo stato dei pin
// ========================================
void timer_pins_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  (void) timer;

  // Leggi lo stato dei pin
  int pin1_state = digitalRead(PIN_1);
  int pin2_state = digitalRead(PIN_2);

  // Crea il messaggio
  char buffer[50];
  snprintf(buffer, sizeof(buffer), "PIN1:%d,PIN2:%d", pin1_state, pin2_state);

  // Pubblica
  pub_msg_pins.data.data = buffer;
  pub_msg_pins.data.size = strlen(buffer);
  pub_msg_pins.data.capacity = sizeof(buffer);

  rcl_ret_t ret = rcl_publish(&publisher_pins, &pub_msg_pins, NULL);

  if (ret == RCL_RET_OK) {
    Serial.print("📍 Pins: ");
    Serial.println(buffer);
  }
}

// ========================================
// Timer callback per pubblicare dati IMU
// ========================================
void timer_imu_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  (void) timer;

  // Leggi dati dal BMI160
  int rslt;
  int16_t accelGyro[6] = {0};

  rslt = bmi160.getAccelGyroData(accelGyro);

  if (rslt == 0) {
    // accelGyro[0-2] = accelerometro X, Y, Z (mg)
    // accelGyro[3-5] = giroscopio X, Y, Z (°/s * 100)

    // Converti in unità standard: m/s² e rad/s
    float accel_x = accelGyro[0] * 0.00981;  // mg to m/s²
    float accel_y = accelGyro[1] * 0.00981;
    float accel_z = accelGyro[2] * 0.00981;

    float gyro_x = accelGyro[3] * 0.0001745;  // (°/s * 100) to rad/s
    float gyro_y = accelGyro[4] * 0.0001745;
    float gyro_z = accelGyro[5] * 0.0001745;

    // Formato JSON per facile parsing
    char buffer[200];
    snprintf(buffer, sizeof(buffer),
             "{\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
             "\"gyro\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}",
             accel_x, accel_y, accel_z,
             gyro_x, gyro_y, gyro_z);

    // Pubblica
    pub_msg_imu.data.data = buffer;
    pub_msg_imu.data.size = strlen(buffer);
    pub_msg_imu.data.capacity = sizeof(buffer);

    rcl_ret_t ret = rcl_publish(&publisher_imu, &pub_msg_imu, NULL);

    if (ret == RCL_RET_OK) {
      Serial.print("📊 IMU: ");
      Serial.println(buffer);
    }
  } else {
    Serial.println("⚠️  Errore lettura BMI160");
  }
}

// ========================================
// Error loop - lampeggia rapidamente
// ========================================
void error_loop() {
  Serial.println("\n❌ ERRORE FATALE - Sistema bloccato");
  Serial.println("Premi RESET per riavviare");
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ========================================
// Macro per gestire errori con debug
// ========================================
#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){ \
    Serial.print("❌ ERRORE in: "); \
    Serial.println(#fn); \
    Serial.print("   Codice errore: "); \
    Serial.println(temp_rc); \
    error_loop(); \
  } \
}

#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){ \
    Serial.print("⚠️  Warning: "); \
    Serial.print(#fn); \
    Serial.print(" -> "); \
    Serial.println(temp_rc); \
  } \
}

// ========================================
// Test UDP verso l'agent
// ========================================
bool testUDPConnection() {
  Serial.println("\n🔍 TEST CONNETTIVITA' AGENT");
  Serial.println("========================================");

  IPAddress agentIPAddr;
  if (!agentIPAddr.fromString(agent_ip)) {
    Serial.println("❌ IP non valido!");
    return false;
  }

  Serial.print("Test invio pacchetto UDP a ");
  Serial.print(agent_ip);
  Serial.print(":");
  Serial.print(agent_port);
  Serial.print("... ");

  WiFiUDP udp;
  udp.begin(0);  // Usa porta random
  udp.beginPacket(agentIPAddr, agent_port);
  udp.write((const uint8_t*)"TEST", 4);
  bool sent = udp.endPacket();
  udp.stop();

  if (sent) {
    Serial.println("✅ OK");
  } else {
    Serial.println("❌ FALLITO");
    return false;
  }

  Serial.println("========================================\n");
  return true;
}

// ========================================
// SETUP
// ========================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Configura i pin digitali come INPUT con pull-up
  pinMode(PIN_1, INPUT_PULLUP);
  pinMode(PIN_2, INPUT_PULLUP);

  // ========================================
  // Inizializzazione PWM per motori
  // ========================================
  Serial.println("\n\n🔧 Inizializzazione driver motori DRV8833...");

  // Configura PWM per Motor 1
  ledcSetup(PWM_MOTOR1_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_MOTOR1_IN2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR1_IN1, PWM_MOTOR1_IN1);
  ledcAttachPin(MOTOR1_IN2, PWM_MOTOR1_IN2);

  // Configura PWM per Motor 2
  ledcSetup(PWM_MOTOR2_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_MOTOR2_IN2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR2_IN1, PWM_MOTOR2_IN1);
  ledcAttachPin(MOTOR2_IN2, PWM_MOTOR2_IN2);

  // Configura PWM per Motor 3
  ledcSetup(PWM_MOTOR3_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_MOTOR3_IN2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR3_IN1, PWM_MOTOR3_IN1);
  ledcAttachPin(MOTOR3_IN2, PWM_MOTOR3_IN2);

  // Configura PWM per Motor 4
  ledcSetup(PWM_MOTOR4_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_MOTOR4_IN2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR4_IN1, PWM_MOTOR4_IN1);
  ledcAttachPin(MOTOR4_IN2, PWM_MOTOR4_IN2);

  // Ferma tutti i motori
  for(int i = 1; i <= 4; i++) {
    setMotorSpeed(i, 0);
  }

  Serial.println("✅ Motori inizializzati e fermati");
  Serial.println();

  Serial.println("🚀 micro-ROS ESP32 Robot Controller");
  Serial.println("==========================================");
  Serial.print("Versione firmware: ");
  Serial.println(__DATE__ " " __TIME__);
  Serial.println();

  // ========================================
  // Connessione WiFi
  // ========================================
  Serial.print("📶 Connessione WiFi a: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 30) {
    delay(500);
    Serial.print(".");
    wifi_attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n❌ Impossibile connettersi al WiFi!");
    error_loop();
  }

  Serial.println("\n✅ WiFi connesso!");
  Serial.print("📍 IP ESP32: ");
  Serial.println(WiFi.localIP());
  Serial.print("📍 Gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("📍 DNS: ");
  Serial.println(WiFi.dnsIP());
  Serial.println();

  // ========================================
  // Inizializzazione BMI160
  // ========================================
  Serial.println("\n🔧 Inizializzazione BMI160...");
  Serial.println("========================================");

  Wire.begin(I2C_SDA, I2C_SCL);

  if (bmi160.softReset() != BMI160_OK) {
    Serial.println("❌ Errore reset BMI160!");
    Serial.println("⚠️  Verifica collegamento I2C:");
    Serial.println("   SDA -> GPIO21");
    Serial.println("   SCL -> GPIO22");
    Serial.println("   VCC -> 3.3V");
    Serial.println("   GND -> GND");
    Serial.println("\nContinuo senza IMU...");
  } else {
    if (bmi160.I2cInit() != BMI160_OK) {
      Serial.println("❌ Errore inizializzazione I2C BMI160!");
      Serial.println("Continuo senza IMU...");
    } else {
      Serial.println("✅ BMI160 inizializzato!");
      Serial.println("   SDA: GPIO21");
      Serial.println("   SCL: GPIO22");
    }
  }
  Serial.println("========================================\n");

  // ========================================
  // Test connettività agent
  // ========================================
  if (!testUDPConnection()) {
    Serial.println("⚠️  ATTENZIONE: Problemi di connettività rilevati");
    Serial.println("Continuo comunque...");
  }

  // ========================================
  // Configurazione micro-ROS Transport
  // ========================================
  Serial.println("🔧 Configurazione trasporto micro-ROS...");
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

  Serial.println("   ✅ Trasporto configurato");
  delay(1000);

  // ========================================
  // Inizializzazione micro-ROS con retry
  // ========================================
  Serial.println("\n🔧 Inizializzazione micro-ROS...");
  Serial.println("========================================");

  allocator = rcl_get_default_allocator();
  Serial.println("✓ Allocator creato");

  // Retry logic per la connessione all'agent
  Serial.println("\nConnessione all'agent...");
  Serial.println("IMPORTANTE: Osserva i log del container Docker!");
  Serial.println("Dovresti vedere una riga tipo:");
  Serial.println("  'session established | client_key: 0xXXXXXXXX, address: 192.168.1.X'");
  Serial.println();

  rcl_ret_t ret;
  const int max_attempts = 20;  // Aumentato a 20 tentativi

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

    // Mostra un punto di progresso
    if (attempt % 5 == 0) {
      Serial.println();
    }

    delay(1000);  // Ridotto a 1 secondo
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

  Serial.println("✓ Support inizializzato");

  // Create node
  Serial.print("Creazione nodo 'esp32_subscriber'... ");
  RCCHECK(rclc_node_init_default(&node, "esp32_subscriber", "", &support));
  Serial.println("✓");

  // Create subscriber chatter
  Serial.print("Creazione subscriber sul topic '/chatter'... ");
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "chatter"
  ));
  Serial.println("✓");

  // Create subscribers motori
  Serial.print("Creazione subscriber sul topic '/m1ctrl'... ");
  RCCHECK(rclc_subscription_init_default(
    &sub_motor1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "m1ctrl"
  ));
  Serial.println("✓");

  Serial.print("Creazione subscriber sul topic '/m2ctrl'... ");
  RCCHECK(rclc_subscription_init_default(
    &sub_motor2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "m2ctrl"
  ));
  Serial.println("✓");

  Serial.print("Creazione subscriber sul topic '/m3ctrl'... ");
  RCCHECK(rclc_subscription_init_default(
    &sub_motor3,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "m3ctrl"
  ));
  Serial.println("✓");

  Serial.print("Creazione subscriber sul topic '/m4ctrl'... ");
  RCCHECK(rclc_subscription_init_default(
    &sub_motor4,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "m4ctrl"
  ));
  Serial.println("✓");

  // Create publisher pins
  Serial.print("Creazione publisher sul topic '/pins'... ");
  RCCHECK(rclc_publisher_init_default(
    &publisher_pins,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "pins"
  ));
  Serial.println("✓");

  // Create publisher IMU
  Serial.print("Creazione publisher sul topic '/imu'... ");
  RCCHECK(rclc_publisher_init_default(
    &publisher_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "imu"
  ));
  Serial.println("✓");

  // Create timer pins (pubblica ogni 1 secondo)
  Serial.print("Creazione timer pins (1000ms)... ");
  RCCHECK(rclc_timer_init_default(
    &timer_pins,
    &support,
    RCL_MS_TO_NS(1000),
    timer_pins_callback
  ));
  Serial.println("✓");

  // Create timer IMU (pubblica ogni 100ms = 10Hz)
  Serial.print("Creazione timer IMU (100ms)... ");
  RCCHECK(rclc_timer_init_default(
    &timer_imu,
    &support,
    RCL_MS_TO_NS(100),
    timer_imu_callback
  ));
  Serial.println("✓");

  // Create executor
  // Handles: 5 subscriptions (chatter + 4 motori) + 2 timers = 7 handles
  Serial.print("Creazione executor (7 handles)... ");
  RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
  Serial.println("✓");

  Serial.print("Aggiunta subscription '/chatter'... ");
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA));
  Serial.println("✓");

  Serial.print("Aggiunta subscription '/m1ctrl'... ");
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor1, &msg_motor1, &motor1_callback, ON_NEW_DATA));
  Serial.println("✓");

  Serial.print("Aggiunta subscription '/m2ctrl'... ");
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor2, &msg_motor2, &motor2_callback, ON_NEW_DATA));
  Serial.println("✓");

  Serial.print("Aggiunta subscription '/m3ctrl'... ");
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor3, &msg_motor3, &motor3_callback, ON_NEW_DATA));
  Serial.println("✓");

  Serial.print("Aggiunta subscription '/m4ctrl'... ");
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor4, &msg_motor4, &motor4_callback, ON_NEW_DATA));
  Serial.println("✓");

  Serial.print("Aggiunta timer pins... ");
  RCCHECK(rclc_executor_add_timer(&executor, &timer_pins));
  Serial.println("✓");

  Serial.print("Aggiunta timer IMU... ");
  RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
  Serial.println("✓");

  // Alloca memoria per i messaggi
  sub_msg.data.data = (char*)malloc(100 * sizeof(char));
  sub_msg.data.size = 0;
  sub_msg.data.capacity = 100;

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

  pub_msg_pins.data.data = (char*)malloc(50 * sizeof(char));
  pub_msg_pins.data.size = 0;
  pub_msg_pins.data.capacity = 50;

  pub_msg_imu.data.data = (char*)malloc(200 * sizeof(char));
  pub_msg_imu.data.size = 0;
  pub_msg_imu.data.capacity = 200;

  Serial.println("========================================");
  Serial.println("✅ SISTEMA PRONTO!");
  Serial.println("========================================");
  Serial.println("📡 Subscribers:");
  Serial.println("   /chatter (std_msgs/String)");
  Serial.println("   /m1ctrl  (std_msgs/String) - Motore 1");
  Serial.println("   /m2ctrl  (std_msgs/String) - Motore 2");
  Serial.println("   /m3ctrl  (std_msgs/String) - Motore 3");
  Serial.println("   /m4ctrl  (std_msgs/String) - Motore 4");
  Serial.println();
  Serial.println("📤 Publishers:");
  Serial.println("   /pins (std_msgs/String) @ 1 Hz");
  Serial.println("   /imu  (std_msgs/String) @ 10 Hz");
  Serial.println();
  Serial.println("🤖 Controllo Motori:");
  Serial.println("   Valori: -1000 (max indietro) a +1000 (max avanti)");
  Serial.println("   Valore 0 = fermo");
  Serial.println();
  Serial.println("📌 Pin Mapping:");
  Serial.println("   Motor 1: GPIO13, GPIO12 (DRV8833 #1)");
  Serial.println("   Motor 2: GPIO14, GPIO27 (DRV8833 #1)");
  Serial.println("   Motor 3: GPIO26, GPIO25 (DRV8833 #2)");
  Serial.println("   Motor 4: GPIO33, GPIO32 (DRV8833 #2)");
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
