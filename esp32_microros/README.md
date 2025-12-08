# ESP32 micro-ROS Robot Controller

Sistema di controllo robotico basato su ESP32 con integrazione micro-ROS per comunicazione con ROS2.

## 📋 Indice

- [Panoramica](#panoramica)
- [Caratteristiche](#caratteristiche)
- [Hardware Richiesto](#hardware-richiesto)
- [Schema Collegamenti](#schema-collegamenti)
- [Installazione](#installazione)
- [Configurazione](#configurazione)
- [Topic ROS2](#topic-ros2)
- [Utilizzo](#utilizzo)
- [Esempi](#esempi)
- [Troubleshooting](#troubleshooting)

## 🎯 Panoramica

Questo progetto implementa un controller robotico su ESP32 che comunica con ROS2 attraverso micro-ROS. Il sistema è capace di:

- Controllare 4 motori DC tramite driver DRV8833
- Leggere dati da sensore IMU (BMI160)
- Monitorare stato pin digitali
- Comunicare via WiFi con un micro-ROS agent

## ✨ Caratteristiche

### Subscribers (riceve comandi da ROS2)
- `/chatter` - Messaggi generici (String)
- `/m1ctrl` - Controllo Motore 1 (-1000 a +1000)
- `/m2ctrl` - Controllo Motore 2 (-1000 a +1000)
- `/m3ctrl` - Controllo Motore 3 (-1000 a +1000)
- `/m4ctrl` - Controllo Motore 4 (-1000 a +1000)

### Publishers (invia dati a ROS2)
- `/pins` - Stato GPIO digitali (1 Hz)
- `/imu` - Dati accelerometro + giroscopio BMI160 (10 Hz)

## 🔧 Hardware Richiesto

### Componenti Principali
- **ESP32 DevKit** (qualsiasi variante)
- **2x DRV8833** - Driver motori DC dual H-bridge
- **4x Motori DC** - Con riduttore (3-10V)
- **BMI160** - Sensore IMU 6-axis (opzionale)
- **Alimentazione** - 3-10V per motori

### Specifiche DRV8833
- Voltaggio in ingresso: 3-10V
- Corrente massima per canale: 1.5A
- 2 canali H-bridge per driver (controlla 2 motori)

### Specifiche BMI160
- Accelerometro a 3 assi
- Giroscopio a 3 assi
- Interfaccia I2C

## 📐 Schema Collegamenti

### ESP32 ↔ DRV8833 #1 (Motori 1 e 2)

```
ESP32          DRV8833 #1
─────────────────────────
GPIO 13   →    AIN1 (Motor 1)
GPIO 12   →    AIN2 (Motor 1)
GPIO 14   →    BIN1 (Motor 2)
GPIO 27   →    BIN2 (Motor 2)
3-10V     →    VCC
GND       →    GND
```

**Collegamenti Motori DRV8833 #1:**
```
Motor 1+  →    AOUT1
Motor 1-  →    AOUT2
Motor 2+  →    BOUT1
Motor 2-  →    BOUT2
```

### ESP32 ↔ DRV8833 #2 (Motori 3 e 4)

```
ESP32          DRV8833 #2
─────────────────────────
GPIO 26   →    AIN1 (Motor 3)
GPIO 25   →    AIN2 (Motor 3)
GPIO 33   →    BIN1 (Motor 4)
GPIO 32   →    BIN2 (Motor 4)
3-10V     →    VCC
GND       →    GND
```

**Collegamenti Motori DRV8833 #2:**
```
Motor 3+  →    AOUT1
Motor 3-  →    AOUT2
Motor 4+  →    BOUT1
Motor 4-  →    BOUT2
```

### ESP32 ↔ BMI160 (I2C)

```
ESP32          BMI160
────────────────────
GPIO 21   →    SDA
GPIO 22   →    SCL
3.3V      →    VCC
GND       →    GND
```

### Pin Monitoraggio

```
GPIO 25   →    Input pullup (PIN_1)
GPIO 26   →    Input pullup (PIN_2)
GPIO 2    →    LED integrato
```

**⚠️ NOTA:** GPIO25 e GPIO26 sono usati sia per DRV8833 #2 che per monitoraggio. Se usi il DRV8833 #2, disabilita il monitoraggio pin o cambia i pin.

## 📦 Installazione

### 1. Prerequisiti

- [PlatformIO](https://platformio.org/) installato
- [Docker](https://www.docker.com/) (per micro-ROS agent)
- ROS2 Humble (opzionale, per testing)

### 2. Clone e Build

```bash
cd /path/to/project
pio lib install
pio run
```

### 3. Upload su ESP32

```bash
pio run --target upload
```

### 4. Monitor Seriale

```bash
pio device monitor
```

## ⚙️ Configurazione

### WiFi e Agent

Modifica il file `src/main.cpp`:

```cpp
const char* ssid = "TUO_SSID";           // Nome WiFi
const char* password = "TUA_PASSWORD";    // Password WiFi
const char* agent_ip = "192.168.1.31";   // IP del micro-ROS agent
const int agent_port = 8888;              // Porta UDP
```

### PWM Motori

```cpp
#define PWM_FREQ 20000        // Frequenza PWM (20 kHz)
#define PWM_RESOLUTION 10     // Risoluzione (10 bit = 0-1023)
```

## 🐳 Avvio micro-ROS Agent

### Con Docker (consigliato per macOS)

```bash
docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:humble udp4 --port 8888
```

### Con ROS2 (Linux)

```bash
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## 📡 Topic ROS2

### Formato Messaggi

Tutti i topic usano `std_msgs/String`.

#### Subscribers (Comandi Motori)

**Formato:** Valore numerico da -1000 a +1000
- `-1000` = Massima velocità indietro
- `0` = Fermo
- `+1000` = Massima velocità avanti

Esempio:
```bash
ros2 topic pub -1 /m1ctrl std_msgs/String "data: '500'"
```

#### Publishers

**`/pins`** - Formato: `"PIN1:x,PIN2:y"`
```
PIN1:1,PIN2:0
```

**`/imu`** - Formato JSON:
```json
{
  "accel": {"x": 0.123, "y": -0.456, "z": 9.810},
  "gyro": {"x": 0.001, "y": -0.002, "z": 0.000}
}
```

Unità di misura:
- Accelerazione: m/s²
- Giroscopio: rad/s

## 🚀 Utilizzo

### 1. Avvia il sistema

1. Carica il firmware su ESP32
2. Avvia micro-ROS agent
3. Resetta ESP32
4. Verifica connessione sul monitor seriale

### 2. Verifica Topic Attivi

```bash
ros2 topic list
```

Output atteso:
```
/chatter
/m1ctrl
/m2ctrl
/m3ctrl
/m4ctrl
/pins
/imu
```

### 3. Controlla un Motore

```bash
# Motore 1 avanti a velocità media
ros2 topic pub -1 /m1ctrl std_msgs/String "data: '500'"

# Motore 2 indietro veloce
ros2 topic pub -1 /m2ctrl std_msgs/String "data: '-1000'"

# Ferma motore 1
ros2 topic pub -1 /m1ctrl std_msgs/String "data: '0'"
```

### 4. Leggi Dati IMU

```bash
ros2 topic echo /imu
```

### 5. Monitora Pin Digitali

```bash
ros2 topic echo /pins
```

## 💡 Esempi

### Controllo Robot 4 Ruote

```bash
# Avanti
ros2 topic pub -1 /m1ctrl std_msgs/String "data: '800'"
ros2 topic pub -1 /m2ctrl std_msgs/String "data: '800'"
ros2 topic pub -1 /m3ctrl std_msgs/String "data: '800'"
ros2 topic pub -1 /m4ctrl std_msgs/String "data: '800'"

# Ruota destra (motori sx avanti, dx indietro)
ros2 topic pub -1 /m1ctrl std_msgs/String "data: '500'"
ros2 topic pub -1 /m2ctrl std_msgs/String "data: '-500'"
ros2 topic pub -1 /m3ctrl std_msgs/String "data: '500'"
ros2 topic pub -1 /m4ctrl std_msgs/String "data: '-500'"

# Ferma tutto
ros2 topic pub -1 /m1ctrl std_msgs/String "data: '0'"
ros2 topic pub -1 /m2ctrl std_msgs/String "data: '0'"
ros2 topic pub -1 /m3ctrl std_msgs/String "data: '0'"
ros2 topic pub -1 /m4ctrl std_msgs/String "data: '0'"
```

### Script Python per Controllo

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.pub_m1 = self.create_publisher(String, 'm1ctrl', 10)
        self.pub_m2 = self.create_publisher(String, 'm2ctrl', 10)
        self.pub_m3 = self.create_publisher(String, 'm3ctrl', 10)
        self.pub_m4 = self.create_publisher(String, 'm4ctrl', 10)

    def set_motor(self, motor_num, speed):
        """Imposta velocità motore (-1000 a +1000)"""
        msg = String()
        msg.data = str(speed)

        if motor_num == 1:
            self.pub_m1.publish(msg)
        elif motor_num == 2:
            self.pub_m2.publish(msg)
        elif motor_num == 3:
            self.pub_m3.publish(msg)
        elif motor_num == 4:
            self.pub_m4.publish(msg)

    def forward(self, speed=800):
        """Muovi in avanti"""
        for i in range(1, 5):
            self.set_motor(i, speed)

    def stop(self):
        """Ferma tutti i motori"""
        for i in range(1, 5):
            self.set_motor(i, 0)

def main():
    rclpy.init()
    controller = RobotController()

    # Esempio utilizzo
    controller.forward(500)
    rclpy.spin_once(controller, timeout_sec=2.0)
    controller.stop()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 🔍 Troubleshooting

### ESP32 non si connette all'agent

**Sintomo:** Errore `FAIL (1)` ripetuto

**Soluzioni:**
1. Verifica che l'agent sia in esecuzione:
   ```bash
   docker ps | grep micro-ros-agent
   ```

2. Verifica mapping porta UDP su macOS:
   ```bash
   docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:humble udp4 --port 8888
   ```
   (NON usare `--net=host` su macOS!)

3. Controlla IP agent nel codice corrisponda all'IP del Mac:
   ```bash
   ifconfig | grep "inet "
   ```

4. Verifica firewall non blocchi porta 8888 UDP

### Motori non si muovono

**Soluzioni:**
1. Verifica alimentazione DRV8833 (3-10V)
2. Controlla collegamenti PWM ESP32 → DRV8833
3. Testa con monitor seriale se riceve comandi:
   ```
   🔧 M1: 500
   ```

### BMI160 non rilevato

**Sintomo:** `❌ Errore reset BMI160!`

**Soluzioni:**
1. Verifica collegamenti I2C (SDA=21, SCL=22)
2. Verifica alimentazione BMI160 (3.3V)
3. Usa scanner I2C per rilevare indirizzo:
   ```cpp
   Wire.begin(21, 22);
   // Scansione indirizzi I2C
   ```

### Performance/Latenza

**Sintomo:** Ritardo nei comandi motori

**Soluzioni:**
1. Riduci frequenza pubblicazione IMU (da 10Hz a 5Hz)
2. Aumenta `PWM_FREQ` se necessario
3. Verifica qualità segnale WiFi

## 📊 Specifiche Tecniche

### Sistema
- **Microcontrollore:** ESP32 (dual-core, 240 MHz)
- **Framework:** Arduino
- **Middleware:** micro-ROS
- **Comunicazione:** WiFi UDP

### Motori
- **Canali PWM:** 8 (2 per motore)
- **Frequenza PWM:** 20 kHz
- **Risoluzione:** 10 bit (0-1023)
- **Controllo:** Bidirezionale con velocità variabile

### Sensori
- **IMU:** BMI160 (I2C)
- **Frequenza lettura:** 10 Hz
- **Range accelerometro:** Configurabile
- **Range giroscopio:** Configurabile

## 📝 Note

- Il sistema continua a funzionare anche se BMI160 non è connesso
- Tutti i motori vengono fermati (velocità 0) all'avvio
- LED integrato (GPIO2) lampeggia alla ricezione di messaggi su `/chatter`
- Monitor seriale mostra debug dettagliato di tutte le operazioni

## 🤝 Contributi

Per bug o miglioramenti, aprire una issue o pull request.

## 📄 Licenza

Progetto educativo - Usa liberamente.

## 🔗 Link Utili

- [micro-ROS Documentation](https://micro.ros.org/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)
- [DRV8833 Datasheet](https://www.ti.com/lit/ds/symlink/drv8833.pdf)
- [BMI160 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi160/)

---

**Versione:** 1.0
**Data:** Dicembre 2025
**Autore:** Sistema ESP32 micro-ROS
