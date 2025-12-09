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

- Controllare 4 motori DC tramite driver DRV8833 (configurazione testata)
- Leggere dati da sensore IMU MPU6050 (accelerometro, giroscopio, temperatura)
- Calcolare angoli di inclinazione (pitch e roll)
- Comunicare via WiFi con un micro-ROS agent

## ✨ Caratteristiche

### Subscribers (riceve comandi da ROS2)
- `/m1ctrl` - Controllo Motor A (-1000 a +1000)
- `/m2ctrl` - Controllo Motor B (-1000 a +1000)
- `/m3ctrl` - Controllo Motor C (-1000 a +1000)
- `/m4ctrl` - Controllo Motor D (-1000 a +1000)

### Publishers (invia dati a ROS2)
- `/imu` - Dati completi MPU6050 in formato JSON (10 Hz):
  - Accelerometro (x, y, z in g)
  - Giroscopio (x, y, z in °/s)
  - Temperatura (°C)
  - Pitch e Roll calcolati (°)

## 🔧 Hardware Richiesto

### Componenti Principali
- **ESP32 DevKit** (qualsiasi variante)
- **2x DRV8833** - Driver motori DC dual H-bridge
- **4x Motori DC** - Con riduttore (3-10V)
- **MPU6050** - Sensore IMU 6-axis (opzionale)
- **Alimentazione** - 3-10V per motori + batteria separata consigliata

### Specifiche DRV8833
- Voltaggio in ingresso: 3-10V
- Corrente massima per canale: 1.5A
- 2 canali H-bridge per driver (controlla 2 motori)
- Frequenza PWM: 1 kHz
- Risoluzione PWM: 8 bit (0-255)

### Specifiche MPU6050
- Accelerometro: ±2g, ±4g, ±8g, ±16g (configurato a ±8g)
- Giroscopio: ±250°/s, ±500°/s, ±1000°/s, ±2000°/s (configurato a ±500°/s)
- Filtro passa-basso digitale: 21 Hz
- Interfaccia I2C (indirizzo 0x68 o 0x69)
- **IMPORTANTE:** Solo 3.3V! NON collegare a 5V

## 📐 Schema Collegamenti

### ESP32 ↔ DRV8833 #1 (Motor A e Motor B)

```
ESP32          DRV8833 #1
─────────────────────────
GPIO 25   →    AIN1 (Motor A)
GPIO 26   →    AIN2 (Motor A)
GPIO 27   →    BIN1 (Motor B)
GPIO 14   →    BIN2 (Motor B)
VM        →    3-10V Batteria
GND       →    GND (comune con ESP32!)
```

**Collegamenti Motori DRV8833 #1:**
```
Motor A+  →    AOUT1
Motor A-  →    AOUT2
Motor B+  →    BOUT1
Motor B-  →    BOUT2
```

### ESP32 ↔ DRV8833 #2 (Motor C e Motor D)

```
ESP32          DRV8833 #2
─────────────────────────
GPIO 32   →    AIN1 (Motor C)
GPIO 33   →    AIN2 (Motor C)
GPIO 12   →    BIN1 (Motor D)
GPIO 13   →    BIN2 (Motor D)
VM        →    3-10V Batteria
GND       →    GND (comune con ESP32!)
```

**Collegamenti Motori DRV8833 #2:**
```
Motor C+  →    AOUT1
Motor C-  →    AOUT2
Motor D+  →    BOUT1
Motor D-  →    BOUT2
```

### ESP32 ↔ MPU6050 (I2C)

```
ESP32          MPU6050
────────────────────
GPIO 21   →    SDA
GPIO 22   →    SCL
3.3V      →    VCC (⚠️ SOLO 3.3V!)
GND       →    GND
```

### Altri Pin

```
GPIO 2    →    LED integrato (diagnostica)
```

**⚠️ IMPORTANTE:**
- Alimenta i DRV8833 con batteria separata (VM) per evitare interferenze
- **GND COMUNE** tra ESP32, MPU6050 e DRV8833 è OBBLIGATORIO
- MPU6050 funziona SOLO a 3.3V (NON collegare a 5V!)
- Se inverti i poli del motore, inverti i fili AOUT1/AOUT2

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
#define PWM_FREQ 1000         // Frequenza PWM (1 kHz) - TESTATA
#define PWM_RESOLUTION 8      // Risoluzione (8 bit = 0-255) - TESTATA
```

### Configurazione MPU6050

```cpp
mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);      // ±8g
mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);           // ±500°/s
mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);        // Filtro 21Hz
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
- `-1000` = Massima velocità indietro (PWM 255)
- `0` = Fermo
- `+1000` = Massima velocità avanti (PWM 255)

Mapping: Valore ricevuto viene scalato da [-1000, +1000] a [-255, +255]

Esempio:
```bash
ros2 topic pub /m1ctrl std_msgs/String "data: '500'" --once
```

#### Publishers

**`/imu`** - Formato JSON completo (pubblicato ogni 100ms @ 10Hz):
```json
{
  "accel": {"x": 0.123, "y": -0.456, "z": 1.002},
  "gyro": {"x": 1.23, "y": -4.56, "z": 0.12},
  "temp": 25.3,
  "pitch": 2.5,
  "roll": -1.8
}
```

Unità di misura:
- **accel**: Accelerazione in **g** (non m/s²)
- **gyro**: Velocità angolare in **°/s** (non rad/s)
- **temp**: Temperatura in **°C**
- **pitch/roll**: Angoli di inclinazione in **°**

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
/m1ctrl
/m2ctrl
/m3ctrl
/m4ctrl
/imu
/parameter_events
/rosout
```

### 3. Controlla un Motore

```bash
# Motor A avanti a velocità media
ros2 topic pub /m1ctrl std_msgs/String "data: '500'" --once

# Motor B indietro veloce
ros2 topic pub /m2ctrl std_msgs/String "data: '-1000'" --once

# Ferma Motor A
ros2 topic pub /m1ctrl std_msgs/String "data: '0'" --once
```

### 4. Leggi Dati IMU

```bash
ros2 topic echo /imu
```

Output esempio:
```
data: '{"accel":{"x":0.012,"y":-0.003,"z":1.001},"gyro":{"x":0.15,"y":-0.23,"z":0.08},"temp":26.2,"pitch":0.3,"roll":-0.2}'
---
```

## 💡 Esempi

### Controllo Robot 4 Ruote

```bash
# Avanti
ros2 topic pub /m1ctrl std_msgs/String "data: '800'" --once &
ros2 topic pub /m2ctrl std_msgs/String "data: '800'" --once &
ros2 topic pub /m3ctrl std_msgs/String "data: '800'" --once &
ros2 topic pub /m4ctrl std_msgs/String "data: '800'" --once &

# Ruota destra (motori sx avanti, dx indietro)
ros2 topic pub /m1ctrl std_msgs/String "data: '500'" --once &
ros2 topic pub /m2ctrl std_msgs/String "data: '-500'" --once &
ros2 topic pub /m3ctrl std_msgs/String "data: '500'" --once &
ros2 topic pub /m4ctrl std_msgs/String "data: '-500'" --once &

# Ferma tutto
ros2 topic pub /m1ctrl std_msgs/String "data: '0'" --once &
ros2 topic pub /m2ctrl std_msgs/String "data: '0'" --once &
ros2 topic pub /m3ctrl std_msgs/String "data: '0'" --once &
ros2 topic pub /m4ctrl std_msgs/String "data: '0'" --once &
```

### Test Singolo Motore (Diagnostica)

```bash
# Test Motor A
echo "Test Motor A avanti..."
ros2 topic pub /m1ctrl std_msgs/String "data: '600'" --once
sleep 2
ros2 topic pub /m1ctrl std_msgs/String "data: '0'" --once

# Test Motor A indietro
echo "Test Motor A indietro..."
ros2 topic pub /m1ctrl std_msgs/String "data: '-600'" --once
sleep 2
ros2 topic pub /m1ctrl std_msgs/String "data: '0'" --once
```

### Lettura Continua IMU

```bash
# Monitora dati IMU in tempo reale
ros2 topic echo /imu --no-arr

# Oppure con filtraggio JSON (richiede jq)
ros2 topic echo /imu --no-arr | grep "data:" | sed 's/data: //' | jq '.'
```

### Script Python per Controllo

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publishers per i motori
        self.pub_m1 = self.create_publisher(String, 'm1ctrl', 10)
        self.pub_m2 = self.create_publisher(String, 'm2ctrl', 10)
        self.pub_m3 = self.create_publisher(String, 'm3ctrl', 10)
        self.pub_m4 = self.create_publisher(String, 'm4ctrl', 10)

        # Subscriber per IMU
        self.sub_imu = self.create_subscription(
            String,
            'imu',
            self.imu_callback,
            10
        )

        self.latest_imu = None

    def imu_callback(self, msg):
        """Callback per ricevere dati IMU"""
        try:
            self.latest_imu = json.loads(msg.data)
            pitch = self.latest_imu['pitch']
            roll = self.latest_imu['roll']
            self.get_logger().info(f'IMU - Pitch: {pitch:.1f}° Roll: {roll:.1f}°')
        except json.JSONDecodeError:
            self.get_logger().error('Errore parsing JSON IMU')

    def set_motor(self, motor_num, speed):
        """Imposta velocità motore (-1000 a +1000)"""
        speed = max(-1000, min(1000, speed))  # Clamp
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

        self.get_logger().info(f'Motor {motor_num}: {speed}')

    def forward(self, speed=800):
        """Muovi in avanti"""
        for i in range(1, 5):
            self.set_motor(i, speed)

    def backward(self, speed=800):
        """Muovi indietro"""
        for i in range(1, 5):
            self.set_motor(i, -speed)

    def turn_right(self, speed=500):
        """Ruota a destra"""
        self.set_motor(1, speed)   # Motor A avanti
        self.set_motor(2, -speed)  # Motor B indietro
        self.set_motor(3, speed)   # Motor C avanti
        self.set_motor(4, -speed)  # Motor D indietro

    def turn_left(self, speed=500):
        """Ruota a sinistra"""
        self.set_motor(1, -speed)  # Motor A indietro
        self.set_motor(2, speed)   # Motor B avanti
        self.set_motor(3, -speed)  # Motor C indietro
        self.set_motor(4, speed)   # Motor D avanti

    def stop(self):
        """Ferma tutti i motori"""
        for i in range(1, 5):
            self.set_motor(i, 0)

def main():
    rclpy.init()
    controller = RobotController()

    # Esempio utilizzo: sequenza di movimenti
    try:
        controller.get_logger().info('Avanti...')
        controller.forward(600)
        time.sleep(2)

        controller.get_logger().info('Ruota destra...')
        controller.turn_right(500)
        time.sleep(1)

        controller.get_logger().info('Indietro...')
        controller.backward(600)
        time.sleep(2)

        controller.get_logger().info('Stop!')
        controller.stop()

        # Leggi IMU per 5 secondi
        controller.get_logger().info('Lettura IMU...')
        end_time = time.time() + 5
        while time.time() < end_time:
            rclpy.spin_once(controller, timeout_sec=0.1)

    except KeyboardInterrupt:
        controller.get_logger().info('Interrotto da utente')
    finally:
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
1. Verifica alimentazione DRV8833 (VM: 3-10V)
2. Controlla **GND comune** tra ESP32, DRV8833 e batteria
3. Controlla collegamenti PWM ESP32 → DRV8833 IN1/IN2
4. Testa con monitor seriale se riceve comandi:
   ```
   Motor A: 127
   ```
5. Se motore gira al contrario, inverti i fili AOUT1/AOUT2

### MPU6050 non rilevato

**Sintomo:** `MPU6050 non rilevato!` o nessun dispositivo I2C trovato

**Soluzioni:**
1. Verifica collegamenti I2C:
   - SDA → GPIO21
   - SCL → GPIO22
   - VCC → **3.3V** (NON 5V!)
   - GND → GND
2. Controlla che MPU6050 sia alimentato (LED acceso se presente)
3. Verifica indirizzo I2C (0x68 o 0x69). Lo scanner I2C integrato mostra:
   ```
   Dispositivo I2C trovato all'indirizzo 0x68
     -> MPU6050 rilevato!
   ```
4. Prova con resistenze pull-up su SDA/SCL (4.7kΩ a 3.3V) se cavi lunghi
5. Sistema funziona anche senza MPU6050 (solo controllo motori)

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
- **Middleware:** micro-ROS (ROS2 Humble)
- **Comunicazione:** WiFi UDP (porta 8888)
- **Nodo ROS:** `esp32_robot`

### Motori
- **Canali PWM:** 8 (2 per motore)
- **Frequenza PWM:** 1 kHz (testata e stabile)
- **Risoluzione:** 8 bit (0-255)
- **Controllo:** Bidirezionale con velocità variabile
- **Range comando:** -1000 a +1000 (mappato su -255 a +255)

### Sensori
- **IMU:** MPU6050 (I2C @ 0x68)
- **Frequenza pubblicazione:** 10 Hz (100ms)
- **Range accelerometro:** ±8g
- **Range giroscopio:** ±500°/s
- **Filtro digitale:** 21 Hz passa-basso
- **Dati aggiuntivi:** Temperatura, Pitch, Roll calcolati

## 📝 Note

- Il sistema continua a funzionare anche se MPU6050 non è connesso (solo controllo motori)
- Tutti i motori vengono fermati (velocità 0) all'avvio per sicurezza
- Scanner I2C integrato all'avvio per diagnostica automatica
- Reinizializzazione automatica MPU6050 ogni 10s in caso di errore di lettura
- LED integrato (GPIO2) lampeggia 3 volte all'avvio per conferma
- Monitor seriale mostra debug dettagliato di tutte le operazioni
- Configurazione hardware testata e verificata funzionante

## 🤝 Contributi

Per bug o miglioramenti, aprire una issue o pull request.

## 📄 Licenza

Progetto educativo - Usa liberamente.

## 🧪 Test e Diagnostica

### Test Completo Sistema

```bash
# 1. Verifica connessione
ros2 node list
# Deve mostrare: /esp32_robot

# 2. Verifica topic
ros2 topic list
# Deve mostrare: /m1ctrl, /m2ctrl, /m3ctrl, /m4ctrl, /imu

# 3. Test singolo motore
ros2 topic pub /m1ctrl std_msgs/String "data: '300'" --once
# Osserva sul serial monitor: "Motor A: 76"

# 4. Test IMU
ros2 topic hz /imu
# Deve mostrare: ~10 Hz

ros2 topic echo /imu --once
# Deve mostrare JSON con dati validi
```

### Diagnostica Monitor Seriale

All'avvio dovresti vedere:

```
micro-ROS ESP32 Robot Controller
MPU6050 + 4x DRV8833 Motors

Scansione bus I2C...
Dispositivo I2C trovato all'indirizzo 0x68
  -> MPU6050 rilevato!

MPU6050 inizializzato con successo!
Tutti i 4 motori pronti!

WiFi connesso!
IP ESP32: 192.168.1.XX

[1/20] SUCCESS!
Support inizializzato
...
SISTEMA PRONTO!
```

### Benchmark Prestazioni

- **Latenza comando motore:** ~10-50ms (dipende da WiFi)
- **Frequenza IMU stabile:** 10 Hz ±0.5 Hz
- **CPU usage:** ~30% (un core dedicato a micro-ROS)
- **Memoria libera:** ~200KB dopo inizializzazione

## 🔗 Link Utili

- [micro-ROS Documentation](https://micro.ros.org/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)
- [DRV8833 Datasheet](https://www.ti.com/lit/ds/symlink/drv8833.pdf)
- [MPU6050 Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- [Adafruit MPU6050 Library](https://github.com/adafruit/Adafruit_MPU6050)

---

**Versione:** 2.0
**Data:** Dicembre 2025
**Hardware:** Testato e verificato funzionante
**Autore:** Sistema ESP32 micro-ROS
