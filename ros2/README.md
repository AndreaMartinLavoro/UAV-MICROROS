# DRONE-ROS - Sistema Distribuito micro-ROS + ROS2

Sistema Docker multi-container per comunicazione tra micro-ROS Agent e nodi ROS2, progettato per architettura ARM64 (Apple Silicon).

## Architettura del Sistema

Il progetto è composto da due container Docker che comunicano tra loro:

```
┌─────────────────────┐         ┌─────────────────────┐
│  microros-agent     │         │  ros2-publisher     │
│                     │         │                     │
│  micro-ROS Agent    │◄───────►│  ROS2 Humble        │
│  UDP Port 8888      │   DDS   │  my_publisher pkg   │
│                     │         │                     │
└──────────┬──────────┘         └─────────────────────┘
           │                              │
           │ UDP 8888                     │
           ▼                              │
    ┌──────────────┐                     │
    │ Microcontroller                    │
    │ (ESP32/STM32)│                     │
    └──────────────┘                     │
                                          │
                        ROS_DOMAIN_ID=0 ──┘
```

### Container 1: micro-ROS Agent
- **Immagine**: `microros/micro-ros-agent:humble`
- **Funzione**: Bridge tra microcontrollori (micro-ROS) e rete ROS2
- **Porta**: UDP 8888
- **Comunicazione**: DDS con nodi ROS2

### Container 2: ROS2 Publisher
- **Immagine**: Custom (build da Dockerfile)
- **Funzione**: Esegue nodi ROS2 standard
- **Package**: `my_publisher`
- **Comunicazione**: DDS con Agent e altri nodi ROS2

## Prerequisiti

- Docker Desktop per Mac (con supporto ARM64)
- Docker Compose
- Almeno 4GB di spazio disco libero

## Struttura del Progetto

```
DRONE-ROS/
├── Dockerfile                 # Immagine container ROS2
├── docker-compose.yml         # Orchestrazione multi-container
├── README.md                  # Questa documentazione
└── ros2_ws/                   # Workspace ROS2
    └── src/
        └── my_publisher/      # Package ROS2 personalizzato
```

## Setup e Build

### 1. Build dei Container

```bash
# Build solo del container ROS2 (l'Agent usa immagine pre-built)
docker-compose build ros2-publisher

# Build forzando la ricompilazione (senza cache)
docker-compose build --no-cache ros2-publisher
```

### 2. Avvio dei Container

```bash
# Avvia entrambi i container in background
docker-compose up -d

# Avvia con log visibili (utile per debug)
docker-compose up

# Avvia solo l'Agent
docker-compose up -d microros-agent

# Avvia solo il container ROS2
docker-compose up -d ros2-publisher
```

### 3. Verifica dello Stato

```bash
# Lista container attivi
docker-compose ps

# Visualizza i log
docker-compose logs

# Segui i log in tempo reale
docker-compose logs -f

# Log di un singolo container
docker-compose logs microros-agent
docker-compose logs ros2-publisher
```

## Gestione dei Container

### Accesso ai Container

```bash
# Accedi al container ROS2
docker exec -it ros2_publisher bash

# Accedi al container Agent
docker exec -it microros_agent bash
```

### Stop e Riavvio

```bash
# Ferma tutti i container
docker-compose down

# Ferma e rimuovi volumi
docker-compose down -v

# Riavvia un singolo container
docker-compose restart ros2-publisher

# Riavvia tutti i container
docker-compose restart
```

## Comandi ROS2 Utili

### Gestione Nodi

```bash
# Lista tutti i nodi attivi
docker exec -it ros2_publisher ros2 node list

# Info su un nodo specifico
docker exec -it ros2_publisher ros2 node info /nome_nodo

# Esegui il nodo publisher
docker exec -it ros2_publisher ros2 run my_publisher publisher_node
```

### Gestione Topic

```bash
# Lista tutti i topic
docker exec -it ros2_publisher ros2 topic list

# Lista topic con tipi di messaggio
docker exec -it ros2_publisher ros2 topic list -t

# Info su un topic specifico
docker exec -it ros2_publisher ros2 topic info /topic_name

# Visualizza messaggi in tempo reale
docker exec -it ros2_publisher ros2 topic echo /topic_name

# Verifica frequenza di pubblicazione
docker exec -it ros2_publisher ros2 topic hz /topic_name

# Pubblica manualmente su un topic
docker exec -it ros2_publisher ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello'"
```

### Debug e Diagnostica

```bash
# Visualizza il grafo dei nodi
docker exec -it ros2_publisher ros2 node list

# Verifica il tipo di un messaggio
docker exec -it ros2_publisher ros2 topic type /topic_name

# Mostra la struttura di un messaggio
docker exec -it ros2_publisher ros2 interface show std_msgs/msg/String

# Verifica connettività tra container
docker exec -it ros2_publisher ping microros-agent

# Verifica porte aperte
docker exec -it microros_agent netstat -uln | grep 8888
```

## Sviluppo del Package my_publisher

### Modifica del Codice

```bash
# 1. Modifica i file nel workspace locale
# Edita: ros2_ws/src/my_publisher/my_publisher/*.py

# 2. Rebuild del container per applicare le modifiche
docker-compose build ros2-publisher

# 3. Riavvia il container
docker-compose up -d ros2-publisher
```

### Ricompilazione del Package

Se vuoi ricompilare senza rifare il build del container:

```bash
# Accedi al container
docker exec -it ros2_publisher bash

# Ricompila il workspace
cd /root/ros2_ws
colcon build --symlink-install

# Source del nuovo build
source install/setup.bash

# Esegui il nodo aggiornato
ros2 run my_publisher publisher_node
```

## Configurazione Avanzata

### Variabili d'Ambiente

Entrambi i container usano:
- `ROS_DOMAIN_ID=0` - Dominio ROS2 (deve essere uguale per comunicare)
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` - Middleware DDS

Per modificare il domain ID, edita [docker-compose.yml](docker-compose.yml):

```yaml
environment:
  - ROS_DOMAIN_ID=42  # Cambia valore
```

### Avvio Automatico del Publisher

Per avviare automaticamente il nodo publisher, decomenta nel [docker-compose.yml](docker-compose.yml):

```yaml
ros2-publisher:
  # ...
  command: ros2 run my_publisher publisher_node
```

## Test del Sistema Completo

### 1. Test Base

```bash
# Avvia i container
docker-compose up -d

# Verifica che i container siano attivi
docker-compose ps

# Lista i nodi (dovrebbe mostrare almeno l'Agent)
docker exec -it ros2_publisher ros2 node list
```

### 2. Test Pubblicazione

```bash
# Terminale 1: Avvia il publisher
docker exec -it ros2_publisher ros2 run my_publisher publisher_node

# Terminale 2: Monitora i topic
docker exec -it ros2_publisher ros2 topic echo /topic_name
```

### 3. Test con Microcontrollore

```bash
# 1. Assicurati che l'Agent sia in ascolto
docker-compose logs microros-agent

# 2. Configura il microcontrollore per connettersi a:
#    - IP: indirizzo del tuo Mac
#    - Porta: 8888
#    - Protocollo: UDP

# 3. Una volta connesso, verifica i nodi
docker exec -it ros2_publisher ros2 node list

# Dovresti vedere il nodo del microcontrollore
```

## Troubleshooting

### I container non comunicano

```bash
# Verifica che siano sulla stessa rete
docker network inspect drone-ros_ros_network

# Verifica ROS_DOMAIN_ID
docker exec -it ros2_publisher printenv | grep ROS_DOMAIN_ID
docker exec -it microros_agent printenv | grep ROS_DOMAIN_ID
```

### Non vedo i topic

```bash
# Verifica che l'Agent sia attivo
docker-compose logs microros-agent

# Verifica il DDS discovery
docker exec -it ros2_publisher ros2 daemon stop
docker exec -it ros2_publisher ros2 daemon start
docker exec -it ros2_publisher ros2 topic list
```

### Errori di build

```bash
# Pulizia completa e rebuild
docker-compose down
docker system prune -f
docker-compose build --no-cache
docker-compose up -d
```

### Il microcontrollore non si connette

```bash
# Verifica che la porta 8888 sia esposta
docker port microros_agent

# Verifica i log dell'Agent
docker-compose logs -f microros-agent

# Testa la connessione UDP
# Dal microcontrollore, prova a pingare l'IP del Mac
```

## Note Importanti

1. **Porta UDP 8888**: Usata per comunicazione Agent ↔ Microcontrollori, non ROS2 ↔ Agent
2. **DDS Discovery**: ROS2 scopre automaticamente l'Agent tramite multicast
3. **ROS_DOMAIN_ID**: Deve essere identico su tutti i nodi che devono comunicare
4. **ARM64**: I container sono configurati per Apple Silicon, modifica `platform` per altre architetture

## Comandi Rapidi di Riferimento

```bash
# Setup iniziale
docker-compose build
docker-compose up -d

# Sviluppo quotidiano
docker exec -it ros2_publisher bash
ros2 run my_publisher publisher_node

# Monitoring
docker-compose logs -f
docker exec -it ros2_publisher ros2 topic list
docker exec -it ros2_publisher ros2 topic echo /topic_name

# Pulizia
docker-compose down
docker system prune -f
```

## Risorse

- [micro-ROS Documentation](https://micro.ros.org/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Docker Compose Reference](https://docs.docker.com/compose/)

## Licenza

TODO: Aggiungi licenza

## Autori

Andrea Martin
