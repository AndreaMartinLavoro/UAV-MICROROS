#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    """
    Publisher semplice che pubblica messaggi sul topic 'chatter'
    """
    def __init__(self):
        # Inizializza il nodo con nome 'simple_publisher'
        super().__init__('simple_publisher')
        
        # Crea un publisher sul topic 'chatter'
        # String = tipo di messaggio
        # 10 = dimensione della coda
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        
        # Crea un timer che chiama timer_callback ogni 0.5 secondi
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # Contatore per i messaggi
        self.count = 0
        
        # Log di avvio
        self.get_logger().info('✅ Publisher avviato! Publishing su /chatter')

    def timer_callback(self):
        """
        Funzione chiamata ogni 0.5 secondi dal timer
        """
        # Crea un nuovo messaggio
        msg = String()
        msg.data = f'Hello ROS2! Messaggio numero: {self.count}'
        
        # Pubblica il messaggio
        self.publisher_.publish(msg)
        
        # Log del messaggio pubblicato
        self.get_logger().info(f'📤 Publishing: "{msg.data}"')
        
        # Incrementa il contatore
        self.count += 1


def main(args=None):
    """
    Funzione principale
    """
    # Inizializza ROS2
    rclpy.init(args=args)
    
    # Crea il nodo publisher
    node = SimplePublisher()
    
    try:
        # Mantiene il nodo attivo
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Gestisce Ctrl+C
        node.get_logger().info('🛑 Publisher fermato dall\'utente')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()