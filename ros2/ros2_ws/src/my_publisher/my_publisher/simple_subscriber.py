#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """
    Subscriber semplice che ascolta messaggi sul topic 'chatter'
    """
    def __init__(self):
        # Inizializza il nodo con nome 'simple_subscriber'
        super().__init__('simple_subscriber')
        
        # Crea un subscriber sul topic 'chatter'
        # String = tipo di messaggio
        # listener_callback = funzione chiamata quando arriva un messaggio
        # 10 = dimensione della coda
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        
        # Previene warning su variabile non usata
        self.subscription
        
        # Log di avvio
        self.get_logger().info('✅ Subscriber avviato! In ascolto su /chatter')

    def listener_callback(self, msg):
        """
        Funzione chiamata ogni volta che arriva un messaggio sul topic
        
        Args:
            msg: Il messaggio ricevuto (tipo String)
        """
        # Log del messaggio ricevuto
        self.get_logger().info(f'📥 Ricevuto: "{msg.data}"')


def main(args=None):
    """
    Funzione principale
    """
    # Inizializza ROS2
    rclpy.init(args=args)
    
    # Crea il nodo subscriber
    node = SimpleSubscriber()
    
    try:
        # Mantiene il nodo attivo
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Gestisce Ctrl+C
        node.get_logger().info('🛑 Subscriber fermato dall\'utente')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()