
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import base64

home = '/home/ubuntu/ws/audio'

class AudioReceiverNode(Node):

    def __init__(self):
        super().__init__('audioin')
        self.subscription = self.create_subscription(
            String,
            'microphone_input',
            self.callback,
            10)
        self.subscription

    def callback(self, msg):
        encoded64_bytes = msg.data.encode('ascii')
        data_bytes = base64.b64decode(encoded64_bytes)

        print('writing audio data to: sound.wav')
        with open(home+"sound.wav", 'wb') as file:
            file.write(data_bytes)
        print('done writing')


def main(args=None):

    rclpy.init(args=args)
    ar = AudioReceiverNode()
    print('ar spinning')
    rclpy.spin(ar)

    ar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()