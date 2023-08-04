
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from ml_inference.inference import Inference_Model

import os
from dotenv import load_dotenv
load_dotenv('/home/ubuntu/ws/src/.env')

import base64

name = 'app_audio'

class AudioReceiverNode(Node):

    def __init__(self):
        super().__init__('audioin')
        self.subscription = self.create_subscription(
            String,
            'microphone_input',
            self.callback,
            10)
        self.subscription

        print('loading inference model')
        self.model = Inference_Model()
        self.model.load_model()

    def callback(self, msg):
        encoded64_bytes = msg.data.encode('ascii')
        data_bytes = base64.b64decode(encoded64_bytes)

        print(f'writing audio data to: {name}.wav')
        with open(f"{ os.environ.get('DATA_PATH') }audio/{name}.wav", 'wb') as file:
            file.write(data_bytes)
        print(f'wrote audio data to: {name}.wav')
        
        command_text = self.model.infer_text(f"{ os.environ.get('DATA_PATH') }audio/{name}.wav")
        


def main(args=None):

    print('creating audio receiver')
    rclpy.init(args=args)
    ar = AudioReceiverNode()
    print('ar spinning')
    rclpy.spin(ar)

    ar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
