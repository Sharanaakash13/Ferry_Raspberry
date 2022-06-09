import serial 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String


class Subscriber(Node):

# Defining the serial port connected to relay board 
Serial_port = '/dev/ttyS0'

# Defining the data to the relay board 
Dock_light = 00000001  # First relay output for docking light
Nav_light = 00000010    # Second relay output for navigation light

# Address of the relay board
address_relay = 1

# Command for the relay board 
cmd_initial = 1  # initializing the relay board 
cmd_toggle =  8  # toggle the light 

    def __init__(self):

        # Node for the Ferry light
        super().__init__('light_subscriber')

        # Checking the serial port 
        if not os.path.exists(self.Serial_port):
            self.get_logger().error("Serial port does not exists :" + self.Serial_port + "command not sent to the relay board")
            rclpy.shutdown()

        # Start the serial communication
        self.ser = serial.Serial(
            port=self.Serial_port, 
            baudrate=19200, 
            parity= serial.PARITY_NONE, 
            stopbits = serial.STOPBITS_ONE, 
            bytesize= serial.EIGHTBITS,
            timeout=1)

        # Creating Boolean subscription 
        self.subscription = self.create_subscription(Bool,'/topic', self.send_serial_cmd, 10)
        self.get_logger().info("Serial communication started")

        # Creating String publisher (Publishing the status)
        self.publisher = self.create_publisher(String,'/topic',10)

    def send_serial_cmd(self,msg):
        
        # Relay board toogles to Docking light if the message is true.
        if msg.data = True:
            self.get_logger().info("Turning on docking light")
            # serial code
            
        else :
            # serial code 
    
    def send_serial(self, send):
        self.ser.write()





    def main():
        rclpy.init(args=args)
        topic_subscriber = Subscriber()
        rclpy.spin(topic_subscriber)
        rclpy.shutdown()

        print('Hi from ba_server_light.')


if __name__ == '__main__':
    main()
