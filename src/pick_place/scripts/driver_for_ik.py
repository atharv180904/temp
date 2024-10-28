#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
import serial
import time
from collections import deque

class ServoControl:
    def __init__(self, port, baudrate=1000000):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)

    def __del__(self):
        self.ser.close()

    def calculate_checksum(self, data):
        return (~sum(data)) & 0xFF

    def send_command(self, command):
        self.ser.write(command)
        time.sleep(0.05)

    def move_to_position(self, id, position, speed=350):
        command = bytes([0xFF, 0xFF, id, 0x09, 0x03, 0x2A])
        command += position.to_bytes(2, byteorder='little')
        command += bytes([0x00, 0x00])  # Add necessary bytes
        command += speed.to_bytes(2, byteorder='little')
        checksum = self.calculate_checksum(command[2:])
        command += bytes([checksum])
        self.send_command(command)
        print(f"Command sent to move servo {id} to position {position} at speed {speed}")

class JointStateListener(Node):
    def __init__(self, port):
        super().__init__('driver')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.subscription
        self.coord_source_subscription = self.create_subscription(Float32MultiArray, '/coord/source', self.coord_source_callback, 100)
        self.coord_destination_subscription = self.create_subscription(Float32MultiArray, '/coord/destination', self.coord_destination_callback, 100)
        self.ack_subscription = self.create_subscription(String, '/servo_movement_acknowledgement', self.ack_callback, 10)
        self.check_action_status_subscription = self.create_subscription(String, '/check_1_action_status', self.check_action_status_callback, 10)
        self.action_status_publisher = self.create_publisher(String, '/action_status', 10)
        self.last_time = self.get_clock().now().nanoseconds
        self.updated_angles = [0.0] * 4  
        self.updated_positions = [0] * 4  
        self.servo_control = ServoControl(port) 
        self.queue = deque(maxlen=5)  
        self.timer = self.create_timer(1.0, self.print_queue)
        self.servo_control.move_to_position(4, 1050) 
        self.get_logger().info('Gripper initialized to default position: 1050')

    def joint_state_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds
        if (current_time - self.last_time) / 1e9 > 0.0001:  
            self.last_time = current_time
            if len(msg.position) > 0:
                self.updated_angles[0] = (-msg.position[0]) + 4.71238898 
            if len(msg.position) > 1:
                self.updated_angles[1] = -(msg.position[1] + 3.141592654)  
            if len(msg.position) > 2:
                self.updated_angles[2] = msg.position[2] + 3.141592654  
            for i in range(3):  
                angle_in_degrees = self.updated_angles[i] * (180.0 / 3.141592654)  
                normalized_angle = angle_in_degrees % 360  
                self.updated_positions[i] = angle_to_position(normalized_angle)  
            servo_ids = [1, 2, 3]
            for i in range(3):
                self.servo_control.move_to_position(servo_ids[i], self.updated_positions[i])
            joint_positions_msg = ", ".join(
                f"{joint}: {self.updated_positions[i]}" 
                for i, joint in enumerate(msg.name[:3])
            )
            joint_positions_msg += f", {msg.name[3]}: {self.updated_positions[3]}" if len(msg.name) > 3 else ""
            self.get_logger().info(f'Joint positions: {joint_positions_msg}')
            
    def coord_source_callback(self, msg):
        self.queue.append(1) 
        self.get_logger().info("Received data from /coord/source")
 
    def coord_destination_callback(self, msg):
        self.queue.append(0) 
        self.get_logger().info("Received data from /coord/destination")
        
    def ack_callback(self, msg):
        if msg.data == "done":
            self.get_logger().info("Received acknowledgment: done")
            if self.queue:
                value = self.queue[0] 
                if value == 1:
                    self.servo_control.move_to_position(4, 573)
                elif value == 0:
                    self.servo_control.move_to_position(4, 1050)

    def check_action_status_callback(self, msg):
        if msg.data == "check0":
            if self.queue:
                value = self.queue[0]
                if value == 0:
                    self.get_logger().info("Destination Coordinates, hence publishing 'done' to /action_status")
                    self.publish_action_status_done()  
                else:
                    self.get_logger().info("Queue front is 1, no action taken")
                self.queue.popleft()  
    
    def publish_action_status_done(self):
        status_msg = String()
        status_msg.data = "done"
        self.action_status_publisher.publish(status_msg)
    
        
    def print_queue(self):
        queue_contents = list(self.queue)
        self.get_logger().info(f'Queue contents: {queue_contents}')

def angle_to_position(angle):
    return int((angle % 360) * 4096 / 360)

def main(args=None):
    rclpy.init(args=args)
    available_port = '/dev/ttyACM0'
    joint_state_listener = JointStateListener(available_port)
    rclpy.spin(joint_state_listener)
    joint_state_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
