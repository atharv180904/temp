import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class CoordinatePublisher(Node):
    def __init__(self):
        super().__init__('coordinate_publisher')
        self.publisher_source = self.create_publisher(Float32MultiArray, '/coord/source', 10)
        self.publisher_destination = self.create_publisher(Float32MultiArray, '/coord/destination', 10)
        self.timer = self.create_timer(0.5, self.publish_coordinates)  # Adjust the timer as needed
        self.selected_topic = None
        self.coordinates = Float32MultiArray()

    def publish_coordinates(self):
        if self.selected_topic is not None:
            self.get_logger().info(f'Publishing to {self.selected_topic}')
            self.coordinates.data = [float(coord) for coord in self.coordinates.data]
            if self.selected_topic == '/coord/source':
                self.publisher_source.publish(self.coordinates)
            elif self.selected_topic == '/coord/destination':
                self.publisher_destination.publish(self.coordinates)

def main(args=None):
    rclpy.init(args=args)
    coordinate_publisher = CoordinatePublisher()

    while True:
        print("\nSelect which topic to publish to:")
        print("1. /coord/source")
        print("2. /coord/destination")
        choice = input("Enter the number of your choice (1/2): ")

        if choice == '1':
            coordinate_publisher.selected_topic = '/coord/source'
        elif choice == '2':
            coordinate_publisher.selected_topic = '/coord/destination'
        else:
            print("Invalid choice, please try again.")
            continue
        coords_input = input("Enter the coordinates (x, y, z) separated by commas: ")
        coords = coords_input.split(',')
        if len(coords) != 3:
            print("Please enter exactly three coordinates.")
            continue
        try:
            coordinate_publisher.coordinates.data = [float(coord) for coord in coords]
        except ValueError:
            print("Please enter valid float values for x, y, and z.")
            continue
        rclpy.spin_once(coordinate_publisher)

if __name__ == '__main__':
    main()
