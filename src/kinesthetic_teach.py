import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class KinestheticTeaching(Node):

    def __init__(self):
        super().__init__('kinesthetic_teaching')

        # Subscriber to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)

        # List to store joint trajectories
        self.joint_trajectory = []

    def joint_states_callback(self, msg):
        # Append joint positions to the trajectory
        self.joint_trajectory.append(msg.position)

    def save_trajectory(self):
        # Save the trajectory data to a file or process further
        with open("trajectory.txt", "w") as file:
            for positions in self.joint_trajectory:
                file.write(', '.join(map(str, positions)) + '\n')

    def on_shutdown(self):
        # Save trajectory when node is shut down
        self.save_trajectory()

def main(args=None):
    rclpy.init(args=args)

    # Create and run the node
    kt_node = KinestheticTeaching()
    rclpy.spin(kt_node)

    # Cleanup
    kt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
