import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from action_file.action import Target

class PointSenderClient(Node):
    def __init__(self):
        super().__init__('point_sender_client')
        self.point_sender_client = ActionClient(self, Target, 'reach_target')  # Connect to the action server
        
        # Points to be sent sequentially
        self.points = [
            (7, 0, 0),  # First point
            #(0,0,0),    # Second point
            # (3.25, 1.85, 126),   # Third point
            # (2.76, 2.49, 198),   # Fourth point
            # (2.76, 3.99, 270),   # Fifth point
        ]
        
        # Point counter to track progress
        self.counter = 0
        
        # Begin sending points
        self.send_next_point()

    def send_next_point(self):
        if self.counter < len(self.points):
            x, y, theta = self.points[self.counter]
            self.get_logger().info(f"Sending point {self.counter + 1}: x={x}, y={y}, theta={theta}")
            self.send_goal(x, y, theta)
        else:
            self.get_logger().info("All points have been sent and processed.")


    ####################################################### Step 1 : Send The Goal ######################################################## 
    
    def send_goal(self, x, y, theta):
    
        goal_msg = Target.Goal()
        goal_msg.target_point.x = float(x)
        goal_msg.target_point.y = float(y)
        goal_msg.target_point.z = float(theta)

        self.point_sender_client.wait_for_server()  # Wait for the action server to be ready
        self.point_sender_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    #################################### Step 2: Check From Server If the goal is accepted or not ########################################## 
    def goal_response_callback(self, future):
        self.goal_handle : ClientGoalHandle =  future.result()
        if self.goal_handle.accepted:
            self.get_logger().info('Goal accepted. Waiting for result...')
            self.goal_handle.get_result_async().add_done_callback(self.result_callback)
        else:
            self.get_logger().info('Goal rejected.')
            return
    ################################ Step 3: Recieve Result From server and send next point if the goal fulfilled ############################ 

    def result_callback(self, future):
        try:
            result = future.result().result
            if result.success:
                self.get_logger().info(f"Point {self.counter + 1} reached successfully.")
                self.counter += 1
                self.send_next_point()  # Send the next point
            else:
                self.get_logger().info(f"Failed to reach point {self.counter + 1}.")
        except Exception as e:
            self.get_logger().error(f"Exception in result callback: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = PointSenderClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
