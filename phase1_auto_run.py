import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion
from gazebo_msgs.srv import SetEntityState
import random
import time
import math

def euler_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_phase1')
        self.client = self.create_client(SetEntityState, '/set_entity_state')

    def random_spawn(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gazebo service...')

        req = SetEntityState.Request()
        req.state.name = 'yahboom_car' 

        x_rand = random.uniform(0.8, 1.4) 
        y_rand = random.uniform(0.8, 1.4)
        yaw_rand = random.uniform(0, 6.28)

        req.state.pose.position.x = x_rand
        req.state.pose.position.y = y_rand
        req.state.pose.position.z = 0.05
        req.state.pose.orientation = euler_to_quaternion(yaw_rand)

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'The robot has teleported to a secure location: ({x_rand:.2f}, {y_rand:.2f})')
        return x_rand, y_rand, yaw_rand

def main():
    rclpy.init()
    
    navigator = BasicNavigator()
    maze_node = MazeSolver()

    real_x, real_y, real_yaw = maze_node.random_spawn()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp.sec = 0
    initial_pose.header.stamp.nanosec = 0

    initial_pose.pose.position.x = real_x
    initial_pose.pose.position.y = real_y
    initial_pose.pose.orientation = euler_to_quaternion(real_yaw)
    
    print(f">>> Send initial position guess: ({real_x:.2f}, {real_y:.2f})...")
    navigator.setInitialPose(initial_pose)

    print(">>> Awaiting Nav2 activation (may take several seconds)...")
    navigator.waitUntilNav2Active() 

    print(">>> Waiting for positioning algorithm to stabilise...")
    time.sleep(2.0)

    print(">>> Clear Costmaps...")
    navigator.clearAllCostmaps()
    time.sleep(1.0)

    print(">>> Initiate spin alignment (AMCL)...")
    navigator.spin(spin_dist=6.28, time_allowance=15)

    if navigator.isTaskComplete():
        print(">>> Spin complete, positioning finished")
    else:
        print(">>> Spin rejected (likely due to excessive expansion radius), attempting direct navigation....")

    # --- 步骤 D: 导航 ---
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp.sec = 0
    goal_pose.header.stamp.nanosec = 0
    goal_pose.pose.position.x = 1.6
    goal_pose.pose.position.y = 0.15
    goal_pose.pose.orientation = euler_to_quaternion(0.0)

    print(f">>> Proceeding to the START area...")
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        time.sleep(0.1) 
        
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(">>> Successfully reached the START zone")
    else:
        print(f">>> Navigation failed, error code: {result}")

    maze_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()