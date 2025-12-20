import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion
from gazebo_msgs.srv import SetEntityState
import random
import time
import math


MAP_PHASE2_PATH = '/home/ubuntu/ros2_ws/src/my_maze/maps/map_phase2.yaml'
def euler_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')
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
        self.get_logger().info(f'Robot teleported to: ({x_rand:.2f}, {y_rand:.2f})')
        return x_rand, y_rand, yaw_rand

def main():
    rclpy.init()
    
    navigator = BasicNavigator()
    maze_node = MazeSolver()

    real_x, real_y, real_yaw = maze_node.random_spawn()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = real_x
    initial_pose.pose.position.y = real_y
    initial_pose.pose.orientation = euler_to_quaternion(real_yaw)
    
    print(f">>> [Phase 1] Setting initial pose: ({real_x:.2f}, {real_y:.2f})...")
    navigator.setInitialPose(initial_pose)

    print(">>> [Phase 1] Waiting for Nav2 to activate...")
    navigator.waitUntilNav2Active() 

    print(">>> [Phase 1] Clearing Costmaps to remove ghosts...")
    navigator.clearAllCostmaps()
    time.sleep(1.0)

    print(">>> [Phase 1] Spinning to converge particles...")
    navigator.spin(spin_dist=3.14, time_allowance=10) 

    if navigator.isTaskComplete():
        print(">>> [Phase 1] Localization converged.")
    else:
        print(">>> [Phase 1] Spin timed out, proceeding anyway...")


    start_pose = PoseStamped()
    start_pose.header.frame_id = 'map'
    start_pose.header.stamp = navigator.get_clock().now().to_msg()
    start_pose.pose.position.x = 1.6
    start_pose.pose.position.y = 0.2
    start_pose.pose.orientation = euler_to_quaternion(0.0)

    print(f">>> [Phase 1] Navigating to START zone (Ignoring Blue Lines)...")
    navigator.goToPose(start_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        # print(f'Distance remaining: {feedback.distance_remaining:.2f}', end='\r')
        
    result = navigator.getResult()
    
    if result == TaskResult.SUCCEEDED:
        print("\n>>> [Success] Reached START zone!")
        print(">>> ====================================")
        print(">>> [Phase 2] Initiating Map Switch...")
        print(">>> ====================================")

        try:
            navigator.changeMap(MAP_PHASE2_PATH)
            print(">>> [Phase 2] Map switch request sent. Waiting for Costmap update...")
            
            time.sleep(4.0) 
            
            navigator.clearAllCostmaps()
            time.sleep(1.0)
            
        except Exception as e:
            print(f">>> [Error] Failed to change map! Check path: {MAP_PHASE2_PATH}")
            print(e)
            return

        end_pose = PoseStamped()
        end_pose.header.frame_id = 'map'
        end_pose.header.stamp = navigator.get_clock().now().to_msg()
        end_pose.pose.position.x = 0.8
        end_pose.pose.position.y = 1.6
        end_pose.pose.orientation = euler_to_quaternion(1.57) # 朝向上方 (90度)

        print(f">>> [Phase 2] Planning path to END (Avoiding Blue Lines)...")
        navigator.goToPose(end_pose)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            # print(f'Phase 2 Dist: {feedback.distance_remaining:.2f}', end='\r')
            
        result2 = navigator.getResult()
        if result2 == TaskResult.SUCCEEDED:
            print("\n>>> [VICTORY] Robot reached END position safely!")
        else:
            print(f"\n>>> [Failure] Phase 2 failed with code: {result2}")
            print(">>> Hint: Check 'inflation_radius' in nav2_params.yaml if path planning fails.")

    else:
        print(f">>> [Failure] Phase 1 failed, error code: {result}")

    maze_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
