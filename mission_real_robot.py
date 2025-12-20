import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion
import time
import math


MAP_PHASE2_PATH = '/home/ubuntu/ros2_ws/src/my_maze/maps/real_map_phase1.yaml'


INITIAL_X = 0.45  
INITIAL_Y = 0.47   
INITIAL_YAW = 0.0 


def euler_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def main():
    rclpy.init()
    
    navigator = BasicNavigator()


    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = INITIAL_X
    initial_pose.pose.position.y = INITIAL_Y
    initial_pose.pose.orientation = euler_to_quaternion(INITIAL_YAW)
    
    print(f">>> [Real World] Setting initial guess to: ({INITIAL_X}, {INITIAL_Y})")
    print(">>> Please ensure the robot is physically at this location!")
    navigator.setInitialPose(initial_pose)

    print(">>> [System] Waiting for Nav2 to activate (check MicroROS Agent)...")
    navigator.waitUntilNav2Active() 

    print(">>> [System] Clearing Costmaps...")
    navigator.clearAllCostmaps()
    time.sleep(1.0)


    print(">>> [Phase 1] Spinning to match scan with map...")
    navigator.spin(spin_dist=3.14, time_allowance=15) 

    if navigator.isTaskComplete():
        print(">>> [Phase 1] Localization spin finished.")
    else:
        print(">>> [Phase 1] Spin timed out (check battery/motors), proceeding...")

    start_pose = PoseStamped()
    start_pose.header.frame_id = 'map'
    start_pose.header.stamp = navigator.get_clock().now().to_msg()
    start_pose.pose.position.x = 1.6
    start_pose.pose.position.y = 0.2
    start_pose.pose.orientation = euler_to_quaternion(0.0)

    print(f">>> [Phase 1] Moving to START zone...")
    navigator.goToPose(start_pose)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0: 
            print(f'>>> [Navigating] Distance remaining: {feedback.distance_remaining:.2f} m')
        time.sleep(0.1)
        
    result = navigator.getResult()
    
    if result == TaskResult.SUCCEEDED:
        print("\n>>> [Success] Reached START zone!")
        print(">>> ====================================")
        print(">>> [Phase 2] Initiating Map Switch...")
        print(">>> ====================================")

        try:
            print(f">>> Loading map from: {MAP_PHASE2_PATH}")
            navigator.changeMap(MAP_PHASE2_PATH)
            print(">>> Map switch request sent. Waiting 5s for Costmap update...")
            
            time.sleep(5.0) 
            
            navigator.clearAllCostmaps()
            time.sleep(1.0)
            
        except Exception as e:
            print(f">>> [Error] Failed to change map! {e}")
            return


        end_pose = PoseStamped()
        end_pose.header.frame_id = 'map'
        end_pose.header.stamp = navigator.get_clock().now().to_msg()
        end_pose.pose.position.x = 0.8
        end_pose.pose.position.y = 1.6
        end_pose.pose.orientation = euler_to_quaternion(1.57)

        print(f">>> [Phase 2] Planning path to END (Using Updated Map)...")
        navigator.goToPose(end_pose)

        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 10 == 0:
                print(f'>>> [Phase 2] Dist to Goal: {feedback.distance_remaining:.2f} m')
            time.sleep(0.1)
            
        result2 = navigator.getResult()
        if result2 == TaskResult.SUCCEEDED:
            print("\n>>> [VICTORY] Robot reached END position!")
        else:
            print(f"\n>>> [Failure] Phase 2 failed with code: {result2}")

    else:
        print(f">>> [Failure] Phase 1 failed, error code: {result}")

    rclpy.shutdown()

if __name__ == '__main__':
    main()