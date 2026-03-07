from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy

def main():
    rclpy.init()
    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()

    # Simulated crop bed waypoints
    crop_beds = [
        (5.0, 2.0),
        (3.0, 5.0),
        (4.0, 6.0),
    ]

    goal_poses = []
    for x, y in crop_beds:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        goal_poses.append(pose)

    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f'Heading to waypoint {feedback.current_waypoint + 1}/{len(goal_poses)}')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('All crop beds visited!')
    elif result == TaskResult.CANCELED:
        print('Mission canceled!')
    elif result == TaskResult.FAILED:
        print('Mission failed!')

    navigator.lifecycleShutdown()

if __name__ == '__main__':
    main()