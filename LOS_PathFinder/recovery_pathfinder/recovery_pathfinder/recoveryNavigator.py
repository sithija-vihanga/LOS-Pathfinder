import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.action import GoalResponse
from recovery_pathfinder.recoveryStatus import RECOVERYSTATUS

class recoveryNavigator:
    def __init__(self, node):
        self.node           = node
        self.goalPublished  = False
        self.client         = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def navigateToGoal(self, key_points):
        if(not self.goalPublished):
            self.node.logger.info(f"Starting navigation to : {key_points[0]}")
            goal_pose                       = PoseStamped()
            goal_pose.header.frame_id       = 'map'
            goal_pose.pose.position.x       = key_points[0][0]
            goal_pose.pose.position.y       = key_points[0][1]
            goal_pose.pose.orientation.w    = 1.0

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose     

            self.client.wait_for_server()
            self.node.logger.info('Action server available. Sending goal...')
            send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedbackCallback)
            send_goal_future.add_done_callback(self.goalResponseCallback)

            self.goalPublished = True

    def goalResponseCallback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.logger().warn('Goal was rejected')
            return

        self.node.logger.info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def feedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.logger.debug(f'Current progress: {feedback.current_pose}')

    def get_result_callback(self, future):
        self.goalPublished      = False
        self.node.RECOVERYMODE  = RECOVERYSTATUS.CHECKPULSE
        self.node.logger.info('Goal reached successfully!')
        self.node.logger.info("Start checking pulse")



