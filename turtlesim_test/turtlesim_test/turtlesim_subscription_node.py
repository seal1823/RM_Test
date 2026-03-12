import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from rclpy.duration import Duration


class TurtleSimController(Node):
    def __init__(self,node_name):
        super().__init__(node_name)

        self.tur_pub = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.sub_pose = self.create_subscription(Pose,"/turtle1/pose",self.callback,10)
        self.current_pose = Pose()
        self.pose_goal_pose = [
            [5.54,5.54],
            [5.54,8.54],
            [6.54,7.54],
            [5.54,6.54],
            [6.54,5.54],
            [7.04,8.54],
            [7.54,5.54],
            [7.84,8.54],
            [8.54,5.54]
        ]
        self.linear_angular = Twist()

        self.angle_error_threshold = 0.01
        self.pos_error_threshold = 0.05

        self.kp = 0.3
        self.ki = 0.0
        self.kd = 0.05
        self.last_pose_error = 0.0
        self.max_I_pose_error = 1.0
        self.min_I_pose_error = -1.0
        self.integral_deadzone = 0.1

    
    def callback(self,msg):
        self.current_pose = msg

    def corrent_theta(self,angle):
        error = angle - self.current_pose.theta
        error = math.atan2(math.sin(error),math.cos(error))
        return error

    def control_angle_closed_loop(self,target_angle,angular_speed = 0.2):
        self.get_logger().info(f"目标旋转角度: {target_angle:.2f} rad")
        while rclpy.ok():
            now_angle_error = self.corrent_theta(target_angle)
            self.get_logger().info(f"当前角度误差: {now_angle_error:.2f} rad")
            if abs(now_angle_error) <= self.angle_error_threshold:
                break

            self.linear_angular.angular.z = now_angle_error
            self.tur_pub.publish(self.linear_angular)

            rclpy.spin_once(self,timeout_sec=0.01)

        self.linear_angular.angular.z = 0.0
        self.tur_pub.publish(self.linear_angular)
        self.get_logger().info("已精准旋转到目标角度")

    def control_dis_closed_loop(self,target_x,target_y):
        self.get_logger().info(f"目标位置: ({target_x:.2f}, {target_y:.2f})")
        pose_error_sum = 0.0
        self.last_pose_error = 0.0

        while rclpy.ok():
            error_x = target_x - self.current_pose.x
            error_y = target_y - self.current_pose.y
            pos_error = math.hypot(error_x,error_y)
            self.get_logger().info(f"当前位置误差: {pos_error:.4f} m")

            if abs(pos_error) <= self.pos_error_threshold:
                break
            
            p_part = self.kp * pos_error
            
            i_part = 0.0
            if self.ki > 0 and abs(pos_error) > self.integral_deadzone:
                pose_error_sum += pos_error * 0.01
                pose_error_sum = max(min(pose_error_sum,self.max_I_pose_error),self.min_I_pose_error)
                i_part = self.ki * pose_error_sum

            d_part = self.kd * (pos_error - self.last_pose_error) / 0.01
            self.last_pose_error = pos_error

            linear_speed = p_part + i_part + d_part
            linear_speed = max(min(linear_speed, 0.5), 0.0)

            self.linear_angular.linear.x = linear_speed
            self.tur_pub.publish(self.linear_angular)

            rclpy.spin_once(self,timeout_sec=0.01)

        self.linear_angular.linear.x = 0.0
        self.tur_pub.publish(self.linear_angular)
        self.get_logger().info("已精准移动到目标点")
    
def main():
    rclpy.init()
    node = TurtleSimController("hai_gui_node")
    node.get_clock().sleep_for(Duration(seconds=1.0))

    for i in range(len(node.pose_goal_pose)-1):
        current_goal = node.pose_goal_pose[i]
        next_goal = node.pose_goal_pose[i+1]

        goal_theta = math.atan2(next_goal[1] - current_goal[1],next_goal[0] - current_goal[0])

        node.control_angle_closed_loop(goal_theta)

        node.control_dis_closed_loop(next_goal[0],next_goal[1])

        if i == len(node.pose_goal_pose)-1:
            node.get_logger().info('到达最后一个目标点，停止运动')
            node.linear_angular.linear.x = 0.0
            node.linear_angular.angular.z = 0.0
            node.tur_pub.publish(node.linear_angular)
            node.get_clock().sleep_for(Duration(seconds=0.5))
            break
            
        
    node.destroy_node()
    rclpy.shutdown()