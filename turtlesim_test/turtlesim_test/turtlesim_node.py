import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from rclpy.time import Duration
from math import pi

class TurtlesimNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.goal_point = [
            [5.54, 5.54],
            [5.54, 8.54],
            [6.54, 7.54],
            [5.54, 6.54],
            [6.54, 5.54],
            [7.04, 8.54],
            [7.54, 5.54],
            [7.84, 8.54],
            [8.54, 5.54]
        ]
        self.tur_pub = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.pose_sub = self.create_subscription(Pose,"/turtle1/pose",self.callback,10)


        self.linear_angel = Twist()
        self.current_pose = Pose()

        self.angel_yuzhi = 0.01
        self.distance_yuzhi = 0.02

        self.linear_kp = 1.0
        self.linear_ki = 0.01
        self.linear_kd = 0.1

        self.last_dis_error = 0.0
        self.dis_error_sum = 0.0
        self.max_dis_error_sum = 0.5
        self.min_dis_error_sum = -0.5

        self.angel_kp = 3.0

        self.T_time = 0.01

    def callback(self,msg):
        self.current_pose = msg

    def angel_pi(self,target_angel):
        if target_angel > pi:
            target_angel -= 2*pi

        elif target_angel < -pi:
            target_angel += 2*pi

        return target_angel
    
    def jisuan_error_angel(self,target_x,target_y):
        target_angel = math.atan2(target_y - self.current_pose.y,
                                 target_x - self.current_pose.x )
        
        error_angel = target_angel - self.current_pose.theta
        error_angel = self.angel_pi(error_angel)

        return error_angel
    
    def control_angel(self,target_angel):
        self.get_logger().info(f"目标旋转角度: {target_angel:.2f} rad")

        while rclpy.ok():
            angel_error = self.angel_pi(target_angel - self.current_pose.theta)
            self.get_logger().info(f"当前角度误差: {angel_error:.4f} rad")

            if abs(angel_error) <= self.angel_yuzhi:
                break

            self.linear_angel.angular.z = self.angel_kp * angel_error
            self.tur_pub.publish(self.linear_angel)

            rclpy.spin_once(self,timeout_sec=self.T_time)

        self.linear_angel.angular.z = 0.0
        self.tur_pub.publish(self.linear_angel)
        self.get_logger().info("已精准旋转到目标角度")

    def control_distance(self,target_x,target_y):
        self.get_logger().info(f"目标位置: ({target_x:.2f}, {target_y:.2f})")

        self.last_dis_error = 0.0
        self.dis_error_sum = 0.0

        while rclpy.ok():
            error_y = target_y - self.current_pose.y
            error_x = target_x - self.current_pose.x
            error_dis = math.hypot(error_y,error_x)
            self.get_logger().info(f"当前位置误差: {error_dis:.4f} m")

            if abs(error_dis) <= self.distance_yuzhi:
                break

            angel_error = self.jisuan_error_angel(target_x,target_y)

            if abs(angel_error) > self.angel_yuzhi:
                self.linear_angel.angular.z = self.angel_kp * angel_error
            else:
                self.linear_angel.angular.z = 0.0
            
            p_part = error_dis * self.linear_kp

            i_part = 0.0
            if self.linear_ki > 0 and error_dis > 0.1:
                self.dis_error_sum += self.T_time * error_dis 
                self.dis_error_sum = max(min(self.max_dis_error_sum,self.dis_error_sum),self.min_dis_error_sum)

                i_part = self.linear_ki * self.dis_error_sum

            d_part = self.linear_kd * (error_dis - self.last_dis_error) / self.T_time
            self.last_dis_error = error_dis

            linear_speed = p_part + i_part + d_part
            linear_speed = max(min(linear_speed,1.0),0.0)

            if abs(angel_error) > 0.5:
                linear_speed *= 0.3

            self.linear_angel.linear.x = linear_speed
            self.tur_pub.publish(self.linear_angel)
            rclpy.spin_once(self,timeout_sec=self.T_time)

        self.linear_angel.linear.x = 0.0
        self.linear_angel.angular.z = 0.0
        self.tur_pub.publish(self.linear_angel)
        self.get_logger().info("已精准移动到目标点")

def main():
    rclpy.init()
    node = TurtlesimNode("turtle_node")
    node.get_clock().sleep_for(Duration(seconds=1.0))

    if len(node.goal_point) > 0:
        first_goal = node.goal_point[0]

        start_angel_goal = math.atan2(first_goal[1] - node.current_pose.y,
                                      first_goal[0] - node.current_pose.x)
        
        node.control_angel(start_angel_goal)

        node.control_distance(first_goal[0],first_goal[1])

        for i in range(len(node.goal_point) - 1):
            current_goal = node.goal_point[i]
            next_goal = node.goal_point[i + 1]

            angel_goal = math.atan2(next_goal[1] - current_goal[1],
                                    next_goal[0] - current_goal[0])
            
            node.control_angel(angel_goal)

            node.control_distance(next_goal[0],next_goal[1])

            node.get_logger().info(f"已到达点 {i+2}: ({next_goal[0]}, {next_goal[1]})")

    node.get_logger().info('所有目标点已到达')
    node.destroy_node()
    rclpy.shutdown()