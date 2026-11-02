#!/usr/bin/env python3

import math
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Bug2Navigation:

    def __init__(self):
        rospy.init_node("bug2_navigation")

        self.cmd_pub = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=10
        )

        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.x = -4.0
        self.y = -4.0
        self.yaw = 0.0
        self.scan = None
        self.have_odom = False

        self.start_x = -4.0
        self.start_y = -4.0
        self.goal_x = 4.0
        self.goal_y = 4.0

        self.state = "GO_TO_GOAL"
        self.hit_goal_distance = None
        self.previous_line_value = None

        rospy.loginfo("Bug2-style navigation controller started")
        rospy.loginfo("Start: (-4, -4), Goal: (4, 4)")

    def scan_callback(self, msg):
        self.scan = msg

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        self.have_odom = True

    def normalize(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def goal_distance(self):
        return math.hypot(
            self.goal_x - self.x,
            self.goal_y - self.y
        )

    def goal_angle(self):
        return math.atan2(
            self.goal_y - self.y,
            self.goal_x - self.x
        )

    def ray(self, angle):
        if self.scan is None:
            return 10.0

        angle = self.normalize(angle)
        index = int(
            (angle - self.scan.angle_min)
            / self.scan.angle_increment
        )

        if index < 0 or index >= len(self.scan.ranges):
            return 10.0

        distance = self.scan.ranges[index]

        if math.isnan(distance):
            return 10.0
        if math.isinf(distance):
            return 10.0
        if distance <= 0:
            return 10.0

        return min(distance, self.scan.range_max)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def line_value(self):
        # The start-to-goal M-line is y = x.
        return self.y - self.x

    def run(self):
        rate = rospy.Rate(15)

        while not rospy.is_shutdown():

            if self.scan is None or not self.have_odom:
                rate.sleep()
                continue

            distance = self.goal_distance()

            if distance < 0.35:
                self.stop()
                rospy.loginfo("GOAL REACHED at (%.2f, %.2f)",
                              self.x, self.y)
                return

            front = min(
                self.ray(-0.20),
                self.ray(-0.10),
                self.ray(0.0),
                self.ray(0.10),
                self.ray(0.20)
            )

            left = min(
                self.ray(1.25),
                self.ray(1.57),
                self.ray(1.90)
            )

            goal_relative_angle = self.normalize(
                self.goal_angle() - self.yaw
            )
            goal_ray = self.ray(goal_relative_angle)

            current_line = self.line_value()

            crossed_m_line = False
            if self.previous_line_value is not None:
                crossed_m_line = (
                    self.previous_line_value * current_line <= 0
                    and abs(current_line) < 0.40
                )

            self.previous_line_value = current_line

            cmd = Twist()

            if self.state == "GO_TO_GOAL":

                if front < 0.70:
                    self.state = "WALL_FOLLOWING"
                    self.hit_goal_distance = distance

                    rospy.loginfo(
                        "Obstacle detected: following boundary"
                    )

                else:
                    heading_error = goal_relative_angle

                    # Turn toward the goal before moving quickly.
                    cmd.angular.z = max(
                        -0.8, min(0.8, 1.4 * heading_error)
                    )

                    if abs(heading_error) > 0.65:
                        cmd.linear.x = 0.0
                    else:
                        cmd.linear.x = 0.20

            if self.state == "WALL_FOLLOWING":

                # Follow the obstacle with it on the left.
                if front < 0.55:
                    cmd.linear.x = 0.03
                    cmd.angular.z = -0.55

                elif left < 0.50:
                    cmd.linear.x = 0.10
                    cmd.angular.z = -0.25

                elif left > 0.90:
                    cmd.linear.x = 0.12
                    cmd.angular.z = 0.30

                else:
                    cmd.linear.x = 0.15
                    cmd.angular.z = 0.0

                # Leave the boundary only after crossing the
                # start-goal line closer to the goal, with a
                # reasonably clear path toward the goal.
                if (
                    crossed_m_line
                    and self.hit_goal_distance is not None
                    and distance < self.hit_goal_distance - 0.30
                    and goal_ray > 1.0
                    and front > 0.65
                ):
                    self.state = "GO_TO_GOAL"
                    self.hit_goal_distance = None

                    rospy.loginfo(
                        "M-line crossed; resuming goal-seeking"
                    )

            self.cmd_pub.publish(cmd)
            rate.sleep()


if __name__ == "__main__":
    try:
        Bug2Navigation().run()
    except rospy.ROSInterruptException:
        pass
    finally:
        pass
