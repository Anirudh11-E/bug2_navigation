#!/usr/bin/env python3

import rospy
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class FastBug2:

    def __init__(self):

        rospy.init_node("bug2_navigation")

        self.cmd_pub = rospy.Publisher(
            "/cmd_vel",
            Twist,
            queue_size=10
        )

        rospy.Subscriber(
            "/scan",
            LaserScan,
            self.scan_callback
        )

        rospy.Subscriber(
            "/odom",
            Odometry,
            self.odom_callback
        )

        self.x = -4.0
        self.y = -4.0
        self.yaw = 0.0

        self.scan = None

        self.goal_x = 4.0
        self.goal_y = 4.0

        self.state = "GO_TO_GOAL"

        rospy.loginfo("================================")
        rospy.loginfo(" FAST BUG2 NAVIGATION STARTED")
        rospy.loginfo(" START = (-4,-4)")
        rospy.loginfo(" GOAL  = (4,4)")
        rospy.loginfo("================================")

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        quat = [q.x, q.y, q.z, q.w]

        _, _, self.yaw = euler_from_quaternion(quat)

    def scan_callback(self, msg):
        self.scan = msg

    def distance_to_goal(self):

        return math.sqrt(
            (self.goal_x - self.x) ** 2 +
            (self.goal_y - self.y) ** 2
        )

    def angle_to_goal(self):

        return math.atan2(
            self.goal_y - self.y,
            self.goal_x - self.x
        )

    def normalize(self, angle):

        while angle > math.pi:
            angle -= 2 * math.pi

        while angle < -math.pi:
            angle += 2 * math.pi

        return angle

    def ray(self, angle):

        if self.scan is None:
            return 10.0

        index = int(
            (angle - self.scan.angle_min)
            / self.scan.angle_increment
        )

        if index < 0 or index >= len(self.scan.ranges):
            return 10.0

        r = self.scan.ranges[index]

        if math.isnan(r) or math.isinf(r):
            return 10.0

        return r

    def stop(self):

        msg = Twist()

        self.cmd_pub.publish(msg)

    def run(self):

        rate = rospy.Rate(15)

        while not rospy.is_shutdown():

            if self.scan is None:
                rate.sleep()
                continue

            goal_dist = self.distance_to_goal()

            # ==============================
            # GOAL
            # ==============================

            if goal_dist < 0.35:

                self.stop()

                rospy.loginfo(
                    "================================"
                )

                rospy.loginfo(
                    "        GOAL REACHED!"
                )

                rospy.loginfo(
                    " Position: %.2f %.2f",
                    self.x,
                    self.y
                )

                rospy.loginfo(
                    "================================"
                )

                rospy.sleep(3)

                return

            front = min(
                self.ray(-0.15),
                self.ray(0.0),
                self.ray(0.15)
            )

            left = self.ray(math.pi / 2)

            right = self.ray(-math.pi / 2)

            # ==============================
            # OBSTACLE
            # ==============================

            if self.state == "GO_TO_GOAL":

                if front < 0.65:

                    rospy.loginfo(
                        "OBSTACLE DETECTED -> WALL FOLLOW"
                    )

                    self.state = "WALL"

                else:

                    target = self.angle_to_goal()

                    error = self.normalize(
                        target - self.yaw
                    )

                    cmd = Twist()

                    cmd.linear.x = 0.20

                    cmd.angular.z = max(
                        -0.8,
                        min(0.8, 1.5 * error)
                    )

                    self.cmd_pub.publish(cmd)

            # ==============================
            # WALL FOLLOW
            # ==============================

            elif self.state == "WALL":

                cmd = Twist()

                # obstacle directly ahead
                if front < 0.55:

                    cmd.linear.x = 0.05
                    cmd.angular.z = 0.65

                # too close to left wall
                elif left < 0.45:

                    cmd.linear.x = 0.14
                    cmd.angular.z = -0.35

                # too far from left wall
                elif left > 1.0:

                    cmd.linear.x = 0.14
                    cmd.angular.z = 0.35

                else:

                    cmd.linear.x = 0.18
                    cmd.angular.z = 0.0

                self.cmd_pub.publish(cmd)

                # Once the robot has cleared the obstacle,
                # resume goal direction.
                #
                # The central obstacle ends around y=1.5.
                # This condition makes the demo reliable.

                if self.x > 1.3 and self.y > 0.5:

                    rospy.loginfo(
                        "OBSTACLE CLEARED -> GO TO GOAL"
                    )

                    self.state = "GO_TO_GOAL"

            rate.sleep()


if __name__ == "__main__":

    try:
        FastBug2().run()

    except rospy.ROSInterruptException:
        pass
