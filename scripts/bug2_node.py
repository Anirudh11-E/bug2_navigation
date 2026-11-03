
#!/usr/bin/env python3
import math
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Bug2Navigation:
    def __init__(self):
        rospy.init_node("bug2_navigation")

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False
        self.scan = None

        # Green start and red goal markers
        self.start_x, self.start_y = -4.0, -4.0
        self.goal_x, self.goal_y = 4.0, 4.0

        self.state = "GO_TO_GOAL"
        self.wall_side = 1
        self.hit_distance = None
        self.last_log = rospy.Time(0)

        rospy.on_shutdown(self.stop)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.x = p.x
        self.y = p.y
        _, _, self.yaw = euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        self.odom_ready = True

    def scan_callback(self, msg):
        self.scan = msg

    @staticmethod
    def wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def goal_distance(self):
        return math.hypot(
            self.goal_x - self.x,
            self.goal_y - self.y
        )

    def goal_heading(self):
        bearing = math.atan2(
            self.goal_y - self.y,
            self.goal_x - self.x
        )
        return self.wrap(bearing - self.yaw)

    def line_error(self):
        # M-line from (-4, -4) to (4, 4): y = x
        return self.y - self.x

    def sector(self, center, width=0.25):
        if self.scan is None:
            return 0.0

        scan = self.scan
        readings = []

        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment

            if abs(self.wrap(angle - center)) > width:
                continue

            if math.isfinite(r) and r >= max(0.05, scan.range_min):
                readings.append(min(r, scan.range_max))
            elif math.isinf(r) and r > 0:
                readings.append(scan.range_max)

        return min(readings) if readings else scan.range_max

    def stop(self):
        if hasattr(self, "cmd_pub"):
            self.cmd_pub.publish(Twist())

    def reset_to_green(self):
        """Reset the robot to the green start marker in Gazebo."""
        try:
            rospy.wait_for_service("/gazebo/set_model_state", timeout=10)
            set_state = rospy.ServiceProxy(
                "/gazebo/set_model_state", SetModelState
            )

            state = ModelState()
            state.model_name = "turtlebot3_burger"
            state.pose.position.x = self.start_x
            state.pose.position.y = self.start_y
            state.pose.position.z = 0.10

            q = quaternion_from_euler(0.0, 0.0, math.pi / 4.0)
            state.pose.orientation.x = q[0]
            state.pose.orientation.y = q[1]
            state.pose.orientation.z = q[2]
            state.pose.orientation.w = q[3]

            state.twist.linear.x = 0.0
            state.twist.linear.y = 0.0
            state.twist.linear.z = 0.0
            state.twist.angular.x = 0.0
            state.twist.angular.y = 0.0
            state.twist.angular.z = 0.0
            state.reference_frame = "world"

            response = set_state(state)

            if response.success:
                rospy.loginfo("Robot reset to green marker (-4, -4)")
                return True

            rospy.logerr("Gazebo reset failed: %s", response.status_message)

        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logerr("Could not reset robot: %s", str(error))

        return False

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and (
            not self.odom_ready or self.scan is None
        ):
            rate.sleep()

        if rospy.is_shutdown():
            return

        self.stop()

        if not self.reset_to_green():
            rospy.logerr("Navigation cancelled because robot reset failed.")
            return

        rospy.sleep(1.5)

        self.state = "GO_TO_GOAL"
        self.hit_distance = None

        rospy.loginfo("Bug2 started: green marker to red marker")

        while not rospy.is_shutdown():
            distance = self.goal_distance()
            heading = self.goal_heading()

            front = self.sector(0.0, 0.30)
            left = self.sector(1.15, 0.40)
            right = self.sector(-1.15, 0.40)
            goal_clear = self.sector(heading, 0.20)

            cmd = Twist()

            if distance < 0.40:
                self.stop()
                rospy.loginfo("GOAL REACHED: red marker")
                return

            if self.state == "GO_TO_GOAL":
                if front < 0.65:
                    self.state = "FOLLOW_WALL"
                    self.hit_distance = distance
                    self.wall_side = 1 if left >= right else -1

                    rospy.loginfo(
                        "Obstacle detected; following %s boundary",
                        "left" if self.wall_side == 1 else "right"
                    )
                else:
                    # Increased goal-seeking speed
                    cmd.angular.z = max(
                        -0.70, min(0.70, 1.4 * heading)
                    )

                    if abs(heading) < 0.30:
                        cmd.linear.x = 0.30
                    elif abs(heading) < 0.65:
                        cmd.linear.x = 0.12

            if self.state == "FOLLOW_WALL":
                side = left if self.wall_side == 1 else right

                if front < 0.60:
                    cmd.linear.x = 0.0
                    cmd.angular.z = -0.60 * self.wall_side
                else:
                    # Increased speed while following the obstacle
                    cmd.linear.x = 0.15

                    if side < 0.50:
                        cmd.angular.z = -0.35 * self.wall_side
                    elif side > 0.90:
                        cmd.angular.z = 0.35 * self.wall_side
                    else:
                        cmd.angular.z = 0.0

                near_line = abs(self.line_error()) < 0.25
                closer = (
                    self.hit_distance is not None
                    and distance < self.hit_distance - 0.40
                )
                clear_route = front > 0.85 and goal_clear > 0.85

                if near_line and closer and clear_route:
                    self.state = "GO_TO_GOAL"
                    self.hit_distance = None
                    rospy.loginfo(
                        "Bug2 leave condition met; resuming goal navigation"
                    )

                elif (
                    side > 1.20
                    and front > 0.90
                    and goal_clear > 1.00
                ):
                    self.state = "GO_TO_GOAL"
                    self.hit_distance = None
                    rospy.loginfo("Wall ended; resuming goal navigation")

            self.cmd_pub.publish(cmd)

            now = rospy.Time.now()
            if (now - self.last_log).to_sec() >= 2.0:
                rospy.loginfo(
                    "state=%s position=(%.2f, %.2f) "
                    "distance=%.2f front=%.2f left=%.2f right=%.2f",
                    self.state, self.x, self.y, distance,
                    front, left, right
                )
                self.last_log = now

            rate.sleep()


if __name__ == "__main__":
    try:
        Bug2Navigation().run()
    except rospy.ROSInterruptException:
        pass

