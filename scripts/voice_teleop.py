#!/usr/bin/env python3
"""Maps recognized voice commands to Scout Mini velocity commands.

Subscribes to /voice_commands (std_msgs/String) and publishes
geometry_msgs/Twist on /cmd_vel at a fixed rate.

- 'forward' keeps driving until 'stop' is heard
  (or the safety timeout expires, if enabled).
- 'turn left' / 'turn right' rotate exactly ~90 degrees using
  odometry feedback, then stop.
- 'go back' / 'turn around' rotate 180 degrees, then drive forward.

The Scout Mini is skid-steer, so turning scrubs the wheels sideways. Its
odometry is integrated from wheel encoders and drifts more through turns than
a differential-drive robot's; the turn tolerance below is loosened to match.
"""

import math

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# 'faster' multiplies speed by this factor, 'slower' divides by it.
SPEED_FACTOR = 3.0
MIN_LIN_SPEED = 0.05
MIN_ANG_SPEED = 0.2

TURN_ANGLE = math.pi / 2       # 90 degrees
# The loop closes on /odom. In simulation that is Gazebo ground truth, so a
# tight tolerance is honest; the final approach runs at MIN_TURN_SPEED, which
# at 50 Hz moves well under a tenth of a degree per cycle, so this cannot
# oscillate. On wheel-only odometry the turn is limited by scrub, not by this.
TURN_TOLERANCE = math.radians(1.0)
MIN_TURN_SPEED = 0.15          # rad/s, keeps the final approach moving

# Default ceiling for the closed-loop 90/180-degree turns, independent of how
# far 'faster' has scaled ang_speed. The turns close the loop on wheel
# odometry, and a skid-steer chassis takes roughly a second to spin up from
# rest: while the wheels scrub, odometry runs ahead of the body, so a fast turn
# can report complete before the robot has physically rotated. Keeping the turn
# slow holds that error down. Override with ~max_turn_speed.
DEFAULT_MAX_TURN_SPEED = 0.25  # rad/s


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class VoiceTeleop(object):
    def __init__(self):
        # Ceilings are deliberately below the Scout Mini's hardware limits
        # (~3.0 m/s, ~2.5 rad/s) -- voice control is not a precise input.
        self.max_lin = rospy.get_param("~max_linear_speed", 1.5)
        self.max_ang = rospy.get_param("~max_angular_speed", 2.0)
        self.lin_speed = rospy.get_param("~linear_speed", 0.3)
        self.ang_speed = rospy.get_param("~angular_speed", 0.6)
        # Stop if no command heard for this many seconds (0 disables).
        # Does not interrupt an in-progress 90-degree turn.
        self.timeout = rospy.get_param("~command_timeout", 0.0)
        self.max_turn_speed = rospy.get_param("~max_turn_speed",
                                              DEFAULT_MAX_TURN_SPEED)

        self.twist = Twist()
        self.last_cmd_time = rospy.Time.now()

        self.yaw = None
        self.target_yaw = None       # set while a turn is running
        self.turn_direction = 0.0
        self.turn_end_time = None    # fallback when odometry is unavailable
        self.forward_after_turn = False

        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("voice_commands", String, self.on_command)
        rospy.Subscriber("odom", Odometry, self.on_odom)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_turn()
            if self.target_yaw is None and self.turn_end_time is None \
               and self.timeout > 0 and \
               (rospy.Time.now() - self.last_cmd_time).to_sec() > self.timeout:
                self.twist = Twist()
            self.pub.publish(self.twist)
            rate.sleep()

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def start_turn(self, direction, angle=TURN_ANGLE, forward_after=False):
        """direction: +1 = left (CCW), -1 = right (CW)."""
        self.twist = Twist()
        self.turn_direction = direction
        self.forward_after_turn = forward_after
        turn_speed = min(self.ang_speed, self.max_turn_speed)
        if self.yaw is not None:
            self.target_yaw = normalize_angle(self.yaw + direction * angle)
            self.turn_end_time = None
        else:
            # No odometry yet: fall back to an open-loop timed turn.
            rospy.logwarn("No odometry received, using timed turn")
            duration = angle / turn_speed
            self.turn_end_time = rospy.Time.now() + rospy.Duration(duration)
            self.target_yaw = None
        self.twist.angular.z = direction * turn_speed

    def cancel_turn(self):
        self.target_yaw = None
        self.turn_end_time = None
        self.turn_direction = 0.0
        self.forward_after_turn = False

    def finish_turn(self, label):
        forward_after = self.forward_after_turn
        self.cancel_turn()
        self.twist = Twist()
        if forward_after:
            self.twist.linear.x = self.lin_speed
            rospy.loginfo("%s complete, moving forward", label)
        else:
            rospy.loginfo("%s complete", label)

    def update_turn(self):
        if self.target_yaw is not None and self.yaw is not None:
            remaining = normalize_angle(self.target_yaw - self.yaw)
            # Overshoot check only near the target: right after a 180-degree
            # command 'remaining' sits at the +/-pi wraparound, where its
            # sign is not meaningful.
            overshot = remaining * self.turn_direction < 0 and \
                abs(remaining) < 1.0
            if abs(remaining) <= TURN_TOLERANCE or overshot:
                self.finish_turn("Turn")
            else:
                # Proportional slowdown near the target for a clean stop.
                speed = min(self.ang_speed, self.max_turn_speed,
                            max(MIN_TURN_SPEED, 2.0 * abs(remaining)))
                self.twist.angular.z = self.turn_direction * speed
        elif self.turn_end_time is not None:
            if rospy.Time.now() >= self.turn_end_time:
                self.finish_turn("Timed turn")

    def on_command(self, msg):
        cmd = msg.data.lower().strip()
        words = set(cmd.split())
        self.last_cmd_time = rospy.Time.now()

        # Priority matters: speed and backward words must be checked before
        # 'go', so 'go faster' / 'go slower' / 'go back' are not mistaken
        # for a forward command.
        if words & {"faster", "fast"} or "speed up" in cmd:
            self.lin_speed = min(self.lin_speed * SPEED_FACTOR, self.max_lin)
            self.ang_speed = min(self.ang_speed * SPEED_FACTOR, self.max_ang)
            self.rescale_current()
            rospy.loginfo("Speed x%.0f -> lin %.2f m/s, ang %.2f rad/s",
                          SPEED_FACTOR, self.lin_speed, self.ang_speed)
        elif words & {"slower", "slow"}:
            self.lin_speed = max(self.lin_speed / SPEED_FACTOR, MIN_LIN_SPEED)
            self.ang_speed = max(self.ang_speed / SPEED_FACTOR, MIN_ANG_SPEED)
            self.rescale_current()
            rospy.loginfo("Speed /%.0f -> lin %.2f m/s, ang %.2f rad/s",
                          SPEED_FACTOR, self.lin_speed, self.ang_speed)
        elif words & {"backward", "back", "reverse", "around"}:
            self.start_turn(1.0, math.pi, forward_after=True)
        elif words & {"forward", "go"}:
            self.cancel_turn()
            self.twist = Twist()
            self.twist.linear.x = self.lin_speed
        elif "left" in cmd:
            self.start_turn(1.0)
        elif "right" in cmd:
            self.start_turn(-1.0)
        elif any(w in cmd for w in ("stop", "halt")):
            self.cancel_turn()
            self.twist = Twist()
        else:
            rospy.logwarn("Unknown command: '%s'", cmd)
            return

        rospy.loginfo("Command '%s' -> lin %.2f m/s, ang %.2f rad/s",
                      cmd, self.twist.linear.x, self.twist.angular.z)

    def rescale_current(self):
        # Keep moving in the current direction at the new speed.
        if self.twist.linear.x > 0:
            self.twist.linear.x = self.lin_speed
        elif self.twist.linear.x < 0:
            self.twist.linear.x = -self.lin_speed
        if self.target_yaw is None and self.turn_end_time is None:
            if self.twist.angular.z > 0:
                self.twist.angular.z = self.ang_speed
            elif self.twist.angular.z < 0:
                self.twist.angular.z = -self.ang_speed


if __name__ == "__main__":
    rospy.init_node("voice_teleop")
    try:
        VoiceTeleop()
    except rospy.ROSInterruptException:
        pass
