#!/usr/bin/python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2012, Melonee Wise
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************
# author: Melonee Wise

import rospy
import roslib

import math
import sys
import tf
import PyKDL

from spheropy.Sphero import Sphero
from spheropy.Exception import SpheroException
from spheropy.DataStream import DataStreamManager
import dynamic_reconfigure.server

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
from sphero.msg import SpheroCollision
from std_msgs.msg import ColorRGBA, Float32, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sphero.cfg import ReconfigConfig


class SpheroNode(object):
    battery_state = {1: "Battery Charging",
                     2: "Battery OK",
                     3: "Battery Low",
                     4: "Battery Critical"}

    ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
                            0, 1e-3, 0, 0, 0, 0,
                            0, 0, 1e6, 0, 0, 0,
                            0, 0, 0, 1e6, 0, 0,
                            0, 0, 0, 0, 1e6, 0,
                            0, 0, 0, 0, 0, 1e3]

    ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
                             0, 1e-3, 0, 0, 0, 0,
                             0, 0, 1e6, 0, 0, 0,
                             0, 0, 0, 1e6, 0, 0,
                             0, 0, 0, 0, 1e6, 0,
                             0, 0, 0, 0, 0, 1e3]

    def __init__(self, default_update_rate=50.0):
        rospy.init_node('sphero_node')
        self.update_rate = default_update_rate
        self.sampling_divisor = int(400 / self.update_rate)
        self.is_connected = False

        self.odom_pub = None
        self.imu_pub = None
        self.collision_pub = None
        self.diag_pub = None
        self.cmd_vel_sub = None
        self.cmd_turn_sub = None
        self.color_sub = None
        self.back_led_sub = None
        self.stabilization_sub = None
        self.heading_sub = None
        self.angular_velocity_sub = None
        self.reconfigure_srv = None
        self.transform_broadcaster = None
        self._init_pubsub()

        self.robot_name = None
        self.robot_bt_addr = None
        self.connect_color_red = None
        self.connect_color_blue = None
        self.connect_color_green = None
        self.cmd_vel_timeout = None
        self.diag_update_rate = None
        self._init_params()

        rospy.loginfo("connect bt_addr %s", self.robot_bt_addr)
        self.robot = Sphero(self.robot_name, self.robot_bt_addr)

        self.imu = Imu()
        self.imu.orientation_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        self.imu.angular_velocity_covariance = [
            1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        self.imu.linear_acceleration_covariance = [
            1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        self.last_cmd_vel_time = rospy.Time.now()
        self.last_diagnostics_time = rospy.Time.now()
        self.cmd_heading = 0
        self.cmd_speed = 0
        self.power_state_msg = "No Battery Info"
        self.power_state = 0

    def _init_pubsub(self):
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
        self.collision_pub = rospy.Publisher(
            'collision', SpheroCollision, queue_size=1)
        self.diag_pub = rospy.Publisher(
            '/diagnostics', DiagnosticArray, queue_size=1)

        self.cmd_vel_sub = rospy.Subscriber(
            'cmd_vel', Twist, self.cmd_vel, queue_size=1)
        self.cmd_turn_sub = rospy.Subscriber(
            'cmd_turn', Float32, self.cmd_turn, queue_size=1)
        self.color_sub = rospy.Subscriber(
            'set_color', ColorRGBA, self.set_color, queue_size=1)
        self.back_led_sub = rospy.Subscriber(
            'set_back_led', Float32, self.set_back_led, queue_size=1)
        self.stabilization_sub = rospy.Subscriber(
            'disable_stabilization', Bool, self.set_stabilization, queue_size=1)
        self.heading_sub = rospy.Subscriber(
            'set_heading', Float32, self.set_heading, queue_size=1)
        self.angular_velocity_sub = rospy.Subscriber(
            'set_angular_velocity', Float32, self.set_angular_velocity, queue_size=1)
        self.reconfigure_srv = dynamic_reconfigure.server.Server(
            ReconfigConfig, self.reconfigure)
        self.transform_broadcaster = tf.TransformBroadcaster()

    def _init_params(self):
        if rospy.has_param('~bt_addr'):
            self.robot_bt_addr = rospy.get_param('bt_addr')
        else:
            rospy.logerr(
                "Must specify sphero address with private param bt_addr")
            sys.exit(1)

        self.robot_name = rospy.get_param("~name", "Sphero")
        self.connect_color_red = rospy.get_param('~connect_red', 0)
        self.connect_color_blue = rospy.get_param('~connect_blue', 0)
        self.connect_color_green = rospy.get_param('~connect_green', 255)
        self.cmd_vel_timeout = rospy.Duration(
            rospy.get_param('~cmd_vel_timeout', 0.6))
        self.diag_update_rate = rospy.Duration(
            rospy.get_param('~diag_update_rate', 1.0))

    def normalize_angle_positive(self, angle):
        return math.fmod(math.fmod(angle, 2.0 * math.pi) + 2.0 * math.pi, 2.0 * math.pi)

    def start(self):

        tries = 0
        while not self.is_connected and tries < 5:
            try:
                self.is_connected = self.robot.connect()
                rospy.loginfo("Connect to Sphero with address: %s",
                              self.robot.bt.target_address)
            except SpheroException as error:
                rospy.logwarn(
                    "Failed to connect to Sphero with error %s", error.messge)
                tries += 1
        if not self.is_connected:
            rospy.logerr("Cannot connect to sphero")
            sys.exit(1)

        # setup streaming
        manager = DataStreamManager()
        manager.acc = True
        manager.imu_angle = True
        manager.gyro = True
        manager.quaternion = True
        manager.odom = True
        manager.velocity = True
        self.robot.set_data_stream(manager, self.update_rate, response=True)
        self.robot.register_sensor_callback(self.handle_data_strm)

        # setup power notification
        self.robot.set_power_notification(True)
        self.robot.register_power_callback(self.handle_power_notify)

        # setup collision detection
        self.robot.start_collision_detection(45, 110, 45, 110, 1000)
        self.robot.register_collision_callback(self.handle_collision)

        # set the ball to connection color
        self.robot.set_rgb_led(
            self.connect_color_red, self.connect_color_green, self.connect_color_blue, 0, False)
        # now start receiving packets
        self.robot.start()

    def spin(self):
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if (now - self.last_cmd_vel_time) > self.cmd_vel_timeout:
                if self.cmd_heading != 0 or self.cmd_speed != 0:
                    self.cmd_heading = 0
                    self.cmd_speed = 0
                    self.robot.roll(int(self.cmd_speed), int(
                        self.cmd_heading))
            if (now - self.last_diagnostics_time) > self.diag_update_rate:
                self.last_diagnostics_time = now
                self.publish_diagnostics(now)
            r.sleep()

    def stop(self):
        # tell the ball to stop moving before quiting
        self.robot.stop()
        rospy.sleep(1.0)
        self.is_connected = False
        self.robot.disconnect()
        self.robot.join()

    def publish_diagnostics(self, time):
        diag = DiagnosticArray()
        diag.header.stamp = time

        stat = DiagnosticStatus(
            name="Battery Status", level=DiagnosticStatus.OK, message=self.power_state_msg)
        if self.power_state == 3:
            stat.level = DiagnosticStatus.WARN
        if self.power_state == 4:
            stat.level = DiagnosticStatus.ERROR
        diag.status.append(stat)

        self.diag_pub.publish(diag)

    def handle_collision(self, data):
        if self.is_connected:
            now = rospy.Time.now()
            collision = SpheroCollision()
            collision.header.stamp = now
            collision.x = data.x
            collision.y = data.y
            collision.z = data.z
            collision.axis = data.axis
            collision.x_magnitude = data.x_magnitude
            collision.y_magnitude = data.y_magnitude
            collision.speed = data.speed
            collision.timestamp = data.timestamp

            self.collision = collision
            self.collision_pub.publish(self.collision)

    def handle_power_notify(self, data):
        if self.is_connected:
            self.power_state = data
            self.power_state_msg = self.battery_state[data]

    def handle_data_strm(self, data):
        # TODO FIX THIS
        if self.is_connected:
            now = rospy.Time.now()
            imu = Imu(header=rospy.Header(frame_id="imu_link"))
            imu.header.stamp = now

            quaternion = data['quaternion']
            imu.orientation.x = quaternion.x
            imu.orientation.y = quaternion.y
            imu.orientation.z = quaternion.z
            imu.orientation.w = quaternion.w

            acceleration = data['acc']
            imu.linear_acceleration.x = acceleration.x
            imu.linear_acceleration.y = acceleration.y
            imu.linear_acceleration.z = acceleration.z

            gyro = data['gyro']
            imu.angular_velocity.x = gyro.x
            imu.angular_velocity.y = gyro.y
            imu.angular_velocity.z = gyro.z

            self.imu = imu
            self.imu_pub.publish(self.imu)

            odom = Odometry(header=rospy.Header(frame_id="odom"),
                            child_frame_id='base_footprint')
            odom.header.stamp = now

            odom = data['odom']
            odom.pose.pose = Pose(Point(odom.x, odom.y, 0.0),
                                  Quaternion(0.0, 0.0, 0.0, 1.0))

            velocity = data['velocity']
            odom.twist.twist = Twist(
                Vector3(velocity.x, velocity.y, 0), Vector3(0, 0, gyro.z))

            odom.pose.covariance = self.ODOM_POSE_COVARIANCE
            odom.twist.covariance = self.ODOM_TWIST_COVARIANCE
            self.odom_pub.publish(odom)

            # need to publish this trasform to show the roll, pitch, and yaw
            # properly
            self.transform_broadcaster.sendTransform((0.0, 0.0, 0.038),
                                                     (quaternion.x, quaternion.y,
                                                      quaternion.z, quaternion.w),
                                                     odom.header.stamp, "base_link", "base_footprint")

    def cmd_vel(self, msg):
        # TODO start here
        if self.is_connected:
            self.last_cmd_vel_time = rospy.Time.now()
            self.cmd_heading = self.normalize_angle_positive(
                math.atan2(msg.linear.x, msg.linear.y)) * 180 / math.pi
            self.cmd_speed = math.sqrt(
                math.pow(msg.linear.x, 2) + math.pow(msg.linear.y, 2))
            self.robot.roll(int(self.cmd_speed), int(
                self.cmd_heading), 1, False)

    def cmd_turn(self, msg):
        if self.is_connected:
            self.robot.roll(0, int(msg.data), 0, False)

    def set_color(self, msg):
        if self.is_connected:
            self.robot.set_rgb_led(
                int(msg.r * 255), int(msg.g * 255), int(msg.b * 255), 0, False)

    def set_back_led(self, msg):
        if self.is_connected:
            self.robot.set_back_led(int(msg.data), False)

    def set_stabilization(self, msg):
        if self.is_connected:
            if not msg.data:
                self.robot.set_stablization(1, False)
            else:
                self.robot.set_stablization(0, False)

    def set_heading(self, msg):
        if self.is_connected:
            heading_deg = int(self.normalize_angle_positive(
                msg.data) * 180.0 / math.pi)
            self.robot.set_heading(heading_deg, False)

    def set_angular_velocity(self, msg):
        if self.is_connected:
            rate = int((msg.data * 180 / math.pi) / 0.784)
            self.robot.set_rotation_rate(rate, False)

    def configure_collision_detect(self, msg):
        pass

    def reconfigure(self, config, level):
        if self.is_connected:
            self.robot.set_rgb_led(int(
                config['red'] * 255), int(config['green'] * 255), int(config['blue'] * 255), 0, False)
        return config


if __name__ == '__main__':
    s = SpheroNode()
    s.start()
    s.spin()
    s.stop()
