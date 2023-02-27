#!/usr/bin/env python
import os
import rospy
import rospkg

from python_qt_binding import loadUi
try:
    # Starting from Qt 5 QWidget is defined in QtWidgets and not QtGui anymore
    from python_qt_binding.QtWidgets import QWidget
    from python_qt_binding.QtGui import QImage, QPixmap
except:
    from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import QTimer, Slot
from python_qt_binding.QtCore import pyqtSlot

import geometry_msgs.msg as geometry_msgs
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import std_msgs.msg as std_msgs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import math
import numpy as np
import cv2

class AutopilotWidget(QWidget):

    def __init__(self, parent):
        # Init QWidget
        super(AutopilotWidget, self).__init__(parent)
        self.setObjectName('Autopilot Widget')

        # set variables
        self._quad_namespace = None
        self._connected = False

        # self._autopilot_feedback_sub = None
        # self._autopilot_feedback = quadrotor_msgs.AutopilotFeedback()
        # self._autopilot_feedback_stamp = rospy.Time.now()

        self._send_setpoint_enu_pub = None
        self._cmd_pub =None 

        self._video_sub = None
        self._video_msg = None
        self._video_sub_stamp = rospy.Time.now()
        self._cv_bridge = CvBridge()
        self.video_flag = False

        self._pose_sub = None
        self._pose_msg = None
        self._pose_sub_stamp = rospy.Time.now()
        self.pose_flag = False

        self.label_pose_content_1 = 'Pose(ryp): not available'
        self.label_pose_content_2 = 'Pose(ryp): not available'
        self.label_pose_content_3 = 'Pose(ryp): not available'
        self.label_pose_content_4 = 'Pose(ryp): not available'
        self.label_pose_none = 'Pose(ryp): not available'
		# load UI
        ui_file = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            '../../resource/fixedwing_widget.ui')
        loadUi(ui_file, self)

        # Timer
        self._update_info_timer = QTimer(self)
        self._update_info_timer.timeout.connect(self.update_gui)
        self._update_info_timer.start(100)

        self.disconnect()


    def video_sub_cb(self, msg):
        self.video_flag = True
        self._video_msg = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        self._video_sub_stamp = rospy.Time.now()

    
    def pose_sub_cb(self, msg):
        self.pose_flag = True
        self._pose_msg = msg
        self._pose_sub_stamp = rospy.Time.now()


    def video_available(self):
        if (rospy.Time.now() - self._video_sub_stamp).to_sec() <= 1.0:
            return True
        return False

    
    def pose_available(self):
        if (rospy.Time.now() - self._pose_sub_stamp).to_sec() <= 1.0:
            return True
        return False        

    
    def connect(self, quad_namespace):
        self._quad_namespace = quad_namespace

        self._send_setpoint_enu_pub = rospy.Publisher('/xtdrone/plane_0/cmd_pose_enu',Pose, queue_size=1)
        self._cmd_pub =rospy.Publisher('/xtdrone/plane_0/cmd',String,queue_size=1)

        self._video_sub = rospy.Subscriber('/airsim_node/drone_1/front_center_custom/Scene', Image, self.video_sub_cb, queue_size=10)
        self._video_sub = None
        self._pose_sub = None

        self.button_take_off.setEnabled(True)
        self.button_return.setEnabled(True)
        self.button_auto_land.setEnabled(True)
        self.button_off.setEnabled(True)
        self.button_land_fly.setEnabled(True)
        self.button_land_glide.setEnabled(True)
        self.button_setpoint_circle.setEnabled(True)
        self.button_setpoint_glide.setEnabled(True)
        self.button_go_to_pose.setEnabled(True)

        self._connected = True

    def disconnect_pub_sub(self, pub):
        if pub is not None:
            pub.unregister()
            pub = None

    def disconnect(self):
        self.disconnect_pub_sub(self._send_setpoint_enu_pub)

        self.button_take_off.setEnabled(False)
        self.button_return.setEnabled(False)
        self.button_auto_land.setEnabled(False)
        self.button_off.setEnabled(False)
        self.button_land_fly.setEnabled(False)
        self.button_land_glide.setEnabled(False)
        self.button_setpoint_circle.setEnabled(False)
        self.button_setpoint_glide.setEnabled(False)
        self.button_go_to_pose.setEnabled(False)
        self.ImageLabel_video
        self._connected = False


    def update_gui(self):
        # if (self._connected):
        if (self._connected and self.video_available() and self.video_flag):
            frame = cv2.cvtColor(self._video_msg, cv2.COLOR_BGR2RGB)
            height, width, bytesPerComponent = frame.shape
            bytesPerLine = bytesPerComponent * width
            q_image = QImage(frame.data,  width, height, bytesPerLine, 
                            QImage.Format_RGB888).scaled(self.ImageLabel_video.width(), self.ImageLabel_video.height())
            self.ImageLabel_video.setPixmap(QPixmap.fromImage(q_image)) 
        
        # if (self._connected):
        if (self._connected and self.pose_available() and self.pose_flag):
            self.label_pose_content_1 = self.label_pose_content_2
            self.label_pose_content_2 = self.label_pose_content_3
            self.label_pose_content_3 = self.label_pose_content_4
            self.label_pose_content_4 = 'Pose(ryp):[%(x)+.2f, y:%(y)+.2f, z:%(z)+.2f]' % {'x' : self._pose_msg.x, 'y' : self._pose_msg.y, 'z' : self._pose_msg.z}
            self.label_pose_1.setText(self.label_pose_content_1)      
            self.label_pose_2.setText(self.label_pose_content_2)    
            self.label_pose_3.setText(self.label_pose_content_3)    
            self.label_pose_4.setText(self.label_pose_content_4)  
        else:
            self.label_pose_1.setText(self.label_pose_none)
            self.label_pose_2.setText(self.label_pose_none)
            self.label_pose_3.setText(self.label_pose_none)
            self.label_pose_4.setText(self.label_pose_none)  



    @Slot(bool)
    def on_button_connect_clicked(self):
        if(self._connected):
            self.disconnect()
            self.button_connect.setText('Connect')
        else:
            quad_namespace = self.namespace_text.text()
            self.connect(quad_namespace)
            self.button_connect.setText('Disconnect')

    @Slot(bool)
    def on_button_take_off_clicked(self):
        self._cmd_pub.publish('AUTO.TAKEOFF')
        self._cmd_pub.publish('ARM')		
        self._cmd_pub.publish('OFFBOARD')

    @Slot(bool)
    def on_button_return_clicked(self):
        self._cmd_pub.publish('AUTO.RTL')

    @Slot(bool)
    def on_button_auto_land_clicked(self):
        self._cmd_pub.publish('AUTO.LAND')

    @Slot(bool)
    def on_button_land_fly_clicked(self):
        self._cmd_pub.publish('loiter')

    @Slot(bool)
    def on_button_land_glide_clicked(self):
        self._cmd_pub.publish('idle')

    @Slot(bool)
    def on_button_go_to_pose_clicked(self):
        try:
            go_to_pose_msg = geometry_msgs.PoseStamped()
            go_to_pose_msg.pose.position.x = float(self.go_to_pose_x.text())
            go_to_pose_msg.pose.position.y = float(self.go_to_pose_y.text())
            go_to_pose_msg.pose.position.z = float(self.go_to_pose_z.text())

            heading = float(self.go_to_pose_heading.text()) / 180.0 * math.pi

            go_to_pose_msg.pose.orientation.w = math.cos(heading / 2.0)
            go_to_pose_msg.pose.orientation.z = math.sin(heading / 2.0)

            self._send_setpoint_enu_pub.publish(go_to_pose_msg)
        except:
            rospy.logwarn("Could not read and send go to pose message!")

    # def autopilot_feedback_available(self):
    #     if (rospy.Time.now() - self._autopilot_feedback_stamp).to_sec() <= 1.0:
    #         return True
    #     return False

    # def get_autopilot_state_name(self, autopilot_state):
    #     if (autopilot_state == self._autopilot_feedback.START):
    #         return "START"
    #     if (autopilot_state == self._autopilot_feedback.HOVER):
    #         return "HOVER"
    #     if (autopilot_state == self._autopilot_feedback.LAND):
    #         return "LAND"
    #     if (autopilot_state == self._autopilot_feedback.EMERGENCY_LAND):
    #         return "EMERGENCY_LAND"
    #     if (autopilot_state == self._autopilot_feedback.BREAKING):
    #         return "BREAKING"
    #     if (autopilot_state == self._autopilot_feedback.GO_TO_POSE):
    #         return "GO_TO_POSE"
    #     if (autopilot_state == self._autopilot_feedback.VELOCITY_CONTROL):
    #         return "VELOCITY_CONTROL"
    #     if (autopilot_state == self._autopilot_feedback.REFERENCE_CONTROL):
    #         return "REFERENCE_CONTROL"
    #     if (autopilot_state == self._autopilot_feedback.TRAJECTORY_CONTROL):
    #         return "TRAJECTORY_CONTROL"
    #     if (autopilot_state == self._autopilot_feedback.COMMAND_FEEDTHROUGH):
    #         return "COMMAND_FEEDTHROUGH"
    #     if (autopilot_state == self._autopilot_feedback.RC_MANUAL):
    #         return "RC_MANUAL"
    #     return "OFF"

    # def get_battery_state_name(self, battery_state):
    #     if (battery_state == self._autopilot_feedback.low_level_feedback.BAT_GOOD):
    #         return "Good"
    #     if (battery_state == self._autopilot_feedback.low_level_feedback.BAT_LOW):
    #         return "Low"
    #     if (battery_state == self._autopilot_feedback.low_level_feedback.BAT_CRITICAL):
    #         return "Critical"
    #     return "Invalid"

    # def get_control_mode_name(self, control_mode):
    #     if (control_mode == self._autopilot_feedback.low_level_feedback.ATTITUDE):
    #         return "Attitude"
    #     if (control_mode == self._autopilot_feedback.low_level_feedback.BODY_RATES):
    #         return "Body Rates"
    #     if (control_mode == self._autopilot_feedback.low_level_feedback.ANGULAR_ACCELERATION):
    #         return "Angular Accelerations"
    #     if (control_mode == self._autopilot_feedback.low_level_feedback.ROTOR_THRUSTS):
    #         return "Rotor Thrusts"
    #     if (control_mode == self._autopilot_feedback.low_level_feedback.RC_MANUAL):
    #         return "RC_Manual"
    #     return "None"

    # def quat_to_euler_angles(self, q):
    #     #  Computes the euler angles from a unit quaternion using the
    #     #  z-y-x convention.
    #     #  euler_angles = [roll pitch yaw]'
    #     #  A quaternion is defined as q = [qw qx qy qz]'
    #     #  where qw is the real part.

    #     euler_angles = np.zeros((3, 1))

    #     euler_angles[0] = np.arctan2(
    #         2*q.w*q.x + 2*q.y*q.z, q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
    #     euler_angles[1] = -np.arcsin(2*q.x*q.z - 2*q.w*q.y)
    #     euler_angles[2] = np.arctan2(
    #         2*q.w*q.z + 2*q.x*q.y, q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
    #     return euler_angles
