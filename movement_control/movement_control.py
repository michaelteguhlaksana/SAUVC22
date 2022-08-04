#!/usr/bin/env python2

'''
Library for handling the AUV
Uses MAVROS and ArduSub

Reference: 
	https://github.com/masoudir/mavros_python_examples
	https://github.com/PX4/PX4-Autopilot/tree/master/integrationtests/python_src/px4_it/mavros
'''

from __future__ import division
import threading
from six.moves import xrange
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from mavros_msgs.msg import Altitude, ParamValue, State, AttitudeTarget
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode, SetModeRequest

from sensor_msgs.msg import Imu
from std_msgs.msg import Header

import rospy


#----


class AUVHandler(object):
	def __init__(self):
		self.rate = 1
		self.connected = False
		self.mode = "STABILIZED"

		self.altitude = Altitude()
		self.imu_data = Imu()
		self.local_position = PoseStamped()
		self.state = State()

		self.att = AttitudeTarget()

		#For checking whteher each subscribed topic is ready to recieve messages
		self.sub_topics_ready = {
			key: False
			for key in [
				'alt', 'local_pos', 'mission_wp', 'state', 'imu'
			]
		}

		# ROS services
		service_timeout = 30
		rospy.loginfo("waiting for ROS services")
		try:
			rospy.wait_for_service('mavros/param/get', service_timeout)
			rospy.wait_for_service('mavros/param/set', service_timeout)
			rospy.wait_for_service('mavros/cmd/arming', service_timeout)
			rospy.wait_for_service('mavros/set_mode', service_timeout)
			rospy.loginfo("ROS services are up")
		except rospy.ROSException:
			self.fail("failed to connect to services")

		self.get_param_srv = rospy.ServiceProxy('mavros/param/get', ParamGet)
		self.set_param_srv = rospy.ServiceProxy('mavros/param/set', ParamSet)
		self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
		self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)

		 # ROS subscribers
		self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude, self.altitude_callback)
		self.imu_data_sub = rospy.Subscriber('mavros/imu/data', Imu, self.imu_data_callback)
		self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_position_callback)
		self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)

		# ROS publishers
		self.att_setpoint_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)




	# Callback functions
	#
	def altitude_callback(self, data):
		self.altitude = data

		# amsl has been observed to be nan while other fields are valid
		if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
			self.sub_topics_ready['alt'] = True

	def imu_data_callback(self, data):
		self.imu_data = data

		if not self.sub_topics_ready['imu']:
			self.sub_topics_ready['imu'] = True

	def local_position_callback(self, data):
		self.local_position = data

		if not self.sub_topics_ready['local_pos']:
			self.sub_topics_ready['local_pos'] = True

	def state_callback(self, data):
		if self.state.armed != data.armed:
			rospy.loginfo("armed state changed from {0} to {1}".format(
				self.state.armed, data.armed))

		if self.state.connected != data.connected:
			rospy.loginfo("connected changed from {0} to {1}".format(
				self.state.connected, data.connected))

		if self.state.mode != data.mode:
			rospy.loginfo("mode changed from {0} to {1}".format(
				self.state.mode, data.mode))

		if self.state.system_status != data.system_status:
			rospy.loginfo("system_status changed from {0} to {1}".format(
				mavutil.mavlink.enums['MAV_STATE'][
					self.state.system_status].name, mavutil.mavlink.enums[
						'MAV_STATE'][data.system_status].name))

		self.state = data

		# mavros publishes a disconnected state message on init
		if not self.sub_topics_ready['state'] and data.connected:
			self.sub_topics_ready['state'] = True


	# Helper methods
	#
	def set_arm(self, arm, timeout):
		"""arm: True to arm or False to disarm, timeout(int): seconds"""

		rospy.loginfo("setting FCU arm: {0}".format(arm))
		old_arm = self.state.armed
		loop_freq = 1  # Hz
		rate = rospy.Rate(loop_freq)
		arm_set = False
		for i in xrange(timeout * loop_freq):
			if self.state.armed == arm:
				arm_set = True
				rospy.loginfo("set arm success | seconds: {0} of {1}".format(
					i / loop_freq, timeout))
				break
			else:
				try:
					res = self.set_arming_srv(arm)
					if not res.success:
						rospy.logerr("failed to send arm command")
				except rospy.ServiceException as e:
					rospy.logerr(e)

			try:
				rate.sleep()
			except rospy.ROSException as e:
				rospy.logerr(e)

	def set_mode(self, mode, timeout):
		"""
		mode: ArduSub mode string, timeout(int): seconds

		modes for APM Copter: http://wiki.ros.org/mavros/CustomModes
		"""

		rospy.loginfo("setting FCU mode: {0}".format(mode))
		old_mode = self.state.mode
		loop_freq = 1  # Hz
		rate = rospy.Rate(loop_freq)
		mode_set = False
		for i in xrange(timeout * loop_freq):
			if self.state.mode == mode:
				mode_set = True
				rospy.loginfo("set mode success | seconds: {0} of {1}".format(
					i / loop_freq, timeout))
				break
			else:
				try:
					res = self.set_mode_srv(0, mode)  # 0 is custom mode 
					if not res.mode_sent:
						rospy.logerr("failed to send mode command")
				except rospy.ServiceException as e:
					rospy.logerr(e)

			try:
				rate.sleep()
			except rospy.ROSException as e:
				rospy.logerr(e)


	def set_param(self, param_id, param_value, timeout):
		"""
		param: ArduSub param string, ParamValue, timeout(int): seconds

		ArduSub Param list:  https://www.ardusub.com/developers/full-parameter-list.html
		"""

		if param_value.integer != 0:
			value = param_value.integer
		else:
			value = param_value.real
		
		rospy.loginfo("setting parameter: {0} with value {1}".
			format(param_id, value))

		loop_freq = 1  # Hz
		rate = rospy.Rate(loop_freq)
		param_set = False
		for i in xrange(timeout * loop_freq):
			try:
				res = self.set_param_srv(param_id, param_value)
				if res.success:
					rospy.loginfo("param {0} set to {1} | seconds: {2} of {3}".
						format(param_id, value, i / loop_freq, timeout))
				break
			except rospy.ServiceException as e:
				rospy.logerr(e)

			try:
				rate.sleep()
			except rospy.ROSException as e:
				rospy.logerr(e)


	def wait_for_topics(self, timeout):
		"""
		wait for all topics to be ready, make sure we're getting topic info
		from all topics by checking dictionary of flag values set in callbacks,
		timeout(int): seconds
		"""

		rospy.loginfo("waiting for subscribed topics to be ready")
		loop_freq = 1  # Hz
		rate = rospy.Rate(loop_freq)
		simulation_ready = False
		for i in xrange(timeout * loop_freq):
			if all(value for value in self.sub_topics_ready.values()):
				simulation_ready = True
				rospy.loginfo("Topics ready | seconds: {0} of {1}".
							  format(i / loop_freq, timeout))
				break

			try:
				rate.sleep()
			except rospy.ROSException as e:
				rospy.logerr(e)


	def send_att(self, func, *args):
		"""
		Sends attitude according to an external function
		The function must return 3 value, the target attitude, thrust and whether to disarm the AUV
		"""
		target, thrust, disarm = func(*args)
		tx, ty, tz = target
		rate = rospy.Rate(10)  # Hz
		self.att.body_rate = Vector3()
		self.att.header = Header()
		self.att.header.frame_id = "base_footprint"
		self.att.orientation = Quaternion(*quaternion_from_euler(tx, ty, tz))
		self.att.thrust = thrust
		self.att.type_mask = 7  # ignore body rate

		#SENDING COMMAND
		self.log_topic_vars()
		if disarm:
			rospy.loginfo("DISARMING ...")
			handler.set_arm(False, 10)

		else:
			self.att.header.stamp = rospy.Time.now()
			self.att_setpoint_pub.publish(self.att)
			try:  # prevent garbage in console output when thread is killed
				rate.sleep()
			except rospy.ROSInterruptException:
				pass


	def start_att_thread(self, func, *args):
		self.att_thread = threading.Thread(target=self.send_att, args=(func, args))
		self.att_thread.daemon = True
		self.att_thread.start()

	def log_topic_vars(self):
		"""log the state of topic variables"""
		rospy.loginfo("========================")
		rospy.loginfo("===== topic values =====")
		rospy.loginfo("========================")
		rospy.loginfo("altitude:\n{}".format(self.altitude))
		rospy.loginfo("========================")
		rospy.loginfo("local_position:\n{}".format(self.local_position))
		rospy.loginfo("========================")
		rospy.loginfo("state:\n{}".format(self.state))
		rospy.loginfo("========================")



