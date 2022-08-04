#!/usr/bin/env python2

from movement_control. movement_control import AUVHandler
import rospy
import time

"""
A file for testing ArduSub movement control

Send a straight heading
"""




rospy.init_node('test_node', anonymous=True)

handler = AUVHandler()

handler.wait_for_topics(5)
handler.set_mode("STABILIZE", 5)
handler.set_arm(True, 5)


handler.log_topic_vars()

START_TIME = time.time()

<<<<<<< HEAD
def const_att_thrust (att):
	"""
	Sends a constant heading and thrust for 10 seconds
	"""
	att = att[0]
	thrust = 0.7
	sec = time.time() - START_TIME
	rospy.loginfo("TIME:: {}".format(sec))
	if sec > 10:
		return (att, 0, True)
	else:
		return (att, thrust, False)

handler.start_att_thread(const_att_thrust, (-0.25, 0.15, 0))
=======
#handler.start_att_thread(const_att_thrust, ((-0.25, 0.15, 0), 0.7) )
while True:
	handler.send_att(const_att_thrust, ((-0.25, 0.15, 0), 0.7) )
>>>>>>> b7e4fe09f49e01707086029f415a42b98cdb32dd





