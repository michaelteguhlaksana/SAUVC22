from movement_control. movement_control import AUVHandler
import rospy
import time

"""
A file for testing ArduSub movement control

Send a straight heading
"""


def const_att_thrust (att, thrust):
	"""
	Sends a constant heading and thrust for 10 seconds
	"""
	sec = time.time() - START_TIME
	if sec > 10:
		return (att, 0, True)
	else:
		return (att, thrust, False)


rospy.init_node('test_node', anonymous=True)

handler = AUVHandler()

handler.wait_for_topic(5)
handler.set_mode("STABILIZED", 5)
handler.set_arm(True, 5)


handler.log_topic_vars()

START_TIME = time.time()
handler.start_att_thread(const_att_thrust, ((-0.25, 0.15, 0), 0.7) )





