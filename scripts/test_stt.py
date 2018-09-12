#!/usr/bin/python

import rospy
from std_srvs.srv import SetBool
from lab_ros_speech_to_text.msg import(
    Speech
)

def stt_cb(msg):
    rospy.loginfo(msg)


def main():

    #enable STT
    rospy.init_node('stt_test_node')
    rospy.wait_for_service('toggle_stt')
   
    rospy.Subscriber('stt', Speech, stt_cb, queue_size=1)

    stt_toggle = rospy.ServiceProxy('toggle_stt', SetBool)
    resp = stt_toggle(True)
    rospy.loginfo('enabled STT: {}'.format(resp))
    rospy.sleep(10)
    stt_toggle(False)
    rospy.sleep(5)
    stt_toggle(True)
    rospy.sleep(10)
    stt_toggle(False)

if __name__ == '__main__':
    main()