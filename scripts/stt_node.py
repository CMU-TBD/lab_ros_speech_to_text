#!/usr/bin/python3

import sys
import time
import getopt
import alsaaudio
from google.cloud import speech
from lab_ros_speech_to_text.msg import Speech as Speech_msg
import io
import rospy
import threading
import actionlib
import signal

from std_srvs.srv import (
    SetBool,
    SetBoolResponse
)

#We create an audio stream object here
class audioStreamingObject():

    def __init__(self):
        self._inp = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NONBLOCK)
        self._inp.setchannels(1)
        self._inp.setrate(16000)
        self._inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
        self._inp.setperiodsize(80)
        self.closed = False

    def read(self, size):

        if self.closed:
            return None

        cur_l, cur_data = self._inp.read()
        while(cur_l < size):
            time.sleep(.001)
            l, data = self._inp.read()
            cur_l += l
            cur_data += data
        #print("returning data:{}".format(cur_l))
        return cur_data

    def close(self):
        self.closed = True


class AudioStreamingController():

    def __init__(self):

        self._stt_pub = rospy.Publisher('stt', Speech_msg, queue_size=10)
        self._enable_stt_serv = rospy.Service('toggle_stt', SetBool, self._toggle_stt)
        self._enable_flag = False
        self._enable_event = threading.Event()

    def _toggle_stt(self, req):
        self._enable_flag = req.data
        if self._enable_flag:
            self._enable_event.set()
        else:
            self._enable_event.clear()
        msg = "STT now {}".format("enabled" if self._enable_flag else "disabled")
        rospy.loginfo(msg)
        return SetBoolResponse(True, msg)

    def _close_loop(self, stream):
        for i in range(0,120):
            time.sleep(0.5)
            if rospy.is_shutdown() or not self._enable_flag:
                break
        stream.close()

    def spin(self):
        while not rospy.is_shutdown():
            #check if STT is enabled
            if self._enable_flag:
                #restart the client and sample
                stream = audioStreamingObject()
                #print(stream.closed)
                rospy.loginfo("start streaming audio to Google")
                #close thread
                self._close_thread = threading.Thread(target=self._close_loop, args=(stream,))
                self._close_thread.start()
                client = speech.Client()
                sample = client.sample(stream=stream,
                    encoding=speech.Encoding.LINEAR16,
                    sample_rate_hertz=16000)
                results = sample.streaming_recognize(
                    language_code='en-US',
                    interim_results=True
                )
                for result in results:
                    for alternative in result.alternatives:
                        msg = Speech_msg()
                        msg.text = str(alternative.transcript)
                        if alternative.confidence is not None:
                            msg.confidence = float(alternative.confidence)
                        else:
                            msg.confidence = 0
                        msg.is_final = result.is_final
                        self._stt_pub.publish(msg)
            
                if not rospy.is_shutdown() and self._enable_flag:
                    rospy.loginfo("restarting google speech api due to time limit")
            else:
                #wait for it to be enabled
                self._enable_event.wait()

def main():
    rospy.init_node('stt_node')
    asc = AudioStreamingController()

    def shutdown_handler():
        asc._enable_event.set()
    rospy.on_shutdown(shutdown_handler)
    
    rospy.loginfo('initialization complete')
    asc.spin()
   

if __name__ == '__main__':
    main()