# lab_ros_speech_to_text
Xiang Zhi Tan - 2018 - zhi.tan@ri.cmu.edu

ROS Speeh-To-Text module. The system uses Google Cloud Speech API to do speech recognition from the default microphone. You will
first need to enable the STT using the ROS service call at `/toggl_stt`.
The service calls takes a `std_srvs/SetBool` service message. Once the STT is enabled, it will start publishing
`lab_ros_speech_to_text\Speech.msg` on the `\stt` topic.


### Dependencies
Make sure you the packages listed in `requirement.txt`. To install them, do `pip install -r requirement.txt`

### Run
1. First change `google_stt.launch` such that the `GOOGLE_APPLICATION_CREDENTIALS` points to your credential and `PYTHONPATH` to the correct python environment.
2. Launch the launch file `roslaunch lab_ros_speech_to_text google_stt.launch`

### Changelog
* 2018/09 - Added ability to toggle STT on and off.