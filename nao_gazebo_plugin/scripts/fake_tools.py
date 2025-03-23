#!/usr/bin/env python3
import rospy
from robot_toolkit_msgs.srv import (
    motion_tools_srv, vision_tools_srv, misc_tools_srv, 
    tablet_service_srv, battery_service_srv, navigation_tools_srv, 
    audio_tools_srv, set_output_volume_srv
)

class NaoMockupNode:
    def __init__(self):
        rospy.init_node('nao_mockup_node', anonymous=True)
        
        # Set up mock services
        self.motion_tools = rospy.Service('/robot_toolkit/motion_tools_srv', motion_tools_srv, self.mock_response)
        self.vision_tools = rospy.Service('/robot_toolkit/vision_tools_srv', vision_tools_srv, self.set_up_vision_tools_callback)
        self.misc_tools = rospy.Service('/robot_toolkit/misc_tools_srv', misc_tools_srv, self.mock_response)
        self.navigation_tools = rospy.Service('/robot_toolkit/navigation_tools_srv', navigation_tools_srv, self.mock_response)
        self.audio_tools = rospy.Service('/robot_toolkit/audio_tools_srv', audio_tools_srv, self.set_up_audio_tools_callback)

    def mock_response(self, req):
        rospy.loginfo(f"Received request: {req}")
        return "OK"

    def set_up_audio_tools_callback(self, msg):
        print("set up audio tools")
        return str("OK"), msg.data.speech_parameters

    def set_up_vision_tools_callback(self, msg):
        print(msg)
        print("set up vision tools")
        return str("OK"), msg.data.camera_parameters
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    mockup_node = NaoMockupNode()
    mockup_node.run()