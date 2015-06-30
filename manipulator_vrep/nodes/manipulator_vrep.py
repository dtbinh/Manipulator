#!/usr/bin/env python
import rospy
import collections
from threading import Thread
from vrep_common.srv import *
from sensor_msgs.msg import *
from std_msgs.msg import *

class ManipulatorVrep:
    def __init__(self):
        get_object_handle_service = rospy.ServiceProxy('/vrep/simRosGetObjectHandle', simRosGetObjectHandle)
        enable_vrep_publisher_service = rospy.ServiceProxy('vrep/simRosEnablePublisher', simRosEnablePublisher)
        start_simulation_service = rospy.ServiceProxy('/vrep/simRosStartSimulation', simRosStartSimulation)
        self.set_joint_target_position_service = rospy.ServiceProxy('/vrep/simRosSetJointTargetPosition', 
                                                                    simRosSetJointTargetPosition)
        if self.start_simulation(start_simulation_service):
            rospy.loginfo("Linked with V-Rep")
            self.set_params(get_object_handle_service)
            self.enable_vrep_publishers(self.handles, enable_vrep_publisher_service)
            rospy.Subscriber("desired_joint_angles_topic", JointState, self.handle_desired_joint_states)
            rospy.logwarn(self.handles)
        rospy.spin()
        
    def set_params(self, get_object_handle_service):
        """
        Sets the params for the simulator node
        """        
        self.handles = self.get_object_handles(get_object_handle_service)
        
    def on_shutdown(self):
        """
        Stops the simulation service on shutdown
        """
        self.stop_simulation_service()
        
    def handle_desired_joint_states(self, request):
        rospy.logwarn(request)
        for i in xrange(len(request.name)):           
            self.set_joint_target_position_service(handle=self.handles[request.name[i]],
                                                   targetPosition=request.position[i])
        
        
    def start_simulation(self, start_simulation_service):
        """
        Starts the simulation service
        """
        rospy.logwarn("Sim: Starting simulation")
        try:
            start_simulation_service()
            return True
        except:
            rospy.logerr("Sim: Failed to call the simulation service. Have you forgotten to start V-Rep or have you started V-Rep after lauching a new roscore?")
        return False
    
    def get_object_handles(self, get_object_handle_service):
        """
        Gets the vrep object handles for the joints
        """        
        handles = {}        
        joints = ["joint1", "joint2", "joint3"]
        for joint in joints:
            handles[joint] = get_object_handle_service(objectName=joint).handle 
        rospy.logwarn("handles " + str(handles))           
        return handles 
    
    def enable_vrep_publishers(self, handles, enable_vrep_publisher_service):
        """
        Enables the joint pose vrep publishers and the /vrep/joint_states publisher
        """
        for key in handles.keys():
            rospy.logwarn(enable_vrep_publisher_service(topicName=key + "_pose_topic",
                                                        queueSize=1,
                                                        streamCmd=8193,
                                                        auxInt1=handles[key],
                                                        auxInt2=-1).effectiveTopicName)
        rospy.logwarn("+++++++++++++++++++++++++++++++++")
        rospy.logwarn(enable_vrep_publisher_service(topicName="joint_states",
                                                     queueSize=1,
                                                     streamCmd=4102,
                                                     auxInt1=-2,                                                     
                                                     auxInt2=-1)) 


   

if __name__ == "__main__":
    rospy.init_node('manipulator_vrep_node')
    ManipulatorVrep()