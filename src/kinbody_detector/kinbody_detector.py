PACKAGE = 'kinbody_detector'
import prpy
import rospy
import os
import tf
import json
import numpy
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import quaternion_matrix

class DetectorException(Exception):
    pass

class KinBodyDetector(object):
    def __init__(self,
                 env,
                 marker_data_path,
                 kinbody_directory,
                 marker_topic,
                 detection_frame='tf_kinect2_rgb',
                 destination_frame='map'):
        
        # Initialize a new ros node if one has not already been created
        try:
            rospy.init_node('kinbody_detector', anonymous=True)
        except rospy.exceptions.ROSException:
            pass
        
        self.env = env
        self.marker_data_path = marker_data_path
        self.kinbody_directory = kinbody_directory
        self.marker_topic = marker_topic
        self.detection_frame = detection_frame
        self.destination_frame = destination_frame
        self.generated_bodies = []
        
        self.listener = tf.TransformListener()
        
        self.ReloadKinbodyData()    
    
    def ReloadKinbodyData(self):
        with open(self.marker_data_path, 'r') as f:
            self.marker_data = json.load(f)
    
    def Update(self, timeout=10):
        marker_message = rospy.wait_for_message(self.marker_topic,
                                                MarkerArray,
                                                timeout=timeout)
        
        added_kinbodies = []
        updated_kinbodies = []
        
        for marker in marker_message.markers:
            if marker.ns in self.marker_data:
                kinbody_file, kinbody_offset = self.marker_data[marker.ns]
                kinbody_offset = numpy.array(kinbody_offset)
                marker_pose = numpy.array(quaternion_matrix([
                        marker.pose.orientation.x,
                        marker.pose.orientation.y,
                        marker.pose.orientation.z,
                        marker.pose.orientation.w]))
                marker_pose[0,3] = marker.pose.position.x
                marker_pose[1,3] = marker.pose.position.y
                marker_pose[2,3] = marker.pose.position.z
                
                self.listener.waitForTransform(
                        self.from_frame,
                        self.to_frame,
                        rospy.Time(),
                        rospy.Duration(timeout))
                frame_trans, frame_rot = self.listener.lookupTransform(
                        self.from_frame,
                        self.to_frame,
                        rospy.Time(0))
                frame_offset = numpy.matrix(quaternion_matrix(frame_rot))
                frame_offset[0,3] = frame_trans[0]
                frame_offset[1,3] = frame_trans[1]
                frame_offset[2,3] = frame_trans[2]
                
                kinbody_pose = numpy.dot(numpy.dot(frame_offset,marker_pose),
                                         kinbody_offset)
                
                kinbody_name = kinbody_file.replace('.kinbody.xml', '')
                kinbody_name = kinbody_name + str(marker.id)
                
                # load the object if it does not exist
                if self.env.GetKinBody(kinbody_name) is None:
                    new_body = prpy.rave.add_object(
                            self.env,
                            kinbody_name,
                            os.path.join(self.kinbody_directory, kinbody_file))
                    added_kinbodies.append(new_body)
                    self.generated_bodies.append(new_body)
                
                body = self.env.GetKinBody(kinbody_name)
                body.SetTransform(kinbody_pose)
                updated_kinbodies.append(body)
        
        return added_kinbodies, updated_kinbodies
