PACKAGE = 'kinbody_detector'
import prpy
import prpy.rave
import rospy
import os
import tf
import json
import numpy
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import quaternion_matrix, euler_from_matrix, euler_matrix

OBJ_HEIGHT ={'table' : 0.7384871317}

class DetectorException(Exception):
    pass

class KinBodyDetector(object):
    def __init__(self,
                 env,
                 marker_data_path,
                 kinbody_directory,
                 marker_topic,
                 detection_frame='head/kinect2_rgb_optical_frame',
                 destination_frame='map',
                 reference_link=None):
        
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
        self.reference_link = reference_link
        self.listener = tf.TransformListener()
        
        self.ReloadKinbodyData()    
    
    def ReloadKinbodyData(self):
        with open(self.marker_data_path, 'r') as f:
            self.marker_data = json.load(f)
    
    def Update(self, timeout=10, **kw_args):
        marker_message = rospy.wait_for_message(self.marker_topic,
                                                MarkerArray,
                                                timeout=timeout)
        
        added_kinbodies = []
        updated_kinbodies = []

        # Example kw_args : {'snapToObject' : {'ns' : 127, 'name' : 'table', 'to_snap' : ['bowl135', 'obj2', ...]}, 'alignUpright' : ['table127', 'plastic_glass124', ...] }
        # We need the snapToObject value to be of a dictionary so we get the TagID separately
        # So we can compare with tagXYZ in the MarkerArray
        # For alignUpright we can just use the expected kinbody names
        marker_list = marker_message.markers
        
        snap_to_obj = False
        if 'snapToObject' in kw_args.keys():
            snap_to_obj = True
            snap_dict = kw_args['snapToObject']
            tag_ID = snap_dict['ns']
            objs_to_snap = snap_dict['to_snap']
            snap_marker_ns = 'tag'+`tag_ID`
            snap_obj_name = snap_dict['name']

            marker_list = [marker for marker in marker_message.markers if marker.ns == snap_marker_ns]
            marker_list.extend([marker for marker in marker_message.markers if marker.ns != snap_marker_ns])
            snap_z_origin= 0.0

        objs_to_align = None
        if 'alignUpright' in kw_args.keys():
            objs_to_align = kw_args['alignUpright']

        for marker in marker_list:
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
                        self.detection_frame,
                        self.destination_frame,
                        rospy.Time(),
                        rospy.Duration(timeout))
                frame_trans, frame_rot = self.listener.lookupTransform(
                        self.destination_frame,
                        self.detection_frame,
                        rospy.Time(0))
                frame_offset = numpy.matrix(quaternion_matrix(frame_rot))
                frame_offset[0,3] = frame_trans[0]
                frame_offset[1,3] = frame_trans[1]
                frame_offset[2,3] = frame_trans[2]
                
                kinbody_pose = numpy.array(numpy.dot(numpy.dot(frame_offset,marker_pose),
                                                     kinbody_offset))

                final_kb_pose = kinbody_pose

                #Transform w.r.t reference link if link present

                if snap_to_obj == True:
                    if marker.ns == snap_marker_ns:
                        snap_z_origin = final_kb_pose[2,3]
                    else:
                        final_kb_pose[2,3] = snap_z_origin + OBJ_HEIGHT[snap_obj_name]

                        
                kinbody_name = kinbody_file.replace('.kinbody.xml', '')
                kinbody_name = kinbody_name + str(marker.id)

                if objs_to_align is not None:
                    if kinbody_name in objs_to_align:
                        ax,ay,az = euler_from_matrix(final_kb_pose[:3,:3])
                        #Align the kinbody upright
                        if kinbody_name.startswith('table'):
                            #Table's identity pose is flipped (NEED TO CORRECT)
                            ax = numpy.pi/2.0
                            ay = 0.0
                        else:
                            ay = 0.0
                        new_rot_mat = euler_matrix(ax,ay,az)
                        for i in range(3):
                            for j in range(3):
                                final_kb_pose[i,j] = new_rot_mat[i,j]

                if snap_to_obj == True:
                    if kinbody_name in objs_to_snap:
                        final_kb_pose[2,3] = snap_z_origin + OBJ_HEIGHT[snap_obj_name]
                
                # load the object if it does not exist
                if self.env.GetKinBody(kinbody_name) is None:
                    new_body = prpy.rave.add_object(
                            self.env,
                            kinbody_name,
                            os.path.join(self.kinbody_directory, kinbody_file))
                    added_kinbodies.append(new_body)
                    self.generated_bodies.append(new_body)
                
                body = self.env.GetKinBody(kinbody_name)

                body.SetTransform(final_kb_pose)

                if snap_to_obj == True:
                    if kinbody_name in objs_to_snap:
                        # Crazy loop to get out of collision
                        snap_obj_tag_name = snap_obj_name+`tag_ID`
                        snap_kb_obj = self.env.GetKinBody(snap_obj_tag_name)
                        while self.env.CheckCollision(snap_kb_obj,body) == True:
                            final_kb_pose[2,3] += 0.01
                            body.SetTransform(final_kb_pose)

                updated_kinbodies.append(body)
        
        return added_kinbodies, updated_kinbodies
