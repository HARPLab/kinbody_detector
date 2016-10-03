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

OBJ_HEIGHT ={'table127' : 0.7384871317}

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
        
        '''
        Class to detect kinbodies - called by the apriltags detector 
        @param env The OpenRAVE environment
        @param marker_data_path A datafile which maps object tag IDs to the relative
            transform between the tag location on the object and the object's origin 
        @param kinbody_directory The parent folder where all kinbody.xml files exist
            (typically in pr-ordata)
        @param marker_topic The name of the ROS topic which publishes information
            about April Tag markers
        @param detection_frame The TF coordinate frame for the sensor 
        @param destination_frame The TF coordinate frame in which to get the detected pose
        @param reference_link The KinBody link relative to which the pose is returned
        '''  

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

    def AlignUpright(self,kinbody_name,kinbody_pose):
        '''
        Aligns the given kinbody with the world z_axis
        Special logic if it is the table kinbody

        @param kinbody_name The name of the body
        @param kinbody_pose The current pose of the kinbody
        @return The new 3x3 rotation matrix
        '''
        r = numpy.arctan2(kinbody_pose[1,0], kinbody_pose[0,0])

        if kinbody_name.startswith('table'):
            new_rot_mat = numpy.array([[ numpy.cos(r), 0.,  numpy.sin(r)],
                                      [ numpy.sin(r), 0., -numpy.cos(r)],
                                      [           0., 1.,           0.]])
        else:
            new_rot_mat = numpy.array([[numpy.cos(r), -numpy.sin(r), 0.],
                                      [numpy.sin(r),  numpy.cos(r), 0.],
                                      [          0.,            0., 1.]])

        return new_rot_mat

    def SnapTo(self,body,snap_body_name):
        '''
        Snaps the kinbody passed to it to a named object. It sets the height of
        the body to the current z value of the named object plus an offset looked up
        (currently from a global list at the top of this file)

        @param body The kinbody object to snap
        @param snap_body_name The name of the object on which to snap 'body'

        No return type - the pose of 'body' is updated internally
        '''
        snap_kb_obj = self.env.GetKinBody(snap_body_name)
        snap_body_pose = snap_kb_obj.GetTransform()
        kb_pose = body.GetTransform()

        # Adjust to origin of snap_body
        snap_z_origin = snap_body_pose[2,3]
        kb_pose[2,3] = snap_z_origin + OBJ_HEIGHT[snap_body_name]
        body.SetTransform(kb_pose)

        # Ensure out of collision
        while self.env.CheckCollision(snap_kb_obj,body) == True:
            kb_pose[2,3] += 0.01
            body.SetTransform(kb_pose)


    
    def ReloadKinbodyData(self):
        '''
        Opens and loads the file that has tag mapping data
        '''
        with open(self.marker_data_path, 'r') as f:
            self.marker_data = json.load(f)
    
    def Update(self, timeout=10, **kw_args):
        '''
        Updates kinbody detections from the next message on the
        April Tags marker topic.
        @param timeout How long to wait for the next message on the AprilTags topic
        The kw_args expects a nested dictionary with the following keys:
        snapToObject - Specifies an object on which other kinbodies will be 
            snapped, i.e. the z-coordinates aligned. Typically this is the table.
            Its value is also a dictionary with the following keys:
                ns - tag namespace for object to snap on
                name - name of object to snap on. 'name'+'ns' should be the unique ID
                to_snap - list of object IDs to snap. If list is ['all'], then all other
                    objects are snapped on 
        alignUpright - Specifies the objects whose z-axis is to be aligned with the 
            world Z axis. The value is a list of object IDs OR ['all'].
            NOTE that the table kinbody has a different axis alignment and so there
            is special logic to deal with it.

        Example kw_args : {'snapToObject' : {'ns' : 127, 'name' : 'table', 'to_snap' : ['all']}, 
                           'alignUpright' : ['table127', 'plastic_glass124'] }
        '''

        marker_message = rospy.wait_for_message(self.marker_topic,
                                                MarkerArray,
                                                timeout=timeout)
        
        added_kinbodies = []
        updated_kinbodies = []


        marker_list = marker_message.markers
        
        # Check if snapping to object is requested and do necessary bookkeeping
        # If snapping is required, the object on which to snap
        # is processed first so as to obtain it's z-coordinate
        snap_to_obj = False
        snapAll = False
        if 'snapToObject' in kw_args.keys():
            snap_to_obj = True
            snap_dict = kw_args['snapToObject']
            tag_ID = snap_dict['ns']
            snap_marker_ns = 'tag'+`tag_ID`
            snap_obj_name = snap_dict['name']

            objs_to_snap = snap_dict['to_snap']
            if objs_to_snap[0] == 'all':
                snapAll = True

            marker_list = [marker for marker in marker_message.markers if marker.ns == snap_marker_ns]
            marker_list.extend([marker for marker in marker_message.markers if marker.ns != snap_marker_ns])
            snap_z_origin= 0.0

        # Check if object alignment is requested and do necessary bookkeeping
        objs_to_align = None
        alignAll = False
        if 'alignUpright' in kw_args.keys():
            objs_to_align = kw_args['alignUpright']
            if objs_to_align[0] == 'all':
                alignAll = True

        for marker in marker_list:
            if marker.ns in self.marker_data:

                # Map kinbody to tag and get initial pose
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
                if self.reference_link is not None:
                    ref_link_pose = self.reference_link.GetTransform()
                    final_kb_pose = numpy.dot(ref_link_pose,kinbody_pose)

                # Load kinbody file
                kinbody_name = kinbody_file.replace('.kinbody.xml', '')
                kinbody_name = kinbody_name + str(marker.id)


                # Align upright block
                if objs_to_align is not None:
                    if alignAll == True or kinbody_name in objs_to_align:
                        final_kb_pose[0:3,0:3] = self.AlignUpright(kinbody_name,final_kb_pose)
                
                # load the object if it does not exist
                if self.env.GetKinBody(kinbody_name) is None:
                    new_body = prpy.rave.add_object(
                            self.env,
                            kinbody_name,
                            os.path.join(self.kinbody_directory, kinbody_file))
                    added_kinbodies.append(new_body)
                    self.generated_bodies.append(new_body)
                else:
                    body = self.env.GetKinBody(kinbody_name)
                    updated_kinbodies.append(body)
                
                body = self.env.GetKinBody(kinbody_name)
                body.SetTransform(final_kb_pose)

                # Snap to specified object - has to happen at end because body needed
                if snap_to_obj == True:
                    if (marker.ns != snap_marker_ns) and (kinbody_name in objs_to_snap or snapAll == True):
                        snap_obj_tag_name = snap_obj_name+`tag_ID`
                        self.SnapTo(body,snap_obj_tag_name)
        
        return added_kinbodies, updated_kinbodies
