# kinbody_detector
Builds kinbodies from MarkerArray 

This page describes the process for using apriltags to detect objects.

What you will need: 1. A physical object. 2. A corresponding kinbody. 3. An apriltag sticker or card.

Setup:

1. First determine the index of the apriltag. If you don't already know it, you can put it in front of Herb and run: rostopic echo /apriltags/marker\_array (assuming the apriltags node is running on HERB, if not, launch it from the herb\_launch package). The index is the id field:

  ...
  ns: tag114
  id: 114  <- This is the tag index
  type: 1
  ...

2. Checkout the kinbody_detector package.

3. Start a local roscore

4. Run:

 rosrun kinbody_detector tag_kinbody.py --body PATH/TO/MY.kinbody.xml --index INDEX --offsetpath PATH/TO/tag_data.json

–width WIDTH

The –index argument is the apriltag index that you found above. The –width argument is the size of the black square on the apriltag sticker or card in meters. The default is 0.048 which is the size of the stickers we most recently printed. The –body argument is the path to the kinbody xml file. The –offsetpath is the location of a json file that will be written to contain the offset from the tag pose to the body pose. You can use the same json file for more than one tag, so it may be a good idea to create a single file for your project.

5. When you run this, rviz should open and automatically load your kinbody plus a textured kinbody representing the apriltag. Move the apriltag to the position that you will affix the tag on the physical object.

6. Once you are happy with the placement, type:

 write()

in the python terminal. This will write the relative offset between the tag and the kinbody to the json file you specified. You can also restore the previously saved value by typing:

 read()

When you are finished, type:

 exit()

7. Continue this process for any other objects you wish to tag. Make sure you don't use the same tag on more than one object.

Detection:

1. Now that you have your offset file created, you convert apriltag detections to kinbodies in OpenRave. In herbpy, you can now run:

 import kinbody_detector.kinbody_detector as kd
 detector = kd.KinBodyDetector(env, '/PATH/TO/tag_data.json', '/PATH/TO/KINBODY/FOLDER', '/apriltags/marker_array')

This creates a detector object that you can use to update the environment with detected kinbodies.

2. To update the environment with the most recent apriltag detections run:

 detector.Update()

3. This system does not update live, so you must continue to run detector.Update() whenever you want to get a new snapshot of the physical world. 

### Tag Printing

Run this command to produce a printable PDF:

```console
$ convert tag36_11_00###.png -density 300 -interpolate Nearest -filter point -resize 708x708 tag36_11_00###.pdf
```
