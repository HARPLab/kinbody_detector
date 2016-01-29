#!/usr/bin/env python
PACKAGE = 'kinbody_detector'
import argparse, herbpy, numpy, openravepy, os, time, prpy, json
import apriltag_kinbody as ak
from openravepy import *
import IPython

class TagException(Exception):
    pass

def write_tag_offset(path, name, index, offset):
    key = 'tag%i'%index
    data = {}
    if os.path.exists(path):
        with open(path, 'r') as f:
            data = json.load(f)
    data[key] = [name, offset.tolist()]
    
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)

def read_tag_offset(path, index):
    key = 'tag%i'%index
    if not os.path.exists(path):
        raise TagException, ('The offset path you specified at the '
                'command line does not exist yet.  You must write it first.')
    
    with open(path, 'r') as f:
        data = json.load(f)
        if key not in data:
            raise TagException, ('The tag index you specified at the '
                    'command line does not have an entry in this file.')
        
        return numpy.array(data[key][1])

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=('Loads a kinbody and '
            'creates an apriltag kinbody so that you can specify the tag '
            'location on an object.'))
    parser.add_argument('-b', '--body', dest='body', type=str,
            help='The body you wish to tag')
    parser.add_argument('-f', '--family', dest='family', type=str,
            default='36h11',
            help='The tag family, the default is "36h11"')
    parser.add_argument('-i', '--index', dest='index', type=int, default=0,
            help='The tag index you wish to attach')
    parser.add_argument('-w', '--width', dest='width', type=float,
            default=0.048,
            help='The width of the black part of the apriltag in meters')
    parser.add_argument('-t', '--thickness', dest='thickness', type=float,
            default=0.001,
            help='The thicnkess of the tag in meters')
    parser.add_argument('-o', '--offsetpath', dest='offsetpath', type=str,
            help='The path to a json file that will store the offset data.')
    args = parser.parse_args()
    
    env = Environment()
    env.SetViewer('rviz')
    
    kinbody = env.ReadKinBodyXMLFile(args.body)
    env.Add(kinbody)
    
    family_parts = args.family.split('h')
    family = (int(family_parts[0]), int(family_parts[1]))
    
    tag_kb_path = ak.BuildApriltagKinbody(family=family,
                                          index=args.index,
                                          width=args.width,
                                          thickness=args.thickness,
                                          name='tag',
                                          path='/tmp')
    
    tag_kinbody = env.ReadKinBodyXMLFile(tag_kb_path)
    env.Add(tag_kinbody)
    
    def write():
        tag_xform = tag_kinbody.GetTransform()
        kinbody_xform = kinbody.GetTransform()
        
        offset = numpy.dot(numpy.linalg.inv(tag_xform), kinbody_xform)
        write_tag_offset(args.offsetpath,
                         os.path.split(args.body)[1],
                         args.index, offset)
    
    def read():
        kinbody_xform = kinbody.GetTransform()
        offset = read_tag_offset(args.offsetpath, args.index)
        tag_kinbody.SetTransform(numpy.dot(kinbody_xform,
                                           numpy.linalg.inv(offset)))
    
    print ''
    print ''
    print 'Run the "write()" command to write the offset to the offset file.'
    print ('Run the "read()" command to read the offset from the file and '
            'apply it.')
    print 'Run the "exit() command when finished.'
    print ''
    print ''
    
    # Attempt to load the current offset
    try:
        read()
    except TagException:
        pass
    
    IPython.embed()
    
