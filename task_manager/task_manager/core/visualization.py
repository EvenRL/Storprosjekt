#from typing import Dict, List
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

def create_cube_markers(detected_cubes, output_frame, now):
    markers = []  # Return a list of markers instead of MarkerArray

    for idx, (color, pose) in enumerate(detected_cubes.items()):
        m = Marker()
        m.header = Header(stamp=now, frame_id=output_frame)
        m.ns = 'cubes'
        m.id = idx
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose = pose
        m.scale.x = 0.05
        m.scale.y = 0.05
        m.scale.z = 0.05
        colors = {
            'red': [1.0, 0.0, 0.0, 1.0],
            'yellow': [1.0, 1.0, 0.0, 1.0],
            'blue': [0.0, 0.0, 1.0, 1.0],
            'green': [0.0, 1.0, 0.0, 1.0],
            'black': [0.1, 0.1, 0.1, 1.0]
        }
        rgba = colors.get(color, [0.5, 0.5, 0.5, 1.0])
        # Set color values
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        # Set lifetime to prevent old markers from lingering
        m.lifetime.sec = 1
        markers.append(m)
    return markers  # Return list of markers


def create_pointing_marker(color, detected_cubes, output_frame, now):
    if color not in detected_cubes:
        return None

    pose = detected_cubes[color]
    # Create a line marker for pointing
    m = Marker()
    m.header = Header(stamp=now, frame_id=output_frame)
    m.ns = 'pointing'
    m.id = 0
    m.type = Marker.ARROW
    m.action = Marker.ADD
    # Set start point at a fixed offset from cube
    m.points.append(Point(
        x=pose.position.x - 0.2,
        y=pose.position.y,
        z=pose.position.z
    ))
    # Set end point at cube
    m.points.append(Point(
        x=pose.position.x,
        y=pose.position.y,
        z=pose.position.z
    ))
    # Set arrow properties
    m.scale.x = 0.01  # shaft diameter
    m.scale.y = 0.02  # head diameter
    m.scale.z = 0.03  # head length
    m.color.r = 1.0  # Pointing color (white)
    m.color.g = 1.0
    m.color.b = 1.0
    m.color.a = 0.8
    m.lifetime.sec = 1
    return m