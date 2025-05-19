# cube_interfaces

This package defines custom message types to interface with the cube_detector package.

## Messages

- `DetectedCube.msg`: Contains a cube's color and 3D pose.
- `DetectedCubeArray.msg`: An array of `DetectedCube` messages with a shared header for frame and timestamp.

### DetectedCube.msg
```
string color
geometry_msgs/Pose pose
```

### DetectedCubeArray.msg
```
cube_interfaces/DetectedCube[] cubes
std_msgs/Header header
```

## Dependencies

- `std_msgs`
- `geometry_msgs`

## Usage

Other packages can use:

```python
from cube_interfaces.msg import DetectedCube, DetectedCubeArray
```
