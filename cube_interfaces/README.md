# cube_interfaces

This package defines custom message types to interface with the cube_detector package.

## Messages

- `DetectedCube.msg`: Contains a cube's color and 3D pose.
- `DetectedCubeArray.msg`: An array of `DetectedCube` messages with a shared header for frame and timestamp.
- `ColorRange.msg`: Contains lower and upper color ranges for a color in hsv format.
- `ColorRangeArray.msg`: An array of `ColorRange`

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

### ColorRange.msg
```
string name
uint8[3] lower
uint8[3] upper
```

### DetectedCubeArray.msg
```
cube_interfaces/ColorRange[] colors
```

## Dependencies

- `std_msgs`
- `geometry_msgs`

## Usage

Other packages can use:

```python
from cube_interfaces.msg import DetectedCube, DetectedCubeArray, ColorRange, ColorRangeArray
```
