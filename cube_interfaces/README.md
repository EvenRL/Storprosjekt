# cube_interfaces

This package defines custom message types for the cube_detector package.

## Messages

- `ColorCube.msg`: Contains a cube's color, detection status and 3D pose.
- `ColorCubeArray.msg`: An array of `ColorCube` messages with a shared header for frame and timestamp.

## Dependencies

- `std_msgs`
- `geometry_msgs`

## Usage

Other packages can use:

```python
from cube_interfaces.msg import ColorCube, ColorCubeArray