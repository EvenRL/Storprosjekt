# cube_detector

This package contains a node for detecting and estimating the pose of colored cubes using a camera.

## Nodes

### `cube_detector.py`

- Detects and estimates pose of colored cubes.
- Publishes `DetectedCubeArray` messages on the `/detected_cubes` topic.
- Each cube includes pose in `camera_frame` and color data.

## Parameters
The node has many parameters which can be edited in the parameters files under /config/.

## Dependencies

- `cube_interfaces`
- `rclpy`
- `opencv`