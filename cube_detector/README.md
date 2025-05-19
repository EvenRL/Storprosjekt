# cube_detector

This package contains a node for detecting and estimating the pose of colored cubes using a camera.

## Nodes

### `cube_detector.py`

- Detects and estimates pose of colored cubes.
- Publishes `DetectedCubeArray` messages on the `/detected_cubes` topic.
- Each cube includes pose (in `camera_link`) and color metadata.

## Parameters


## Dependencies

- `cube_interfaces`
- `rclpy`