# Tools Usage Guide: Intel RealSense (ROS2)

## Camera Control

### `start_camera(serial_number=None)`
Start the RealSense camera ROS2 node.

**Parameters**:
- `serial_number`: Optional specific camera S/N

---

### `stop_camera()`
Stop the camera node.

---

### `capture_frame()`
Capture a single frame from all enabled streams.

**Returns**: Frame data with color and depth.

---

## Configuration

### `set_exposure(exposure_ms)`
Set camera exposure time in milliseconds.

### `align_depth_to_color(enable)`
Enable or disable depth-to-color alignment.

### `get_camera_info()`
Get camera intrinsics and calibration data.
