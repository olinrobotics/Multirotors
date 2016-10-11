### Photogrammetry

### Scripts

- calculate_dimensions.py

This code implements the basic trigonometric calculation of dimensions in the Z-plane, with fixed known parameters.

- calibration.py

Calibrates the camera (i.e. obtains its intrinsic parameters, etc.) by first collecting samples and processing them.

- mission_image_capture.py

Not Implemented. This code is meant to collect image data during the mission and couple it with altitude information from the vehicle.

- mission_post_processing.py

This is similar to calculate_dimensions.py, but currently works with the image under images_for_processing folder.

- post_process.py

This script rectifies each frame from a video given the intrinsic/extrinsic parameters of the camera.

- stream_process.py

This script is similar to post_process.py, but runs live with the camera.

### Usage

For a point-and-click GUI for estimating dimensions:
 
```bash
python mission_post_processing.py
```
