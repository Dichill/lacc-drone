# Camera Calibration Guide

Camera calibration is essential for accurate ArUco marker detection and positioning. The Raspberry Pi camera lens causes distortion that needs to be corrected.

## Why Calibrate?

- **Improved Accuracy**: Corrects lens distortion for precise marker detection
- **Better Positioning**: More accurate center point calculations for landing
- **Distance Estimation**: Enables accurate distance measurements to markers

## Quick Start

### 1. Print a Checkerboard Pattern

You need a checkerboard calibration pattern with **9x6 inner corners**.

**Download options:**
- https://github.com/opencv/opencv/blob/master/doc/pattern.png
- https://calib.io/pages/camera-calibration-pattern-generator

**Tips:**
- Print on stiff paper or mount on cardboard
- Ensure it's flat (no wrinkles or bends)
- Larger is better (A4 or Letter size minimum)

### 2. Run Calibration Script

On your Raspberry Pi:

```bash
cd /path/to/drone-client
python calibrate_camera.py
```

### 3. Capture Calibration Images

**Hold the checkerboard at:**
- Different angles (tilted left/right, up/down)
- Different distances (close and far)
- Different positions (center, corners, edges of frame)

**When you see green corners detected:**
- Press **SPACE** to capture the frame
- You'll hear/see confirmation

**Capture at least 20 images** for good calibration quality.

Press **'q'** when done to calculate calibration.

### 4. Check Results

The script will show:
- Camera matrix
- Distortion coefficients
- Mean reprojection error

**Reprojection error guide:**
- < 0.5 pixels: Excellent ✓
- < 1.0 pixels: Good ✓
- > 1.0 pixels: Try again with more images ⚠

### 5. Test Calibration (Optional)

```bash
python calibrate_camera.py test
```

This shows original vs undistorted video side-by-side.

## Using Calibration

Once calibrated, `main.py` will automatically:
1. Load `camera_calibration.npz` on startup
2. Apply undistortion to all frames
3. Detect ArUco markers on corrected images

You'll see: `✓ Camera calibration loaded from camera_calibration.npz`

## Tips for Good Calibration

### ✓ Do:
- Capture 20+ images
- Vary angles significantly
- Cover all areas of the frame
- Keep checkerboard flat
- Ensure good lighting
- Keep checkerboard fully visible

### ✗ Don't:
- Move too fast (images will blur)
- Use bent or wrinkled pattern
- Capture similar angles repeatedly
- Cover part of the checkerboard
- Use poor lighting

## Troubleshooting

### "No checkerboard detected"
- Ensure entire pattern is visible
- Improve lighting
- Check pattern size matches (9x6 inner corners)
- Move closer or farther from camera
- Clean camera lens

### High reprojection error (> 1.0)
- Capture more images (30-40)
- Vary angles more
- Recalibrate from scratch
- Check pattern is flat

### Calibration file not loading
- Ensure `camera_calibration.npz` is in the same directory as `main.py`
- Check file permissions
- Verify file wasn't corrupted

## Advanced: Custom Checkerboard

If using a different checkerboard size, edit `calibrate_camera.py`:

```python
CHECKERBOARD_SIZE: Tuple[int, int] = (9, 6)  # Change these numbers
```

Count the **inner corners** (intersections), not squares!

## Recalibration

Recalibrate if:
- Camera position/mount changes
- Lens is adjusted or replaced
- Calibration is older than 6 months
- Accuracy seems degraded

Simply run `python calibrate_camera.py` again to create a new calibration.

---

**Note**: Calibration file is drone-specific. If you have multiple drones, calibrate each one separately and keep track of which calibration goes with which drone.

