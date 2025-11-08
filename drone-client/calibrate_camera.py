"""
Camera Calibration Script for Raspberry Pi Camera 
https://github.com/Carson-Stark/AutonomousDrone

This script calibrates the camera using a checkerboard pattern to correct
lens distortion for accurate ArUco marker detection.

Instructions:
1. Print a checkerboard pattern (9x6 inner corners)
   You can use: https://github.com/opencv/opencv/blob/master/doc/pattern.png
   Or generate one with: https://calib.io/pages/camera-calibration-pattern-generator

2. Run this script: python calibrate_camera.py

3. Hold the checkerboard in front of the camera at different:
   - Angles (tilted left/right, up/down)
   - Distances (close and far)
   - Positions (center, corners, edges)

4. Press SPACE to capture a frame when you see a good detection
5. Capture at least 20 images from different angles/positions
6. Press 'q' when done to calculate calibration

The calibration data will be saved to camera_calibration.npz
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
import os

# Checkerboard dimensions (inner corners)
CHECKERBOARD_SIZE: Tuple[int, int] = (9, 6)  # columns, rows
SQUARE_SIZE: float = 1.0  # Size of square in arbitrary units

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def prepare_object_points() -> np.ndarray:
    """Prepare object points for the checkerboard pattern."""
    objp: np.ndarray = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD_SIZE[0], 0:CHECKERBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE
    return objp


def calibrate_camera_interactive() -> None:
    """Interactive camera calibration using checkerboard pattern."""
    
    # Arrays to store object points and image points
    objpoints: List[np.ndarray] = []  # 3D points in real world space
    imgpoints: List[np.ndarray] = []  # 2D points in image plane
    
    objp: np.ndarray = prepare_object_points()
    
    # Try to import picamera2 for Raspberry Pi, fallback to regular camera
    try:
        from picamera2 import Picamera2
        print("Using Raspberry Pi Camera")
        camera = Picamera2()
        camera.configure(camera.create_preview_configuration(main={"size": (640, 480)}))
        camera.start()
        use_picamera: bool = True
        cap = None
    except ImportError:
        print("Picamera2 not available, using default camera")
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        use_picamera = False
        camera = None
    
    frame_size: Optional[Tuple[int, int]] = None
    capture_count: int = 0
    
    print("\n" + "=" * 60)
    print("CAMERA CALIBRATION")
    print("=" * 60)
    print(f"\nCheckerboard size: {CHECKERBOARD_SIZE[0]}x{CHECKERBOARD_SIZE[1]} (inner corners)")
    print("\nInstructions:")
    print("  - Hold checkerboard at different angles and distances")
    print("  - Press SPACE when you see green corners detected")
    print("  - Capture at least 20 good images")
    print("  - Press 'q' when done to calculate calibration")
    print("\n" + "=" * 60 + "\n")
    
    while True:
        # Capture frame
        if use_picamera:
            frame = camera.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame")
                break
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if frame_size is None:
            frame_size = (gray.shape[1], gray.shape[0])
        
        # Find checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_SIZE, None)
        
        display_frame = frame.copy()
        
        if ret:
            # Refine corner positions
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # Draw corners
            cv2.drawChessboardCorners(display_frame, CHECKERBOARD_SIZE, corners_refined, ret)
            
            # Add instruction text
            cv2.putText(display_frame, "Checkerboard detected! Press SPACE to capture", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(display_frame, "No checkerboard detected", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Show capture count
        cv2.putText(display_frame, f"Captured: {capture_count}/20+", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.imshow("Camera Calibration", display_frame)
        
        key: int = cv2.waitKey(1) & 0xFF
        
        if key == ord(" ") and ret:
            # Save this detection
            objpoints.append(objp)
            imgpoints.append(corners_refined)
            capture_count += 1
            print(f"✓ Captured frame {capture_count}")
            
        elif key == ord("q"):
            if capture_count < 10:
                print(f"\n⚠ Warning: Only {capture_count} images captured. Recommended: 20+")
                print("Continue anyway? (y/n): ", end="", flush=True)
                # Note: Can't easily read input here without blocking, so just proceed
                print("Proceeding with calibration...")
            break
    
    # Cleanup
    cv2.destroyAllWindows()
    if use_picamera and camera:
        camera.stop()
    elif cap:
        cap.release()
    
    if capture_count < 3:
        print("\n✗ Error: Need at least 3 captured images for calibration")
        return
    
    print(f"\nCalculating calibration from {capture_count} images...")
    
    # Calibrate camera
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, frame_size, None, None
    )
    
    if ret:
        print("\n✓ Calibration successful!")
        print(f"\nCamera Matrix:\n{camera_matrix}")
        print(f"\nDistortion Coefficients:\n{dist_coeffs}")
        
        # Calculate reprojection error
        total_error: float = 0.0
        for i in range(len(objpoints)):
            imgpoints_reprojected, _ = cv2.projectPoints(
                objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
            )
            error = cv2.norm(imgpoints[i], imgpoints_reprojected, cv2.NORM_L2) / len(imgpoints_reprojected)
            total_error += error
        
        mean_error: float = total_error / len(objpoints)
        print(f"\nMean reprojection error: {mean_error:.4f} pixels")
        
        if mean_error < 0.5:
            print("✓ Excellent calibration quality!")
        elif mean_error < 1.0:
            print("✓ Good calibration quality")
        else:
            print("⚠ Calibration quality could be improved (try more images)")
        
        # Save calibration data
        calibration_file: str = "camera_calibration.npz"
        np.savez(
            calibration_file,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            frame_size=frame_size
        )
        
        print(f"\n✓ Calibration saved to {calibration_file}")
        print("\nYou can now use this calibration in main.py for accurate ArUco detection")
        
    else:
        print("\n✗ Calibration failed")


def test_calibration() -> None:
    """Test the saved calibration by showing undistorted video."""
    
    calibration_file: str = "camera_calibration.npz"
    
    if not os.path.exists(calibration_file):
        print(f"✗ Calibration file not found: {calibration_file}")
        print("Run calibration first!")
        return
    
    # Load calibration
    calib_data = np.load(calibration_file)
    camera_matrix = calib_data["camera_matrix"]
    dist_coeffs = calib_data["dist_coeffs"]
    
    print("✓ Calibration loaded")
    print("\nShowing original (left) and undistorted (right) video")
    print("Press 'q' to exit")
    
    # Try to import picamera2 for Raspberry Pi, fallback to regular camera
    try:
        from picamera2 import Picamera2
        camera = Picamera2()
        camera.configure(camera.create_preview_configuration(main={"size": (640, 480)}))
        camera.start()
        use_picamera: bool = True
        cap = None
    except ImportError:
        cap = cv2.VideoCapture(0)
        use_picamera = False
        camera = None
    
    while True:
        if use_picamera:
            frame = camera.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            ret, frame = cap.read()
            if not ret:
                break
        
        # Undistort the frame
        h, w = frame.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix, dist_coeffs, (w, h), 1, (w, h)
        )
        undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)
        
        # Crop the image
        x, y, w_roi, h_roi = roi
        undistorted = undistorted[y:y+h_roi, x:x+w_roi]
        
        # Resize to match original for side-by-side comparison
        undistorted_resized = cv2.resize(undistorted, (frame.shape[1], frame.shape[0]))
        
        # Create side-by-side comparison
        comparison = np.hstack([frame, undistorted_resized])
        
        cv2.putText(comparison, "Original", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(comparison, "Undistorted", (frame.shape[1] + 10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow("Calibration Test", comparison)
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    
    cv2.destroyAllWindows()
    if use_picamera and camera:
        camera.stop()
    elif cap:
        cap.release()


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "test":
        test_calibration()
    else:
        calibrate_camera_interactive()

