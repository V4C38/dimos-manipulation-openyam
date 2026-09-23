#!/usr/bin/env python3
"""Local browser stream for the calibrated OpenYAM wrist camera."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import cv2
import numpy as np


CAMERA = "/dev/v4l/by-id/usb-USB_CAMERA_4K_USB_CAMERA_4K_01.00.00-video-index0"
CONFIG = Path(__file__).with_name("openyam_bench.json")


def undistort_map(size):
    try:
        calibration = json.loads(CONFIG.read_text()).get("wrist_camera_calibration")
        if not calibration or calibration.get("model") != "opencv_fisheye":
            return None
        if calibration.get("image_size") != list(size):
            return None
        matrix = np.asarray(calibration["camera_matrix"], dtype=np.float64)
        distortion = np.asarray(calibration["distortion_coefficients"], dtype=np.float64).reshape(4, 1)
        new_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            matrix, distortion, size, np.eye(3), balance=0.35)
        return cv2.fisheye.initUndistortRectifyMap(
            matrix, distortion, np.eye(3), new_matrix, size, cv2.CV_16SC2)
    except (OSError, KeyError, TypeError, ValueError, cv2.error):
        return None


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            page = b'<html><title>OpenYAM wrist camera</title><body style="margin:0;background:#111"><div style="color:white;padding:8px">Undistorted stream when workspace calibration is available; <a style="color:white" href="/raw">raw view</a></div><img src="/stream" style="width:100%;height:auto"></body></html>'
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.end_headers()
            self.wfile.write(page)
            return
        if self.path not in ("/stream", "/raw"):
            self.send_error(404)
            return
        camera = cv2.VideoCapture(CAMERA, cv2.CAP_V4L2)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        rectification = undistort_map((1920, 1080)) if self.path == "/stream" else None
        self.send_response(200)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                ok, image = camera.read()
                if not ok:
                    break
                if rectification is not None:
                    image = cv2.remap(image, rectification[0], rectification[1], cv2.INTER_LINEAR)
                ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if not ok:
                    continue
                payload = encoded.tobytes()
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(payload)).encode() + b"\r\n\r\n" + payload + b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            camera.release()


if __name__ == "__main__":
    ThreadingHTTPServer(("0.0.0.0", 8765), Handler).serve_forever()
