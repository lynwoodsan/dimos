
import sys
import time
import json
import numpy as np
import cv2
import open3d as o3d
import signal
import threading
import argparse
import os
from dataclasses import dataclass
from unittest.mock import MagicMock
from flask import Flask, Response, render_template_string

# --- FLASK APP ---
app = Flask(__name__)

# --- DEPENDENCY SETUP (Mocks) ---
# ... (Same Dependency Mocking Logic as before to keep imports happy) ...
try:
    import dimos_lcm
except ImportError:
    dimos_lcm = MagicMock()
    sys.modules["dimos_lcm"] = dimos_lcm
    sys.modules["dimos_lcm.std_msgs"] = MagicMock()
    sys.modules["dimos_lcm.std_msgs.Header"] = MagicMock()
    sys.modules["dimos_lcm.sensor_msgs"] = MagicMock()
    sys.modules["dimos_lcm.sensor_msgs.Image"] = MagicMock()
    sys.modules["dimos_lcm.sensor_msgs.PointCloud2"] = MagicMock()
    sys.modules["dimos_lcm.geometry_msgs"] = MagicMock()
    sys.modules["dimos_lcm.vision_msgs"] = MagicMock()
    sys.modules["dimos_lcm.builtin_interfaces"] = MagicMock()
    sys.modules["rerun"] = MagicMock()
    sys.modules["reactivex"] = MagicMock()
    sys.modules["moondream"] = MagicMock()

try:
    import rclpy
except ImportError:
    sys.modules["rclpy"] = MagicMock()
    sys.modules["cv_bridge"] = MagicMock()

# --- IMPORTS ---
try:
    import pyrealsense2 as rs
except ImportError:
    print("Error: pyrealsense2 not installed.")
    sys.exit(1)

from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.perception.detection.type.detection2d import ImageDetections2D

# --- UTILS ---
def load_calibration(calibration_file):
    try:
        with open(calibration_file, 'r') as f:
            calib = json.load(f)
        trans = calib.get("translation_m", {})
        ts = np.array([trans.get("x", 0), trans.get("y", 0), trans.get("z", 0)])
        rot_q = calib.get("rotation_quat_wxyz", {})
        w, x, y, z = rot_q.get("w", 1), rot_q.get("x", 0), rot_q.get("y", 0), rot_q.get("z", 0)
        # Rotation matrix from quaternion (standard formula)
        xx, xy, xz, xw = x*x, x*y, x*z, x*w
        yy, yz, yw = y*y, y*z, y*w
        zz, zw = z*z, z*w
        R = np.array([
            [1 - 2*(yy+zz), 2*(xy-zw),     2*(xz+yw)],
            [2*(xy+zw),     1 - 2*(xx+zz), 2*(yz-xw)],
            [2*(xz-yw),     2*(yz+xw),     1 - 2*(xx+yy)]
        ])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = ts
        return T
    except Exception as e:
        print(f"Warning: Failed to load calibration: {e}")
        return np.eye(4)

class MockImage:
    def __init__(self, cv_image):
        self._cv_image = cv_image
        self.ts = time.time()
        self.frame_id = "camera_color_frame"
        self.height, self.width = cv_image.shape[:2]

    def to_opencv(self):
        return self._cv_image
        
    @property
    def shape(self):
        return self._cv_image.shape

@dataclass
class DetectedObject3D:
    track_id: int
    name: str
    confidence: float
    pointcloud: o3d.geometry.PointCloud
    bbox_2d: tuple
    center_3d: np.ndarray

# 2D to 3D Projection (Simplified for script)
def project_to_3d(detections_2d, color_image, depth_image, intrinsics, extrinsics):
    objects_3d = []
    if depth_image.dtype == np.uint16:
        depth_m = depth_image.astype(np.float32) / 1000.0
    else:
        depth_m = depth_image.astype(np.float32)
    
    color_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    h, w = depth_image.shape[:2]

    for det in detections_2d.detections:
        if isinstance(det, Detection2DSeg) and det.mask is not None:
             mask = det.mask
        else:
             mask = np.zeros((h, w), dtype=np.uint8)
             x1, y1, x2, y2 = map(int, det.bbox)
             mask[max(0, y1):min(h, y2), max(0, x1):min(w, x2)] = 255
        
        # Simple erosion
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.erode(mask.astype(np.uint8), kernel)
        
        masked_depth = depth_m.copy()
        masked_depth[mask == 0] = 0
        
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color_rgb),
            o3d.geometry.Image(masked_depth),
            depth_scale=1.0, depth_trunc=3.0, convert_rgb_to_intensity=False
        )
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)
        
        if len(pcd.points) < 10:
            continue
        
        # Outlier removal
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
        
        pcd.transform(extrinsics)
        objects_3d.append(DetectedObject3D(
            track_id=det.track_id if hasattr(det, 'track_id') else -1,
            name=det.name, confidence=det.confidence,
            pointcloud=pcd, bbox_2d=det.bbox, center_3d=pcd.get_center()
        ))
    return objects_3d

# --- PIPELINE THREAD ---
class PipelineThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.running = False
        self.lock = threading.Lock()
        self.latest_frame = None  # JPEG encoded
        self.segmentation_enabled = False
        self.yolo_model = None
        self.last_error = None
        
    def init_camera(self):
        print("Initializing RealSense...")
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
            
            profile = self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            
            # Intrinsics
            sp = profile.get_stream(rs.stream.color).as_video_stream_profile()
            intr = sp.get_intrinsics()
            self.o3d_intrinsics = o3d.camera.PinholeCameraIntrinsic(
                intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy
            )
            
            # Extrinsics
            self.extrinsics = load_calibration("dimos/hardware/camera/realsense/eye_in_hand_calibration_xarm6.json")
            return True
        except Exception as e:
            self.last_error = str(e)
            print(f"Camera Init Failed: {e}")
            return False

    def init_yolo(self):
        print("Initializing YOLO...")
        import torch
        device = "cuda:0" if torch.cuda.is_available() else "cpu"
        print(f"Using Device: {device}")
        
        # Patch for direct loading
        class RobustYoloe(Yoloe2DDetector):
            def __init__(self, model_name, device):
                 from ultralytics import YOLOE as UltraYOLOE
                 self.model = UltraYOLOE(model_name)
                 self.model.to(device)
                 self.device = device
                 self.prompt_mode = YoloePromptMode.LRPC
            
            # Override to increase confidence
            def process_image(self, image):
                track_kwargs = {
                    "source": image.to_opencv(),
                    "device": self.device,
                    "conf": 0.60, # Reduced noise (was 0.5)
                    "iou": 0.6,
                    "persist": True,
                    "verbose": False,
                }
                results = self.model.track(**track_kwargs)
                return ImageDetections2D.from_ultralytics_result(image, results)


        # Use a BETTER model (Large) as requested
        model_name = "yolov8l-seg.pt" 
        try:
            print(f"Loading High-Quality Model: {model_name}...")
            self.yolo_model = RobustYoloe(model_name, device)
            print("YOLO Loaded (Large Model)")
        except:
            print("Fallback to standard yolov8n-seg.pt")
            self.yolo_model = RobustYoloe("yolov8n-seg.pt", device)

    def handle_click(self, norm_x, norm_y):
        """Handle click in normalized coordinates (0-1)"""
        if not self.latest_detections:
            return None
            
        # Convert to pixel coordinates (1280x720)
        img_w, img_h = 1280, 720
        click_x = int(norm_x * img_w)
        click_y = int(norm_y * img_h)
        
        print(f"Click at: {click_x}, {click_y}")
        
        # Find object
        for det in self.latest_detections.detections:
            x1, y1, x2, y2 = map(int, det.bbox)
            if x1 <= click_x <= x2 and y1 <= click_y <= y2:
                self.selected_id = det.track_id if hasattr(det, 'track_id') else -1
                print(f"Selected Object ID: {self.selected_id} ({det.name})")
                return self.selected_id
                
        # If clicked background, clear selection
        self.selected_id = None
        print("Selection Cleared (Background Click)")
        return None

    def run(self):
        if not self.init_camera():
            return
            
        # Lazy init YOLO only if we want, but let's init now for speed
        self.init_yolo()
        
        self.running = True
        self.latest_detections = None
        self.selected_id = None
        self.latest_selected_pcd = None
        
        print("Pipeline Thread Running...")
        
        while self.running:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    continue
                    
                color_img = np.asanyarray(color_frame.get_data())
                depth_img = np.asanyarray(depth_frame.get_data())
                
                vis_img = color_img.copy()
                
                # YOLO
                if self.segmentation_enabled and self.yolo_model:
                     mock_img = MockImage(color_img)
                     detections = self.yolo_model.process_image(mock_img)
                     self.latest_detections = detections
                     
                     # 3D ROI MASKING Logic
                     selected_obj_3d = None
                     
                     # Visualize
                     for det in detections.detections:
                         track_id = det.track_id if hasattr(det, 'track_id') else -1
                         is_selected = (self.selected_id is not None and track_id == self.selected_id)
                         
                         # Color: Red if selected, Green if normal
                         color = (0, 0, 255) if is_selected else (0, 255, 0)
                         thickness = 4 if is_selected else 2
                         
                         x1, y1, x2, y2 = map(int, det.bbox)
                         cv2.rectangle(vis_img, (x1, y1), (x2, y2), color, thickness)
                         
                         label = f"{det.name} {track_id}"
                         if is_selected: 
                             label += " [SELECTED]"
                             
                         cv2.putText(vis_img, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                         
                         if hasattr(det, 'mask') and det.mask is not None:
                             mask = det.mask.astype(bool)
                             # Overlay
                             # If selected, make it very visible
                             overlay_color = np.array([0, 0, 255]) if is_selected else np.array([0, 255, 0])
                             alpha = 0.6 if is_selected else 0.4
                             vis_img[mask] = vis_img[mask] * (1-alpha) + overlay_color * alpha

                         # If this is the selected object, project IT to 3D
                         if is_selected:
                             print(f"- Projecting ROI for {det.name} (ID: {track_id})", flush=True)
                             # Optimization: only project the selected one for ROI feedback
                             roi_detections = ImageDetections2D(image=mock_img, detections=[det])
                             objects_3d = project_to_3d(
                                 roi_detections, 
                                 color_img, 
                                 depth_img, 
                                 self.o3d_intrinsics, 
                                 self.extrinsics
                             )
                             if objects_3d:
                                 selected_obj_3d = objects_3d[0]
                                 with self.lock:
                                     self.latest_selected_pcd = selected_obj_3d.pointcloud

                else:
                    cv2.putText(vis_img, "CAMERA OK - Segmentation OFF", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                    self.latest_detections = None
                    self.latest_selected_pcd = None
                
                # Encode for Web
                ret, buffer = cv2.imencode('.jpg', vis_img)
                if ret:
                    with self.lock:
                        self.latest_frame = buffer.tobytes()
                        
            except Exception as e:
                print(f"Loop Error: {e}")
                time.sleep(1)

    def stop(self):
        self.running = False
        if hasattr(self, 'pipeline'):
            self.pipeline.stop()

# --- GLOBAL PIPELINE ---
pipeline_thread = PipelineThread()

# --- FLASK ROUTES ---
from flask import request

@app.route('/')
def index():
    return render_template_string("""
    <html>
    <head>
        <title>Dimos Perception Stream</title>
        <style>
            body { font-family: sans-serif; text-align: center; background: #222; color: #fff; }
            .container { position: relative; display: inline-block; }
            img { border: 2px solid #555; max-width: 100%; cursor: crosshair; }
            .btn { padding: 10px 20px; font-size: 18px; cursor: pointer; background: #4CAF50; color: white; border: none; border-radius: 5px; margin: 10px;}
            .btn:hover { background: #45a049; }
            .btn.clear { background: #f44336; }
            .status { margin-top: 10px; color: #aaa; }
        </style>
        <script>
            function toggleSeg() {
                fetch('/toggle_segmentation').then(response => response.text()).then(data => {
                    document.getElementById('status').innerText = "Status: " + data;
                });
            }
            
            function clearSelection() {
                fetch('/clear_selection');
            }

            function onImageClick(event) {
                // Get click coordinates relative to image
                const rect = event.target.getBoundingClientRect();
                const x = event.clientX - rect.left;
                const y = event.clientY - rect.top;
                
                // Normalize (0-1)
                const normX = x / rect.width;
                const normY = y / rect.height;
                
                console.log("Click:", normX, normY);
                fetch(`/select_object?x=${normX}&y=${normY}`)
                    .then(response => response.text())
                    .then(data => {
                         console.log("Selection:", data);
                         document.getElementById('sel_status').innerText = "Selected: " + data;
                    });
            }
        </script>
    </head>
    <body>
        <h1>Dimos RealSense Pipeline</h1>
        <div class="container">
            <img src="/video_feed" onclick="onImageClick(event)" />
        </div>
        <br>
        <button class="btn" onclick="toggleSeg()">Toggle Segmentation (Large Model)</button>
        <button class="btn clear" onclick="clearSelection()">Clear Selection</button>
        <div id="status" class="status">Status: Ready</div>
        <div id="sel_status" class="status" style="color: yellow">Selected: None</div>
        <br>
        <div id="roi_feedback" style="margin-top: 20px;">
            <a href="/download_pcd" target="_blank">
                <button class="btn" style="background: #2196F3;">Download Selected 3D ROI (.pcd)</button>
            </a>
            <p style="font-size: 0.8em; color: #888;">(Click an object first, then download to verify the segmented 3D cloud)</p>
        </div>
    </body>
    </html>
    """)

def gen_frames():
    while True:
        with pipeline_thread.lock:
            frame = pipeline_thread.latest_frame
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.033) # ~30 FPS cap

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/toggle_segmentation')
def toggle_segmentation():
    pipeline_thread.segmentation_enabled = not pipeline_thread.segmentation_enabled
    state = "ON" if pipeline_thread.segmentation_enabled else "OFF"
    print(f"Segmentation toggled to: {state}")
    return f"Segmentation {state}"

@app.route('/select_object')
def select_object():
    x = float(request.args.get('x', 0))
    y = float(request.args.get('y', 0))
    selected = pipeline_thread.handle_click(x, y)
    return str(selected) if selected is not None else "None"

@app.route('/clear_selection')
def clear_selection():
    pipeline_thread.selected_id = None
    pipeline_thread.latest_selected_pcd = None
    return "Cleared"

@app.route('/download_pcd')
def download_pcd():
    with pipeline_thread.lock:
        pcd = pipeline_thread.latest_selected_pcd
    if pcd is None or pcd.is_empty():
        return "No ROI selected or pointcloud empty", 404
    
    # Save to a temporary file locally and serve it
    filename = "selected_roi.pcd"
    o3d.io.write_point_cloud(filename, pcd)
    
    from flask import send_file
    return send_file(filename, as_attachment=True)

def main():
    # Handle Cleanup
    def signal_handler(sig, frame):
        print("Stopping...")
        pipeline_thread.stop()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    # Start Pipeline
    pipeline_thread.start()
    
    # Start Web Server
    # Host 0.0.0.0 allows external access (e.g. from your laptop via toggle/port forward)
    print("Starting Web Server at http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000, threaded=True)

if __name__ == "__main__":
    main()
