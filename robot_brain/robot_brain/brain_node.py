"""Robot Brain node: Vision + LLM task planner."""

import base64
import json
import threading
from io import BytesIO

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from cv_bridge import CvBridge
    _CV_BRIDGE_OK = True
except ImportError:
    _CV_BRIDGE_OK = False

from robot_brain.prompts import get_scene_prompt, get_task_prompt

# HTTP request without external deps
from urllib.request import urlopen, Request
from urllib.error import URLError


class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')

        # Parameters
        self.declare_parameter('ollama_url', 'http://localhost:11434')
        self.declare_parameter('model_name', 'llava:7b')
        self.declare_parameter('inference_timeout_sec', 120.0)
        self.declare_parameter('rgb_topic', '/zed/zed_node/rgb/color/rect/image')
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('max_image_width', 1280)
        self.declare_parameter('depth_grid_rows', 3)
        self.declare_parameter('depth_grid_cols', 3)

        self._ollama_url = self.get_parameter('ollama_url').value
        self._model = self.get_parameter('model_name').value
        self._timeout = self.get_parameter('inference_timeout_sec').value
        self._jpeg_quality = self.get_parameter('jpeg_quality').value
        self._max_width = self.get_parameter('max_image_width').value
        self._depth_rows = self.get_parameter('depth_grid_rows').value
        self._depth_cols = self.get_parameter('depth_grid_cols').value

        if _CV_BRIDGE_OK:
            self._bridge = CvBridge()
        else:
            self._bridge = None
            self.get_logger().warn('cv_bridge not available, using numpy fallback')

        # Latest frames
        self._rgb_frame = None
        self._depth_frame = None
        self._frame_lock = threading.Lock()

        # Inference lock (one at a time)
        self._inference_lock = threading.Lock()

        # QoS for camera topics (best effort, keep last 1)
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value

        self._rgb_sub = self.create_subscription(
            Image, rgb_topic, self._rgb_callback, camera_qos)
        self._depth_sub = self.create_subscription(
            Image, depth_topic, self._depth_callback, camera_qos)

        # Command input topic
        self._cmd_sub = self.create_subscription(
            String, '/robot_brain/command', self._command_callback, 10)

        # Task plan output topic
        self._plan_pub = self.create_publisher(String, '/robot_brain/task_plan', 10)

        # Status topic
        self._status_pub = self.create_publisher(String, '/robot_brain/status', 10)

        # Scene description service
        self._scene_srv = self.create_service(
            Trigger, '/robot_brain/describe_scene', self._scene_service_callback)

        self.get_logger().info(
            f'Brain node started — model={self._model}, '
            f'rgb={rgb_topic}, depth={depth_topic}')
        self._publish_status('ready')

    # ── Image conversion ──────────────────────────────────────────────

    def _imgmsg_to_cv2(self, msg: Image) -> np.ndarray:
        """Convert ROS Image to OpenCV numpy array."""
        if self._bridge is not None:
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Fallback: manual conversion
        dtype = np.uint8
        channels = 1
        if msg.encoding in ('bgr8', 'rgb8'):
            channels = 3
        elif msg.encoding in ('bgra8', 'rgba8'):
            channels = 4
        elif msg.encoding == '32FC1':
            dtype = np.float32
            channels = 1
        elif msg.encoding == '16UC1':
            dtype = np.uint16
            channels = 1

        img = np.frombuffer(msg.data, dtype=dtype)
        if channels > 1:
            img = img.reshape(msg.height, msg.width, channels)
        else:
            img = img.reshape(msg.height, msg.width)

        return img

    # ── Callbacks ─────────────────────────────────────────────────────

    def _rgb_callback(self, msg: Image):
        try:
            frame = self._imgmsg_to_cv2(msg)
            # Convert to BGR if needed
            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'rgba8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            elif msg.encoding == 'bgra8':
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            with self._frame_lock:
                self._rgb_frame = frame
        except Exception as e:
            self.get_logger().error(f'RGB conversion failed: {e}')

    def _depth_callback(self, msg: Image):
        try:
            frame = self._imgmsg_to_cv2(msg)
            with self._frame_lock:
                self._depth_frame = frame
        except Exception as e:
            self.get_logger().error(f'Depth conversion failed: {e}')

    def _command_callback(self, msg: String):
        command = msg.data.strip()
        if not command:
            return
        self.get_logger().info(f'Received command: {command}')
        # Run inference in background thread
        thread = threading.Thread(
            target=self._process_command, args=(command,), daemon=True)
        thread.start()

    def _scene_service_callback(self, request, response):
        self.get_logger().info('Scene description requested')
        result = self._run_inference(get_scene_prompt())
        if result is not None:
            response.success = True
            response.message = result
        else:
            response.success = False
            response.message = 'Inference failed — check logs'
        return response

    # ── Core logic ────────────────────────────────────────────────────

    def _process_command(self, command: str):
        """Process a natural language command (runs in background thread)."""
        self._publish_status('thinking')

        if command.lower() in ('/scene', 'scene', 'describe', 'leírás'):
            prompt = get_scene_prompt()
        else:
            prompt = get_task_prompt(command)

        result = self._run_inference(prompt)

        if result is not None:
            plan_msg = String()
            plan_msg.data = result
            self._plan_pub.publish(plan_msg)
            self.get_logger().info('Task plan published')
        else:
            err_msg = String()
            err_msg.data = json.dumps({
                'error': True,
                'message': 'LLM inference failed',
            })
            self._plan_pub.publish(err_msg)

        self._publish_status('ready')

    def _run_inference(self, prompt: str) -> str | None:
        """Capture current frame, call Ollama, return response text."""
        if not self._inference_lock.acquire(blocking=False):
            self.get_logger().warn('Inference already in progress, skipping')
            return None

        try:
            # Grab current frames
            with self._frame_lock:
                rgb = self._rgb_frame.copy() if self._rgb_frame is not None else None
                depth = self._depth_frame.copy() if self._depth_frame is not None else None

            if rgb is None:
                self.get_logger().error('No RGB frame available')
                return None

            # Encode image to base64 JPEG
            img_b64 = self._encode_image(rgb)

            # Build depth context string
            depth_info = self._summarize_depth(depth) if depth is not None else ''

            # Add depth info to prompt if available
            full_prompt = prompt
            if depth_info:
                full_prompt += (
                    f'\n\nDEPTH INFORMATION (distance in meters, NaN=no data):\n'
                    f'{depth_info}'
                )

            # Call Ollama
            response_text = self._call_ollama(full_prompt, img_b64)
            return response_text

        except Exception as e:
            self.get_logger().error(f'Inference error: {e}')
            return None
        finally:
            self._inference_lock.release()

    def _encode_image(self, frame: np.ndarray) -> str:
        """Resize and encode BGR frame to base64 JPEG."""
        h, w = frame.shape[:2]
        if w > self._max_width:
            scale = self._max_width / w
            frame = cv2.resize(frame, (self._max_width, int(h * scale)))

        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
        _, buf = cv2.imencode('.jpg', frame, encode_params)
        return base64.b64encode(buf.tobytes()).decode('utf-8')

    def _summarize_depth(self, depth: np.ndarray) -> str:
        """Create a grid summary of depth values."""
        h, w = depth.shape[:2]
        rows, cols = self._depth_rows, self._depth_cols
        cell_h, cell_w = h // rows, w // cols

        lines = []
        for r in range(rows):
            row_vals = []
            for c in range(cols):
                cell = depth[
                    r * cell_h:(r + 1) * cell_h,
                    c * cell_w:(c + 1) * cell_w
                ]
                # Depth may be float32 (meters) or uint16 (mm)
                cell_f = cell.astype(np.float32)
                valid = cell_f[np.isfinite(cell_f) & (cell_f > 0)]
                if len(valid) > 0:
                    median_m = float(np.median(valid))
                    # If values are > 100, assume millimeters
                    if median_m > 100:
                        median_m /= 1000.0
                    row_vals.append(f'{median_m:.2f}m')
                else:
                    row_vals.append('N/A')
            label = ['top', 'mid', 'bot'][r] if rows == 3 else f'row{r}'
            lines.append(f'  {label}: {" | ".join(row_vals)}')

        col_labels = ['left', 'center', 'right'] if cols == 3 else [
            f'col{c}' for c in range(cols)]
        header = '  ' + '  |  '.join(f'{lbl:>6s}' for lbl in col_labels)
        return header + '\n' + '\n'.join(lines)

    def _call_ollama(self, prompt: str, image_b64: str) -> str | None:
        """Call Ollama API with image and prompt."""
        url = f'{self._ollama_url}/api/generate'

        payload = json.dumps({
            'model': self._model,
            'prompt': prompt,
            'images': [image_b64],
            'stream': False,
            'options': {
                'temperature': 0.1,
                'num_predict': 2048,
            },
        }).encode('utf-8')

        req = Request(url, data=payload, method='POST')
        req.add_header('Content-Type', 'application/json')

        self.get_logger().info(f'Calling Ollama ({self._model})...')

        try:
            with urlopen(req, timeout=self._timeout) as resp:
                body = json.loads(resp.read().decode('utf-8'))
                response_text = body.get('response', '')
                total_dur = body.get('total_duration', 0)
                dur_sec = total_dur / 1e9 if total_dur else 0
                self.get_logger().info(
                    f'Ollama response received ({dur_sec:.1f}s)')
                return response_text
        except URLError as e:
            self.get_logger().error(f'Ollama request failed: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'Ollama error: {e}')
            return None

    # ── Helpers ───────────────────────────────────────────────────────

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
