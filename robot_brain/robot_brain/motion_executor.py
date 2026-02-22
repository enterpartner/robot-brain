"""Motion executor: translates Brain task plans into joint movements."""

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


# ── Joint names (must match URDF) ────────────────────────────────
LEFT_ARM_JOINTS = [
    'left_shoulder_yaw_joint',
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_elbow_pitch_joint',
    'left_wrist_yaw_joint',
    'left_wrist_pitch_joint',
    'left_wrist_roll_joint',
    'left_gripper_joint',
]

RIGHT_ARM_JOINTS = [
    'right_shoulder_yaw_joint',
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_elbow_pitch_joint',
    'right_wrist_yaw_joint',
    'right_wrist_pitch_joint',
    'right_wrist_roll_joint',
    'right_gripper_joint',
]

HEAD_JOINTS = [
    'head_pan_joint',
    'head_tilt_joint',
]

ALL_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + HEAD_JOINTS

# ── Predefined poses ─────────────────────────────────────────────
# Values: [shoulder_yaw, shoulder_pitch, shoulder_roll, elbow_pitch,
#          wrist_yaw, wrist_pitch, wrist_roll, gripper]

HOME = [0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0]

POSES = {
    # ── Right arm poses ──
    'right_raise': [0.0, -1.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'right_wave_a': [0.0, -1.8, 0.0, -0.8, 0.0, 0.5, 0.0, 0.0],
    'right_wave_b': [0.0, -1.8, 0.0, -0.8, 0.0, -0.5, 0.0, 0.0],
    'right_forward': [0.0, -0.8, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0],
    'right_side': [0.0, 0.0, -1.4, 0.0, 0.0, 0.0, 0.0, 0.0],
    'right_grab_ready': [0.0, -0.5, 0.0, -1.2, 0.0, -0.3, 0.0, 0.04],
    'right_grab_close': [0.0, -0.5, 0.0, -1.2, 0.0, -0.3, 0.0, 0.0],
    'right_home': HOME.copy(),

    # ── Left arm poses ──
    'left_raise': [0.0, -1.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'left_wave_a': [0.0, -1.8, 0.0, -0.8, 0.0, 0.5, 0.0, 0.0],
    'left_wave_b': [0.0, -1.8, 0.0, -0.8, 0.0, -0.5, 0.0, 0.0],
    'left_forward': [0.0, -0.8, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0],
    'left_side': [0.0, 0.0, 1.4, 0.0, 0.0, 0.0, 0.0, 0.0],
    'left_grab_ready': [0.0, -0.5, 0.0, -1.2, 0.0, -0.3, 0.0, 0.04],
    'left_grab_close': [0.0, -0.5, 0.0, -1.2, 0.0, -0.3, 0.0, 0.0],
    'left_home': HOME.copy(),
}

# ── Command → pose mapping keywords ─────────────────────────────
# Maps keywords in LLM actions or direct text to pose sequences
COMMAND_MAP = {
    # Raise / emeld fel
    ('raise', 'right'): ['right_raise'],
    ('raise', 'left'): ['left_raise'],
    ('emeld', 'jobb'): ['right_raise'],
    ('emeld', 'bal'): ['left_raise'],
    ('raise', 'both'): ['left_raise', 'right_raise'],
    ('emeld', 'mindkét'): ['left_raise', 'right_raise'],
    ('emeld', 'mind'): ['left_raise', 'right_raise'],
    ('kéz', 'fel', 'jobb'): ['right_raise'],
    ('kéz', 'fel', 'bal'): ['left_raise'],
    ('kar', 'fel', 'jobb'): ['right_raise'],
    ('kar', 'fel', 'bal'): ['left_raise'],

    # Wave / integess
    ('wave', 'right'): ['right_wave_a', 'right_wave_b', 'right_wave_a', 'right_wave_b', 'right_home'],
    ('wave', 'left'): ['left_wave_a', 'left_wave_b', 'left_wave_a', 'left_wave_b', 'left_home'],
    ('integes',): ['right_wave_a', 'right_wave_b', 'right_wave_a', 'right_wave_b', 'right_home'],
    ('wave',): ['right_wave_a', 'right_wave_b', 'right_wave_a', 'right_wave_b', 'right_home'],

    # Forward / nyújtsd
    ('forward', 'right'): ['right_forward'],
    ('forward', 'left'): ['left_forward'],
    ('reach', 'right'): ['right_forward'],
    ('reach', 'left'): ['left_forward'],
    ('nyújt', 'jobb'): ['right_forward'],
    ('nyújt', 'bal'): ['left_forward'],

    # Grab / fogd meg
    ('grasp', 'right'): ['right_grab_ready', 'right_grab_close'],
    ('grasp', 'left'): ['left_grab_ready', 'left_grab_close'],
    ('grab', 'right'): ['right_grab_ready', 'right_grab_close'],
    ('grab', 'left'): ['left_grab_ready', 'left_grab_close'],
    ('fogd', 'jobb'): ['right_grab_ready', 'right_grab_close'],
    ('fogd', 'bal'): ['left_grab_ready', 'left_grab_close'],

    # Home / alap
    ('home',): ['right_home', 'left_home'],
    ('alap',): ['right_home', 'left_home'],
    ('reset',): ['right_home', 'left_home'],
    ('pihenj',): ['right_home', 'left_home'],

    # Side / oldalra
    ('side', 'right'): ['right_side'],
    ('side', 'left'): ['left_side'],
}


class MotionExecutor(Node):
    def __init__(self):
        super().__init__('motion_executor')

        self.declare_parameter('interpolation_hz', 30.0)
        self.declare_parameter('motion_duration_sec', 1.5)

        self._hz = self.get_parameter('interpolation_hz').value
        self._motion_dur = self.get_parameter('motion_duration_sec').value

        # Current joint positions (start at home)
        self._current = {}
        for j in LEFT_ARM_JOINTS:
            self._current[j] = HOME[LEFT_ARM_JOINTS.index(j)]
        for j in RIGHT_ARM_JOINTS:
            self._current[j] = HOME[RIGHT_ARM_JOINTS.index(j)]
        for j in HEAD_JOINTS:
            self._current[j] = 0.0

        self._lock = threading.Lock()
        self._executing = False

        # Joint state publisher
        self._js_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribe to brain task plan
        self._plan_sub = self.create_subscription(
            String, '/robot_brain/task_plan', self._plan_callback, 10)

        # Direct motion command topic
        self._cmd_sub = self.create_subscription(
            String, '/motion_executor/command', self._direct_command_callback, 10)

        # Also listen to brain commands directly for simple keyword matching
        self._brain_cmd_sub = self.create_subscription(
            String, '/robot_brain/command', self._brain_command_callback, 10)

        # Publish joint states at fixed rate
        self._timer = self.create_timer(1.0 / self._hz, self._publish_joints)

        self.get_logger().info('Motion executor started — ready for commands')

    def _publish_joints(self):
        """Publish current joint states at fixed rate."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self._lock:
            msg.name = list(self._current.keys())
            msg.position = [self._current[j] for j in msg.name]
        self._js_pub.publish(msg)

    def _plan_callback(self, msg: String):
        """Parse brain task plan and execute motions."""
        try:
            data = json.loads(msg.data)
            if data.get('error'):
                return
            plan = data.get('plan', [])
            if not plan:
                return
            thread = threading.Thread(
                target=self._execute_plan, args=(plan,), daemon=True)
            thread.start()
        except json.JSONDecodeError:
            # Not JSON, try keyword matching on raw text
            self._try_keyword_match(msg.data)

    def _brain_command_callback(self, msg: String):
        """Try to match brain commands directly to poses (bypass LLM for known motions)."""
        cmd = msg.data.strip().lower()
        # Skip scene/status commands
        if cmd in ('/scene', 'scene', '/status', 'status', 'describe', 'leírás'):
            return
        self._try_keyword_match(cmd)

    def _direct_command_callback(self, msg: String):
        """Direct pose command (e.g. 'right_raise' or keywords)."""
        cmd = msg.data.strip().lower()
        # Try as a direct pose name first
        if cmd in POSES:
            arm = 'right' if cmd.startswith('right_') else 'left'
            joints = RIGHT_ARM_JOINTS if arm == 'right' else LEFT_ARM_JOINTS
            thread = threading.Thread(
                target=self._interpolate_to,
                args=(joints, POSES[cmd]), daemon=True)
            thread.start()
            return
        # Try keyword matching
        self._try_keyword_match(cmd)

    def _try_keyword_match(self, text: str):
        """Match text against keyword patterns to find poses."""
        text_lower = text.lower()
        best_match = None
        best_score = 0

        for keywords, pose_sequence in COMMAND_MAP.items():
            score = sum(1 for kw in keywords if kw in text_lower)
            if score == len(keywords) and score > best_score:
                best_score = score
                best_match = pose_sequence

        if best_match:
            self.get_logger().info(f'Matched command: {best_match}')
            thread = threading.Thread(
                target=self._execute_pose_sequence,
                args=(best_match,), daemon=True)
            thread.start()

    def _execute_plan(self, plan: list):
        """Execute a list of plan steps from the LLM."""
        for step in plan:
            action = step.get('action', '')
            params = step.get('parameters', {})

            if action == 'move_arm':
                arm = params.get('arm', 'right')
                # For now, try to guess a reasonable pose based on the target position
                z = params.get('z', 0.0)
                if z is not None and z > 0.8:
                    pose_name = f'{arm}_raise'
                elif z is not None and z > 0.3:
                    pose_name = f'{arm}_forward'
                else:
                    pose_name = f'{arm}_grab_ready'

                if pose_name in POSES:
                    joints = RIGHT_ARM_JOINTS if arm == 'right' else LEFT_ARM_JOINTS
                    self._interpolate_to(joints, POSES[pose_name])
                    time.sleep(0.3)

            elif action in ('grasp', 'grab'):
                arm = params.get('arm', 'right')
                joints = RIGHT_ARM_JOINTS if arm == 'right' else LEFT_ARM_JOINTS
                self._interpolate_to(joints, POSES.get(f'{arm}_grab_close', HOME))
                time.sleep(0.3)

            elif action in ('release', 'open'):
                arm = params.get('arm', 'right')
                joints = RIGHT_ARM_JOINTS if arm == 'right' else LEFT_ARM_JOINTS
                self._interpolate_to(joints, POSES.get(f'{arm}_grab_ready', HOME))
                time.sleep(0.3)

            elif action == 'look_at':
                pan = params.get('pan', 0.0)
                tilt = params.get('tilt', 0.0)
                if pan is None:
                    pan = 0.0
                if tilt is None:
                    tilt = 0.0
                self._interpolate_to(HEAD_JOINTS, [pan, tilt])
                time.sleep(0.3)

            elif action == 'wait':
                secs = params.get('seconds', 1.0)
                if secs:
                    time.sleep(float(secs))

        self.get_logger().info('Plan execution complete')

    def _execute_pose_sequence(self, pose_names: list):
        """Execute a sequence of named poses."""
        for pose_name in pose_names:
            if pose_name not in POSES:
                continue

            arm = 'right' if pose_name.startswith('right_') else 'left'
            joints = RIGHT_ARM_JOINTS if arm == 'right' else LEFT_ARM_JOINTS

            # For 'both arms' commands that have mixed sides
            if 'left' in pose_name:
                joints = LEFT_ARM_JOINTS
            if 'right' in pose_name:
                joints = RIGHT_ARM_JOINTS

            self._interpolate_to(joints, POSES[pose_name])
            time.sleep(0.3)  # Pause between poses

    def _interpolate_to(self, joint_names: list, target: list):
        """Smoothly interpolate joints to target over motion_duration."""
        if self._executing:
            self.get_logger().warn('Motion already in progress, queuing...')
            while self._executing:
                time.sleep(0.05)

        self._executing = True
        try:
            with self._lock:
                start = [self._current.get(j, 0.0) for j in joint_names]

            steps = int(self._hz * self._motion_dur)
            for i in range(1, steps + 1):
                t = i / steps
                # Smooth ease-in-out
                t = t * t * (3.0 - 2.0 * t)

                with self._lock:
                    for idx, j in enumerate(joint_names):
                        self._current[j] = start[idx] + t * (target[idx] - start[idx])

                time.sleep(1.0 / self._hz)
        finally:
            self._executing = False


def main(args=None):
    rclpy.init(args=args)
    node = MotionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
