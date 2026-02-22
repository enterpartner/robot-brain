"""Interactive CLI for the Robot Brain."""

import json
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class BrainCLI(Node):
    def __init__(self):
        super().__init__('brain_cli')

        # Publisher for commands
        self._cmd_pub = self.create_publisher(String, '/robot_brain/command', 10)

        # Subscribe to task plans
        self._plan_sub = self.create_subscription(
            String, '/robot_brain/task_plan', self._plan_callback, 10)

        # Subscribe to status
        self._status_sub = self.create_subscription(
            String, '/robot_brain/status', self._status_callback, 10)

        # Scene description service client
        self._scene_client = self.create_client(
            Trigger, '/robot_brain/describe_scene')

        self._status = 'unknown'

    def _plan_callback(self, msg: String):
        """Display received task plan."""
        print('\n' + '=' * 60)
        try:
            data = json.loads(msg.data)
            if data.get('error'):
                print(f'ERROR: {data.get("message", "unknown error")}')
            else:
                print(json.dumps(data, indent=2, ensure_ascii=False))
        except json.JSONDecodeError:
            # Raw text response
            print(msg.data)
        print('=' * 60)
        print('\nrobot> ', end='', flush=True)

    def _status_callback(self, msg: String):
        self._status = msg.data

    def run_interactive(self):
        """Main interactive loop."""
        # Spin ROS in background
        spin_thread = threading.Thread(
            target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()

        print('=' * 60)
        print('  Robot Brain — Interactive CLI')
        print('=' * 60)
        print('Commands:')
        print('  /scene     — Describe current scene')
        print('  /status    — Show brain status')
        print('  /quit      — Exit')
        print('  <anything> — Send as task command')
        print('=' * 60)
        print()

        while True:
            try:
                user_input = input('robot> ').strip()
            except (EOFError, KeyboardInterrupt):
                print('\nExiting...')
                break

            if not user_input:
                continue

            if user_input.lower() in ('/quit', '/exit', '/q'):
                print('Exiting...')
                break

            if user_input.lower() == '/status':
                print(f'  Brain status: {self._status}')
                continue

            if user_input.lower() == '/scene':
                self._request_scene()
                continue

            # Send as command
            msg = String()
            msg.data = user_input
            self._cmd_pub.publish(msg)
            print(f'  Command sent. Waiting for response...')

    def _request_scene(self):
        """Call the describe_scene service."""
        if not self._scene_client.wait_for_service(timeout_sec=2.0):
            # Fallback: send as command topic
            print('  Scene service not available, sending via topic...')
            msg = String()
            msg.data = '/scene'
            self._cmd_pub.publish(msg)
            print('  Waiting for response...')
            return

        request = Trigger.Request()
        future = self._scene_client.call_async(request)

        print('  Requesting scene description...')
        # Wait for result (blocking, but we're in the input thread)
        rclpy.spin_until_future_complete(self, future, timeout_sec=180.0)

        if future.result() is not None:
            result = future.result()
            print('\n' + '=' * 60)
            if result.success:
                try:
                    data = json.loads(result.message)
                    print(json.dumps(data, indent=2, ensure_ascii=False))
                except json.JSONDecodeError:
                    print(result.message)
            else:
                print(f'ERROR: {result.message}')
            print('=' * 60)
        else:
            print('  Scene request timed out.')


def main(args=None):
    rclpy.init(args=args)
    cli = BrainCLI()
    try:
        cli.run_interactive()
    except Exception as e:
        print(f'Error: {e}')
    finally:
        cli.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
