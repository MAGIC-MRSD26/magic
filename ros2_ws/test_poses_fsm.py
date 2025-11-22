#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
import time
from datetime import datetime
import csv
import subprocess
import signal
import os
import shlex

'''
To run this test, you must comment out all of the waitForKeypresses in FSM as well as planner. 
'''

class ObjectPoseTester(Node):
    def __init__(self):
        super().__init__('object_pose_tester')
        
        # Create publisher for cylinder pose
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/cylinder_pose',
            10
        )
        
        # Create subscriber for FSM state feedback
        self.state_subscription = self.create_subscription(
            String,
            '/fsm_state',
            self.state_callback,
            10
        )
        
        # Test parameters - 0.02 increments from -0.1 to 0.1
        # self.x_range = [round(x * 0.02, 2) for x in range(-5, 6)]  # -0.1 to 0.1
        self.x_range = [0.0, 0.1]
        # self.y_range = [round(y * 0.02, 2) for y in range(-5, 6)]  # -0.1 to 0.1
        self.y_range = [0.0, 0.1]
        self.yaw_range = list(range(35, 56, 5))
        
        # Results tracking
        self.results = []
        self.current_test = None
        self.last_state = None
        self.state_history = []
        
        # New State Timeout Tracking
        self.state_timeout_s = 30.0  # Default timeout for any single state
        self.state_start_time = time.time() 
        
        # Process management
        self.rviz_process = None
        self.rviz_command = None
        self.fsm_process = None
        self.fsm_command = None
        
        total_combinations = len(self.x_range) * len(self.y_range) * len(self.yaw_range)
        
        self.get_logger().info('Object Pose Tester initialized')
        self.get_logger().info(f'Total combinations: {total_combinations}')
    
    def state_callback(self, msg):
        """Callback to receive FSM state updates and manage state timer"""
        new_state = msg.data
        self.get_logger().info(f'Received FSM state: {new_state}')
        
        # Check for state transition: Reset timer if the state has changed
        if new_state != self.last_state:
            self.state_start_time = time.time()
        
        self.last_state = new_state
        
        if self.current_test is not None:
            # Track state history
            self.state_history.append(self.last_state)
            self.current_test['last_state'] = self.last_state
            
            # Check for terminal states
            if self.last_state == 'SUCCEEDED':
                self.current_test['result'] = 'PASS'
                self.current_test['failed_at'] = 'N/A'
            elif self.last_state == 'FAILED':
                self.current_test['result'] = 'FAIL'
                # Get the state before FAILED
                if len(self.state_history) >= 2:
                    self.current_test['failed_at'] = self.state_history[-2]
                else:
                    self.current_test['failed_at'] = 'FAILED_IMMEDIATELY'
    
    # --- Process Management Methods (Unchanged) ---
    
    def kill_all_rviz_processes(self):
        """Kill all RViz and related processes"""
        self.get_logger().info('Killing all RViz/launch processes...')
        
        # Kill RViz
        subprocess.run(['pkill', '-9', '-f', 'rviz2'], 
                       stdout=subprocess.DEVNULL, 
                       stderr=subprocess.DEVNULL)
        
        # Kill launch files
        subprocess.run(['pkill', '-9', '-f', 'robot.launch'], 
                       stdout=subprocess.DEVNULL, 
                       stderr=subprocess.DEVNULL)
        
        # Kill any moveit related processes
        subprocess.run(['pkill', '-9', '-f', 'move_group'], 
                       stdout=subprocess.DEVNULL, 
                       stderr=subprocess.DEVNULL)
        
        time.sleep(2)
    
    def start_rviz(self):
        """Start the RViz/simulation"""
        if self.rviz_process is not None:
            self.get_logger().warn('RViz is already running')
            return False
        
        try:
            self.get_logger().info('Starting RViz/simulation...')
            
            self.rviz_process = subprocess.Popen(
                self.rviz_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            # Wait longer for RViz to initialize
            time.sleep(8)
            
            # Check if process is still running
            if self.rviz_process.poll() is not None:
                self.get_logger().error('RViz process exited immediately')
                return False
            
            self.get_logger().info('RViz/simulation started successfully')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to start RViz: {e}')
            return False
    
    def stop_rviz(self):
        """Stop the RViz/simulation"""
        if self.rviz_process is None:
            return True
        
        try:
            self.get_logger().info('Stopping RViz/simulation...')
            
            # Try graceful shutdown first
            pgid = os.getpgid(self.rviz_process.pid)
            os.killpg(pgid, signal.SIGTERM)
            
            try:
                self.rviz_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('Graceful shutdown failed, forcing kill...')
                os.killpg(pgid, signal.SIGKILL)
                self.rviz_process.wait(timeout=2)
            
        except Exception as e:
            self.get_logger().error(f'Error stopping RViz process: {e}')
        
        finally:
            # Always do aggressive cleanup
            self.kill_all_rviz_processes()
            self.rviz_process = None
            self.get_logger().info('RViz/simulation stopped')
            time.sleep(3)
        
        return True
    
    def restart_rviz(self):
        """Restart the RViz/simulation"""
        self.get_logger().info('Restarting RViz/simulation...')
        self.stop_rviz()
        return self.start_rviz()
    
    def start_fsm(self):
        """Start the FSM node"""
        if self.fsm_process is not None:
            self.get_logger().warn('FSM is already running')
            return False
        
        try:
            self.get_logger().info('Starting FSM node...')
            
            self.fsm_process = subprocess.Popen(
                self.fsm_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            # Wait for FSM to initialize
            time.sleep(3)
            
            # Check if process is still running
            if self.fsm_process.poll() is not None:
                self.get_logger().error('FSM process exited immediately')
                return False
            
            self.get_logger().info('FSM node started successfully')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to start FSM: {e}')
            return False
    
    def stop_fsm(self):
        """Stop the FSM node"""
        if self.fsm_process is None:
            return True
        
        try:
            self.get_logger().info('Stopping FSM node...')
            
            os.killpg(os.getpgid(self.fsm_process.pid), signal.SIGTERM)
            
            try:
                self.fsm_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('FSM did not terminate gracefully, forcing kill')
                os.killpg(os.getpgid(self.fsm_process.pid), signal.SIGKILL)
                self.fsm_process.wait()
            
            self.fsm_process = None
            self.get_logger().info('FSM node stopped')
            time.sleep(2)
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to stop FSM: {e}')
            return False
    
    def restart_fsm(self):
        """Restart the FSM node"""
        self.get_logger().info('Restarting FSM node...')
        self.stop_fsm()
        return self.start_fsm()
    
    def restart_all(self):
        """Restart both RViz and FSM"""
        self.get_logger().info('Restarting all processes...')
        self.stop_fsm()
        self.stop_rviz()
        
        if not self.start_rviz():
            return False
        if not self.start_fsm():
            return False
        
        return True
    
    def publish_pose(self, x, y, yaw_deg):
        """Publish a pose with given x, y, and yaw in degrees"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        
        # Position
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        
        # Orientation - convert yaw from degrees to radians for quaternion
        yaw_rad = math.radians(yaw_deg)
        
        # Simple Z-axis rotation quaternion
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        
        self.pose_publisher.publish(msg)
        self.get_logger().info(f'Published pose: x={x:.2f}, y={y:.2f}, yaw={yaw_deg}°')
    
    # --- Core Testing Logic (Revised) ---

    def run_automatic_test(self, delay=2.0, state_timeout_s=30.0, restart_every=1, restart_rviz_on_fail=True):
        """Automatic mode - publish all poses with state-based timeout"""
        
        self.state_timeout_s = state_timeout_s
        self.get_logger().info(f'\nStarting automatic test...')
        self.get_logger().info(f'Delay between poses: {delay}s')
        self.get_logger().info(f'Timeout per state: {self.state_timeout_s}s')
        
        test_count = 0
        total = len(self.x_range) * len(self.y_range) * len(self.yaw_range)
        
        start_time = datetime.now()
        
        # Start RViz and FSM initially
        if not self.start_rviz():
            self.get_logger().error('Failed to start RViz, aborting tests')
            return
        
        if not self.start_fsm():
            self.get_logger().error('Failed to start FSM, aborting tests')
            return
        
        time.sleep(2) # Allow topics to connect
        
        for x in self.x_range:
            for y in self.y_range:
                for yaw in self.yaw_range:
                    test_count += 1
                    
                    # Restart FSM periodically if requested
                    if restart_every > 0 and test_count > 1 and (test_count - 1) % restart_every == 0:
                        self.get_logger().info(f'\nRestarting FSM after {restart_every} test(s)...')
                        if not self.restart_fsm():
                            self.get_logger().error('Failed to restart FSM, aborting tests')
                            break
                    
                    # Initialize current test
                    self.current_test = {
                        'test_num': test_count,
                        'x': x,
                        'y': y,
                        'yaw': yaw,
                        'result': 'RUNNING',
                        'failed_at': 'UNKNOWN',
                        'last_state': 'NONE',
                        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    }
                    
                    self.get_logger().info(f'\n{"="*60}')
                    self.get_logger().info(f'Test {test_count}/{total} (State Timeout: {self.state_timeout_s}s)')
                    self.get_logger().info(f'{"="*60}')
                    
                    # Reset state tracking and timer for the start of the new test
                    self.last_state = None
                    self.state_history = []
                    self.state_start_time = time.time()
                    
                    # Publish the pose
                    self.publish_pose(x, y, yaw)
                    
                    # Wait for completion or state timeout
                    timed_out = False
                    
                    while self.current_test['result'] == 'RUNNING':
                        rclpy.spin_once(self, timeout_sec=0.1)
                        
                        # Check for state-specific timeout
                        if self.last_state is not None:
                            elapsed_in_state = time.time() - self.state_start_time
                            
                            if elapsed_in_state > self.state_timeout_s:
                                timed_out = True
                                self.current_test['result'] = 'TIMEOUT'
                                self.current_test['failed_at'] = self.last_state # The state it got stuck in
                                self.get_logger().warn(f'State "{self.last_state}" timed out after {elapsed_in_state:.2f}s!')
                                break
                    
                    # If failed or timed out, record details and restart if necessary
                    if self.current_test['result'] == 'FAIL' or timed_out:
                        if self.current_test['failed_at'] == 'UNKNOWN':
                            # Fallback: use state history
                            if len(self.state_history) >= 2:
                                self.current_test['failed_at'] = self.state_history[-2]
                            elif len(self.state_history) >= 1:
                                self.current_test['failed_at'] = self.state_history[0]
                            else:
                                self.current_test['failed_at'] = 'NO_STATE_RECEIVED'
                        
                        # Restart RViz and FSM on failure or timeout
                        if restart_rviz_on_fail:
                            self.get_logger().info('Test failed/timed out, restarting all processes...')
                            if not self.restart_all():
                                self.get_logger().error('Failed to restart processes, aborting tests')
                                break
                    
                    # Add to results
                    self.results.append(self.current_test.copy())
                    
                    # Log result
                    result_str = f"{self.current_test['result']}"
                    if self.current_test['result'] in ['FAIL', 'TIMEOUT']:
                        result_str += f" at {self.current_test['failed_at']}"
                    self.get_logger().info(f'Result: {result_str}')
                    
                    # Save results periodically (every 100 tests)
                    if test_count % 100 == 0:
                        self.save_results(f'pose_test_results_partial_{test_count}.csv')
                    
                    if test_count < total:
                        time.sleep(delay)
        
        # Stop all processes at the end
        self.stop_fsm()
        self.stop_rviz()
        
        end_time = datetime.now()
        duration = end_time - start_time
        
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'✓ Completed all {test_count} test combinations!')
        self.get_logger().info(f'Total time: {duration}')
        self.get_logger().info(f'{"="*60}\n')
        
        # Save final results
        self.save_results('pose_test_results_final.csv')
        self.print_results_table()
    
    # --- Helper Methods (Unchanged) ---
    
    def save_results(self, filename):
        """Save results to CSV file"""
        try:
            with open(filename, 'w', newline='') as csvfile:
                fieldnames = ['test_num', 'x', 'y', 'yaw', 'result', 'failed_at', 'last_state', 'timestamp']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                writer.writeheader()
                for result in self.results:
                    writer.writerow(result)
            
            self.get_logger().info(f'Results saved to {filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to save results: {e}')
    
    def print_results_table(self, max_rows=50):
        """Print results in a formatted table"""
        print(f'\n{"="*80}')
        print(f'DETAILED RESULTS (showing first and last {max_rows} entries)')
        print(f'{"="*80}')
        print(f'{"Test#":>6} {"X":>7} {"Y":>7} {"Yaw":>5} {"Result":>10} {"Failed At":>20} {"Timestamp":>20}')
        print(f'{"-"*80}')
        
        # Show first max_rows entries
        for i, result in enumerate(self.results[:max_rows]):
            print(f'{result["test_num"]:6d} '
                  f'{result["x"]:7.2f} '
                  f'{result["y"]:7.2f} '
                  f'{result["yaw"]:5d} '
                  f'{result["result"]:>10s} '
                  f'{result["failed_at"]:>20s} '
                  f'{result["timestamp"]:>20s}')
        
        # If there are more results, show ellipsis
        if len(self.results) > 2 * max_rows:
            print(f'{"...":^80}')
            print(f'{"(middle entries omitted)":^80}')
            print(f'{"...":^80}')
        
        # Show last max_rows entries
        if len(self.results) > max_rows:
            for result in self.results[-max_rows:]:
                print(f'{result["test_num"]:6d} '
                      f'{result["x"]:7.2f} '
                      f'{result["y"]:7.2f} '
                      f'{result["yaw"]:5d} '
                      f'{result["result"]:>10s} '
                      f'{result["failed_at"]:>20s} '
                      f'{result["timestamp"]:>20s}')
        
        print(f'{"="*80}\n')
        print(f'Full results saved to CSV files.')
        print(f'Use spreadsheet software to view all {len(self.results)} entries.\n')
    
    def cleanup(self):
        """Cleanup resources"""
        self.stop_fsm()
        self.stop_rviz()


def main(args=None):
    rclpy.init(args=args)
    
    tester = ObjectPoseTester()
    
    try:
        # Get RViz command
        default_rviz = "ros2 launch two_arm_moveit2_config robot.launch.py robot_ip:=xxx.xxx.xxx.xxx use_fake_hardware:=true"
        print("\nEnter the command to launch RViz/simulation.")
        print(f"Default: {default_rviz}")
        rviz_cmd = input("\nRViz command (press Enter for default): ").strip()
        tester.rviz_command = shlex.split(rviz_cmd if rviz_cmd else default_rviz)
        
        # Get FSM command
        default_fsm = "ros2 run moveit_go dual_arm_fsm"
        print("\nEnter the command to launch your FSM node.")
        print(f"Default: {default_fsm}")
        fsm_cmd = input("\nFSM command (press Enter for default): ").strip()
        tester.fsm_command = shlex.split(fsm_cmd if fsm_cmd else default_fsm)
        
        delay = input("\nEnter delay between poses in seconds (default 2.0): ")
        delay = float(delay) if delay else 2.0
        
        # New input for state timeout
        state_timeout_input = input("Enter timeout per state in seconds (default 60.0): ")
        state_timeout = float(state_timeout_input) if state_timeout_input else 60.0
        
        restart = input("Restart FSM every N tests (0 for never, 1 for every test, default 1): ")
        restart_every = int(restart) if restart else 1
        
        restart_rviz = input("Restart RViz on failure? (y/n, default y): ")
        restart_rviz_on_fail = restart_rviz.lower() != 'n'
        
        print(f"\nStarting in 3 seconds...")
        time.sleep(3)
        
        # Pass state_timeout to the testing function
        tester.run_automatic_test(delay, state_timeout, restart_every, restart_rviz_on_fail)
    
    except KeyboardInterrupt:
        tester.get_logger().info('\n\nTesting interrupted by user')
        tester.save_results('pose_test_results_interrupted.csv')
        tester.print_results_table()
    except Exception as e:
        tester.get_logger().error(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        tester.cleanup()
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()