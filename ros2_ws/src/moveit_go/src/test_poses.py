#!/usr/bin/env python3
"""
Improved Position Variation Test Script with auto-keypress support
Tests x, y, and yaw variations using grid search only.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
import numpy as np
import time
import csv
from datetime import datetime
import json
import subprocess
import os


class ImprovedPositionTester(Node):
    """Test node with better FSM integration"""
    
    def __init__(self, auto_keypress=False):
        super().__init__('position_variation_tester')
        
        self.auto_keypress = auto_keypress
        
        # Publisher for cylinder pose
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/cylinder_pose',
            10
        )
        
        # Subscriber to FSM state
        self.state_subscriber = self.create_subscription(
            String,
            '/fsm_state',
            self.state_callback,
            10
        )
        
        # Test tracking
        self.current_fsm_state = "HOME"
        self.previous_state = "HOME"
        self.test_timeout = 300.0  # 5 minutes per test
        self.test_start_time = None
        self.state_stuck_time = None
        
        # Results
        self.results = []
        
        self.get_logger().info("Improved Position Variation Tester initialized")
        if auto_keypress:
            self.get_logger().info("Auto-keypress mode: ENABLED")
        
    def state_callback(self, msg):
        """Track current FSM state"""
        if self.current_fsm_state != msg.data:
            self.get_logger().info(f"FSM: {self.current_fsm_state} → {msg.data}")
            self.previous_state = self.current_fsm_state
            self.state_stuck_time = time.time()
        
        self.current_fsm_state = msg.data
        
    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion (x, y, z, w)"""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return (x, y, z, w)
    
    def send_keypress(self):
        """Send a space keypress to the FSM terminal"""
        if self.auto_keypress:
            try:
                # This sends a space key to help FSM progress
                # Note: This is a hacky solution. Better to modify FSM.
                subprocess.run(['xdotool', 'search', '--name', 'motion_planning', 
                               'key', 'space'], 
                              capture_output=True, timeout=1.0)
            except:
                pass  # xdotool might not be available
    
    def create_pose_msg(self, x, y, yaw):
        """Create a PoseStamped message"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.78
        
        # Convert yaw to quaternion (roll=0, pitch=0)
        quat = self.quaternion_from_euler(0.0, 0.0, yaw)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        return pose_msg
    
    def publish_test_pose(self, x, y, yaw):
        """Publish a test pose configuration"""
        pose_msg = self.create_pose_msg(x, y, yaw)
        
        # Publish multiple times to ensure it's received
        for _ in range(10):
            self.pose_publisher.publish(pose_msg)
            time.sleep(0.05)
        
        self.get_logger().info(
            f"Published pose: x={x:.3f}, y={y:.3f}, yaw={np.degrees(yaw):.1f}°"
        )
    
    def check_test_success(self):
        """
        Check if test reached PLACE state successfully
        Returns: 'SUCCESS', 'FAILED', 'TIMEOUT', or 'RUNNING'
        """
        # Success conditions
        if self.current_fsm_state in ["PLACE", "PLAN_TO_PLACE", "PLAN_RETRACT", "MOVE_RETRACT"]:
            return 'SUCCESS'
        
        # Failure condition
        elif self.current_fsm_state == "FAILED":
            return 'FAILED'
        
        # Timeout condition
        elif time.time() - self.test_start_time > self.test_timeout:
            return 'TIMEOUT'
        
        # Check if stuck at HOME for too long (might need keypress)
        elif (self.current_fsm_state == "HOME" and 
              self.state_stuck_time and 
              time.time() - self.state_stuck_time > 10.0):
            self.get_logger().warn("FSM stuck at HOME - may need keypress")
            if self.auto_keypress:
                self.send_keypress()
            return 'RUNNING'
        
        else:
            return 'RUNNING'
    
    def run_single_test(self, x, y, yaw):
        """Run a single test configuration"""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Testing: x={x:.3f}m, y={y:.3f}m, yaw={np.degrees(yaw):.1f}°")
        self.get_logger().info(f"{'='*60}")
        
        # Reset tracking
        self.state_stuck_time = time.time()
        
        # Publish the test pose
        self.publish_test_pose(x, y, yaw)
        
        # Wait for FSM to receive
        time.sleep(2.0)
        
        # Try sending initial keypress if in auto mode
        if self.auto_keypress:
            self.send_keypress()
            time.sleep(0.5)
        
        # Start timing
        self.test_start_time = time.time()
        
        # Monitor FSM execution
        result_status = 'RUNNING'
        last_log_time = time.time()
        
        while result_status == 'RUNNING':
            rclpy.spin_once(self, timeout_sec=0.5)
            result_status = self.check_test_success()
            
            # Log progress every 10 seconds
            if time.time() - last_log_time > 10.0:
                elapsed = time.time() - self.test_start_time
                self.get_logger().info(
                    f"Progress: {elapsed:.0f}s elapsed, State: {self.current_fsm_state}"
                )
                last_log_time = time.time()
                
                # Try keypress periodically in auto mode
                if self.auto_keypress:
                    self.send_keypress()
        
        execution_time = time.time() - self.test_start_time
        
        # Record results
        result = {
            'x': x,
            'y': y,
            'yaw_rad': yaw,
            'yaw_deg': np.degrees(yaw),
            'status': result_status,
            'final_state': self.current_fsm_state,
            'execution_time': execution_time,
            'timestamp': datetime.now().isoformat()
        }
        
        self.get_logger().info(f"Result: {result_status} in {execution_time:.1f}s")
        
        return result
    
    def save_results(self, filename_base='position_test_results'):
        """Save results to CSV and JSON"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Save as CSV
        csv_filename = f"{filename_base}_{timestamp}.csv"
        with open(csv_filename, 'w', newline='') as csvfile:
            if self.results:
                fieldnames = self.results[0].keys()
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.results)
        
        # Save as JSON
        json_filename = f"{filename_base}_{timestamp}.json"
        with open(json_filename, 'w') as jsonfile:
            json.dump(self.results, jsonfile, indent=2)
        
        self.get_logger().info(f"\nResults saved to:")
        self.get_logger().info(f"  - {csv_filename}")
        self.get_logger().info(f"  - {json_filename}")
        
        # Print summary
        self.print_summary()
    
    def print_summary(self):
        """Print summary statistics"""
        if not self.results:
            return
        
        total = len(self.results)
        success = sum(1 for r in self.results if r['status'] == 'SUCCESS')
        failed = sum(1 for r in self.results if r['status'] == 'FAILED')
        timeout = sum(1 for r in self.results if r['status'] == 'TIMEOUT')
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info("TEST SUMMARY")
        self.get_logger().info(f"{'='*60}")
        self.get_logger().info(f"Total tests: {total}")
        self.get_logger().info(f"Successful: {success} ({100*success/total:.1f}%)")
        self.get_logger().info(f"Failed: {failed} ({100*failed/total:.1f}%)")
        self.get_logger().info(f"Timeout: {timeout} ({100*timeout/total:.1f}%)")
        self.get_logger().info(f"{'='*60}\n")


def generate_test_grid(x_range, y_range, yaw_range):
    """Generate a grid of test positions"""
    x_values = np.linspace(x_range[0], x_range[1], x_range[2])
    y_values = np.linspace(y_range[0], y_range[1], y_range[2])
    yaw_values = np.radians(np.linspace(yaw_range[0], yaw_range[1], yaw_range[2]))
    
    test_configs = []
    for x in x_values:
        for y in y_values:
            for yaw in yaw_values:
                test_configs.append((x, y, yaw))
    
    return test_configs


def main(args=None):
    """Main test execution - Grid Search Only"""
    rclpy.init(args=args)
    
    print("\n" + "="*70)
    print("FSM Position Variation Testing - IMPROVED VERSION")
    print("="*70)
    
    # Ask about keypress handling
    print("\nYour FSM likely has keypress prompts that pause execution.")
    print("Choose how to handle this:")
    print("  1. Manual - You press space/enter during testing (tedious)")
    print("  2. Auto - Script tries to send keypresses (experimental)")
    print("  3. Skip test - Run diagnostic first")
    
    mode = input("\nChoose mode (1/2/3): ").strip()
    
    if mode == '3':
        print("\nPlease run: python3 diagnose_setup.py")
        return
    
    auto_keypress = (mode == '2')
    
    if auto_keypress:
        print("\n⚠️  Auto-keypress is experimental and may not work.")
        print("If tests still timeout, you'll need to modify your FSM.")
    
    tester = ImprovedPositionTester(auto_keypress=auto_keypress)
    
    print("\n" + "="*70)
    print("Enter test parameters:")
    print("-" * 70)
    
    # Get ranges (with defaults)
    x_min = float(input("X min (meters) [default 0.4]: ") or "0.4")
    x_max = float(input("X max (meters) [default 0.6]: ") or "0.6")
    x_num = int(input("Number of X points [default 3]: ") or "3")
    
    y_min = float(input("\nY min (meters) [default -0.2]: ") or "-0.2")
    y_max = float(input("Y max (meters) [default 0.2]: ") or "0.2")
    y_num = int(input("Number of Y points [default 5]: ") or "5")
    
    yaw_min = float(input("\nYaw min (degrees) [default -30]: ") or "-30")
    yaw_max = float(input("Yaw max (degrees) [default 30]: ") or "30")
    yaw_num = int(input("Number of Yaw points [default 3]: ") or "3")
    
    # Generate test grid
    test_configs = generate_test_grid(
        x_range=(x_min, x_max, x_num),
        y_range=(y_min, y_max, y_num),
        yaw_range=(yaw_min, yaw_max, yaw_num)
    )
    
    total_tests = len(test_configs)
    estimated_time = total_tests * 3.0  # 3 minutes per test
    
    print("\n" + "="*70)
    print(f"TEST CONFIGURATION:")
    print(f"  X range: [{x_min}, {x_max}] with {x_num} points")
    print(f"  Y range: [{y_min}, {y_max}] with {y_num} points")
    print(f"  Yaw range: [{yaw_min}°, {yaw_max}°] with {yaw_num} points")
    print(f"  Total tests: {total_tests}")
    print(f"  Estimated time: {estimated_time/60:.1f} minutes")
    print(f"  Timeout per test: 5 minutes")
    print("="*70)
    
    # Confirm
    confirm = input("\nStart testing? (y/n): ").strip().lower()
    
    if confirm != 'y':
        print("Test cancelled.")
        tester.destroy_node()
        rclpy.shutdown()
        return
    
    # Run tests
    print("\nStarting tests...")
    print("TIP: Watch your FSM terminal. Press space if it's waiting.\n")
    start_time = time.time()
    
    try:
        for i, (x, y, yaw) in enumerate(test_configs, 1):
            print(f"\n{'#'*70}")
            print(f"Test {i}/{total_tests}")
            print(f"{'#'*70}")
            
            result = tester.run_single_test(x, y, yaw)
            tester.results.append(result)
            
            # Save intermediate results every 5 tests
            if i % 5 == 0:
                tester.save_results('position_test_intermediate')
                print(f"\nIntermediate results saved. Progress: {i}/{total_tests}")
            
            # Brief pause between tests
            time.sleep(2.0)
        
        total_time = time.time() - start_time
        
        # Save final results
        print("\n" + "="*70)
        print("ALL TESTS COMPLETED!")
        print("="*70)
        tester.save_results('position_test_final')
        
        print(f"\nTotal time: {total_time/60:.1f} minutes")
        print(f"Average time per test: {total_time/total_tests:.1f} seconds")
        
    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print("TEST INTERRUPTED BY USER")
        print("="*70)
        if tester.results:
            tester.save_results('position_test_interrupted')
            print("\nPartial results have been saved.")
    
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()