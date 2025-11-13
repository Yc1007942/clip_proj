#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import math
import time

class ForceMonitor(Node):
    def __init__(self):
        super().__init__('force_monitor')
        
        # Subscribe to force-torque sensor
        self.wrench_sub = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.wrench_callback,
            10
        )
        
        # Storage for data
        self.current_wrench = None
        self.force_history = []
        self.max_history = 10
        
        # Statistics
        self.message_count = 0
        self.start_time = time.time()
        
        print(" Force-Torque Monitor Started")
        print(" Subscribing to: /force_torque_sensor_broadcaster/wrench")
        print("Press Ctrl+C to stop\n")
        
        # Timer for periodic display
        self.timer = self.create_timer(1.0, self.display_stats)
        
    def wrench_callback(self, msg):
        """Process incoming force-torque data"""
        self.current_wrench = msg
        self.message_count += 1
        
        # Extract force and torque
        force = msg.wrench.force
        torque = msg.wrench.torque
        
        # Calculate magnitudes
        force_magnitude = math.sqrt(force.x**2 + force.y**2 + force.z**2)
        torque_magnitude = math.sqrt(torque.x**2 + torque.y**2 + torque.z**2)
        
        # Store in history for averaging
        self.force_history.append({
            'force': {'x': force.x, 'y': force.y, 'z': force.z, 'mag': force_magnitude},
            'torque': {'x': torque.x, 'y': torque.y, 'z': torque.z, 'mag': torque_magnitude},
            'timestamp': time.time()
        })
        
        # Keep only recent history
        if len(self.force_history) > self.max_history:
            self.force_history.pop(0)
    
    def display_stats(self):
        """Display current force-torque readings and statistics"""
        if not self.current_wrench:
            print(" No force-torque data received")
            return
        
        # Calculate averages
        if self.force_history:
            avg_force_x = sum(h['force']['x'] for h in self.force_history) / len(self.force_history)
            avg_force_y = sum(h['force']['y'] for h in self.force_history) / len(self.force_history)
            avg_force_z = sum(h['force']['z'] for h in self.force_history) / len(self.force_history)
            avg_force_mag = sum(h['force']['mag'] for h in self.force_history) / len(self.force_history)
            
            avg_torque_x = sum(h['torque']['x'] for h in self.force_history) / len(self.force_history)
            avg_torque_y = sum(h['torque']['y'] for h in self.force_history) / len(self.force_history)
            avg_torque_z = sum(h['torque']['z'] for h in self.force_history) / len(self.force_history)
            avg_torque_mag = sum(h['torque']['mag'] for h in self.force_history) / len(self.force_history)
        
        # Current readings
        current = self.current_wrench.wrench
        current_force_mag = math.sqrt(current.force.x**2 + current.force.y**2 + current.force.z**2)
        current_torque_mag = math.sqrt(current.torque.x**2 + current.torque.y**2 + current.torque.z**2)
        
        # Display
        elapsed = time.time() - self.start_time
        frequency = self.message_count / elapsed if elapsed > 0 else 0
        
        print(f"\n{'='*60}")
        print(f" Runtime: {elapsed:.1f}s | Messages: {self.message_count} | Freq: {frequency:.1f} Hz")
        print(f"{'='*60}")
        
        print(" CURRENT READINGS:")
        print(f"   Force:  X={current.force.x:+7.3f}  Y={current.force.y:+7.3f}  Z={current.force.z:+7.3f}  |F|={current_force_mag:6.3f} N")
        print(f"   Torque: X={current.torque.x:+7.3f}  Y={current.torque.y:+7.3f}  Z={current.torque.z:+7.3f}  |T|={current_torque_mag:6.3f} Nm")
        
        if len(self.force_history) >= 3:
            print(f"\n AVERAGED ({len(self.force_history)} samples):")
            print(f"   Force:  X={avg_force_x:+7.3f}  Y={avg_force_y:+7.3f}  Z={avg_force_z:+7.3f}  |F|={avg_force_mag:6.3f} N")
            print(f"   Torque: X={avg_torque_x:+7.3f}  Y={avg_torque_y:+7.3f}  Z={avg_torque_z:+7.3f}  |T|={avg_torque_mag:6.3f} Nm")
        
        # Threshold warnings (using values from waypoint_multi.py)
        force_threshold = 30.0  # N
        torque_threshold = 10.0  # Nm
        
        warnings = []
        if abs(current.force.x) > force_threshold:
            warnings.append(f"Force X: {current.force.x:.1f}N > {force_threshold}N")
        if abs(current.force.y) > force_threshold:
            warnings.append(f"Force Y: {current.force.y:.1f}N > {force_threshold}N")
        if abs(current.force.z) > force_threshold:
            warnings.append(f"Force Z: {current.force.z:.1f}N > {force_threshold}N")
        if abs(current.torque.x) > torque_threshold:
            warnings.append(f"Torque X: {current.torque.x:.1f}Nm > {torque_threshold}Nm")
        if abs(current.torque.y) > torque_threshold:
            warnings.append(f"Torque Y: {current.torque.y:.1f}Nm > {torque_threshold}Nm")
        if abs(current.torque.z) > torque_threshold:
            warnings.append(f"Torque Z: {current.torque.z:.1f}Nm > {torque_threshold}Nm")
        if current_force_mag > 60.0:
            warnings.append(f"Force magnitude: {current_force_mag:.1f}N > 60.0N")
        
        if warnings:
            print(f"\n THRESHOLD WARNINGS:")
            for warning in warnings:
                print(f"   {warning}")
        else:
            print(f"\n All readings within thresholds")

def main():
    rclpy.init()
    
    try:
        monitor = ForceMonitor()
        print(" Starting force-torque monitoring...")
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n Force monitoring stopped by user")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()