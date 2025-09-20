#!/usr/bin/env python3
"""
only for sete.json and setf.json, obtain value for x,y coordinates
Socket receiver implementation for real-time wire position updates
"""
import json
import socket
import time
import threading
from datetime import datetime


class WirePositionReceiver:
    def __init__(self, host='localhost', port=9999, timeout=50.0, max_retries=100, retry_delay=1.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.max_retries = max_retries
        self.retry_delay = retry_delay
        self.latest_x = -0.4539  # Default fallback coordinates
        self.latest_y = 0.4059
        self.latest_timestamp = None
        self.connection_active = False
        self.server_socket = None
        self.client_connection = None
        
    def setup_server(self):
        """Setup server socket to receive wire positions"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            print(f"🔌 Wire position server listening on {self.host}:{self.port}")
            print("Waiting for Basler wire detection system to connect...")
            return True
        except Exception as e:
            print(f"❌ Failed to setup server socket: {e}")
            return False
    
    def try_receive_position(self):
        """Try to receive the latest wire position from connected client"""
        if not self.setup_server():
            return None, None
            
        try:
            # Accept connection from Basler script
            self.client_connection, addr = self.server_socket.accept()
            print(f"✅ Connected to wire detection system at {addr}")
            self.connection_active = True
            
            # Set timeout for receiving data
            self.client_connection.settimeout(self.timeout)
            
            buffer = ""
            start_time = time.time()
            
            while time.time() - start_time < self.timeout:
                try:
                    data = self.client_connection.recv(1024).decode('utf-8')
                    if not data:
                        break
                    
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.strip():
                            try:
                                wire_data = json.loads(line.strip())
                                
                                # Update latest coordinates
                                self.latest_x = wire_data['x']
                                self.latest_y = wire_data['y']
                                self.latest_timestamp = wire_data['timestamp']
                                
                                timestamp_str = datetime.fromtimestamp(self.latest_timestamp).strftime("%H:%M:%S.%f")[:-3]
                                print(f"📍 Received wire position: X={self.latest_x:.4f}, Y={self.latest_y:.4f} at {timestamp_str}")
                                
                                # We got fresh coordinates, return them
                                return self.latest_x, self.latest_y
                                
                            except (json.JSONDecodeError, KeyError) as e:
                                print(f"⚠️  Error parsing wire data: {e}")
                                continue
                
                except socket.timeout:
                    break
                except Exception as e:
                    print(f"⚠️  Error receiving data: {e}")
                    break
            
        except Exception as e:
            print(f"❌ Server error: {e}")
        
        finally:
            if self.client_connection:
                self.client_connection.close()
            if self.server_socket:
                self.server_socket.close()
        
        print(f"❌ Failed to receive wire position")
        self.connection_active = False
        return None, None


# Global receiver instance
_receiver = WirePositionReceiver(max_retries=100, retry_delay=1.0)


def get_target_coordinates():
    """
    Get the target X,Y coordinates for waypoint updates
    First tries to get real-time coordinates from wire detection system,
    falls back to default coordinates if connection fails
    Returns: tuple (x, y) 
    """
    print("\n🎯 Getting target coordinates...")
    
    # Try to get real-time coordinates
    x, y = _receiver.try_receive_position()
    
    if x is not None and y is not None:
        print(f"✅ Using real-time coordinates: X={x:.4f}, Y={y:.4f}")
        x = x -0.01
        y = y + 0.01
        return x, y
    else:
        # Fallback to default coordinates
        x, y = _receiver.latest_x, _receiver.latest_y
        print(f"⚠️  Using fallback coordinates: X={x:.4f}, Y={y:.4f}")
        print("   (Start wire detection system for real-time coordinates)")
        return x, y

def main():
    x, y = get_target_coordinates()
    print(f"Target coordinates: X={x:.4f}, Y={y:.4f}")
    
if __name__ == "__main__":
    main()
