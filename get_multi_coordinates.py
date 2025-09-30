
import json
import socket
import time
import threading
from datetime import datetime


class WirePositionReceiver:
    def __init__(self, host='localhost', port=9999, timeout=30.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.latest_x = -0.4539  # Default fallback coordinates
        self.latest_y = 0.4059
        self.latest_timestamp = None
        self.connection_active = False
        self.server_socket = None
        self.client_connection = None
        self.position_count = 0
        
    def setup_server(self):
        if self.server_socket is not None:
            return True  
            
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            print(f" Wire position server listening on {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f" Failed to setup server socket: {e}")
            return False
    
    def connect_client(self):
        if self.connection_active and self.client_connection:
            return True
            
        if not self.setup_server():
            return False
            
        try:
            self.client_connection, addr = self.server_socket.accept()
            print(f" Connected to wire detection system at {addr}")
            self.client_connection.settimeout(self.timeout)
            self.connection_active = True
            return True
        except Exception as e:
            print(f" Failed to connect client: {e}")
            return False
    
    def request_next_position(self, clip_number=None):
        if not self.connection_active or not self.client_connection:
            if not self.connect_client():
                return None, None
        
        try:
            # Send request for next position
            request_msg = {
                "action": "get_next_position",
                "clip_number": clip_number,
                "position_count": self.position_count
            }
            request_str = json.dumps(request_msg) + '\n'
            self.client_connection.send(request_str.encode('utf-8'))
            
            buffer = ""
            start_time = time.time()
            
            while time.time() - start_time < self.timeout:
                try:
                    data = self.client_connection.recv(1024).decode('utf-8')
                    if not data:
                        self.connection_active = False
                        return None, None
                    
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.strip():
                            try:
                                wire_data = json.loads(line.strip())
                                
                                # Check if this is a position response
                                if 'x' in wire_data and 'y' in wire_data:
                                    self.latest_x = wire_data['x']
                                    self.latest_y = wire_data['y']
                                    self.latest_timestamp = wire_data.get('timestamp', time.time())
                                    self.position_count += 1
                                    
                                    timestamp_str = datetime.fromtimestamp(self.latest_timestamp).strftime("%H:%M:%S.%f")[:-3]
                                    print(f" Received position #{self.position_count}: X={self.latest_x:.4f}, Y={self.latest_y:.4f} at {timestamp_str}")
                                    
                                    return self.latest_x, self.latest_y
                                
                                elif 'status' in wire_data:
                                    print(f" Status: {wire_data['status']}")
                                    if wire_data['status'] == 'no_more_positions':
                                        print(" No more clip positions available")
                                        return None, None
                                        
                            except (json.JSONDecodeError, KeyError) as e:
                                print(f"  Error parsing wire data: {e}")
                                continue
                
                except socket.timeout:
                    print(" Timeout waiting for position response")
                    break
                except Exception as e:
                    print(f"  Error receiving data: {e}")
                    break
            
        except Exception as e:
            print(f" Error requesting position: {e}")
            self.connection_active = False
        
        return None, None
    
    def close_connection(self):
        try:
            if self.client_connection:
                close_msg = json.dumps({"action": "close"}) + '\n'
                self.client_connection.send(close_msg.encode('utf-8'))
                self.client_connection.close()
                self.client_connection = None
                
            if self.server_socket:
                self.server_socket.close()
                self.server_socket = None
                
            self.connection_active = False
        except Exception as e:
            print(f"  Error closing connection: {e}")
    
    def reset_position_count(self):
        self.position_count = 0
        print(" Position counter reset")


_receiver = WirePositionReceiver(timeout=30.0)


def get_target_coordinates(clip_number=None):
    print(f"\n coordinates for clip {clip_number or 'next'}...")
    x, y = _receiver.request_next_position(clip_number)
    
    if x is not None and y is not None:
        print(f" Using real-time coordinates: X={x:.4f}, Y={y:.4f}")
        return x, y
    else:
        if _receiver.position_count == 0:
            x, y = _receiver.latest_x, _receiver.latest_y
            print(f"  Using fallback coordinates: X={x:.4f}, Y={y:.4f}")
            print("   (Start wire detection system for real-time coordinates)")
            return x, y
        else:
            return None, None


def reset_coordinate_sequence():
    _receiver.reset_position_count()


def close_coordinate_connection():
    _receiver.close_connection()


def main():
    try:
        # Test getting multiple positions
        for i in range(1, 4):
            x, y = get_target_coordinates(f"clip{i}")
            if x is None:
                break
            print(f"Clip {i} coordinates: X={x:.4f}, Y={y:.4f}")
            time.sleep(1)  
    finally:
        close_coordinate_connection()


if __name__ == "__main__":
    main()
