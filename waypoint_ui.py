"""
 /home/yc/panasonic/clip_proj/.venv/bin/python -m streamlit run waypoint_ui.py --server.headless true --server.enableCORS false --server.enableXsrfProtection false
"""

import streamlit as st
import subprocess
import threading
import time
import json
import os
import cv2
import numpy as np
from PIL import Image
import queue
import signal

# Configure Streamlit page
st.set_page_config(
    page_title="Panasonic",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="collapsed"
)

# Custom CSS for dark theme with glowing elements
st.markdown("""
<style>
    /* Dark theme background */
    .stApp {
        background: linear-gradient(135deg, #0f0f0f 0%, #1a1a2e 50%, #16213e 100%);
        color: #ffffff !important;
    }
    
    /* Main container styling */
    .main-container {
        background: rgba(0, 0, 0, 0.8);
        border-radius: 20px;
        padding: 20px;
        border: 2px solid #00ff88;
        box-shadow: 0 0 30px rgba(0, 255, 136, 0.3);
        margin: 10px 0;
    }
    
    /* Title styling with glow */
    .main-title {
        color: #00ff88;
        text-align: center;
        font-size: 3rem;
        font-weight: bold;
        margin-bottom: 30px;
        text-shadow: 0 0 20px rgba(0, 255, 136, 0.8);
        animation: pulse-glow 2s infinite alternate;
    }
    
    @keyframes pulse-glow {
        from { text-shadow: 0 0 20px rgba(0, 255, 136, 0.8); }
        to { text-shadow: 0 0 30px rgba(0, 255, 136, 1.0); }
    }
    
    /* Video container */
    .video-container {
        background: rgba(0, 0, 0, 0.9);
        border-radius: 15px;
        padding: 15px;
        border: 2px solid #00aaff;
        box-shadow: 0 0 25px rgba(0, 170, 255, 0.4);
        margin-bottom: 20px;
    }
    
    /* Control panel */
    .control-panel {
        background: rgba(0, 0, 0, 0.9);
        border-radius: 15px;
        padding: 20px;
        border: 2px solid #ff6b00;
        box-shadow: 0 0 25px rgba(255, 107, 0, 0.4);
        margin-bottom: 20px;
    }
    
    /* Status indicators with better visibility */
    .status-success {
        color: #00ff88;
        font-weight: bold;
        font-size: 1.1rem;
        text-shadow: 0 0 15px rgba(0, 255, 136, 0.8);
        background: rgba(0, 255, 136, 0.1);
        padding: 8px 12px;
        border-radius: 8px;
        border: 1px solid rgba(0, 255, 136, 0.3);
    }
    
    .status-error {
        color: #ff6666;
        font-weight: bold;
        font-size: 1.1rem;
        text-shadow: 0 0 15px rgba(255, 102, 102, 0.8);
        background: rgba(255, 102, 102, 0.1);
        padding: 8px 12px;
        border-radius: 8px;
        border: 1px solid rgba(255, 102, 102, 0.3);
    }
    
    .status-warning {
        color: #ffcc00;
        font-weight: bold;
        font-size: 1.1rem;
        text-shadow: 0 0 15px rgba(255, 204, 0, 0.8);
        background: rgba(255, 204, 0, 0.1);
        padding: 8px 12px;
        border-radius: 8px;
        border: 1px solid rgba(255, 204, 0, 0.3);
    }
    
    /* Input styling with better visibility */
    .stTextInput > div > div > input {
        background: rgba(30, 30, 30, 0.95) !important;
        color: #ffffff !important;
        border: 2px solid #00ff88 !important;
        border-radius: 10px !important;
        box-shadow: 0 0 15px rgba(0, 255, 136, 0.3) !important;
        font-weight: bold !important;
        font-size: 1.1rem !important;
    }
    
    .stTextInput > div > div > input:focus {
        border-color: #00aaff !important;
        box-shadow: 0 0 25px rgba(0, 170, 255, 0.5) !important;
    }
    
    /* Number input styling */
    .stNumberInput > div > div > input {
        background: rgba(30, 30, 30, 0.95) !important;
        color: #ffffff !important;
        border: 2px solid #ff6b00 !important;
        border-radius: 10px !important;
        box-shadow: 0 0 15px rgba(255, 107, 0, 0.3) !important;
        font-weight: bold !important;
        font-size: 1.1rem !important;
    }
    
    /* Button styling with enhanced visibility */
    .stButton > button {
        background: linear-gradient(45deg, #00ff88, #00aaff) !important;
        color: #000000 !important;
        font-weight: bold !important;
        border: none !important;
        border-radius: 15px !important;
        box-shadow: 0 0 20px rgba(0, 255, 136, 0.5) !important;
        transition: all 0.3s ease !important;
        padding: 15px 30px !important;
        font-size: 1.2rem !important;
        text-shadow: 0 0 5px rgba(0, 0, 0, 0.8) !important;
    }
    
    .stButton > button:hover {
        transform: translateY(-2px) !important;
        box-shadow: 0 0 35px rgba(0, 255, 136, 0.9) !important;
    }
    
    /* Log container with better contrast */
    .log-container {
        background: rgba(10, 10, 10, 0.98);
        border-radius: 10px;
        padding: 15px;
        border: 1px solid #444;
        box-shadow: inset 0 0 10px rgba(0, 0, 0, 0.8);
        font-family: 'Courier New', monospace;
        color: #ffffff;
        height: 300px;
        overflow-y: auto;
    }
    
    /* Section headers with better visibility */
    .section-header {
        color: #ffffff;
        font-size: 1.6rem;
        font-weight: bold;
        margin-bottom: 15px;
        text-shadow: 0 0 20px rgba(0, 170, 255, 0.8);
        background: rgba(0, 170, 255, 0.1);
        padding: 10px 15px;
        border-radius: 10px;
        border: 1px solid rgba(0, 170, 255, 0.3);
    }
    
    /* General text improvements */
    .stMarkdown, .stText, p, div {
        color: #ffffff !important;
    }
    
    /* Markdown text styling */
    .stMarkdown p {
        color: #ffffff !important;
        font-weight: 500 !important;
        font-size: 1.05rem !important;
    }
    
    /* Bold text for labels */
    .stMarkdown strong {
        color: #00aaff !important;
        text-shadow: 0 0 10px rgba(0, 170, 255, 0.6);
        font-size: 1.1rem;
    }
</style>
""", unsafe_allow_html=True)

class WaypointControllerUI:
    def __init__(self):
        self.process = None
        self.is_running = False
        self.log_queue = queue.Queue()
        self.video_enabled = True
        
    def start_video_capture(self):
        """Initialize video capture"""
        try:
            # Try different camera indices
            for i in range(3):
                cap = cv2.VideoCapture(i)
                if cap.isOpened():
                    return cap
            # If no camera found, return None
            return None
        except Exception as e:
            st.error(f"Failed to initialize camera: {e}")
            return None
    
    def get_video_frame(self, cap):
        """Get a frame from video capture"""
        if cap is None:
            # Generate a sophisticated placeholder frame
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            
            # Create gradient background
            for y in range(480):
                intensity = int(30 + (y / 480) * 50)
                frame[y, :] = [intensity//3, intensity//2, intensity]
            
            # Draw robot silhouette/outline
            cv2.circle(frame, (320, 240), 80, (0, 255, 136), 3)
            cv2.rectangle(frame, (280, 200), (360, 280), (0, 255, 136), 2)
            cv2.line(frame, (320, 160), (320, 200), (0, 255, 136), 4)
            cv2.circle(frame, (320, 150), 10, (0, 255, 136), -1)
            
            # Add informative text
            cv2.putText(frame, "CAMERA NOT CONNECTED", (120, 350), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(frame, "UR5e Robot Workspace View", (140, 380), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 136), 2)
            cv2.putText(frame, "Connect camera for live feed", (150, 410), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            
            # Add status indicator
            status_color = (0, 255, 136) if self.is_running else (255, 170, 0)
            status_text = "EXECUTING WAYPOINTS" if self.is_running else "READY TO EXECUTE"
            cv2.putText(frame, f"STATUS: {status_text}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
            
            return frame
        
        ret, frame = cap.read()
        if not ret:
            # Generate error frame with better visibility
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            frame[:] = [20, 20, 40]  # Dark blue background
            cv2.putText(frame, "CAMERA ERROR", (180, 220), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
            cv2.putText(frame, "Check camera connection", (160, 260), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 100, 100), 2)
            return frame
        
        # Add overlay information to live feed
        cv2.putText(frame, "UR5e LIVE FEED", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame, f"STATUS: {'EXECUTING' if self.is_running else 'READY'}", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 
                   (0, 255, 136) if self.is_running else (255, 170, 0), 2)
        
        return frame
    
    def validate_json_file(self, filename):
        """Validate if the JSON file exists and is properly formatted"""
        if not filename.endswith('.json'):
            return False, "File must have .json extension"
        
        if not os.path.exists(filename):
            return False, f"File '{filename}' not found"
        
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            if 'waypoints' not in data:
                return False, "JSON file must contain 'waypoints' array"
            
            waypoints = data['waypoints']
            if not waypoints:
                return False, "Waypoints array is empty"
            
            for i, waypoint in enumerate(waypoints):
                if 'joints' not in waypoint:
                    return False, f"Waypoint {i+1} missing 'joints' array"
                if len(waypoint['joints']) != 6:
                    return False, f"Waypoint {i+1} must have 6 joint values"
            
            return True, f"Valid waypoint file with {len(waypoints)} waypoints"
            
        except json.JSONDecodeError as e:
            return False, f"Invalid JSON format: {e}"
        except Exception as e:
            return False, f"File validation error: {e}"
    
    def execute_waypoint_sequence(self, filename, delay=0.05):
        """Execute waypoint sequence in a separate thread"""
        def run_controller():
            try:
                self.is_running = True
                cmd = [
                    "python3", "waypoint_controller.py"
                ]
                
                # Create process with pipes for communication
                self.process = subprocess.Popen(
                    cmd,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    cwd="/home/yc/panasonic/clip_proj",
                    bufsize=1
                )
                
                # Wait for controller to be ready
                time.sleep(2)
                
                # Send execute command
                execute_cmd = f"execute {filename} {delay}\n"
                self.process.stdin.write(execute_cmd)
                self.process.stdin.flush()
                
                # Read output
                while self.process.poll() is None:
                    line = self.process.stdout.readline()
                    if line:
                        self.log_queue.put(line.strip())
                
                # Send quit command
                self.process.stdin.write("quit\n")
                self.process.stdin.flush()
                
            except Exception as e:
                self.log_queue.put(f"❌ Error: {e}")
            finally:
                self.is_running = False
                self.process = None
        
        thread = threading.Thread(target=run_controller)
        thread.daemon = True
        thread.start()
    
    def stop_execution(self):
        """Stop the current execution"""
        if self.process:
            try:
                self.process.stdin.write("quit\n")
                self.process.stdin.flush()
                self.process.terminate()
                time.sleep(1)
                if self.process.poll() is None:
                    self.process.kill()
            except:
                pass
        self.is_running = False
        self.process = None
    
    def get_logs(self):
        """Get accumulated logs"""
        logs = []
        while not self.log_queue.empty():
            try:
                logs.append(self.log_queue.get_nowait())
            except queue.Empty:
                break
        return logs

# Initialize the controller and session state
if 'controller' not in st.session_state:
    st.session_state.controller = WaypointControllerUI()
    st.session_state.logs = []
    st.session_state.last_video_update = 0
    st.session_state.video_frame = None

controller = st.session_state.controller

# Main title
st.markdown('<h1 class="main-title">Panasonic</h1>', unsafe_allow_html=True)

# Create two columns for layout
col1, col2 = st.columns([3, 2])

# Left column - Video feed
with col1:
    st.markdown('<div class="section-header"> Live Feed</div>', unsafe_allow_html=True)
    st.markdown('<div class="video-container">', unsafe_allow_html=True)
    
    video_placeholder = st.empty()
    
    # Initialize video capture
    if 'cap' not in st.session_state:
        st.session_state.cap = controller.start_video_capture()
    
    # Update video frame only every 2 seconds to reduce flickering
    current_time = time.time()
    if (current_time - st.session_state.last_video_update > 2.0 or 
        st.session_state.video_frame is None):
        frame = controller.get_video_frame(st.session_state.cap)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        st.session_state.video_frame = frame_rgb
        st.session_state.last_video_update = current_time
    
    # Display cached video frame
    video_placeholder.image(st.session_state.video_frame, channels="RGB", width="stretch")
    
    st.markdown('</div>', unsafe_allow_html=True)

# Right column - Control panel
with col2:
    st.markdown('<div class="section-header">Control Panel</div>', unsafe_allow_html=True)
    st.markdown('<div class="control-panel">', unsafe_allow_html=True)
    
    # File input
    st.markdown("**pick up:**")
    json_filename = st.text_input(
        "Enter filename:",
        value="setA.json",
        placeholder="e.g., seta.json, myWaypoints.json",
        label_visibility="collapsed"
    )
    
    # Delay input
    st.markdown("**Inter-waypoint Delay (seconds):**")
    delay = st.number_input(
        "Delay:",
        min_value=0.0,
        max_value=5.0,
        value=0.05,
        step=0.01,
        format="%.3f",
        label_visibility="collapsed"
    )
    
    # Validate file
    if json_filename:
        is_valid, message = controller.validate_json_file(json_filename)
        if is_valid:
            st.markdown(f'<p class="status-success"> {message}</p>', unsafe_allow_html=True)
        else:
            st.markdown(f'<p class="status-error"> {message}</p>', unsafe_allow_html=True)
    
    st.markdown("</div>", unsafe_allow_html=True)
    
    # Control buttons
    col_btn1, col_btn2 = st.columns(2)
    
    with col_btn1:
        if st.button("START", disabled=controller.is_running or not json_filename):
            if json_filename:
                is_valid, _ = controller.validate_json_file(json_filename)
                if is_valid:
                    st.session_state.logs.append(f" Starting execution of {json_filename}")
                    controller.execute_waypoint_sequence(json_filename, delay)
                    st.rerun()
    
    with col_btn2:
        if st.button(" STOP", disabled=not controller.is_running):
            controller.stop_execution()
            st.session_state.logs.append(" Execution stopped")
            st.rerun()
    
    # Status indicator
    if controller.is_running:
        st.markdown('<p class="status-success">🟢 EXECUTING WAYPOINTS</p>', unsafe_allow_html=True)
    else:
        st.markdown('<p class="status-warning">🟡 READY</p>', unsafe_allow_html=True)

# Bottom section - Logs
st.markdown('<div class="section-header"> Execution Logs</div>', unsafe_allow_html=True)

# Get new logs from controller only if there are any
new_logs = controller.get_logs()
if new_logs:
    st.session_state.logs.extend(new_logs)
    # Force a rerun only when we have new logs
    if controller.is_running:
        st.rerun()

# Display logs (use cached logs to avoid flickering)
log_container = st.container()
with log_container:
    st.markdown('<div class="log-container">', unsafe_allow_html=True)
    
    # Show recent logs (last 20)
    recent_logs = st.session_state.logs[-20:] if len(st.session_state.logs) > 20 else st.session_state.logs
    
    if recent_logs:
        for log in recent_logs:
            st.text(log)
    else:
        st.text(" Ready to execute waypoint sequences...")
        st.text(" Load a JSON waypoint file and click START")
    
    st.markdown('</div>', unsafe_allow_html=True)

# Controlled refresh for live updates - only when necessary
if controller.is_running:
    # Only refresh if we haven't refreshed recently
    if 'last_refresh' not in st.session_state:
        st.session_state.last_refresh = 0
    
    current_time = time.time()
    if current_time - st.session_state.last_refresh > 1.0:  # Refresh every 1 second max
        st.session_state.last_refresh = current_time
        time.sleep(0.5)  # Brief pause before refresh
        st.rerun()

# Cleanup on app shutdown
def cleanup():
    if 'controller' in st.session_state:
        st.session_state.controller.stop_execution()
    if 'cap' in st.session_state and st.session_state.cap:
        st.session_state.cap.release()

# Register cleanup
import atexit
atexit.register(cleanup)
