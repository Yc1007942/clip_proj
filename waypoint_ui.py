"""
 streamlit run waypoint_ui.py --server.headless true --server.enableCORS false --server.enableXsrfProtection false
"""

import streamlit as st
import subprocess
import threading
import time
import json
import cv2
import numpy as np
import queue
from pathlib import Path
from html import escape

st.set_page_config(
    page_title="Panasonic",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="collapsed"
)
st.markdown("""
<style>
:root {
    --bg-primary: #0b1120;
    --bg-secondary: rgba(15, 23, 42, 0.82);
    --border: rgba(148, 163, 184, 0.22);
    --accent: #38bdf8;
    --accent-strong: #6366f1;
    --success: #34d399;
    --warning: #fbbf24;
    --error: #f87171;
    --text-primary: #e2e8f0;
    --text-muted: #94a3b8;
}

.stApp::before,
.stApp::after {
    content: "";
    position: fixed;
    border-radius: 999px;
    filter: blur(60px);
    opacity: 0.4;
    pointer-events: none;
    z-index: 0;
}

.stApp::before {
    width: 420px;
    height: 420px;
    top: -140px;
    left: -120px;
    background: radial-gradient(circle, rgba(56, 189, 248, 0.55), transparent 65%);
    animation: floatOrb1 16s ease-in-out infinite;
}

.stApp::after {
    width: 520px;
    height: 520px;
    bottom: -200px;
    right: -160px;
    background: radial-gradient(circle, rgba(99, 102, 241, 0.48), transparent 65%);
    animation: floatOrb2 22s ease-in-out infinite;
}

.background-orbs {
    position: fixed;
    inset: 0;
    z-index: 0;
    pointer-events: none;
}

.background-orbs span {
    position: absolute;
    border-radius: 999px;
    filter: blur(80px);
    opacity: 0.22;
    mix-blend-mode: screen;
    animation: glowPulse 12s ease-in-out infinite;
}

.background-orbs .orb-a { width: 280px; height: 280px; top: 20%; left: 10%; background: rgba(14, 165, 233, 0.6);} 
.background-orbs .orb-b { width: 340px; height: 340px; top: 65%; left: 35%; background: rgba(14, 116, 144, 0.55);} 
.background-orbs .orb-c { width: 260px; height: 260px; top: 40%; right: 8%; background: rgba(139, 92, 246, 0.55);} 

.stApp {
    background:
        radial-gradient(circle at 15% 15%, rgba(56, 189, 248, 0.09), transparent 45%),
        radial-gradient(circle at 85% 10%, rgba(99, 102, 241, 0.14), transparent 40%),
        linear-gradient(135deg, #050b1a 0%, #0f172a 50%, #030711 100%);
    color: var(--text-primary) !important;
    font-family: "Inter", "Segoe UI", system-ui, -apple-system, BlinkMacSystemFont, sans-serif;
}

.block-container {
    width: 100%;
    max-width: none;
    padding: 2rem 4vw 3rem;
    position: relative;
    z-index: 1;
}

.main-title {
    color: #f8fafc;
    text-align: center;
    font-size: 2.5rem;
    font-weight: 700;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    margin: 0 0 1.5rem 0;
    text-shadow: 0 20px 40px rgba(8, 47, 73, 0.35);
}

.section-header {
    display: inline-flex;
    align-items: center;
    gap: 0.5rem;
    margin-bottom: 1rem;
    font-size: 0.8rem;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 0.18rem;
    color: var(--text-muted);
}

.section-header::before {
    content: "";
    display: inline-block;
    width: 2.25rem;
    height: 2px;
    border-radius: 999px;
    background: var(--accent);
    opacity: 0.8;
}

.video-container,
.control-panel,
.log-container {
    position: relative;
    background: var(--bg-secondary);
    border-radius: 22px;
    padding: 1.35rem 1.6rem;
    border: 1px solid rgba(148, 163, 184, 0.18);
    box-shadow: 0 32px 55px rgba(7, 11, 25, 0.35);
    backdrop-filter: blur(18px);
}

.video-container::after,
.control-panel::after,
.log-container::after {
    content: "";
    position: absolute;
    inset: 0;
    border-radius: inherit;
    pointer-events: none;
    border: 1px solid rgba(148, 163, 184, 0.08);
}

.video-container {
    min-height: 420px;
}

.control-panel {
    min-height: 420px;
}

.log-container {
    margin-top: 1rem;
    font-family: "JetBrains Mono", "Fira Code", monospace;
    color: var(--text-primary);
    height: 320px;
    overflow-y: auto;
    background: rgba(11, 18, 33, 0.76);
}

.stMarkdown, .stText, p, span, label {
    color: var(--text-primary) !important;
}

.stMarkdown strong {
    color: #f8fafc !important;
    font-weight: 600 !important;
}

.helper-text {
    color: var(--text-muted) !important;
    font-size: 0.82rem;
    margin: 0.6rem 0 0;
}

.metric-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
    gap: 1.1rem;
    margin: 0 0 1.5rem 0;
}

.metric-card {
    position: relative;
    padding: 1.1rem 1.25rem;
    border-radius: 18px;
    background: rgba(13, 21, 38, 0.78);
    border: 1px solid rgba(148, 163, 184, 0.28);
    box-shadow: 0 22px 45px rgba(5, 9, 25, 0.35);
    overflow: hidden;
}

.metric-card::after {
    content: "";
    position: absolute;
    inset: 0;
    border-radius: inherit;
    background: linear-gradient(135deg, rgba(148, 163, 184, 0.08), transparent 55%);
    pointer-events: none;
}

.metric-label {
    font-size: 0.75rem;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 0.16rem;
    color: var(--text-muted);
}

.metric-value {
    margin-top: 0.45rem;
    font-size: 1.45rem;
    font-weight: 600;
    color: #f9fafb;
    letter-spacing: 0.02em;
}

.metric-value--sm {
    font-size: 1rem;
    font-weight: 500;
    line-height: 1.35;
}

.metric-helper {
    margin-top: 0.35rem;
    font-size: 0.8rem;
    color: var(--text-muted);
}

.metric-pill {
    display: inline-flex;
    align-items: center;
    gap: 0.4rem;
    font-size: 0.72rem;
    font-weight: 600;
    letter-spacing: 0.12rem;
    text-transform: uppercase;
    padding: 0.4rem 0.8rem;
    border-radius: 999px;
    border: 1px solid transparent;
    margin-top: 0.6rem;
}

.metric-pill--idle {
    color: var(--warning);
    background: rgba(251, 191, 36, 0.12);
    border-color: rgba(251, 191, 36, 0.28);
}

.metric-pill--running {
    color: var(--success);
    background: rgba(34, 197, 94, 0.14);
    border-color: rgba(34, 197, 94, 0.28);
}

.metric-pill--error {
    color: var(--error);
    background: rgba(248, 113, 113, 0.14);
    border-color: rgba(248, 113, 113, 0.28);
}

.stTextInput > div > div > input,
.stNumberInput > div > div > input {
    background: rgba(12, 22, 41, 0.78) !important;
    color: var(--text-primary) !important;
    border: 1px solid transparent !important;
    border-radius: 14px !important;
    padding: 0.75rem 1rem !important;
    font-weight: 500 !important;
    transition: border 0.2s ease, box-shadow 0.2s ease;
    box-shadow: inset 0 0 0 1px rgba(148, 163, 184, 0.16) !important;
}

.stTextInput > div > div > input:focus,
.stNumberInput > div > div > input:focus {
    border: 1px solid rgba(56, 189, 248, 0.8) !important;
    box-shadow: 0 0 26px rgba(56, 189, 248, 0.2) !important;
}

.stButton > button {
    width: 100%;
    max-width: 180px;
    background: linear-gradient(135deg, var(--accent) 0%, var(--accent-strong) 100%) !important;
    color: #0b1120 !important;
    font-weight: 600 !important;
    border-radius: 16px !important;
    border: none !important;
    padding: 0.75rem 1.6rem !important;
    font-size: 0.95rem !important;
    letter-spacing: 0.02em;
    box-shadow: 0 18px 32px rgba(56, 189, 248, 0.35) !important;
    transition: transform 0.18s ease, box-shadow 0.18s ease;
}

.stButton > button:hover {
    transform: translateY(-1px) scale(1.01) !important;
    box-shadow: 0 28px 48px rgba(56, 189, 248, 0.45) !important;
}

.status-success,
.status-warning,
.status-error {
    display: inline-flex;
    align-items: center;
    gap: 0.55rem;
    font-weight: 600;
    font-size: 0.85rem;
    padding: 0.55rem 0.9rem;
    border-radius: 999px;
    border: 1px solid transparent;
    backdrop-filter: blur(10px);
    margin-top: 0.75rem;
}

.status-success {
    color: var(--success);
    background: rgba(52, 211, 153, 0.12);
    border-color: rgba(52, 211, 153, 0.25);
}

.status-warning {
    color: var(--warning);
    background: rgba(251, 191, 36, 0.12);
    border-color: rgba(251, 191, 36, 0.25);
}

.status-error {
    color: var(--error);
    background: rgba(248, 113, 113, 0.12);
    border-color: rgba(248, 113, 113, 0.25);
}

.status-card {
    display: flex;
    align-items: flex-start;
    gap: 0.75rem;
    margin: 1.25rem 0 0.75rem;
    padding: 1rem 1.25rem;
    border-radius: 18px;
    border: 1px solid rgba(148, 163, 184, 0.22);
    background: rgba(15, 23, 42, 0.72);
    box-shadow: inset 0 0 0 1px rgba(148, 163, 184, 0.12);
}

.status-card__indicator {
    position: relative;
    width: 12px;
    height: 12px;
    border-radius: 999px;
    margin-top: 0.25rem;
    box-shadow: 0 0 0 6px rgba(56, 189, 248, 0.12);
}

.status-card__indicator::after {
    content: "";
    position: absolute;
    inset: -6px;
    border-radius: inherit;
    border: 1px solid currentColor;
    opacity: 0.35;
    animation: ripple 2.8s ease-out infinite;
}

.status-card__content {
    display: flex;
    flex-direction: column;
    gap: 0.2rem;
}

.status-card__title {
    display: flex;
    align-items: center;
    gap: 0.45rem;
    font-size: 1rem;
    font-weight: 600;
    letter-spacing: 0.03em;
}

.status-card__subtitle {
    font-size: 0.82rem;
    color: var(--text-muted);
}

.status-card--running {
    border-color: rgba(52, 211, 153, 0.3);
    background: linear-gradient(135deg, rgba(34, 197, 94, 0.12), rgba(15, 23, 42, 0.72));
}

.status-card--running .status-card__indicator {
    background: var(--success);
    box-shadow: 0 0 0 6px rgba(52, 211, 153, 0.15);
}

.status-card--idle {
    border-color: rgba(251, 191, 36, 0.28);
    background: linear-gradient(135deg, rgba(251, 191, 36, 0.12), rgba(15, 23, 42, 0.72));
}

.status-card--idle .status-card__indicator {
    background: var(--warning);
    box-shadow: 0 0 0 6px rgba(251, 191, 36, 0.16);
}

@keyframes floatOrb1 {
    0%, 100% { transform: translate3d(0, 0, 0) scale(1); }
    40% { transform: translate3d(30px, 20px, 0) scale(1.05); }
    70% { transform: translate3d(-25px, 30px, 0) scale(0.95); }
}

@keyframes floatOrb2 {
    0%, 100% { transform: translate3d(0, 0, 0) scale(1); }
    50% { transform: translate3d(-40px, -30px, 0) scale(1.08); }
}

@keyframes glowPulse {
    0%, 100% { transform: translateY(0) scale(1); opacity: 0.22; }
    50% { transform: translateY(-12px) scale(1.08); opacity: 0.32; }
}

@keyframes ripple {
    0% { transform: scale(0.6); opacity: 0.6; }
    100% { transform: scale(1.6); opacity: 0; }
}
</style>
""", unsafe_allow_html=True)

st.markdown('<div class="background-orbs"><span class="orb-a"></span><span class="orb-b"></span><span class="orb-c"></span></div>', unsafe_allow_html=True)

class WaypointControllerUI:
    def __init__(self):
        self.process = None
        self.is_running = False
        self.log_queue = queue.Queue()
        self.base_dir = Path(__file__).resolve().parent
    
    def resolve_json_path(self, filename):
        """Return absolute path to the JSON file within the project"""
        path = Path(filename)
        if not path.is_absolute():
            path = self.base_dir / path
        return path
        
    def start_video_capture(self):
        """Initialize video capture"""
        try:
            # Try different camera indices
            for i in range(3):
                cap = cv2.VideoCapture(i)
                if cap.isOpened():
                    return cap
            return None
        except Exception as e:
            st.error(f"Failed to initialize camera: {e}")
            return None
    
    def get_video_frame(self, cap):
        """Get a frame from video capture"""
        if cap is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            for y in range(480):
                intensity = int(30 + (y / 480) * 50)
                frame[y, :] = [intensity//3, intensity//2, intensity]
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
        if not filename.lower().endswith('.json'):
            return False, "File must have .json extension"

        path = self.resolve_json_path(filename)

        if not path.exists():
            return False, f"File '{path.name}' not found"

        try:
            with path.open('r') as f:
                data = json.load(f)

            waypoints = data.get('waypoints')
            if not isinstance(waypoints, list) or not waypoints:
                return False, "JSON file must contain a non-empty 'waypoints' array"

            for i, waypoint in enumerate(waypoints):
                pose = waypoint.get('pose')
                if not isinstance(pose, list) or len(pose) != 6:
                    return False, f"Waypoint {i + 1} must include a 6-value 'pose' array"
                joints = waypoint.get('joints')
                if joints is not None and len(joints) != 6:
                    return False, f"Waypoint {i + 1} has {len(joints)} joints; expected 6"

            return True, f"Valid waypoint file with {len(waypoints)} waypoints"

        except json.JSONDecodeError as e:
            return False, f"Invalid JSON format: {e}"
        except Exception as e:
            return False, f"File validation error: {e}"
    
    def execute_waypoint_sequence(self, filename, velocity_scale=0.3):
        """Execute waypoint sequence using the MoveIt service-based controller"""

        def run_controller():
            try:
                self.is_running = True
                file_path = self.resolve_json_path(filename)

                if not file_path.exists():
                    self.log_queue.put(f" File '{file_path}' not found")
                    return

                self.log_queue.put(f" Launching MoveIt controller for {file_path.name}")

                cmd = [
                    "python3",
                    "-u",
                    "waypoint_service_moveit.py",
                    "--file",
                    str(file_path),
                    "--velocity-scale",
                    f"{velocity_scale:.3f}"
                ]

                self.process = subprocess.Popen(
                    cmd,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    cwd=str(self.base_dir),
                    bufsize=1
                )

                # Auto-confirm dynamic coordinate updates when needed
                filename_lower = file_path.name.lower()
                if filename_lower.startswith("sete") or filename_lower.startswith("setf"):
                    try:
                        self.process.stdin.write("y\n")
                        self.process.stdin.flush()
                    except Exception as confirm_error:
                        self.log_queue.put(f" Auto-confirm failed: {confirm_error}")

                # Stream process output to the UI log
                while True:
                    line = self.process.stdout.readline()
                    if line:
                        self.log_queue.put(line.rstrip())
                    elif self.process.poll() is not None:
                        break

                return_code = self.process.wait()
                self.log_queue.put(f"Process exited with code {return_code}")

            except Exception as e:
                self.log_queue.put(f" Error: {e}")
            finally:
                if self.process:
                    try:
                        if self.process.stdin and not self.process.stdin.closed:
                            self.process.stdin.close()
                    except Exception:
                        pass
                    try:
                        if self.process.stdout and not self.process.stdout.closed:
                            self.process.stdout.close()
                    except Exception:
                        pass
                self.is_running = False
                self.process = None

        thread = threading.Thread(target=run_controller, name="waypoint-runner", daemon=True)
        thread.start()
    
    def stop_execution(self):
        """Stop the current execution"""
        if self.process:
            try:
                if self.process.stdin and not self.process.stdin.closed:
                    self.process.stdin.close()
                self.process.terminate()
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
            except Exception:
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
    st.session_state.cleanup_registered = False

controller = st.session_state.controller

if "json_filename_input" not in st.session_state:
    st.session_state.json_filename_input = "seta.json"

if "velocity_scale_input" not in st.session_state:
    st.session_state.velocity_scale_input = 0.30

active_file_name = st.session_state.json_filename_input
file_display = escape(Path(active_file_name).name) if active_file_name else "—"
velocity_display = f"{st.session_state.velocity_scale_input:.2f}×"

raw_last_log = st.session_state.logs[-1] if st.session_state.logs else "Awaiting first execution"
display_log = raw_last_log
if len(display_log) > 75:
    display_log = f"{display_log[:72]}…"
last_log_display = escape(display_log)

status_badge_label = "Running" if controller.is_running else "Idle"
status_badge_class = "metric-pill metric-pill--running" if controller.is_running else "metric-pill metric-pill--idle"
status_icon = "" if controller.is_running else "_"

lower_log = raw_last_log.lower()
if "error" in lower_log or "" in raw_last_log:
    status_badge_label = "Attention"
    status_badge_class = "metric-pill metric-pill--error"
    status_icon = ""

metrics_html = f"""
    <div class="metric-grid">
        <div class="metric-card">
            <span class="metric-label">Active File</span>
            <span class="metric-value">{file_display}</span>
            <span class="metric-helper">Current waypoint library</span>
        </div>
        <div class="metric-card">
            <span class="metric-label">Velocity Scale</span>
            <span class="metric-value">{velocity_display}</span>
            <span class="metric-helper">Motion multiplier for the MoveIt plan</span>
        </div>
        <div class="metric-card">
            <span class="metric-label">Controller Status</span>
            <span class="metric-value metric-value--sm">{last_log_display}</span>
            <span class="{status_badge_class}">{status_icon} {status_badge_label}</span>
        </div>
    </div>
"""

# Main title
st.markdown('<h1 class="main-title">Panasonic</h1>', unsafe_allow_html=True)

st.markdown(metrics_html, unsafe_allow_html=True)

# Create two columns for layout
col1, col2 = st.columns([3, 2])

# Left column - Video feed
with col1:
    st.markdown('<div class="section-header">Live Feed</div>', unsafe_allow_html=True)
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
    video_placeholder.image(st.session_state.video_frame, channels="RGB", width='stretch')
    st.markdown('<p class="helper-text">Live camera feed with a fallback visualization when the camera is offline.</p>', unsafe_allow_html=True)
    
    st.markdown('</div>', unsafe_allow_html=True)

# Right column - Control panel
with col2:
    st.markdown('<div class="section-header">Control Panel</div>', unsafe_allow_html=True)
    st.markdown('<div class="control-panel">', unsafe_allow_html=True)
    
    # File input
    st.markdown("**Task:**")
    st.text_input(
        "Enter filename:",
        value=st.session_state.json_filename_input,
        placeholder="e.g., setf.json, setz.json",
        label_visibility="collapsed",
        key="json_filename_input"
    )
    json_filename = st.session_state.json_filename_input

    # Velocity scale input
    st.markdown("**Velocity Scale (0.05 – 1.0):**")
    st.number_input(
        "Velocity Scale:",
        min_value=0.05,
        max_value=1.0,
        value=st.session_state.velocity_scale_input,
        step=0.05,
        format="%.2f",
        label_visibility="collapsed",
        key="velocity_scale_input"
    )
    velocity_scale = st.session_state.velocity_scale_input
    st.markdown('<p class="helper-text">Tune motion speed for the MoveIt controller. Lower values produce slower, safer motion.</p>', unsafe_allow_html=True)
    
    # Validate file
    if json_filename:
        is_valid, message = controller.validate_json_file(json_filename)
        if is_valid:
            st.markdown(f'<p class="status-success"> {message}</p>', unsafe_allow_html=True)
        else:
            st.markdown(f'<p class="status-error"> {message}</p>', unsafe_allow_html=True)

    status_state = "running" if controller.is_running else "idle"
    status_title = "Executing motion plan" if controller.is_running else "Ready to execute"
    status_icon = "" if controller.is_running else "ok"
    status_subtitle = (
        "Robot motion in progress via the MoveIt controller."
        if controller.is_running else
        "Select task file and press START to begin."
    )

    status_card_html = f"""
        <div class="status-card status-card--{status_state}">
            <span class="status-card__indicator"></span>
            <div class="status-card__content">
                <span class="status-card__title">{status_icon} {status_title}</span>
                <span class="status-card__subtitle">{status_subtitle}</span>
            </div>
        </div>
    """
    st.markdown(status_card_html, unsafe_allow_html=True)

    st.markdown("</div>", unsafe_allow_html=True)
    
    # Control buttons
    col_btn1, col_btn2 = st.columns(2)
    
    with col_btn1:
        if st.button("START", disabled=controller.is_running or not json_filename):
            if json_filename:
                is_valid, _ = controller.validate_json_file(json_filename)
                if is_valid:
                    st.session_state.logs.append(f" Starting execution of {json_filename} at {velocity_scale:.2f} velocity scale")
                    controller.execute_waypoint_sequence(json_filename, velocity_scale)
                    st.rerun()

    with col_btn2:
        if st.button(" STOP", disabled=not controller.is_running):
            controller.stop_execution()
            st.session_state.logs.append("🛑 Execution stopped")
            st.rerun()
    
# Bottom section - Logs
st.markdown('<div class="section-header">Execution Logs</div>', unsafe_allow_html=True)

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

st.markdown('<p class="helper-text">Streaming recent output from the MoveIt controller and waypoint runner.</p>', unsafe_allow_html=True)

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

if not st.session_state.get('cleanup_registered', False):
    atexit.register(cleanup)
    st.session_state.cleanup_registered = True
