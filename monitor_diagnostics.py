#!/usr/bin/env python3
"""
Monitor diagnostics from global_localization_node
Tracks ROS logs and system metrics during execution
"""

import subprocess
import threading
import time
import re
import os
import signal
from collections import defaultdict
from datetime import datetime

class DiagnosticsMonitor:
    def __init__(self, scenario_name):
        self.scenario_name = scenario_name
        self.diagnostics = defaultdict(list)
        self.running = True
        self.process = None
        self.monitor_thread = None
        self.start_time = None
        self.end_time = None
        
    def log_line(self, level, msg):
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"[{timestamp}] [{self.scenario_name}] {level}: {msg}")
    
    def parse_diagnostic_log(self, line):
        """Parse [DIAGNOSTIC] patterns from ROS logs"""
        if "[DIAGNOSTIC]" in line:
            # Extract metrics from log line
            match = re.search(r'Frames: (\d+).*Empty: (\d+).*Total: ([\d.]+) ms.*read: ([\d.]+) ms.*process: ([\d.]+) ms.*json: ([\d.]+) ms.*pub: ([\d.]+) ms', line)
            if match:
                return {
                    'frames': int(match.group(1)),
                    'empty': int(match.group(2)),
                    'total_ms': float(match.group(3)),
                    'read_ms': float(match.group(4)),
                    'process_ms': float(match.group(5)),
                    'json_ms': float(match.group(6)),
                    'pub_ms': float(match.group(7))
                }
        return None
    
    def parse_detection_log(self, line):
        """Parse [DETECTION] patterns from ROS logs"""
        if "[DETECTION]" in line:
            match = re.search(
                r'Initial: (\d+) markers.*Tables B4 rescue: (\d+).*Tables after: (\d+).*Rescue helped: (\d+).*solvePnP ok: (\d+).*fail: (\d+)',
                line
            )
            if match:
                return {
                    'initial_markers': int(match.group(1)),
                    'table_before_rescue': int(match.group(2)),
                    'table_after_rescue': int(match.group(3)),
                    'rescue_helped': int(match.group(4)),
                    'solvepnp_ok': int(match.group(5)),
                    'solvepnp_fail': int(match.group(6))
                }
        return None
    
    def monitor_logs(self):
        """Monitor ROS logs for diagnostics"""
        try:
            ros_logs = subprocess.Popen(
                ['ros2', 'topic', 'echo', '/rosout', '--no-arr'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            
            for line in ros_logs.stdout:
                if not self.running:
                    break
                
                if 'global_localization_node' in line or 'camera_map_visualizer_node' in line:
                    # Parse diagnostic line
                    diag = self.parse_diagnostic_log(line)
                    if diag:
                        self.diagnostics['timing'].append(diag)
                    
                    # Parse detection line
                    detect = self.parse_detection_log(line)
                    if detect:
                        self.diagnostics['detection'].append(detect)
            
            ros_logs.terminate()
        except Exception as e:
            self.log_line("ERROR", f"Monitor thread error: {e}")
    
    def get_process_stats(self, pid):
        """Get CPU and memory usage for a process"""
        try:
            result = subprocess.run(
                ['ps', '-p', str(pid), '-o', 'pid,vsz,rss,%cpu,%mem,comm'],
                capture_output=True,
                text=True
            )
            if result.stdout.strip().split('\n')[1:]:
                return result.stdout.strip().split('\n')[1].split()
        except:
            pass
        return None
    
    def get_pid(self, name):
        """Find process ID by name"""
        try:
            result = subprocess.run(
                ['pgrep', '-f', name],
                capture_output=True,
                text=True
            )
            if result.stdout:
                return int(result.stdout.strip().split('\n')[0])
        except:
            pass
        return None
    
    def print_summary(self):
        """Print collected diagnostics summary"""
        print("\n" + "="*80)
        print(f"SCENARIO: {self.scenario_name}")
        print("="*80)
        
        if self.diagnostics['timing']:
            last_timing = self.diagnostics['timing'][-1]
            avg_timing = self.compute_avg(self.diagnostics['timing'])
            print("\nTiming Statistics (last measurement):")
            print(f"  Frames processed: {last_timing['frames']}")
            print(f"  Empty frames: {last_timing['empty']}")
            print(f"  Frame read time: {last_timing['read_ms']:.2f} ms")
            print(f"  Frame process time: {last_timing['process_ms']:.2f} ms")
            print(f"  JSON build time: {last_timing['json_ms']:.2f} ms")
            print(f"  Publish time: {last_timing['pub_ms']:.2f} ms")
            print(f"  Total time: {last_timing['total_ms']:.2f} ms")
            print("\nTiming Statistics (average):")
            print(f"  Frame read time: {avg_timing['read_ms']:.2f} ms")
            print(f"  Frame process time: {avg_timing['process_ms']:.2f} ms")
            print(f"  JSON build time: {avg_timing['json_ms']:.2f} ms")
            print(f"  Publish time: {avg_timing['pub_ms']:.2f} ms")
            print(f"  Total time: {avg_timing['total_ms']:.2f} ms")
        
        if self.diagnostics['detection']:
            last_detect = self.diagnostics['detection'][-1]
            avg_detect = self.compute_avg_detect(self.diagnostics['detection'])
            print("\nDetection Statistics (last measurement):")
            print(f"  Initial markers detected: {last_detect['initial_markers']}")
            print(f"  Table markers before rescue: {last_detect['table_before_rescue']}")
            print(f"  Table markers after rescue: {last_detect['table_after_rescue']}")
            print(f"  Times rescue pass helped: {last_detect['rescue_helped']}")
            print(f"  solvePnP successes: {last_detect['solvepnp_ok']}")
            print(f"  solvePnP failures: {last_detect['solvepnp_fail']}")
            print("\nDetection Statistics (average per measurement):")
            print(f"  Avg initial markers: {avg_detect['initial_markers']:.1f}")
            print(f"  Avg table markers before rescue: {avg_detect['table_before_rescue']:.1f}")
            print(f"  Avg table markers after rescue: {avg_detect['table_after_rescue']:.1f}")
            
            total_detect = len(self.diagnostics['detection'])
            total_rescue_helped = sum(d['rescue_helped'] for d in self.diagnostics['detection'])
            if total_rescue_helped > 0:
                print(f"\n  Total measurements: {total_detect}")
                print(f"  Total rescue passes that helped: {total_rescue_helped}")
                print(f"  Rescue pass effectiveness: {(total_rescue_helped / total_detect * 100):.1f}%")
        
        print("\n")
    
    def compute_avg(self, timing_list):
        """Compute average of timing measurements"""
        if not timing_list:
            return {}
        keys = timing_list[0].keys()
        avg = {}
        for key in keys:
            if key != 'frames' and key != 'empty':
                avg[key] = sum(t[key] for t in timing_list) / len(timing_list)
        return avg
    
    def compute_avg_detect(self, detect_list):
        """Compute average of detection measurements"""
        if not detect_list:
            return {}
        keys = detect_list[0].keys()
        avg = {}
        for key in keys:
            avg[key] = sum(d[key] for d in detect_list) / len(detect_list)
        return avg


def run_scenario(scenario_name, commands):
    """Run a test scenario"""
    print(f"\n{'='*80}")
    print(f"Starting scenario: {scenario_name}")
    print(f"{'='*80}\n")
    
    monitor = DiagnosticsMonitor(scenario_name)
    monitor.start_time = time.time()
    
    # Start log monitoring in background
    monitor_thread = threading.Thread(target=monitor.monitor_logs, daemon=True)
    monitor_thread.start()
    
    # Run commands
    processes = []
    try:
        for cmd in commands:
            print(f"Starting: {' '.join(cmd)}")
            p = subprocess.Popen(cmd)
            processes.append(p)
        
        # Let it run for 30 seconds
        print(f"Running for 30 seconds...")
        time.sleep(30)
        
    finally:
        monitor.running = False
        monitor.end_time = time.time()
        
        # Kill all processes
        for p in processes:
            try:
                p.terminate()
                p.wait(timeout=5)
            except:
                p.kill()
        
        # Give time for logs to flush
        time.sleep(2)
        
        # Print summary
        monitor.print_summary()
        return monitor


if __name__ == '__main__':
    import sys
    
    # Source ros setup
    os.system('source /home/isaac/PFE_Eurobot_2026/server_ws/install/setup.bash')
    
    print("Table Marker Detection Diagnostic Monitor")
    print("This will run two scenarios and compare diagnostics\n")
    
    # Scenario 1: global_localization_node alone
    monitor1 = run_scenario(
        "global_localization_node ALONE",
        [['ros2', 'run', 'camera_localization', 'global_localization_node']]
    )
    
    time.sleep(5)
    
    # Kill any remaining nodes
    os.system('pkill -f global_localization_node')
    os.system('pkill -f camera_map_visualizer')
    time.sleep(2)
    
    # Scenario 2: both nodes running
    monitor2 = run_scenario(
        "global_localization_node + camera_map_visualizer",
        [
            ['ros2', 'run', 'camera_localization', 'global_localization_node'],
            ['ros2', 'run', 'bringup', 'camera_map_visualizer']
        ]
    )
    
    # Kill any remaining nodes
    os.system('pkill -f global_localization_node')
    os.system('pkill -f camera_map_visualizer')
    
    print("\n" + "="*80)
    print("COMPARISON")
    print("="*80)
    
    if monitor1.diagnostics['detection'] and monitor2.diagnostics['detection']:
        m1_rescue = sum(d['rescue_helped'] for d in monitor1.diagnostics['detection'])
        m2_rescue = sum(d['rescue_helped'] for d in monitor2.diagnostics['detection'])
        m1_count = len(monitor1.diagnostics['detection'])
        m2_count = len(monitor2.diagnostics['detection'])
        
        print(f"\nRescue pass effectiveness:")
        print(f"  Scenario 1 (alone): {m1_rescue}/{m1_count} measurements had rescue pass help")
        print(f"  Scenario 2 (together): {m2_rescue}/{m2_count} measurements had rescue pass help")
        
        if m1_count > 0 and m2_count > 0:
            m1_rate = m1_rescue / m1_count
            m2_rate = m2_rescue / m2_count
            diff = ((m2_rate - m1_rate) / max(m1_rate, 0.001)) * 100
            print(f"\n  Improvement when running together: {diff:+.1f}%")
