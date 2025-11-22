#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from radar_msgs.msg import RadarScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
from scipy.optimize import linear_sum_assignment

class RadarEKFTrackerNode(Node):
    def __init__(self):
        super().__init__('radar_ekf_tracker_node')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            RadarScan,
            'radar/scan',
            self.listener_callback,
            qos_profile
        )
        
        # FIXED: Changed to MarkerArray for proper visualization
        self.publisher = self.create_publisher(MarkerArray, 'radar/tracks', 10)
        
        # FIXED: Track states with unique IDs
        self.track_states = {}  # {track_id: {'state': np.array, 'cov': np.array, 'last_update': time}}
        self.next_track_id = 1
        self.max_track_age = 2.0  # Remove tracks not seen for 2 seconds
        
        self.get_logger().info('EKF Tracker started, listening on /radar/scan')
        
    def listener_callback(self, msg):
        dt = 0.05  # Time step (20Hz)
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Extract measurements
        measurements = []
        for detection in msg.returns:
            # Convert polar to Cartesian
            x = detection.range * math.cos(detection.azimuth)
            y = detection.range * math.sin(detection.azimuth)
            vx = detection.doppler_velocity * math.cos(detection.azimuth)
            vy = detection.doppler_velocity * math.sin(detection.azimuth)
            measurements.append(np.array([x, y, vx, vy]))
        
        if not measurements:
            self.publish_tracks(current_time)
            return
        
        # Predict step for all existing tracks
        for track_id in list(self.track_states.keys()):
            self.predict_track(track_id, dt)
        
        # FIXED: Data association using Hungarian algorithm
        if self.track_states and measurements:
            cost_matrix = self.compute_cost_matrix(measurements)
            track_ids = list(self.track_states.keys())
            
            if len(track_ids) > 0 and len(measurements) > 0:
                row_ind, col_ind = linear_sum_assignment(cost_matrix)
                
                matched_tracks = set()
                matched_measurements = set()
                
                # Update matched tracks
                for track_idx, meas_idx in zip(row_ind, col_ind):
                    if cost_matrix[track_idx, meas_idx] < 10.0:  # Gating threshold
                        track_id = track_ids[track_idx]
                        self.update_track(track_id, measurements[meas_idx], current_time)
                        matched_tracks.add(track_id)
                        matched_measurements.add(meas_idx)
                
                # Create new tracks for unmatched measurements
                for meas_idx, measurement in enumerate(measurements):
                    if meas_idx not in matched_measurements:
                        self.create_new_track(measurement, current_time)
        else:
            # No existing tracks, create new ones
            for measurement in measurements:
                self.create_new_track(measurement, current_time)
        
        # Remove old tracks
        self.remove_old_tracks(current_time)
        
        # Publish tracks
        self.publish_tracks(current_time)
    
    def predict_track(self, track_id, dt):
        """Predict track state using constant velocity model"""
        track = self.track_states[track_id]
        state = track['state']
        P = track['cov']
        
        # State transition matrix (constant velocity)
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Process noise
        q = 0.5  # Process noise parameter
        Q = q * np.array([
            [dt**4/4, 0, dt**3/2, 0],
            [0, dt**4/4, 0, dt**3/2],
            [dt**3/2, 0, dt**2, 0],
            [0, dt**3/2, 0, dt**2]
        ])
        
        # Predict
        track['state'] = F @ state
        track['cov'] = F @ P @ F.T + Q
    
    def update_track(self, track_id, measurement, current_time):
        """Update track with measurement (Kalman update)"""
        track = self.track_states[track_id]
        state = track['state']
        P = track['cov']
        
        # Measurement matrix (observe position and velocity)
        H = np.eye(4)
        
        # Measurement noise
        R = np.diag([0.5, 0.5, 0.3, 0.3])  # Position and velocity noise
        
        # Innovation
        y = measurement - H @ state
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        
        # Update
        track['state'] = state + K @ y
        track['cov'] = (np.eye(4) - K @ H) @ P
        track['last_update'] = current_time
    
    def create_new_track(self, measurement, current_time):
        """Create new track from measurement"""
        track_id = self.next_track_id
        self.next_track_id += 1
        
        # Initialize state
        self.track_states[track_id] = {
            'state': measurement.copy(),
            'cov': np.diag([1.0, 1.0, 1.0, 1.0]),
            'last_update': current_time
        }
        self.get_logger().info(f'Created new track {track_id}')
    
    def compute_cost_matrix(self, measurements):
        """Compute cost matrix for data association"""
        track_ids = list(self.track_states.keys())
        cost_matrix = np.zeros((len(track_ids), len(measurements)))
        
        for i, track_id in enumerate(track_ids):
            state = self.track_states[track_id]['state']
            for j, measurement in enumerate(measurements):
                # Euclidean distance
                cost_matrix[i, j] = np.linalg.norm(state[:2] - measurement[:2])
        
        return cost_matrix
    
    def remove_old_tracks(self, current_time):
        """Remove tracks that haven't been updated recently"""
        to_remove = []
        for track_id, track in self.track_states.items():
            if current_time - track['last_update'] > self.max_track_age:
                to_remove.append(track_id)
        
        for track_id in to_remove:
            del self.track_states[track_id]
            self.get_logger().info(f'Removed old track {track_id}')
    
    def publish_tracks(self, current_time):
        """Publish tracked objects as MarkerArray"""
        marker_array = MarkerArray()
        
        for track_id, track in self.track_states.items():
            state = track['state']
            
            # Position marker
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "base_link"
            marker.ns = "radar_tracks"
            marker.id = track_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(state[0])
            marker.pose.position.y = float(state[1])
            marker.pose.position.z = 0.5
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.lifetime.sec = 1
            marker_array.markers.append(marker)
            
            # Velocity arrow
            arrow = Marker()
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.header.frame_id = "base_link"
            arrow.ns = "radar_velocities"
            arrow.id = track_id + 10000
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = []
            
            # Start point
            start = Marker().pose.position
            start.x = float(state[0])
            start.y = float(state[1])
            start.z = 0.5
            arrow.points.append(start)
            
            # End point (velocity vector)
            end = Marker().pose.position
            end.x = float(state[0] + state[2])
            end.y = float(state[1] + state[3])
            end.z = 0.5
            arrow.points.append(end)
            
            arrow.scale.x = 0.1
            arrow.scale.y = 0.2
            arrow.scale.z = 0.0
            arrow.color.a = 1.0
            arrow.color.r = 0.0
            arrow.color.g = 1.0
            arrow.color.b = 0.0
            arrow.lifetime.sec = 1
            marker_array.markers.append(arrow)
            
            # Text label
            text = Marker()
            text.header.stamp = self.get_clock().now().to_msg()
            text.header.frame_id = "base_link"
            text.ns = "radar_labels"
            text.id = track_id + 20000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(state[0])
            text.pose.position.y = float(state[1])
            text.pose.position.z = 1.0
            text.scale.z = 0.3
            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.text = f"ID:{track_id}\nV:{np.linalg.norm(state[2:4]):.1f}m/s"
            text.lifetime.sec = 1
            marker_array.markers.append(text)
        
        self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RadarEKFTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

