#!/usr/bin/env python3
"""Cone SLAM v6.0 — color-agnostic, robust data association, no ghost cones"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
import gtsam
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, Bool
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

CONFIRM_HITS     = 4
STABILITY_THRESH = 0.25
CANDIDATE_ALPHA  = 0.25
MAX_CAND_RANGE   = 15.0
MAX_CAND_AGE     = 30


class ConeSLAM(Node):
    def __init__(self):
        super().__init__('cone_slam')
        self.declare_parameter('association_threshold', 1.5)
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_state     = np.zeros(3)
        self.robot_history   = []
        self.last_stamp      = None
        self.total_distance  = 0.0
        self.last_tracked_pose = None
        self.loop_closed_sent  = False
        self.graph             = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        self.pose_id           = 0
        self.last_graph_odom   = None
        self.prior_noise  = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-6, 1e-6, 1e-6]))
        self.odom_noise   = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.10, 0.10, 0.05]))
        self.meas_noise   = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.08, 0.12]))
        self.map_to_odom_pose = gtsam.Pose2(0.0, 0.0, 0.0)
        x0key = gtsam.symbol('x', 0)
        self.graph.add(gtsam.PriorFactorPose2(x0key, gtsam.Pose2(0,0,0), self.prior_noise))
        self.initial_estimates.insert(x0key, gtsam.Pose2(0,0,0))
        self.candidates = {}
        self.confirmed  = []
        self.cand_id    = 0
        self.create_subscription(Odometry,  '/odometry/filtered', self.odom_callback,  10)
        self.create_subscription(PoseArray, '/cones/poses',        self.cones_callback, 10)
        self.pub_map  = self.create_publisher(MarkerArray, '/map/cones_markers',   10)
        self.pub_cones= self.create_publisher(PoseArray,  '/map/cones_confirmed', 10)
        self.pub_loop = self.create_publisher(Bool,       '/map/loop_closed',     10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.1, self._check_time)
        self.get_logger().info('Cone SLAM v6.0 started [color-agnostic GTSAM Graph-SLAM]')

    def _check_time(self):
        now = self.get_clock().now().nanoseconds
        if self.last_stamp is not None and (now - self.last_stamp)/1e9 < -0.5:
            self.get_logger().warn('Skok czasu — reset SLAM')
            self._reset_slam()
        self.last_stamp = now

    def _reset_slam(self):
        self.candidates = {}; self.confirmed = []; self.robot_history = []
        self.graph = gtsam.NonlinearFactorGraph(); self.initial_estimates = gtsam.Values()
        self.pose_id = 0
        x0key = gtsam.symbol('x', 0)
        self.graph.add(gtsam.PriorFactorPose2(x0key, gtsam.Pose2(0,0,0), self.prior_noise))
        self.initial_estimates.insert(x0key, gtsam.Pose2(0,0,0))
        self.last_graph_odom = None; self.map_to_odom_pose = gtsam.Pose2(0,0,0)
        self.total_distance = 0.0; self.last_tracked_pose = None; self.loop_closed_sent = False

    def odom_callback(self, msg: Odometry):
        self.robot_state[0] = msg.pose.pose.position.x
        self.robot_state[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_state[2] = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y**2+q.z**2))
        if self.last_tracked_pose is not None:
            self.total_distance += math.hypot(self.robot_state[0]-self.last_tracked_pose[0],
                                              self.robot_state[1]-self.last_tracked_pose[1])
        self.last_tracked_pose = (self.robot_state[0], self.robot_state[1])
        if self.total_distance > 18.0 and not self.loop_closed_sent:
            if math.hypot(self.robot_state[0], self.robot_state[1]) < 3.0:
                self.loop_closed_sent = True
                m = Bool(); m.data = True; self.pub_loop.publish(m)
                self.get_logger().info('!!! DETEKCJA ZAMKNIĘCIA PĘTLI !!!')
        self.robot_history.append((self.robot_state[0], self.robot_state[1]))
        if len(self.robot_history) > 50: self.robot_history.pop(0)
        dead = [k for k,c in self.candidates.items() if c['age'] > MAX_CAND_AGE]
        for k in dead: del self.candidates[k]

    def _global_xy(self, lx, ly, ref):
        rx,ry,ryaw = ref.x(), ref.y(), ref.theta()
        return rx+lx*math.cos(ryaw)-ly*math.sin(ryaw), ry+lx*math.sin(ryaw)+ly*math.cos(ryaw)

    def cones_callback(self, msg: PoseArray):
        try:
            self.tf_buffer.lookup_transform('odom','lidar_link',rclpy.time.Time(),
                                            timeout=rclpy.duration.Duration(seconds=0.0))
        except Exception: return
        if not msg.poses: return
        current_odom = gtsam.Pose2(self.robot_state[0], self.robot_state[1], self.robot_state[2])
        if self.last_graph_odom is None:
            self.last_graph_odom = current_odom
        else:
            self.pose_id += 1
            pk = gtsam.symbol('x', self.pose_id-1); ck = gtsam.symbol('x', self.pose_id)
            delta = self.last_graph_odom.between(current_odom)
            self.graph.add(gtsam.BetweenFactorPose2(pk, ck, delta, self.odom_noise))
            self.initial_estimates.insert(ck, self.initial_estimates.atPose2(pk).compose(delta))
            self.last_graph_odom = current_odom
        cpk = gtsam.symbol('x', self.pose_id)
        opt = self.initial_estimates.atPose2(cpk)
        thresh = self.get_parameter('association_threshold').value
        graph_updated = False
        for pose in msg.poses:
            lx,ly = pose.position.x, pose.position.y
            r = math.hypot(lx, ly)
            if r > MAX_CAND_RANGE or lx < -0.5: continue
            bearing = math.atan2(ly, lx)
            gx, gy  = self._global_xy(lx, ly, opt)
            # Asocjacja z potwierdzonymi
            best_d, best_i = thresh, -1
            for i, cone in enumerate(self.confirmed):
                lk = gtsam.symbol('l', cone['id'])
                if not self.initial_estimates.exists(lk): continue
                cp = self.initial_estimates.atPoint2(lk)
                d  = math.hypot(gx-cp[0], gy-cp[1])
                if d < best_d: best_d, best_i = d, i
            if best_i >= 0:
                self.graph.add(gtsam.BearingRangeFactor2D(cpk,
                    gtsam.symbol('l', self.confirmed[best_i]['id']),
                    gtsam.Rot2(bearing), r, self.meas_noise))
                graph_updated = True; continue
            # Asocjacja z kandydatami
            best_d, best_k = thresh, None
            for k, cand in self.candidates.items():
                d = math.hypot(gx-cand['x'], gy-cand['y'])
                if d < best_d: best_d, best_k = d, k
            if best_k is not None:
                cand = self.candidates[best_k]
                px, py = cand['x'], cand['y']
                nx = (1-CANDIDATE_ALPHA)*px + CANDIDATE_ALPHA*gx
                ny = (1-CANDIDATE_ALPHA)*py + CANDIDATE_ALPHA*gy
                cand['hits'] += 1; cand['age'] = 0
                if math.hypot(nx-px, ny-py) < STABILITY_THRESH: cand['stable_hits'] += 1
                else: cand['stable_hits'] = max(0, cand['stable_hits']-1)
                cand['x'], cand['y'] = nx, ny
                if cand['stable_hits'] >= CONFIRM_HITS:
                    cid = len(self.confirmed)
                    lk  = gtsam.symbol('l', cid)
                    self.initial_estimates.insert(lk, gtsam.Point2(nx, ny))
                    self.graph.add(gtsam.BearingRangeFactor2D(cpk, lk,
                        gtsam.Rot2(bearing), r, self.meas_noise))
                    self.confirmed.append({'id':cid, 'color':'unknown'})
                    graph_updated = True; del self.candidates[best_k]
                    self.get_logger().info(f'✓ GTSAM #{cid} ({nx:.2f}, {ny:.2f}) color=unknown')
            else:
                if r <= MAX_CAND_RANGE:
                    self.candidates[self.cand_id] = {'x':gx,'y':gy,'hits':1,'stable_hits':0,'age':0}
                    self.cand_id += 1
        for cand in self.candidates.values(): cand['age'] += 1
        if self.pose_id > 0 and graph_updated:
            try:
                opt2 = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimates).optimize()
                self.initial_estimates = opt2
                op = self.initial_estimates.atPose2(cpk)
                self.map_to_odom_pose = op.compose(current_odom.inverse())
            except Exception as e: self.get_logger().error(f'GTSAM: {e}')
        self.publish_map(msg.header.stamp)

    def publish_map(self, stamp):
        ma = MarkerArray(); pa = PoseArray()
        pa.header.stamp = stamp; pa.header.frame_id = 'map'
        clr = Marker(); clr.action = Marker.DELETEALL; ma.markers.append(clr)
        mid = 0
        for cone in self.confirmed:
            lk = gtsam.symbol('l', cone['id'])
            if not self.initial_estimates.exists(lk): continue
            cp = self.initial_estimates.atPoint2(lk); cx,cy = cp[0],cp[1]
            m = Marker(); m.header = Header(stamp=stamp, frame_id='map')
            m.ns='confirmed'; m.id=mid; mid+=1; m.type=Marker.CYLINDER; m.action=Marker.ADD
            m.pose.position.x=cx; m.pose.position.y=cy; m.pose.position.z=0.1625
            m.pose.orientation.w=1.0; m.scale.x=m.scale.y=0.305; m.scale.z=0.325
            m.color.r=0.7; m.color.g=0.7; m.color.b=0.7; m.color.a=1.0; m.lifetime.sec=0
            ma.markers.append(m)
            p=Pose(); p.position.x=cx; p.position.y=cy; p.orientation.w=1.0; pa.poses.append(p)
        for k,cand in self.candidates.items():
            m=Marker(); m.header=Header(stamp=stamp,frame_id='map')
            m.ns='candidates'; m.id=mid; mid+=1; m.type=Marker.CYLINDER; m.action=Marker.ADD
            m.pose.position.x=cand['x']; m.pose.position.y=cand['y']; m.pose.position.z=0.1
            m.pose.orientation.w=1.0; m.scale.x=m.scale.y=m.scale.z=0.18
            m.color.r=1.0; m.color.g=0.5; m.color.b=0.0; m.color.a=0.4; m.lifetime.sec=1
            ma.markers.append(m)
        self.pub_map.publish(ma); self.pub_cones.publish(pa)
        t=TransformStamped(); t.header.stamp=stamp; t.header.frame_id='map'
        t.child_frame_id='odom'
        t.transform.translation.x=self.map_to_odom_pose.x()
        t.transform.translation.y=self.map_to_odom_pose.y()
        th=self.map_to_odom_pose.theta()
        t.transform.rotation.z=math.sin(th/2); t.transform.rotation.w=math.cos(th/2)
        self.tf_broadcaster.sendTransform(t)
        if self.pose_id % 10 == 0:
            self.get_logger().info(
                f'total_dist={self.total_distance:.1f}m dist_to_start={math.hypot(self.robot_state[0],self.robot_state[1]):.1f}m')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ConeSLAM())
    rclpy.shutdown()

if __name__ == '__main__':
    main()