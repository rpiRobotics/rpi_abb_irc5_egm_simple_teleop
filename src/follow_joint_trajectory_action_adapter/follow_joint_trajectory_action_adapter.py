import rospy
import numpy as np
from scipy.interpolate import PchipInterpolator
import threading

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from sensor_msgs.msg import JointState
from actionlib import action_server

class FollowJointTrajectoryActionAdapter(object):
    
    def __init__(self):        
        self._t = 0
        
        self._action = action_server.ActionServer("joint_trajectory_action", FollowJointTrajectoryAction, self.goal_cb, self.cancel_cb, auto_start=False)
        
        self._action.start()
        
        self._joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self._current_joint_angles=[0,0,0,0,0,0]
        
        self._lock = self._action.lock
        
        self._trajectory_valid = False
        self._trajectory_interp = None
        self._trajectory_t = 0
        self._trajectory_max_t = 0
        self._trajectory_gh = None
        
        
    def goal_cb(self, gh):
        with self._lock:
                        
            g = gh.get_goal()
            if g.trajectory.joint_names != ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']:
                gh.set_rejected(text="Invalid joint names")
                print "Invalid joint names"
                return
            
            start_joint_angles = np.array(g.trajectory.points[0].positions)
            if np.any(np.abs(start_joint_angles - self._current_joint_angles) > np.deg2rad(5)):
                gh.set_rejected()
                return
            else:
                gh.set_accepted()
            
            interps=[]
            t = np.array([p.time_from_start.to_sec() for p in g.trajectory.points])
            for j in xrange(6):
                x = np.array([p.positions[j] for p in g.trajectory.points])
                pchip = PchipInterpolator(t,x)
                interps.append(pchip) 
            
            self._abort_trajectory()
            
            self._trajectory_gh = gh
            self._trajectory_max_t = t[-1]
            self._trajectory_interp = interps
            self._trajectory_valid = True            
  
    def cancel_cb(self, gh):
        
        with self._lock:
            self._reset_trajectory()
            gh.set_canceled()
        
    def _publish_joint_state(self, joint_angles):
        js = JointState()        
        js.header.stamp = rospy.Time.now()
        js.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']      
        js.position = joint_angles  
        js.velocity = [0,0,0,0,0,0]
        js.effort = [0,0,0,0,0,0]
        self._joint_pub.publish(js)
        
    def _publish_current_joint_state(self):
        
        self._publish_joint_state(self._current_joint_angles)
    
    def _set_current_joint_angles(self, joint_angles):
        self._current_joint_angles = joint_angles
        self._publish_current_joint_state()
        
        if self._trajectory_valid:
            #If the specified trajectory is too far away from current joint angles, abort it
            trajectory_angles=self._get_trajectory_joint_angles(self._trajectory_t)
            #TODO: improve trajectory error tracking
            if (np.any(np.abs(trajectory_angles-joint_angles) > np.deg2rad(45))):
                print "Trajectory aborted due to tracking error: " + str(np.rad2deg(trajectory_angles-joint_angles)) 
                self._abort_trajectory()
            else:            
                fb = FollowJointTrajectoryFeedback()
                fb.header.stamp = rospy.Time.now()
                fb.desired.positions = self._get_trajectory_joint_angles(self._trajectory_max_t)
                fb.desired.time_from_start = rospy.Duration(secs=self._trajectory_t)
                fb.actual = fb.desired                
                gh=self._trajectory_gh
                gh.publish_feedback(fb)
    
    @property
    def current_joint_angles(self):
        with self._lock:
            return self._current_joint_angles
        
    @current_joint_angles.setter
    def current_joint_angles(self, joint_angles):
        with self._lock:
            self._set_current_joint_angles(joint_angles)
        
        
    @property
    def trajectory_valid(self):
        with self._lock:
            return self._trajectory_valid
    
    @property
    def trajectory_time(self):
        with self._lock:
            return self._trajectory_t
        
    @property
    def trajectory_max_time(self):
        with self._lock:
            return self._trajectory_max_t
        
    def _set_trajectory_time(self, t):
        if not self._trajectory_valid:
            return False, None
            
        if t > self._trajectory_max_t or t < 0.0:
            return False, None
        
        self._trajectory_t = t
        
        if t >= self._trajectory_max_t:
            trajectory_angles = self._get_trajectory_joint_angles(t)
            self._trajectory_complete()
            return True, trajectory_angles
        
        return True, self._get_trajectory_joint_angles(t)
        
    def _get_trajectory_joint_angles(self, t):        
        return np.array([interp(t) for interp in self._trajectory_interp])
        
    def set_trajectory_time(self,t):
        with self._lock:            
            return self._set_trajectory_time(self, t)
            
    def increment_trajectory_time(self, dt):
        with self._lock: 
            if not self._trajectory_valid:
                return False, None
            
            t2 = self._trajectory_t + dt
            if t2 < 0: t2 = 0
            if t2 > self._trajectory_max_t: t2 = self._trajectory_max_t
            
            return self._set_trajectory_time(t2)
    
    def _reset_trajectory(self):
        self._trajectory_valid=False
        self._trajectory_max_t=0
        self._trajectory_t=0
        self._trajectory_interp=None
        self._trajectory_gh=None
    
    def _trajectory_complete(self):
        
        gh = self._trajectory_gh
        res = gh.get_default_result()
        res.error_code = FollowJointTrajectoryResult.SUCCESSFUL        
        gh.set_succeeded(res)
        
        self._reset_trajectory()
        
    
    def _abort_trajectory(self):
        
        gh = self._trajectory_gh
        if gh is not None:
            gh.set_aborted()
        
        self._reset_trajectory()
        
    def abort_trajectory(self):
        with self._lock:
            if self._trajectory_valid:
                self._abort_trajectory()
