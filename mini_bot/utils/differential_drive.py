import numpy as np
import copy

def wrap_angle(angle):
    """ Wraps angle between -pi and pi 

    @type  angle: float or numpy array
    @param angle: angle in radinas

    @rtype:   float or numpy array
    @return:  angle in radinas between -Pi to Pi
    """
    if isinstance(angle, float) or isinstance(angle, int):
        return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )
    elif isinstance(angle, np.ndarray):
        return (angle + np.pi) % (2 * np.pi ) - np.pi 
    elif isinstance(angle, list):
        ret = []
        for i in angle:
            ret.append(wrap_angle(i))
        return ret
    else:
        raise NameError('wrap_angle')
    

class BaseRobot:
    X = 0
    Y = 1
    THETA = 2

    def __init__(self, x:float=0, y:float=0, theta:float=0, dt:float=0.2):
        self.set_pose(x, y, theta)
        self.v = 0
        self.w = 0
        self.dt = dt

    def set_pose(self, x:float=0, y:float=0, theta:float=0) -> None:
        self.pose = (x, y, wrap_angle(theta))
    
    def set_velocity(self, v:float=0, w:float=0) -> None:
        self.v = v
        self.w = w
    
    def get_pose(self):
        return copy.copy(self.pose)
    
    def get_position(self):
        return (self.pose[self.X], self.pose[self.Y])
    
    def get_theta(self):
        return self.pose[self.THETA]
    
    def get_velocity(self):
        return (self.v, self.w)
    
    def get_state(self):
        return self.pose + (self.v, self.w)
    
    def move(self, cmd_vel:tuple):
        self.v, self.w = cmd_vel
        x, y, theta = self.pose
        x += self.v*np.cos(theta)*self.dt
        y += self.v*np.sin(theta)*self.dt
        theta = wrap_angle(theta + self.w*self.dt)
        self.set_pose(x, y, theta) 
    
    def __str__(self):
        return "[({}, {}), {}]".format(self.x, self.y, self.theta)
        

class DifferentialWheel(BaseRobot):
    def __init__(self, x:float=0, y:float=0, theta:float=0, dt:float=0.2, length=0.287, radius=0.033, max_v=0.6, max_w=2.0):
        super().__init__(x, y, theta, dt)
        self.l = length  
        self.radius = radius
        self.max_v = max_v
        self.max_w = max_w

    def get_velocities(self, wl, wr): # rad/s
        # Wheels rad/s to m/s
        vl = (wl * (self.radius)) 
        vr = (wr * (self.radius))

        # Wheel velocity to robot linear and angular velocity 
        v = (vl + vr)/2
        w = (vl - vr)/(self.l)
        return v, w

    def get_inv_velocity(self, cmd_vel:tuple):
        # Robot linear and angular velocity to wheel velocity
        v, w = cmd_vel
        vl = v + (w * self.l)/2
        vr = v - (w * self.l)/2

        # m/s to rad/s
        wl = vl / self.radius
        wr = vr / self.radius
        return wl, wr
    
    def move(self, cmd_vel:tuple) -> None:
        assert len(cmd_vel) == 2, "Command velocity must be a tuple with two elements v and w."
        v, w = cmd_vel
        v = np.clip(v, -self.max_v, self.max_v)
        w = np.clip(w, -self.max_w, self.max_w)
        super().move((v, w))
    
    def move_wheels(self, wl:float, wr:float) -> None: # wheel_left and wheel_right in rad/s
        # Get robot speed from individual wheels speed 
        v, w = self.get_velocities(wl, wr)

        # Integrate robot position and orientation
        self.move((v, w))
