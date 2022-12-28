import rospy
import time

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import numpy as np
import math

class robot:
    __base_ang_spd = 0.3
    __base_lin_spd = 0.15
    __tgt_pt = np.zeros(2)
    __lid_sense = 10
    __rad = 0.0
    __alpha = 0.0
    __center = np.zeros(2)
    __tgt_speed = np.zeros(2)
    __rays = []
    __obses = [[]]
    __new_data = False

    def __init__(self, rad):
        self.rad = rad

    def detect_obses(self):
        _obses = [[]]
        for r in range(len(self.__rays)):
            if self.__rays[r].length < self.__lid_sense:
                _obs = []
                _obs.append(self.__rays[r])
                last_ray_id = r
                #print("rays len =" + str(len(self.__rays)))
                for i in range(r+1, len(self.__rays)):
                    if(i >= len(self.__rays)):
                        break
                    #print("i =" + str(i))
                    vect = np.asarray(self.__rays[i].end_pt - _obs[-1].end_pt)
                    #print("vect = ", str(vect))
                    if(math.sqrt(math.pow(vect[0], 2) +  math.pow(vect[1], 2))) < 2.5 * self.__rad:
                        _obs.append(self.__rays[i])
                        last_ray_id = i
                r = last_ray_id
                _obses.append(_obs)
        self.__obses = _obses

    def calc_tgt_speed(self):
        _speed = np.zeros(2)
        vect = np.asarray(self.__center - self.__tgt_pt)
        if(math.sqrt(math.pow(vect[0], 2) +  math.pow(vect[1], 2))) < 0.25:
            self.__tgt_pt = self.__center
            self.__tgt_speed = np.zeros(2)
            return
        else:
            min_dist_obs = 10000
            min_ang_obs = 0.0
            esc_angle = 0
            for obstacle in self.__obses:
                for ray in obstacle:
                    #print("ray len= " + str(ray.length) + " ray ang =" + str(ray.alpha))
                    if ray.length < min_dist_obs:
                        min_dist_obs = ray.length
                        min_ang_obs = ray.alpha
            print("minDistObs = " + str(min_dist_obs))
            print("minAngObs = " + str(min_ang_obs))
            esc_coef = math.exp(-(min_dist_obs - 0.5)/10)
            if min_ang_obs>0:
                esc_angle = min_ang_obs - math.pi/2
            elif min_ang_obs < 0:
                esc_angle = min_ang_obs + math.pi/2

            tgt_angle = self.get_min_angle() * (1-esc_coef) + esc_angle * esc_coef
            print("tgt ang= " + str(tgt_angle) + " esc ang = " + str(esc_angle) + " min ang = " + str(self.get_min_angle()))
            print("robot pose = " + str(self.__center))

            if tgt_angle > math.pi / 80:
                _speed[1] = self.__base_ang_spd
            elif tgt_angle < -math.pi / 80:
                _speed[1] = -self.__base_ang_spd
            else:
                _speed[1] = 0  

            if abs(tgt_angle) < math.pi/8:
                _speed[0] = self.__base_lin_spd
            else:
                _speed[0] = 0
            
            print(_speed)
            self.__tgt_speed = np.asarray(_speed)

    def get_min_angle(self):
        output = math.atan2(self.__tgt_pt[1] - self.__center[1], self.__tgt_pt[0] - self.__center[0]) - self.__alpha
        if output > math.pi:
            output -= 2*math.pi
        if output < -math.pi:
            output += 2*math.pi
        return output

    @property
    def center(self):
        return self.__center
    
    
    def set_center(self, x, y):
        #print("robot odom: x=" + str(x) + " y=" +str(y))
        self.__center = np.asarray([x, y])

    @property
    def rays(self):
        return self.__rays
    
    def set_rays(self, ranges, start_ang, ang_inc):
        self.__rays = []
        #print("rays len: " + str(len(ranges)))
        self.__new_data = True
        for i in range(len(ranges)):
            _ray = ray(ranges[i], start_ang + ang_inc*i)
            self.__rays.append(_ray)
        
        #print("len __rays=" + str(len(ranges)))
        #print("start ang = " + str(start_ang))
        #print("inc angle = " + str(ang_inc))

    @property
    def alpha(self):
        return self.__alpha
    
    @alpha.setter
    def alpha(self, alpha):
        if alpha >= math.pi * 2:
            self.__alpha = alpha - math.pi * 2
        elif alpha <= math.pi * 2:
            self.__alpha = alpha + math.pi * 2
        else:
            self.__alpha = alpha

    @property
    def tgt_point(self):
        return self.__tgt_pt
    
    
    def set_tgt_point(self, x, y):
        self.__tgt_pt = np.asarray([x, y])

    def getSpeed(self):
        lin_speed = self.__tgt_speed[0]
        ang_speed = self.__tgt_speed[1]
        return lin_speed, ang_speed

    def is_robot_arrive(self):
        if self.__tgt_pt == self.__center:
            return True
        else:
            return False

    def handler(self):
        if self.__new_data:
            self.__new_data = False
            self.detect_obses()
            self.calc_tgt_speed()

class ray:
    __alpha = 0.0
    __length = 0.0
    __maxLength = 10000
    __start_pt = np.zeros(2)

    def __init__(self, length, alpha):
        self.alpha= alpha
        self.length = length


    def rotation_m(self, alpha):
        return np.asarray(([math.cos(alpha), -math.sin(alpha)],
                           [math.sin(alpha), math.cos(alpha)]))

    @property
    def alpha(self):
        return self.__alpha
    
    @alpha.setter
    def alpha(self, alpha):
        if alpha >= math.pi * 2:
            self.__alpha = alpha - math.pi * 2
        elif alpha <= -math.pi * 2:
            self.__alpha = alpha + math.pi * 2
        else:
            self.__alpha = alpha

    @property
    def length(self):
        return self.__length

    @length.setter
    def length(self, length):
        if abs(length) <= self.__maxLength:
            self.__length = abs(length)
        else:
            self.__length = self.__maxLength

    @property
    def end_pt(self):
        return (self.__start_pt + (self.rotation_m(self.__alpha)*np.asarray([1, 0]))*self.__length)[:,0]


