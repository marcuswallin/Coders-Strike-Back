import sys
import math
import numpy as np


def sigmoid_fcn(val, k1, shift):
    angle_abs = abs(val)
    f = 1-  1 / (1 + math.exp(-k1*(angle_abs - shift)))
    return f

def normalize(vec):
    return vec / np.linalg.norm(vec)

def get_radial_velocity_ratio(radial_velocity, velocity):
    if velocity == 0:
        return 1
    ratio = radial_velocity / velocity
    if ratio < 0:
        ratio = 0
    elif ratio > 1:
        ratio = 1
    return ratio

def get_distance(vec1, vec2):
    d = vec1 - vec2
    return np.linalg.norm(d) 

def find_best_opponent(opp1, opp2):
    if opp1.get_checkp_count() > opp2.get_checkp_count():
        return opp1
    elif opp2.get_checkp_count() > opp1.get_checkp_count():
        return opp2
    
    if opp1.distance_to_checkp() < opp2.distance_to_checkp():
        return opp1
    else: 
        return opp2
    
class Pod:
    
    def __init__(self, checkpoints):
        '''input_list is the list of information integers given about each pod.'''
        self._pos = None
        self._v = None 
        self._angle = None
        self._checkp_nr = None
        self._boost_used = False
        
        self._checkpoints = checkpoints
        self._old_checkp_nr = 1
        self._lap_count = 0


    def angle_to_point(self, point):
        angle_rad = self._angle / 360 * 2 * np.pi
        dir = normalize(np.array([np.cos(angle_rad), np.sin(angle_rad)]))
        checkp_vector = normalize(point - self._pos)
        angle = np.arccos(np.dot(dir, checkp_vector))
        
        if np.cross(checkp_vector, dir) < 0:
            angle = -angle
        return angle/(2*np.pi)*360

    def next_checkp_coord(self ):
        return self._checkpoints[self._checkp_nr]
    
    def next_next_checkp(self):
        return self._checkpoints[(self._checkp_nr+1)%checkp_count]

    def predict_next_pos(self):
        return self._pos + self._v * 5
    
    def distance_to_checkp(self):
        return get_distance(self._pos, self.next_checkp_coord())

    def get_checkp_count(self):
        nr_in_lap = (self._checkp_nr-1)
        if self._checkp_nr == 0 and self._old_checkp_nr != 1:
            nr_in_lap = checkp_count -1
        return self._lap_count * checkp_count + nr_in_lap
        
    def update_pod(self, input_list):
        self._pos = np.array([input_list[0], input_list[1]])
        self._v = np.array([input_list[2], input_list[3]]) 
        self._angle = input_list[4]
        self._checkp_nr = input_list[5]

        if self._checkp_nr != self._old_checkp_nr:
            self._old_checkp_nr = self._checkp_nr
            #starting a new lap if next checkpoint is 1
            if self._checkp_nr == 1:
                self._lap_count += 1

    #returns whehter or not the robot can glide to the next pos.
    def simulate_glide_checkp(self):
        simulated_position = self._pos
        i = 1
        next_checkp = self.next_checkp_coord()
        while i < 6 and get_distance(simulated_position, next_checkp) > 600:
            simulated_position = simulated_position + self._v * 0.85
            i += 1

        return i < 6

    def get_aim_point(self, point): 
        direction = point - self._pos
        old_pos = self._pos - self._v

        old_dist = get_distance(old_pos, point)
        radial_velocity = old_dist - self.distance_to_checkp()

        radial_ratio = get_radial_velocity_ratio(float(radial_velocity), float(np.linalg.norm(self._v)))
        aim_dist = 800*(1-radial_ratio)
        print(radial_ratio, file=sys.stderr)
        normalized_dir = normalize(direction) 

        v = np.array([0,0])
        if self.angle_to_point(self.next_checkp_coord()) < 0:
            v = np.array([-normalized_dir[1], normalized_dir[0]])
        else:
            v = np.array([normalized_dir[1], - normalized_dir[0]])

        return point + v*aim_dist

    #CHANGE WHEN AND HOW BOOST IS USED
    def use_boost(self):
        use_boost = False
        angle_abs = abs(self.angle_to_point(self.next_checkp_coord()))
      #  print(str(angle_abs) + " " + str(np.linalg.norm(self._v)) + " " + str(self.distance_to_checkp()), file=sys.stderr)
        if not self._boost_used and np.linalg.norm(self._v) < 250 \
            and self.distance_to_checkp() > 5000 and angle_abs / 180 < 0.05:
            self._boost_used = True
            use_boost = True
        return use_boost

    def use_shield(self, opponent1, opponent2):
        opp1_next_pos = opponent1._pos + 1.8*opponent1._v*0.85
        opp2_next_pos = opponent2._pos + 1.8*opponent2._v*0.85
        me_next_pos = self._pos + 1.8*self._v*0.85
        use_shield = False
        print(get_distance(me_next_pos, opp1_next_pos), file=sys.stderr)
        if  get_distance(me_next_pos, opp1_next_pos) < 800 or \
            get_distance(me_next_pos, opp2_next_pos) < 800:
            use_shield = True
        return use_shield

#------------------------------------------------------------------------------
##INITIALISATION --------------------------------------------------------------
nr_laps = int(input())
checkp_count = int(input())
checkpoints = []

for i in range(checkp_count):
    checkp_coord = [int(i) for i in input().split()]
    checkpoints.append(np.array([checkp_coord[0], checkp_coord[1]]))
    print(checkpoints, file=sys.stderr)

#old_checkp_nr = [1, 1, 1, 1]

me1 = Pod(checkpoints)
me2 = Pod(checkpoints)
    
opponent1 = Pod(checkpoints)
opponent2 = Pod(checkpoints)
pods = [me1, me2, opponent1, opponent2]

#------------------------------------------------------------------------------
#LOOP--------------------------------------------------------------------------
#------------------------------------------------------------------------------
while True:

    #update pod information----------------------------------------------------
    for i, pod in enumerate(pods):
        pod.update_pod([int(i) for i in input().split()])
    
    checkp_coord1 = me1.next_checkp_coord()
    checkp_coord2 = me2.next_checkp_coord()
    
    
    can_glide = me1.simulate_glide_checkp()
     
    #find enemy position
    best_opp = find_best_opponent(opponent1, opponent2)
    best_opp_pos = best_opp.predict_next_pos()
    
    aim = me1.get_aim_point(me1.next_checkp_coord())
    if can_glide:
        aim = me1.next_next_checkp()

    thrust1 = int(math.ceil(sigmoid_fcn(me1.angle_to_point(aim), 0.1, 70) * 100.0))
    thrust2 = int(math.ceil(sigmoid_fcn(me2.angle_to_point(best_opp_pos), 0.1, 70) * 100.0))

    if me1.use_boost():
        thrust1 = "BOOST"
    
    if me1.use_shield(opponent1, opponent2):
        thrust1 = "SHIELD"
    if me2.use_shield(opponent1, opponent2):
        thrust2 = "SHIELD"

    print(str(int(aim[0])) + " " + str(int(aim[1])) + " " +str(thrust1))
    print(str(best_opp_pos[0]) + " " + str(best_opp_pos[1]) + " " +str(thrust2))
