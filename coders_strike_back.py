import sys
import math
import numpy as np


def sigmoid_fcn(val, k1, shift):
    angle_abs = abs(val)
    f = 1-  1 / (1 + math.exp(-k1*(angle_abs - shift)))
    return f
 

def get_radial_velocity_ratio(radial_velocity, velocity):
    ratio = radial_velocity / velocity
    if ratio < 0:
        ratio = 0
    elif ratio > 1:
        ratio = 1
    return ratio


def get_aim_point(velocity, pos, radial_velocity, checkpoint, angle_to_cp):
    
    direction = checkpoint - pos
    radial_ratio = get_radial_velocity_ratio(radial_velocity, velocity)
    aim_dist = 700*(1-radial_ratio)
    normalized_dir = direction / np.linalg.norm(direction) 
    print(normalized_dir, file=sys.stderr)
    print(radial_velocity / velocity, file=sys.stderr)
    v = np.array([0,0])
    if angle_to_cp > 0:
        v = np.array([-normalized_dir[1], normalized_dir[0]])
    else:
        v = np.array([normalized_dir[1], - normalized_dir[0]])
    print(v*aim_dist, file=sys.stderr)
    print(checkpoint + v*aim_dist, file=sys.stderr)
    
    return checkpoint + v*aim_dist
    
#NOT YET IMPLEMENTED SHIELD
 
thrust = 0
boost_used = False
count = 0
direction = np.array([0,0])

old_checkpoint_dist = 0
old_dist = 0

old_pos = np.array([0,0])

while True:

    x, y, next_checkpoint_x, next_checkpoint_y, next_checkpoint_dist, next_checkpoint_angle = [int(i) for i in input().split()]
    opponent_x, opponent_y = [int(i) for i in input().split()]
    
    checkpoint = np.array([next_checkpoint_x, next_checkpoint_y])
    direction = np.array([x - old_pos[0], y - old_pos[1]])
    v_radial = old_checkpoint_dist - next_checkpoint_dist
    velocity = np.linalg.norm(direction)
    pos = np.array([x,y])

    aim = get_aim_point(velocity, pos ,v_radial, checkpoint, next_checkpoint_angle)
    
    
    thrust = int(math.ceil(sigmoid_fcn(next_checkpoint_angle, 0.1, 70) * 100.0))
    angle_abs = abs(next_checkpoint_angle)
    if not boost_used  and velocity < 250 and next_checkpoint_dist > 5000 and angle_abs / 180 < 0.05:
        thrust = 'BOOST'
        boost_used = True
    
    old_pos = np.array([x, y])
    old_checkpoint_dist = next_checkpoint_dist

    print(next_checkpoint_angle, file=sys.stderr)
  #  print(v_radial, file=sys.stderr)

    count += 1
  
    print(str(int(round(aim[0]))) + " " + str(int(round(aim[1]))) + " " +str(thrust))
    
