import math
import numpy as np
        

def reward_function(params):
    is_distance_unpardonable, distance_component = calculate_distance_component(params)
    is_heading_unpardonable, heading_component = calculate_heading_component(params)
    is_steering_unpardonable, steering_component = calculate_steering_component(params)

    is_unpardonable = is_action_unpardonable(params)
    if is_unpardonable \
        or is_distance_unpardonable \
        or is_heading_unpardonable \
        or is_steering_unpardonable :
        reward = 1e-3
    else:
        reward = (distance_component + heading_component + steering_component) **2
    return reward
    

STRAIGHT_WAYPOINTS = {

}

CURVE_WAYPOINTS = {
    
}   

################## HELPER FUNCTIONS ###################

def distance_to_line(pt0, pt1, pt2):
    x0, y0 = pt0
    x1, y1 = pt1
    x2, y2 = pt2
    numerator = abs( (x2-x1) * (y1-y0) - (x1-x0) * (y2-y1) )
    denominator = ( (x2-x1)**2 + (y2-y1)**2) ** 0.5
    if denominator == 0:
        return ( (x1-x0)**2 + (y1-y0)**2 ) ** 0.5
    else:
        return numerator / denominator
    
    
#######################################################################
#                                                                     #
#                                                                     #
#                       Unpardonable Actions                          #
#                                                                     #
#                                                                     # 
#######################################################################
def is_action_unpardonable(params):
    is_offtrack = params['is_offtrack']
    is_reversed = params["is_reversed"]
    
    if is_offtrack:
        return True
        
    if is_reversed:
        return True
    
    return False
        

#######################################################################
#                                                                     #
#                                                                     #
#                        Speed Component                              #
#                                                                     #
#                                                                     # 
#######################################################################
def calculate_speed_progress_steps_component(params):
    all_wheels_on_track = params['all_wheels_on_track']
    x = params['x']
    y = params['y']
    heading = params['heading']
    progress = params['progress']
    steps = params['steps']
    speed = params['speed']
    steering_angle = params['steering_angle']
    track_width = params['track_width']
    closest_waypoints = params['closest_waypoints']
    waypoints = params['waypoints']

    # get indices of 2 closest points on the optimal racing line
    distance_array = ((RACING_TRACK[:,0]-x)**2 + (RACING_TRACK[:,1]-y)**2)**0.5 
    closest_index, second_closest_index = np.argsort(distance_array)[:2]

    is_speed_unpardonable = False
    SPEED_MULTIPLE = 15

    target_time = 8    # seconds
    target_steps = target_time * 12.5    # 12.5 steps/s
    speed_reward = SPEED_MULTIPLE * (progress/100 * target_steps/steps)**2    # normalized approximate speed

    # extra reward if fast enough in straigt line
    if (closest_index in STRAIGHT_WAYPOINTS) \
        and (second_closest_index in STRAIGHT_WAYPOINTS) \
        and (speed >= 3.4):
        speed_reward += 10
    
    # extra reward if finish lap
    if progress == 100:
        speed_reward += 10

    # unpardonable if too many steps
    if steps > 140:
        is_speed_unpardonable = True
        speed_reward = 1e-3
    
    return is_speed_unpardonable, speed_reward


#######################################################################
#                                                                     #
#                                                                     #
#                        Distance Component                           #
#                                                                     #
#                                                                     # 
#######################################################################
def calculate_distance_component(params):
    all_wheels_on_track = params['all_wheels_on_track']
    x = params['x']
    y = params['y']
    heading = params['heading']
    progress = params['progress']
    steps = params['steps']
    speed = params['speed']
    steering_angle = params['steering_angle']
    track_width = params['track_width']

    # get indices of 2 closest points on the optimal racing line
    distance_array = ((RACING_TRACK[:,0]-x)**2 + (RACING_TRACK[:,1]-y)**2)**0.5 
    closest_index, second_closest_index = np.argsort(distance_array)[:2]

    # get optimal [x, y, speed, time] for closest and second closest points
    optimals = RACING_TRACK[closest_index]
    optimals_second = RACING_TRACK[second_closest_index]

    # reward if car goes close to optimal racing line
    DISTANCE_MULTIPLE = 10

    dist = distance_to_line([x, y], optimals[0:2], optimals_second[0:2], )
    
    is_distance_unpardonable = False
    if dist > track_width * 0.15:
        is_distance_unpardonable = True
        distance_reward = 1e-3
    else:
        distance_reward = 1 - dist/(track_width*0.15)
        distance_reward = distance_reward * DISTANCE_MULTIPLE
    return is_distance_unpardonable, distance_reward


#######################################################################
#                                                                     #
#                                                                     #
#                        Heading Component                            #
#                                                                     #
#                                                                     # 
#######################################################################
def calculate_heading_component(params):
    all_wheels_on_track = params['all_wheels_on_track']
    x = params['x']
    y = params['y']
    heading = params['heading']
    progress = params['progress']
    steps = params['steps']
    speed = params['speed']
    steering_angle = params['steering_angle']
    track_width = params['track_width']

    # get indices of 2 closest points on the optimal racing line
    distance_array = ((RACING_TRACK[:,0]-x)**2 + (RACING_TRACK[:,1]-y)**2)**0.5 
    closest_index, second_closest_index = np.argsort(distance_array)[:2]

    # get optimal [x, y, speed, time] for closest and second closest points
    optimals = RACING_TRACK[closest_index]
    optimals_second = RACING_TRACK[second_closest_index]

    # target next point on racing line
    n_pts_away_idx = (closest_index + 2) % len(RACING_TRACK)
    n_pts_away = RACING_TRACK[n_pts_away_idx]

    # calculate target heading direction
    target_heading_radians = math.atan2(n_pts_away[1]-y, n_pts_away[0]-x)
    target_heading_degree = math.degrees(target_heading_radians)

    # calculate heading direction difference
    heading_direction_diff = abs(target_heading_degree - heading)
    if heading_direction_diff > 180:
        heading_direction_diff = 360 - heading_direction_diff

    HEADING_MULTIPLIER = 10
    
    is_heading_unpardonable = False
    # calculate heading reward based on heading direction difference
    if abs(heading_direction_diff) <= 20:
        heading_reward = math.cos( abs(heading_direction_diff ) * ( math.pi / 180 ) ) ** 4
        heading_reward = HEADING_MULTIPLIER * heading_reward
    else:
        is_heading_unpardonable = True
        heading_reward = 1e-3
    return is_heading_unpardonable, heading_reward


#######################################################################
#                                                                     #
#                                                                     #
#                        Steering Component                           #
#                                                                     #
#                                                                     # 
#######################################################################
def calculate_steering_component(params):
    all_wheels_on_track = params['all_wheels_on_track']
    x = params['x']
    y = params['y']
    heading = params['heading']
    progress = params['progress']
    steps = params['steps']
    speed = params['speed']
    steering_angle = params['steering_angle']
    track_width = params['track_width']

    # get indices of 2 closest points on the optimal racing line
    distance_array = ((RACING_TRACK[:,0]-x)**2 + (RACING_TRACK[:,1]-y)**2)**0.5 
    closest_index, second_closest_index = np.argsort(distance_array)[:2]

    # calculate target steering angle
    n_pts_away_idx = (closest_index + 5) % len(RACING_TRACK)
    n_pts_away = RACING_TRACK[n_pts_away_idx]

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    target_steering_radians = math.atan2(n_pts_away[1] - y, n_pts_away[0] - x)

    # Convert to degree
    target_steering_degree = math.degrees(target_steering_radians)
    steering_direction = heading + steering_angle

    # Calculate the difference between the track direction and the heading direction of the car
    steering_direction_diff = abs(target_steering_degree - steering_direction)
    if steering_direction_diff > 180:
        steering_direction_diff = 360 - steering_direction_diff

    STEERING_MULTIPLIER = 10

    is_steering_unpardonable = False
    if abs(steering_direction_diff) <= 20:
        steering_reward = math.cos( abs(steering_direction_diff ) * ( math.pi / 180 ) ) ** 4
        steering_reward = STEERING_MULTIPLIER * steering_reward
    else:
        is_steering_unpardonable = True
        steering_reward = 1e-3
    return is_steering_unpardonable, steering_reward


#######################################################################
#                                                                     #
#                                                                     #
#                        Optimal Track                                #
#                                                                     #
#                                                                     # 
#######################################################################
# Optimal racing line for the 2022 AWS re:invent champion track
# Each row: [x,y]
RACING_TRACK = np.array([
    [ 0.01111836,  1.13948501],
    [ 0.13520309,  1.00073701],
    [ 0.26845672,  0.85147937],
    [ 0.40978946,  0.69296496],
    [ 0.56476936,  0.51880509],
    [ 0.7341353 ,  0.32811834],
    [ 0.91683666,  0.12203327],
    [ 1.11062966, -0.09700346],
    [ 1.31099251, -0.32442756],
    [ 1.51342951, -0.54952628],
    [ 1.72028326, -0.76965278],
    [ 1.93371608, -0.98199594],
    [ 2.15571513, -1.18368112],
    [ 2.38809342, -1.3717577 ],
    [ 2.63253204, -1.54309453],
    [ 2.88519165, -1.69130414],
    [ 3.11849603, -1.85972246],
    [ 3.32521964, -2.05500975],
    [ 3.49340164, -2.27824387],
    [ 3.60829275, -2.51890629],
    [ 3.66247621, -2.75973265],
    [ 3.65820257, -2.98641469],
    [ 3.60157844, -3.18941893],
    [ 3.498367  , -3.36138839],
    [ 3.35307904, -3.4941456 ],
    [ 3.17634992, -3.58703481],
    [ 2.97480354, -3.63807831],
    [ 2.7540063 , -3.64428499],
    [ 2.52006614, -3.60237652],
    [ 2.28077485, -3.5101923 ],
    [ 2.03037103, -3.45894491],
    [ 1.7775013 , -3.44164559],
    [ 1.52343421, -3.45030199],
    [ 1.26882376, -3.47744367],
    [ 1.01401186, -3.51545155],
    [ 0.78785814, -3.55167615],
    [ 0.56933487, -3.56709682],
    [ 0.36465055, -3.54812988],
    [ 0.17980516, -3.48650887],
    [ 0.02325569, -3.37532501],
    [-0.08692003, -3.2051105 ],
    [-0.15238374, -2.99415509],
    [-0.1773475 , -2.75372026],
    [-0.17636884, -2.49656552],
    [-0.18030125, -2.22040869],
    [-0.20300324, -1.95356564],
    [-0.25449661, -1.70356042],
    [-0.34016339, -1.47827895],
    [-0.46102257, -1.28570048],
    [-0.61456247, -1.13380253],
    [-0.79545263, -1.0308591 ],
    [-0.99548233, -0.98649016],
    [-1.20086749, -1.0119989 ],
    [-1.39870808, -1.09045672],
    [-1.58090221, -1.21906679],
    [-1.73942716, -1.39529142],
    [-1.86602187, -1.6155834 ],
    [-1.95300875, -1.87388833],
    [-1.99507654, -2.16083002],
    [-1.99163144, -2.45910585],
    [-2.04165517, -2.73745693],
    [-2.14117152, -2.98065598],
    [-2.28125653, -3.1793216 ],
    [-2.45204053, -3.32854275],
    [-2.6442585 , -3.42535311],
    [-2.84910047, -3.46628154],
    [-3.05627895, -3.4461862 ],
    [-3.25542631, -3.37156365],
    [-3.43810612, -3.24599374],
    [-3.59614811, -3.07281791],
    [-3.72149577, -2.85656557],
    [-3.80702519, -2.60410455],
    [-3.84829862, -2.32515614],
    [-3.84559056, -2.03130234],
    [-3.80443857, -1.73337512],
    [-3.7340432 , -1.43873179],
    [-3.64474171, -1.14936138],
    [-3.54405515, -0.86401521],
    [-3.43753179, -0.58087689],
    [-3.33809257, -0.30945932],
    [-3.25217898, -0.04458027],
    [-3.1865999 ,  0.21263419],
    [-3.14592177,  0.46235597],
    [-3.13356039,  0.70462767],
    [-3.15260271,  0.93906706],
    [-3.20723573,  1.16472223],
    [-3.28827962,  1.38377482],
    [-3.38766673,  1.59817679],
    [-3.49697531,  1.81004347],
    [-3.60078718,  2.0498248 ],
    [-3.67666608,  2.29188816],
    [-3.71762335,  2.53253132],
    [-3.71896075,  2.76642753],
    [-3.67828948,  2.98714446],
    [-3.59495372,  3.18734585],
    [-3.46958966,  3.35851469],
    [-3.30417804,  3.49019778],
    [-3.10321277,  3.56834926],
    [-2.88211363,  3.59846005],
    [-2.64854903,  3.57998873],
    [-2.40954454,  3.51329851],
    [-2.17169557,  3.40031938],
    [-1.94065466,  3.24580786],
    [-1.71992663,  3.05768588],
    [-1.509843  ,  2.84603835],
    [-1.30724964,  2.62167251],
    [-1.10694718,  2.39539057],
    [-0.90912721,  2.17228422],
    [-0.71541405,  1.95420934],
    [-0.52631773,  1.74170819],
    [-0.34033597,  1.53294474],
    [-0.15917737,  1.32994478],
    [ 0.01111836,  1.13948501]
])
