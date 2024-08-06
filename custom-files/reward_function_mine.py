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
RACING_TRACK = np.array([[ 0.01111836,  1.13948501],
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
       [ 3.1189107 , -1.85930103],
       [ 3.33161856, -2.04983862],
       [ 3.5158145 , -2.26902329],
       [ 3.65591177, -2.5175006 ],
       [ 3.73348984, -2.77806641],
       [ 3.74305423, -3.02771856],
       [ 3.69029572, -3.25188731],
       [ 3.58219964, -3.44054409],
       [ 3.42465524, -3.58315737],
       [ 3.23121267, -3.67852364],
       [ 3.01043826, -3.72344867],
       [ 2.77000775, -3.71291549],
       [ 2.51909879, -3.64129946],
       [ 2.28043707, -3.51185544],
       [ 2.0321491 , -3.43513598],
       [ 1.77901977, -3.40231913],
       [ 1.52382735, -3.40758564],
       [ 1.26839549, -3.44654808],
       [ 1.01401186, -3.51545155],
       [ 0.77052765, -3.60670641],
       [ 0.53361507, -3.6547963 ],
       [ 0.31039949, -3.64892499],
       [ 0.11026151, -3.58393659],
       [-0.05290591, -3.45595298],
       [-0.15229067, -3.25999498],
       [-0.2006704 , -3.02752395],
       [-0.20464482, -2.77018469],
       [-0.18115018, -2.49996352],
       [-0.16489118, -2.2117363 ],
       [-0.17202782, -1.93114174],
       [-0.21333136, -1.66507387],
       [-0.29504108, -1.42209203],
       [-0.41885165, -1.21211987],
       [-0.58199737, -1.04583737],
       [-0.77764507, -0.93444951],
       [-0.99454453, -0.8896192 ],
       [-1.21465547, -0.92356777],
       [-1.42324994, -1.01554611],
       [-1.6110983 , -1.1612801 ],
       [-1.76942067, -1.35625284],
       [-1.88994143, -1.59430307],
       [-1.96612134, -1.86645001],
       [-1.99507654, -2.16083002],
       [-1.97955365, -2.46299033],
       [-2.01717393, -2.75403385],
       [-2.10870762, -3.01621075],
       [-2.24745827, -3.23562155],
       [-2.42352608, -3.40360619],
       [-2.62649795, -3.51466828],
       [-2.84590155, -3.56380665],
       [-3.06938724, -3.54427483],
       [-3.28356448, -3.46171998],
       [-3.47728887, -3.32125303],
       [-3.6402027 , -3.12856371],
       [-3.76315252, -2.89168385],
       [-3.83989781, -2.62155253],
       [-3.86904497, -2.33090844],
       [-3.85489163, -2.03182594],
       [-3.80612755, -1.73315752],
       [-3.73345509, -1.43890045],
       [-3.64480651, -1.14933336],
       [-3.54432053, -0.86391809],
       [-3.43591555, -0.58152708],
       [-3.3233463 , -0.30078982],
       [-3.21690638, -0.03345756],
       [-3.12702857,  0.22809725],
       [-3.06441569,  0.48208046],
       [-3.03590866,  0.72816489],
       [-3.0466528 ,  0.96544913],
       [-3.10228116,  1.19181998],
       [-3.2039404 ,  1.40583587],
       [-3.3380069 ,  1.6110719 ],
       [-3.49697531,  1.81004347],
       [-3.63840697,  2.03707722],
       [-3.74470535,  2.28069124],
       [-3.80715142,  2.53563272],
       [-3.81880943,  2.79192526],
       [-3.77668095,  3.03660562],
       [-3.68224073,  3.25657497],
       [-3.54006889,  3.44022156],
       [-3.35598421,  3.57646869],
       [-3.13760961,  3.65254764],
       [-2.90080896,  3.67467111],
       [-2.65461679,  3.64376651],
       [-2.40706496,  3.56193258],
       [-2.16512954,  3.43341662],
       [-1.93388155,  3.26520884],
       [-1.71535556,  3.06680536],
       [-1.50806803,  2.84881997],
       [-1.30724964,  2.62167251],
       [-1.10694718,  2.39539057],
       [-0.90912721,  2.17228422],
       [-0.71541405,  1.95420934],
       [-0.52631773,  1.74170819],
       [-0.34033597,  1.53294474],
       [-0.15917737,  1.32994478],
       [ 0.01111836,  1.13948501]])
