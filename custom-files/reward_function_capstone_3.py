# speed range: 1.5 - 4
import math

################## HELPER FUNCTIONS ###################


def dist_2_points(x1, x2, y1, y2):
    return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5


def closest_2_racing_points_index(racing_coords, car_coords):

    # Calculate all distances to racing points
    distances = []
    for i in range(len(racing_coords)):
        distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                 y1=racing_coords[i][1], y2=car_coords[1])
        distances.append(distance)

    # Get index of the closest racing point
    closest_index = distances.index(min(distances))

    # Get index of the second closest racing point
    distances_no_closest = distances.copy()
    distances_no_closest[closest_index] = 999
    second_closest_index = distances_no_closest.index(
        min(distances_no_closest))

    return [closest_index, second_closest_index]


def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):

    # Calculate the distances between 2 closest racing points
    a = abs(dist_2_points(x1=closest_coords[0],
                          x2=second_closest_coords[0],
                          y1=closest_coords[1],
                          y2=second_closest_coords[1]))

    # Distances between car and closest and second closest racing point
    b = abs(dist_2_points(x1=car_coords[0],
                          x2=closest_coords[0],
                          y1=car_coords[1],
                          y2=closest_coords[1]))
    c = abs(dist_2_points(x1=car_coords[0],
                          x2=second_closest_coords[0],
                          y1=car_coords[1],
                          y2=second_closest_coords[1]))

    # Calculate distance between car and racing line (goes through 2 closest racing points)
    # try-except in case a=0 (rare bug in DeepRacer)
    try:
        distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                        (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
    except:
        distance = b

    return distance

# Calculate which one of the closest racing points is the next one and which one the previous one


def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

    # Virtually set the car more into the heading direction
    heading_vector = [math.cos(math.radians(
        heading)), math.sin(math.radians(heading))]
    new_car_coords = [car_coords[0]+heading_vector[0],
                      car_coords[1]+heading_vector[1]]

    # Calculate distance from new car coords to 2 closest racing points
    distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                x2=closest_coords[0],
                                                y1=new_car_coords[1],
                                                y2=closest_coords[1])
    distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                       x2=second_closest_coords[0],
                                                       y1=new_car_coords[1],
                                                       y2=second_closest_coords[1])

    if distance_closest_coords_new <= distance_second_closest_coords_new:
        next_point_coords = closest_coords
        prev_point_coords = second_closest_coords
    else:
        next_point_coords = second_closest_coords
        prev_point_coords = closest_coords

    return [next_point_coords, prev_point_coords]


def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

    # Calculate the direction of the center line based on the closest waypoints
    next_point, prev_point = next_prev_racing_point(closest_coords,
                                                    second_closest_coords,
                                                    car_coords,
                                                    heading)

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(
        next_point[1] - prev_point[1], next_point[0] - prev_point[0])

    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - heading)
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    return direction_diff


class Reward:
    def __init__(self):
        self.first_racingpoint_index = 0

    def reward_function(self, params):

        # planned speed based on waypoints
        # manually adjust the list for better performance, e.g. lower the speed before turning
        # above_three_five = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 93, 94, 95, 114, 115, 116, 117, 118]
        # above_three = [12, 41, 71, 91, 92, 96, 97, 112, 113]
        # above_two_five = [13, 14, 40, 63, 64, 65, 66, 67, 68, 69, 70, 72, 73, 74, 88, 89, 90, 98, 99, 110, 111]
        # above_two = [38, 39, 75, 76, 86, 87, 100, 101, 108, 109]
        # below_two = [15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 77, 78, 79, 80, 81, 82, 83, 84, 85, 102, 103, 104, 105, 106, 107]
        # planned speed based on waypoints

        turn_track = list(range(14, 18)) + list(range(81, 91))
        strong_turn_track = list(range(18, 70)) + list(range(92, 100))

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = abs(params['steering_angle'])
        track_width = params['track_width']
        is_offtrack = params['is_offtrack']
        is_crashed = params['is_crashed']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################
        reward = 1e-3

        # Zero reward if off track ##
        if is_offtrack is True:
            return reward
        if is_crashed:
            return reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            return reward

        # Reward if car goes close to optimal racing line
        def get_distance_reward(threshold, distance, multiplier):
            distance_reward = max(0, 1 - (distance / threshold))
            return distance_reward * multiplier

        DIST_THRESH = track_width * 0.5
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])

        if closest_index in turn_track:
            reward += get_distance_reward(DIST_THRESH, dist, 2)
        elif closest_index in strong_turn_track:
            reward += get_distance_reward(DIST_THRESH, dist, 5)
        else:
            reward += get_distance_reward(DIST_THRESH, dist, 1)

        # Reward if speed falls within optimal range
        PENALTY_RATIO = 0.9
        SPEED_DIFF_NO_REWARD = 1
        speed_diff = abs(optimals[2]-speed)
        if speed_diff > SPEED_DIFF_NO_REWARD:
            return 1e-3

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # def get_speed_reward(ceiling, threshold, diff):
        #     return ceiling - diff/threshold
        # if closest_index in above_three_five:
        #     if speed >= 3.5:
        #         reward += get_speed_reward(0.5, SPEED_DIFF_NO_REWARD, speed_diff)
        #     if steering_angle > 3:
        #         reward *= PENALTY_RATIO
        # elif closest_index in above_three:
        #     if speed >= 3:
        #         reward += get_speed_reward(0.5, SPEED_DIFF_NO_REWARD, speed_diff)
        #     if steering_angle > 8:
        #         reward *= PENALTY_RATIO
        # elif closest_index in above_two_five:
        #     if speed >= 2.5:
        #         reward += get_speed_reward(0.8, SPEED_DIFF_NO_REWARD, speed_diff)
        #     if steering_angle > 15:
        #         reward *= PENALTY_RATIO
        # elif closest_index in above_two:
        #     if speed >= 2:
        #         reward += get_speed_reward(1, SPEED_DIFF_NO_REWARD, speed_diff)
        # else:
        #     if speed < 2:
        #         reward += get_speed_reward(3, SPEED_DIFF_NO_REWARD, speed_diff)

        # Incentive for finishing the lap in less steps ##
        # REWARD_FOR_FASTEST_TIME = 2000 # should be adapted to track length and other rewards
        # TARGET_STEPS = 110
        # if progress == 100:
        #     reward += REWARD_FOR_FASTEST_TIME / (steps - TARGET_STEPS)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward()


def reward_function(params):
    return reward_object.reward_function(params)

#################### RACING LINE ######################


# Optimal racing line for the Spain track
# Each row: [x,y,speed,timeFromPreviousPoint]
racing_track = [[0.01112, 1.13949, 4.0, 0.06387],
                [0.1352, 1.00074, 4.0, 0.04653],
                [0.26846, 0.85148, 4.0, 0.05002],
                [0.40979, 0.69296, 3.87568, 0.0548],
                [0.56477, 0.51881, 2.64635, 0.0881],
                [0.73414, 0.32812, 2.13154, 0.11965],
                [0.91684, 0.12203, 1.83247, 0.15029],
                [1.11063, -0.097, 1.63067, 0.17935],
                [1.31099, -0.32443, 1.48061, 0.20471],
                [1.51343, -0.54953, 1.36156, 0.22235],
                [1.72028, -0.76965, 1.25992, 0.23975],
                [1.93372, -0.982, 1.16417, 0.25861],
                [2.15572, -1.18368, 1.01251, 0.29623],
                [2.38809, -1.37176, 0.88421, 0.3381],
                [2.63253, -1.54309, 0.7946, 0.37567],
                [2.88519, -1.6913, 0.72652, 0.40319],
                [3.11891, -1.8593, 0.67294, 0.42772],
                [3.33162, -2.04984, 0.62742, 0.45515],
                [3.51581, -2.26902, 0.58946, 0.4857],
                [3.65591, -2.5175, 0.58946, 0.48392],
                [3.73349, -2.77807, 0.58946, 0.46121],
                [3.74305, -3.02772, 0.58946, 0.42383],
                [3.6903, -3.25189, 0.58946, 0.39068],
                [3.5822, -3.44054, 0.58946, 0.36886],
                [3.42466, -3.58316, 0.62606, 0.33943],
                [3.23121, -3.67852, 0.65989, 0.32683],
                [3.01044, -3.72345, 0.69569, 0.32385],
                [2.77001, -3.71292, 0.73739, 0.32637],
                [2.5191, -3.6413, 0.78573, 0.33208],
                [2.28044, -3.51186, 0.82675, 0.3284],
                [2.03215, -3.43514, 0.87476, 0.29708],
                [1.77902, -3.40232, 0.72191, 0.35357],
                [1.52383, -3.40759, 0.61911, 0.41228],
                [1.2684, -3.44655, 0.55044, 0.46941],
                [1.01401, -3.51545, 0.5, 0.5271],
                [0.77053, -3.60671, 0.5, 0.52005],
                [0.53362, -3.6548, 0.5, 0.48349],
                [0.3104, -3.64892, 0.5, 0.44659],
                [0.11026, -3.58394, 0.5, 0.42085],
                [-0.05291, -3.45595, 0.5, 0.41475],
                [-0.15229, -3.25999, 0.66311, 0.33135],
                [-0.20067, -3.02752, 0.81347, 0.2919],
                [-0.20464, -2.77018, 0.78049, 0.32975],
                [-0.18115, -2.49996, 0.70573, 0.38434],
                [-0.16489, -2.21174, 0.64791, 0.44556],
                [-0.17203, -1.93114, 0.60178, 0.46642],
                [-0.21333, -1.66507, 0.56308, 0.47818],
                [-0.29504, -1.42209, 0.56308, 0.45527],
                [-0.41885, -1.21212, 0.56308, 0.4329],
                [-0.582, -1.04584, 0.56308, 0.41371],
                [-0.77765, -0.93445, 0.56308, 0.39983],
                [-0.99454, -0.88962, 0.56308, 0.39335],
                [-1.21466, -0.92357, 0.66077, 0.33705],
                [-1.42325, -1.01555, 0.69553, 0.32777],
                [-1.6111, -1.16128, 0.73635, 0.32288],
                [-1.76942, -1.35625, 0.785, 0.31995],
                [-1.88994, -1.5943, 0.77355, 0.34493],
                [-1.96612, -1.86645, 0.72209, 0.39138],
                [-1.99508, -2.16083, 0.6795, 0.43532],
                [-1.97955, -2.46299, 0.64315, 0.47043],
                [-2.01717, -2.75403, 0.60955, 0.48145],
                [-2.10871, -3.01621, 0.60955, 0.45558],
                [-2.24746, -3.23562, 0.60955, 0.42589],
                [-2.42353, -3.40361, 0.60955, 0.39923],
                [-2.6265, -3.51467, 0.60955, 0.37958],
                [-2.8459, -3.56381, 0.60955, 0.36886],
                [-3.06939, -3.54427, 0.64107, 0.34994],
                [-3.28356, -3.46172, 0.67756, 0.33877],
                [-3.47729, -3.32125, 0.71893, 0.33284],
                [-3.6402, -3.12856, 0.76868, 0.32826],
                [-3.76315, -2.89168, 0.82963, 0.32169],
                [-3.8399, -2.62155, 0.90653, 0.30977],
                [-3.86904, -2.33091, 1.00934, 0.2894],
                [-3.85489, -2.03183, 1.15422, 0.25941],
                [-3.80613, -1.73316, 1.38279, 0.21885],
                [-3.73346, -1.4389, 1.67144, 0.18134],
                [-3.64481, -1.14933, 1.23596, 0.24502],
                [-3.54432, -0.86392, 1.01082, 0.29935],
                [-3.43592, -0.58153, 0.87548, 0.34551],
                [-3.32335, -0.30079, 0.78116, 0.3872],
                [-3.21691, -0.03346, 0.76754, 0.37489],
                [-3.12703, 0.2281, 0.76754, 0.36033],
                [-3.06442, 0.48208, 0.76754, 0.34081],
                [-3.03591, 0.72816, 0.76754, 0.32276],
                [-3.04665, 0.96545, 0.76754, 0.30946],
                [-3.10228, 1.19182, 0.76754, 0.3037],
                [-3.20394, 1.40584, 0.82236, 0.28811],
                [-3.33801, 1.61107, 0.77033, 0.31823],
                [-3.49698, 1.81004, 0.72588, 0.35085],
                [-3.63841, 2.03708, 0.68782, 0.38888],
                [-3.74471, 2.28069, 0.65309, 0.40698],
                [-3.80715, 2.53563, 0.62266, 0.42154],
                [-3.81881, 2.79193, 0.62266, 0.41203],
                [-3.77668, 3.03661, 0.62266, 0.39874],
                [-3.68224, 3.25657, 0.62266, 0.38446],
                [-3.54007, 3.44022, 0.62266, 0.37299],
                [-3.35598, 3.57647, 0.62266, 0.36781],
                [-3.13761, 3.65255, 0.70152, 0.32964],
                [-2.90081, 3.67467, 0.7522, 0.31618],
                [-2.65462, 3.64377, 0.81503, 0.30444],
                [-2.40706, 3.56193, 0.89576, 0.29107],
                [-2.16513, 3.43342, 1.00494, 0.2726],
                [-1.93388, 3.26521, 1.16625, 0.24519],
                [-1.71536, 3.06681, 1.43466, 0.20573],
                [-1.50807, 2.84882, 2.05342, 0.14649],
                [-1.30725, 2.62167, 4.0, 0.0758],
                [-1.10695, 2.39539, 4.0, 0.07555],
                [-0.90913, 2.17228, 4.0, 0.07454],
                [-0.71541, 1.95421, 4.0, 0.07292],
                [-0.52632, 1.74171, 4.0, 0.07111],
                [-0.34034, 1.53294, 4.0, 0.0699],
                [-0.15918, 1.32994, 4.0, 0.06802]]
