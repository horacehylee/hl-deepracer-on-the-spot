import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
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

        # Gives back indexes that lie between start and end index of a cyclical list
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(
                first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum(
                [times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time /
                                  current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[0.01112, 1.13949, 4.0, 0.06387],
                        [0.1352, 1.00074, 4.0, 0.04653],
                        [0.26846, 0.85148, 4.0, 0.05002],
                        [0.40979, 0.69296, 4.0, 0.05309],
                        [0.56477, 0.51881, 4.0, 0.05828],
                        [0.73414, 0.32812, 3.41046, 0.07478],
                        [0.91684, 0.12203, 2.93195, 0.09393],
                        [1.11063, -0.097, 2.60907, 0.11209],
                        [1.31099, -0.32443, 2.36898, 0.12794],
                        [1.51343, -0.54953, 2.1785, 0.13897],
                        [1.72028, -0.76965, 2.01587, 0.14984],
                        [1.93372, -0.982, 1.86267, 0.16163],
                        [2.15572, -1.18368, 1.62002, 0.18514],
                        [2.38809, -1.37176, 1.41474, 0.21131],
                        [2.63253, -1.54309, 1.27137, 0.23479],
                        [2.88519, -1.6913, 1.16243, 0.25199],
                        [3.11891, -1.8593, 1.07671, 0.26733],
                        [3.33162, -2.04984, 1.00387, 0.28447],
                        [3.51581, -2.26902, 0.94314, 0.30356],
                        [3.65591, -2.5175, 0.94314, 0.30245],
                        [3.73349, -2.77807, 0.94314, 0.28826],
                        [3.74305, -3.02772, 0.94314, 0.2649],
                        [3.6903, -3.25189, 0.94314, 0.24418],
                        [3.5822, -3.44054, 0.94314, 0.23054],
                        [3.42466, -3.58316, 1.00169, 0.21215],
                        [3.23121, -3.67852, 1.05582, 0.20427],
                        [3.01044, -3.72345, 1.11311, 0.2024],
                        [2.77001, -3.71292, 1.17983, 0.20398],
                        [2.5191, -3.6413, 1.25717, 0.20755],
                        [2.28044, -3.51186, 1.32281, 0.20525],
                        [2.03215, -3.43514, 1.39961, 0.18567],
                        [1.77902, -3.40232, 1.15506, 0.22098],
                        [1.52383, -3.40759, 0.99057, 0.25768],
                        [1.2684, -3.44655, 0.88071, 0.29338],
                        [1.01401, -3.51545, 0.8, 0.32944],
                        [0.77053, -3.60671, 0.8, 0.32503],
                        [0.53362, -3.6548, 0.8, 0.30218],
                        [0.3104, -3.64892, 0.8, 0.27912],
                        [0.11026, -3.58394, 0.8, 0.26303],
                        [-0.05291, -3.45595, 0.8, 0.25922],
                        [-0.15229, -3.25999, 1.06097, 0.20709],
                        [-0.20067, -3.02752, 1.30155, 0.18244],
                        [-0.20464, -2.77018, 1.24879, 0.2061],
                        [-0.18115, -2.49996, 1.12916, 0.24021],
                        [-0.16489, -2.21174, 1.03666, 0.27848],
                        [-0.17203, -1.93114, 0.96285, 0.29152],
                        [-0.21333, -1.66507, 0.90092, 0.29887],
                        [-0.29504, -1.42209, 0.90092, 0.28454],
                        [-0.41885, -1.21212, 0.90092, 0.27056],
                        [-0.582, -1.04584, 0.90092, 0.25857],
                        [-0.77765, -0.93445, 0.90092, 0.24989],
                        [-0.99454, -0.88962, 0.90092, 0.24584],
                        [-1.21466, -0.92357, 1.05724, 0.21066],
                        [-1.42325, -1.01555, 1.11285, 0.20485],
                        [-1.6111, -1.16128, 1.17816, 0.2018],
                        [-1.76942, -1.35625, 1.256, 0.19997],
                        [-1.88994, -1.5943, 1.23768, 0.21558],
                        [-1.96612, -1.86645, 1.15534, 0.24461],
                        [-1.99508, -2.16083, 1.0872, 0.27208],
                        [-1.97955, -2.46299, 1.02904, 0.29402],
                        [-2.01717, -2.75403, 0.97527, 0.3009],
                        [-2.10871, -3.01621, 0.97527, 0.28474],
                        [-2.24746, -3.23562, 0.97527, 0.26618],
                        [-2.42353, -3.40361, 0.97527, 0.24952],
                        [-2.6265, -3.51467, 0.97527, 0.23724],
                        [-2.8459, -3.56381, 0.97527, 0.23054],
                        [-3.06939, -3.54427, 1.02571, 0.21871],
                        [-3.28356, -3.46172, 1.08409, 0.21173],
                        [-3.47729, -3.32125, 1.15029, 0.20803],
                        [-3.6402, -3.12856, 1.22988, 0.20517],
                        [-3.76315, -2.89168, 1.32741, 0.20106],
                        [-3.8399, -2.62155, 1.45045, 0.19361],
                        [-3.86904, -2.33091, 1.61494, 0.18087],
                        [-3.85489, -2.03183, 1.84675, 0.16213],
                        [-3.80613, -1.73316, 2.21246, 0.13678],
                        [-3.73346, -1.4389, 2.67431, 0.11334],
                        [-3.64481, -1.14933, 1.97754, 0.15314],
                        [-3.54432, -0.86392, 1.61731, 0.18709],
                        [-3.43592, -0.58153, 1.40076, 0.21594],
                        [-3.32335, -0.30079, 1.24986, 0.242],
                        [-3.21691, -0.03346, 1.22807, 0.2343],
                        [-3.12703, 0.2281, 1.22807, 0.2252],
                        [-3.06442, 0.48208, 1.22807, 0.21301],
                        [-3.03591, 0.72816, 1.22807, 0.20172],
                        [-3.04665, 0.96545, 1.22807, 0.19342],
                        [-3.10228, 1.19182, 1.22807, 0.18981],
                        [-3.20394, 1.40584, 1.31577, 0.18007],
                        [-3.33801, 1.61107, 1.23253, 0.1989],
                        [-3.49698, 1.81004, 1.16141, 0.21928],
                        [-3.63841, 2.03708, 1.10051, 0.24305],
                        [-3.74471, 2.28069, 1.04495, 0.25436],
                        [-3.80715, 2.53563, 0.99626, 0.26346],
                        [-3.81881, 2.79193, 0.99626, 0.25752],
                        [-3.77668, 3.03661, 0.99626, 0.24921],
                        [-3.68224, 3.25657, 0.99626, 0.24028],
                        [-3.54007, 3.44022, 0.99626, 0.23312],
                        [-3.35598, 3.57647, 0.99626, 0.22988],
                        [-3.13761, 3.65255, 1.12243, 0.20602],
                        [-2.90081, 3.67467, 1.20352, 0.19761],
                        [-2.65462, 3.64377, 1.30404, 0.19027],
                        [-2.40706, 3.56193, 1.43321, 0.18192],
                        [-2.16513, 3.43342, 1.60791, 0.17038],
                        [-1.93388, 3.26521, 1.866, 0.15324],
                        [-1.71536, 3.06681, 2.29545, 0.12858],
                        [-1.50807, 2.84882, 3.28548, 0.09156],
                        [-1.30725, 2.62167, 4.0, 0.0758],
                        [-1.10695, 2.39539, 4.0, 0.07555],
                        [-0.90913, 2.17228, 4.0, 0.07454],
                        [-0.71541, 1.95421, 4.0, 0.07292],
                        [-0.52632, 1.74171, 4.0, 0.07111],
                        [-0.34034, 1.53294, 4.0, 0.0699],
                        [-0.15918, 1.32994, 4.0, 0.06802]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0  # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

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

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(
            self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME,
                               reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3

        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        # should be adapted to track length and other rewards
        REWARD_FOR_FASTEST_TIME = 1500
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward

        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################

        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" %
                  (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward()  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
