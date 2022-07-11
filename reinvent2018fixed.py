import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1 - x2) ** 2 + abs(y1 - y2) ** 2) ** 0.5

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
                distance = abs(-(a ** 4) + 2 * (a ** 2) * (b ** 2) + 2 * (a ** 2) * (c ** 2) -
                               (b ** 4) + 2 * (b ** 2) * (c ** 2) - (c ** 4)) ** 0.5 / (2 * a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0] + heading_vector[0],
                              car_coords[1] + heading_vector[1]]

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
            if start is None:
                start = closest_index
            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count - 1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time / current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the 2018 track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[3.07857,0.7234,3.2,0.04483],
                        [3.22295,0.71246,3.2,0.04525],
                        [3.36865,0.70402,3.2,0.04561],
                        [3.51539,0.69762,3.2,0.0459],
                        [3.66294,0.69287,3.2,0.04613],
                        [3.81112,0.68942,3.2,0.04632],
                        [3.95978,0.68698,3.2,0.04646],
                        [4.10881,0.68536,3.2,0.04658],
                        [4.25813,0.68454,3.2,0.04666],
                        [4.4074,0.68487,3.2,0.04665],
                        [4.55614,0.68678,3.2,0.04648],
                        [4.704,0.69061,3.2,0.04622],
                        [4.85072,0.69669,3.2,0.04589],
                        [4.99598,0.70537,3.2,0.04548],
                        [5.13949,0.71702,3.1441,0.04579],
                        [5.28093,0.73198,2.99692,0.04746],
                        [5.41997,0.75056,2.81078,0.0499],
                        [5.55628,0.77305,2.57432,0.05367],
                        [5.68961,0.79959,2.38821,0.05693],
                        [5.81987,0.83014,2.13351,0.06271],
                        [5.94681,0.86481,1.91826,0.0686],
                        [6.07015,0.90377,1.70891,0.07569],
                        [6.18943,0.94729,1.5267,0.08317],
                        [6.30396,0.99586,1.33303,0.09332],
                        [6.41305,1.04984,1.33303,0.09131],
                        [6.51548,1.10999,1.33303,0.08911],
                        [6.60983,1.17694,1.2,0.09641],
                        [6.69419,1.25136,1.2,0.09375],
                        [6.76624,1.33366,1.2,0.09115],
                        [6.82221,1.42397,1.2,0.08854],
                        [6.86523,1.51907,1.2,0.08698],
                        [6.89274,1.61832,1.2,0.08582],
                        [6.90063,1.72008,1.25561,0.08129],
                        [6.89071,1.82141,1.31099,0.07766],
                        [6.86585,1.92062,1.31099,0.07801],
                        [6.82793,2.01677,1.31099,0.07884],
                        [6.77364,2.10731,1.47028,0.0718],
                        [6.70615,2.19179,1.61668,0.06688],
                        [6.62745,2.27016,1.7578,0.06318],
                        [6.53892,2.34243,1.98083,0.0577],
                        [6.4423,2.4092,2.18742,0.05369],
                        [6.33878,2.4709,2.428,0.04964],
                        [6.2294,2.52805,2.72594,0.04527],
                        [6.11518,2.58128,3.12658,0.0403],
                        [5.99717,2.63131,3.2,0.04006],
                        [5.87631,2.67889,3.2,0.04059],
                        [5.75359,2.72482,3.2,0.04095],
                        [5.62981,2.76984,3.2,0.04116],
                        [5.49795,2.81748,3.2,0.04381],
                        [5.36653,2.86607,3.2,0.04379],
                        [5.23582,2.91617,3.2,0.04375],
                        [5.10609,2.96836,3.2,0.0437],
                        [4.97753,3.02305,3.2,0.04366],
                        [4.8503,3.08056,3.2,0.04363],
                        [4.72449,3.14103,3.2,0.04362],
                        [4.6001,3.20451,3.2,0.04364],
                        [4.47711,3.27091,3.2,0.04368],
                        [4.3554,3.33998,3.2,0.04373],
                        [4.23479,3.41134,3.2,0.04379],
                        [4.11504,3.48448,3.2,0.04385],
                        [3.99593,3.55896,3.2,0.0439],
                        [3.87735,3.63453,3.18039,0.04421],
                        [3.76113,3.7075,2.93388,0.04678],
                        [3.64439,3.77888,2.75264,0.04971],
                        [3.52693,3.84793,2.61896,0.05202],
                        [3.40855,3.91383,2.52022,0.05376],
                        [3.28907,3.97573,2.45157,0.05489],
                        [3.16841,4.03287,2.4139,0.05531],
                        [3.04654,4.08445,2.4139,0.05482],
                        [2.92352,4.12983,2.4139,0.05432],
                        [2.79954,4.16845,2.4139,0.0538],
                        [2.67484,4.19988,2.4139,0.05328],
                        [2.54972,4.22385,2.33725,0.0545],
                        [2.42455,4.24027,2.2162,0.05696],
                        [2.29967,4.24929,2.05076,0.06105],
                        [2.17537,4.25144,1.91759,0.06483],
                        [2.05199,4.24673,1.70962,0.07222],
                        [1.92988,4.23511,1.50117,0.08171],
                        [1.80954,4.21626,1.33113,0.09151],
                        [1.69159,4.1896,1.33113,0.09084],
                        [1.57699,4.15419,1.33113,0.09011],
                        [1.46688,4.10924,1.33113,0.08935],
                        [1.36327,4.053,1.33113,0.08856],
                        [1.26935,3.98333,1.33113,0.08785],
                        [1.18961,3.89844,1.42458,0.08176],
                        [1.12366,3.8017,1.59614,0.07335],
                        [1.06963,3.69616,1.74427,0.06797],
                        [1.02636,3.58353,1.88984,0.06385],
                        [0.99299,3.46502,2.03412,0.06053],
                        [0.96886,3.3416,2.17896,0.05771],
                        [0.95341,3.21418,2.32353,0.05524],
                        [0.94612,3.08358,2.46294,0.05311],
                        [0.94652,2.95069,2.58825,0.05135],
                        [0.95419,2.81634,2.70689,0.04971],
                        [0.96868,2.68141,2.77958,0.04882],
                        [0.98967,2.5467,2.82717,0.04823],
                        [1.01688,2.41293,2.82087,0.04839],
                        [1.05003,2.28075,2.64963,0.05143],
                        [1.0888,2.15069,2.47762,0.05478],
                        [1.13289,2.02315,2.25579,0.05982],
                        [1.18207,1.89846,2.05056,0.06536],
                        [1.23616,1.77692,1.87714,0.07087],
                        [1.29518,1.65891,1.62092,0.0814],
                        [1.35951,1.54509,1.62092,0.08066],
                        [1.42956,1.4362,1.62092,0.07987],
                        [1.50611,1.33343,1.62092,0.07906],
                        [1.59004,1.23816,1.62092,0.07833],
                        [1.68227,1.15201,1.62092,0.07786],
                        [1.785,1.07848,1.89629,0.06662],
                        [1.89513,1.01457,2.09791,0.06069],
                        [2.01127,0.95888,2.29171,0.0562],
                        [2.13249,0.91042,2.47149,0.05282],
                        [2.25813,0.86849,2.6827,0.04937],
                        [2.38758,0.8324,2.86066,0.04698],
                        [2.52045,0.80169,3.05712,0.04461],
                        [2.65635,0.77591,3.2,0.04323],
                        [2.79492,0.75461,3.2,0.04381],
                        [2.93578,0.73728,3.2,0.04435]]

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
        distance_reward = max(1e-3, 1 - (dist / (track_width * 0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff / (SPEED_DIFF_NO_REWARD)) ** 2) ** 2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 15
        FASTEST_TIME = 7
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME * (FASTEST_TIME) /
                                           (STANDARD_TIME - FASTEST_TIME)) * (
                                            steps_prediction - (STANDARD_TIME * 15 + 1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3

        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2] - speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 400  # should be adapted to track length and other rewards
        STANDARD_TIME = 15  # seconds (time that is easily done by model)
        FASTEST_TIME = 7  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15 * (STANDARD_TIME - FASTEST_TIME))) * (steps - STANDARD_TIME * 15))
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
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
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