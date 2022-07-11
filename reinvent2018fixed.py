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
        racing_track = [[2.88739, 0.72647, 3.98849, 0.06818],
                        [3.16759, 0.70479, 4.0, 0.07026],
                        [3.45517, 0.69218, 4.0, 0.07196],
                        [3.75325, 0.68581, 3.77521, 0.07897],
                        [4.07281, 0.68361, 3.15803, 0.10119],
                        [4.5, 0.68376, 2.74886, 0.15541],
                        [4.55, 0.68378, 2.42015, 0.02066],
                        [5.11738, 0.6908, 2.15197, 0.26368],
                        [5.44798, 0.71123, 1.90448, 0.17392],
                        [5.71127, 0.74223, 1.68885, 0.15697],
                        [5.94137, 0.78496, 1.46599, 0.15965],
                        [6.14913, 0.84078, 1.46599, 0.14674],
                        [6.33676, 0.91067, 1.46599, 0.13658],
                        [6.50352, 0.99484, 1.42842, 0.13077],
                        [6.64763, 1.09336, 1.3, 0.13428],
                        [6.76715, 1.2064, 1.3, 0.12655],
                        [6.8579, 1.33509, 1.3, 0.12113],
                        [6.92194, 1.47647, 1.3, 0.11939],
                        [6.96027, 1.62797, 1.3, 0.12022],
                        [6.9669, 1.78881, 1.3, 0.12382],
                        [6.92977, 1.95515, 1.36135, 0.1252],
                        [6.8538, 2.1191, 1.36135, 0.13273],
                        [6.72693, 2.26842, 1.68002, 0.11662],
                        [6.56583, 2.39791, 1.94962, 0.10602],
                        [6.38076, 2.50633, 2.29824, 0.09333],
                        [6.18037, 2.59603, 2.85205, 0.07698],
                        [5.97126, 2.67207, 4.0, 0.05563],
                        [5.75829, 2.7411, 3.52258, 0.06356],
                        [5.55881, 2.81131, 3.52258, 0.06003],
                        [5.36088, 2.88624, 3.52258, 0.06008],
                        [5.16456, 2.96629, 3.52258, 0.06019],
                        [4.96989, 3.05191, 3.52258, 0.06037],
                        [4.77697, 3.14378, 3.52258, 0.06066],
                        [4.58661, 3.2454, 4.0, 0.05395],
                        [4.39799, 3.3542, 4.0, 0.05444],
                        [4.21046, 3.4676, 3.55007, 0.06173],
                        [4.02348, 3.58333, 3.01133, 0.07303],
                        [3.85069, 3.68988, 2.64103, 0.07686],
                        [3.68265, 3.79114, 2.64103, 0.07429],
                        [3.51884, 3.8857, 2.64103, 0.07161],
                        [3.35641, 3.97362, 2.61944, 0.07051],
                        [3.19259, 4.05427, 2.45884, 0.07426],
                        [3.02555, 4.12572, 2.29479, 0.07917],
                        [2.85392, 4.18548, 2.11207, 0.08604],
                        [2.67755, 4.234, 1.96027, 0.09332],
                        [2.49619, 4.27141, 1.71517, 0.10797],
                        [2.3088, 4.29611, 1.53552, 0.12309],
                        [2.11374, 4.30523, 1.53552, 0.12717],
                        [1.90856, 4.29409, 1.53552, 0.13382],
                        [1.68968, 4.25391, 1.53552, 0.14493],
                        [1.45388, 4.16915, 1.53552, 0.16319],
                        [1.21119, 4.00653, 1.53552, 0.19025],
                        [1.01923, 3.74402, 1.61197, 0.20175],
                        [0.92221, 3.42051, 2.15478, 0.15675],
                        [0.88927, 3.10444, 2.50133, 0.12704],
                        [0.89601, 2.82076, 2.47437, 0.11468],
                        [0.92405, 2.56281, 2.27496, 0.11405],
                        [0.96605, 2.3246, 2.11534, 0.11435],
                        [1.01803, 2.11229, 1.86323, 0.11732],
                        [1.08079, 1.91513, 1.67506, 0.12352],
                        [1.15514, 1.73108, 1.67506, 0.1185],
                        [1.24162, 1.56015, 1.67506, 0.11436],
                        [1.34113, 1.40324, 1.67506, 0.11092],
                        [1.45473, 1.26109, 1.67506, 0.10863],
                        [1.58653, 1.13641, 1.67506, 0.10831],
                        [1.74473, 1.03229, 2.03258, 0.09318],
                        [1.92656, 0.94305, 2.29484, 0.08826],
                        [2.13282, 0.86779, 2.57199, 0.08537],
                        [2.36411, 0.8068, 2.96619, 0.08064],
                        [2.61751, 0.75992, 3.4335, 0.07505]]

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