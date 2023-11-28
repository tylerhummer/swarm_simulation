import struct
import math
import numpy as np
import random

# function for going stright for a set amount of time
def go_straight(robot):
    start_time = round(robot.get_clock(),2)

    # print("go straight start time is ", start_time)

    # make moving time set to a specified amount
    # should make the amonut of moving time scale with the vector magnitude later
    while robot.get_clock() < (start_time + 2):
        robot.set_vel(20,20)
    
    # tell robot to stop moving
    robot.set_vel(0,0)



# funtion that takes in a desired turn angle and turns the robot 
# by the desired number of degrees
def rotate(robot,desired_turn):
    start_time = round(robot.get_clock(),2)
    # print("start time ", start_time)
    # using robot.set_vel(-10,10), robot turns 12 degrees / 0.6 sec.

    # calculate the amount of time to havev motors turning based on desired number of degrees
    turn_time = abs((0.6/12.15)*desired_turn)
    # print("the turn time is ", turn_time)

    while robot.get_clock() < (start_time + turn_time):
        # if the robot should be turning counter-clockwise
        if desired_turn > 0:
            # set right wheel foward and left wheel backwards
            robot.set_vel(-5,5)

        # if the robot should be turning clockwise
        else:
            # set the left wheel forward and right wheel backwards
            robot.set_vel(5,-5)

        # print("current time ", round(robot.get_clock(),2))
        current_pose = robot.get_pose()
        # print("current angle ", round(math.degrees(current_pose[2]),2))
    
    # set the velocities back to zero to stop turning
    robot.set_vel(0,0)
    

# function for calculating the x,y unit vectors for the gravity input
def gravity_vector(position):
    # takes the current robot position as the input

    # x_portion of vector = origin - current position x
    x_vector = (0 - position[0])

    # y_portion of vector = origin - current position y
    y_vector = (0 - position[1])

    # calculate the magnitude of the vector
    magnitude = abs(math.sqrt((x_vector**2)+(y_vector**2)))

    # scale by magnitude to get unit vectors
    x_unit_vector = x_vector/magnitude
    y_unit_vector = y_vector/magnitude

    # return x and y values of unit vector
    return x_unit_vector,y_unit_vector

# function for adding randomness to behavior
def random_vector():
    rand_x = random.uniform(-1,1)
    rand_y = random.uniform(-1,1)

    return rand_x, rand_y

# function for communicating and calculating the repulsion portion of controller
def repulsion_vector(robot, radius):

    # initialize an array with all zero values... limit the number of 
    # robots to communicate with to 5
    repulse_array = np.full((10,4), 0, dtype=float)
    i = 0

    # get the start time for communicating
    receive_start = round(robot.get_clock(),2)

    # receive messages for a set amount of time in the control cycle
    while robot.get_clock() < (receive_start + 2):
        my_pose = robot.get_pose()
        msg=struct.pack('ff',my_pose[0],my_pose[1])
        robot.send_msg(msg)  # send pose x,y in message

        msgs = robot.recv_msg()
        if len(msgs) > 0:
            # receive message with x,y coord. of other robots
            pose_rxed = struct.unpack('ff', msgs[0][:8])
            # add received position to buffer array
            test_pose = np.array([pose_rxed[0], pose_rxed[1]])
            # print("test_pose ", test_pose)


        # determine if the robot who sent message is within our own radius to repulse
        distance = math.sqrt((my_pose[0]-test_pose[0])**2 + (my_pose[1]-test_pose[1])**2)
        # print("distance between robots is ", distance)

        # if the robot is within the radius
        if distance <= (2*radius):
            # print("the robots are too close")
            # check to see if both values are already in the array
            check = np.isin(test_pose, repulse_array)
            # print("checking if value is in array", check)

            # if both value are not in the array, add it in position i
            if np.all(check) == False:
                # print(test_pose)
                repulse_array[i][0] = float(test_pose[0])
                repulse_array[i][1] = float(test_pose[1])
                
                # put in my_pose - received_pose
                # print("my pose is ", my_pose)

                repulse_array[i][2] = my_pose[0] - test_pose[0]
                repulse_array[i][3] = my_pose[1] - test_pose[1]
                # increment i
                # print(i)
                i += 1
                # print(repulse_array)
            
            # add exit condition to while loop for if the repulse array is full
            if i > 9:
                # print("repulse buffer full")
                break
    
    #repulse_array = np.ma.array(repulse_array, mask=np.isnan(repulse_array)) # Use a mask to mark the NaNs in repulse array

    # sum up all the values in the repulse array
    vector_vals = np.sum(repulse_array, axis=0)

    #check to see if these make sense
    # print("vector values", (vector_vals))

    # # if there are no robots within communication range (i.e. all NaN values)
    # if (math.isnan(vector_vals[2])==True) and (math.isnan(vector_vals[3])==True):
    #     # return (0,0) for the repulse vectors
    #     return 0,0
    
    # else:
        # return the summed up x,y vector values from repulsion, ***not unit vectors***
    return vector_vals[2], vector_vals[3]




# function for combining all portions of the input vectors to a single angle output
def goal(gravity_x, gravity_y, repulse_x, repulse_y,rand_x,rand_y):

    # convert repulse vectors to a unit vector
    repulse_magnitude = abs(math.sqrt((repulse_x**2)+(repulse_y**2)))
    if repulse_magnitude != 0:
        # print("repulse magnitude ", repulse_magnitude)
        repulse_x_unit = repulse_x/repulse_magnitude
        repulse_y_unit = repulse_y/repulse_magnitude
        # print("repulse x_unit vector", repulse_x_unit)
        # print("repulse y_unit vector", repulse_y_unit)

        # set scale limit to the repulsion portion of controller to 3
        if repulse_magnitude < 5:
            repulse_x_vector = repulse_magnitude * repulse_x_unit
            repulse_y_vector = repulse_magnitude * repulse_y_unit

        # if the magnitude is too big, scale by max of 3
        else:
            repulse_x_vector = 5 * repulse_x_unit
            repulse_y_vector = 5 * repulse_y_unit

    else:
        repulse_x_vector = 0
        repulse_y_vector = 0


    # add up unit gravity, scaled up repulse, and scaled down random portions of the controller
    output_x = (10 * repulse_x_vector) + gravity_x + (0.25 * rand_x)
    output_y = (10 * repulse_y_vector) + gravity_y + (0.25 * rand_y)
    # print("output x ", output_x, " output y ", output_y) 

    desired_angle = math.atan2(output_y,output_x)
    print(math.degrees(desired_angle))

    return desired_angle



def usr(robot):
    # initialize all the gradient, distance, and error values to really high numbers
    id = robot.assigned_id


    while True:
        robot.delay(10)
        if id == 0:
            # set robots with id_0 to green
            robot.set_led(0,100,0)
            # set radius of id_0 = 0.2
            radius = 0.3

            # get current position
            current_pose = robot.get_pose()
            print(current_pose)

            # get the repulsion vectors based on communication with other robots
            repulse_x, repulse_y = repulsion_vector(robot, radius)

            # calculate the gravity direction vector
            gravity_x, gravity_y = gravity_vector(current_pose)

            # get random (i.e. shaking) vector
            rand_x, rand_y = random_vector()

            # goal direction = current angle - goal angle
            turn_dir = math.degrees(goal(gravity_x,gravity_y,repulse_x,repulse_y, rand_x,rand_y)-current_pose[2])
            # print("my turn angle is ", turn_dir)
            
            # execute turn
            rotate(robot, turn_dir)

            # move forward
            go_straight(robot)
            
            robot.delay(1000)

        if id == 1:
            robot.set_led(100,0,0)
            radius = 0.85

            # get current position
            current_pose = robot.get_pose()
            print(current_pose)

            # get the repulsion vectors based on communication with other robots
            repulse_x, repulse_y = repulsion_vector(robot, radius)

            # calculate the gravity direction vector
            gravity_x, gravity_y = gravity_vector(current_pose)

            # get random (i.e. shaking) vector
            rand_x, rand_y = random_vector()

            # goal direction = current angle - goal angle
            turn_dir = math.degrees(goal(gravity_x,gravity_y,repulse_x,repulse_y, rand_x,rand_y)-current_pose[2])
            # print("my turn angle is ", turn_dir)
            
            # execute turn
            rotate(robot, turn_dir)

            # move forward
            go_straight(robot)
            
            robot.delay(1000)
        
        if id == 2:
            robot.set_led(0,0,100)
            radius = 1.25
            # get current position
            current_pose = robot.get_pose()
            print(current_pose)

            # get the repulsion vectors based on communication with other robots
            repulse_x, repulse_y = repulsion_vector(robot, radius)

            # calculate the gravity direction vector
            gravity_x, gravity_y = gravity_vector(current_pose)

            # get random (i.e. shaking) vector
            rand_x, rand_y = random_vector()

            # goal direction = current angle - goal angle
            turn_dir = math.degrees(goal(gravity_x,gravity_y,repulse_x,repulse_y, rand_x,rand_y)-current_pose[2])
            # print("my turn angle is ", turn_dir)
            
            # execute turn
            rotate(robot, turn_dir)

            # move forward
            go_straight(robot)
            
            robot.delay(1000)

        # General Algorithm

        # 1. Sense "gravity", a.k.a. the center of the arena. Make it a unit vector

        # 2. Sense the other robots whose virtual radii are overlapping robot's own radius

        # 3. Add in random unit vector to the equation 

        # 4. Sum up the total vectors into a single  vector (desired direction, and an amount of time to move for)

        # 5. Determine current direction heading.

        # 6. Turn from current heading towards the desired heading (using the shortest path) until it reaches it

        # 7. Stop turning.

        # 8. Go straight for x amount of time, determined by magnitude of the vector computed above.

        # 9. Stop moving.

        # 10. Go back to step 1.

        # robot 1 is bottom left corner seed
        # use robot 1 to generate hopcount 1

        # robot 2 is top left corner seed
        # use robot 2 to generate hopcount 2

        end_pose = robot.get_pose()

        msg=struct.pack('ff',end_pose[0],end_pose[1])    
        robot.send_msg(msg)  # send pose x,y in message