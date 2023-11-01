# use the postion values to determine the distance between the robots
def calc_distance (x_0, y_0, x_1, y_1):
    x_dist = (x_1 - x_0)**2
    y_dist = (y_1 - y_0)**2

    distance = (x_dist + y_dist)**0.5
    return distance

def usr(robot):
    import struct
    desired_distance = .3 #will vary from 0.3-0.5
    current_distance = desired_distance
    past_distance = current_distance
    # had to use global variables to get around variable re-writing breaking the sim
    # I'm sure there is a better way to deal with this?
    r_0_x = 0
    r_0_y = 0
    r_1_x = 0
    r_1_y = 0
    # set base speed for the wheels
    base_speed = desired_distance * 10
    left_wheel = base_speed
    right_wheel = base_speed

    
    while True:
        if (robot.id == 0):
            # if we received a message, print out info in message
            msgs = robot.recv_msg()
            if len(msgs) > 0:
                pose_rxed = struct.unpack('ffi', msgs[0][:12])
                #print('robot ', robot.id, ' received position ', pose_rxed[0], pose_rxed[1], ' from robot ', pose_rxed[2])
                r_1_x = pose_rxed[0]
                r_1_y = pose_rxed[1]


            pose_t = robot.get_pose()
            # if there is a new postion sensor update, print out and transmit the info
            if pose_t:  # check pose is valid before using
                print('The x,y postion of robot ', robot.id, ' is ', pose_t[0], pose_t[1])
                r_0_x = pose_t[0]
                r_0_y = pose_t[1]
                current_distance = calc_distance(r_1_x, r_1_y, r_0_x, r_0_y)
                print("current distance = ", current_distance)
                if current_distance > desired_distance:
                    robot.set_led(100,0,0)

                else:
                    robot.set_led(0,100,0)
                robot.send_msg(struct.pack('ffi', pose_t[0], pose_t[1], robot.id))  # send pose x,y in message
                #print("calculate distance")
         
         
        if (robot.id == 1):
            # if we received a message, print out info in message
            msgs = robot.recv_msg()
            if len(msgs) > 0:
                pose_rxed = struct.unpack('ffi', msgs[0][:12])
                #print('robot ', robot.id, ' received position ', pose_rxed[0], pose_rxed[1], ' from robot ', pose_rxed[2])
                r_0_x = pose_rxed[0]
                r_0_y = pose_rxed[1]
                #blink led if message is received  
                # robot.set_led(0,100,0)
                # robot.delay(10)
                # robot.set_led(0,0,0)

            pose_t = robot.get_pose()
            # if there is a new postion sensor update, print out and transmit the info
            if pose_t:  # check pose is valid before using
                #print('The x,y postion of robot ', robot.id, ' is ', pose_t[0], pose_t[1])
                r_1_x = pose_t[0]
                r_1_y = pose_t[1]
                
                # get the current distance between the robots
                current_distance = calc_distance(r_1_x, r_1_y, r_0_x, r_0_y)
                # print("current distance = ", current_distance)
                # print("past distance = ", past_distance)



                # if the current distance is farther than the desired distance
                if current_distance > desired_distance:
                    # print("farther than desired distance")
                    #set led to red
                    robot.set_led(100,0,0)
                    
                    # if current_distance is closer to desired than past_distance (getting closer)
                    if abs(current_distance-desired_distance) <= abs(past_distance-desired_distance):
                        # move in a wide circle
                        # print("turn slowly")
                        left_wheel = base_speed + 1
                        # print("left wheel speed =", left_wheel)
                        right_wheel =  base_speed - 1
                        # print("right wheel speed =", right_wheel)
                    
                    # if the current_distance is farther from desired_distance than past_distance
                    else:
                        # move in a tighter circle
                        # print("turn quickly")
                        left_wheel = base_speed + 2
                        # print("left wheel speed =", left_wheel)
                        right_wheel = base_speed - 2
                        # print("right wheel speed =", right_wheel)

                    robot.set_vel(left_wheel,right_wheel)

                                   

                # else: i.e. the current distance is closer than desired distance
                else:
                    #set led to green
                    robot.set_led(0,100,0)
                    # print("closer than distance required")
                    
                    # if the current distance is getting closer to desired distance than past_distance
                    # print("abs current - desired", abs(current_distance-desired_distance))
                    # print("abs past - desired", abs(past_distance-desired_distance))
                    if abs(current_distance-desired_distance) < abs(past_distance-desired_distance):
                        #move in a wide circle 
                        # print("keep turning")
                        left_wheel = base_speed + 1
                        # print("left wheel speed =", left_wheel)
                        right_wheel = base_speed
                        # print("right wheel speed =", right_wheel)

                    # if the current_distance is farther from desired_distance than past_distance
                    else:
                        # go straight
                        # print("go straight")
                        left_wheel = base_speed
                        # print("left wheel speed =", left_wheel)
                        right_wheel = base_speed
                        # print("right wheel speed =", right_wheel)

                    robot.set_vel(left_wheel,right_wheel)
                    
                    
                past_distance = current_distance
                robot.send_msg(struct.pack('ffi', pose_t[0], pose_t[1], robot.id))  # send pose x,y in message
            #move wheels
            