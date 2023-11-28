import struct

def usr(robot):
    # initialize all the gradient, distance, and error values to really high numbers
    smoothing = True
    gradient_1 = 1000
    send_gradient_1 = 1000
    gradient_2 = 1000
    send_gradient_2 = 1000
    x_coord = 100
    y_coord = 100
    d_j_1 = 1000
    d_j_2 = 1000
    position_error = 10000
    r = 0
    g = 0
    b = 0
    gradient_1_buffer = [0,0,0,0,0,0]
    gradient_2_buffer = [0,0,0,0,0,0]
    avg_1 = 0
    avg_2 = 0
    s_1 = 0
    s_2 = 0
    buffer_index = 0

    while True:
        robot.delay(10)

        # robot 1 is bottom left corner seed
        # use robot 1 to generate hopcount 1

        if robot.assigned_id==1:
            gradient_1 = 0
            x_coord = 0.0
            y_coord = 0.0
            #print("my gradient 1 val is ", gradient_1)
            send_gradient_1 = gradient_1
            msg=struct.pack('ffff',x_coord,y_coord,send_gradient_1, send_gradient_2)
            #print("seed zero sending gradient 1 value", send_gradient_1)
            #robot.delay(1000)
            r = 100
            g = 100
            b = 100
            robot.set_led(r,g,b)       
            robot.send_msg(msg)
            robot.delay(1000)
            
        # robot 2 is top left corner seed
        # use robot 2 to generate hopcount 2

        elif robot.assigned_id==2:
            gradient_2 = 0
            x_coord = 0.0
            y_coord = 15.0
            send_gradient_2 = gradient_2
            msg=struct.pack('ffff',0.0,0.0,send_gradient_1, send_gradient_2)
            r = 100
            g = 100
            b = 100
            robot.set_led(r,g,b)       
            robot.send_msg(msg)
            robot.delay(1000)
        
        else:           
            msgs = robot.recv_msg()      
            if len(msgs) > 0:
                for i in range(len(msgs)):
                    #robot.delay(1000)
                    received=struct.unpack('ffff',msgs[i][:16])
                    #print(received)
                    # receive message with positions (for debugging) and gradient values
                    x_pos = received[0]
                    y_pos = received[1]
                    rec_gradient_1 = received[2]
                    rec_gradient_2 = received[3]
                    
                    # if the received gradient 1 value is less than the current value
                    if rec_gradient_1 < gradient_1:
                        # update this robot's gradient 1 value
                        gradient_1 = rec_gradient_1

                    # if the received gradient 2 value is less than the current value
                    if rec_gradient_2 < gradient_2:
                        # update the robot's gradient 2 value
                        gradient_2 = rec_gradient_2
                    
                    if smoothing == False:
                        # Wait until the robot receives a hopcount value from each of the seed robots to start calculating position vals
                        if (gradient_1 < 100) and (gradient_2 < 100):
                            
                            # Relatively small search space, go through all possible (x,y) coordinate values
                            for x in range(16):
                                for y in range(16):
                                    
                                    #calculate distance from each seed robot by searching x and y vals
                                    # d_j_i = sqrt( (x_i - x_j)^2 + (y_i - y_j)^2 )
                                    d_j_1 = ((0.0 - x)**2 + (0.0 - y)**2)**0.5
                                    d_j_2 = ((0.0 - x)**2 + (15.0 - y)**2)**0.5

                                    # calculate distance error, hopcount values are best possible estimation for distance from each seed robot
                                    calculated_position_error = ((d_j_1 - gradient_1)**2) + ((d_j_2 - gradient_2)**2)
                                    
                                    # minimize the position error calculation
                                    if calculated_position_error < position_error:
                                        # if the calculated error is less than the current error, update value
                                        position_error = calculated_position_error
                                        # remember the the corresponding (x,y)
                                        x_coord = x
                                        y_coord = y
                        
                        # Break the space up into blocks and have various on/off rules for making the N... simplest way to do it
                        if (x_coord < 4) or (x_coord >= 11):
                            r = 100
                            g = 100
                            b = 100
                    
                        elif (x_coord >= 4) and (x_coord < 9) and (y_coord < 12) and (y_coord > 7):
                            r = 100
                            b = 100
                            g = 100

                        elif (x_coord >= 8) and (x_coord < 11) and (y_coord <= 8) and (y_coord > 4):
                            r = 100
                            b = 100
                            g = 100

                        elif (x_coord >= 5) and (x_coord < 10) and (y_coord <= 11) and (y_coord >= 6):
                            r = 100
                            b = 100
                            g = 100

                        elif (x_coord > 8) and (x_coord < 11) and (y_coord < 6) and (y_coord > 3):
                            r = 100
                            b = 100
                            g = 100

                        elif (x_coord > 1) and (x_coord < 6) and (y_coord < 12) and (y_coord > 9):
                            r = 100
                            b = 100
                            g = 100
                        
                        elif (x_coord >= 6) and (x_coord <= 11) and (y_coord <= 5):
                            r = 0
                            b = 0
                            g = 0
                        
                        else:
                            r = 0
                            b = 0
                            g = 0
                    
                    # only do this if the smoothing effect is turned on
                    if smoothing == True:
                        # Wait until the robot receives a hopcount value from each of the seed robots to start calculating position vals
                        if (gradient_1 < 30) and (gradient_2 < 30) and (rec_gradient_1 < 100) and (rec_gradient_2 < 100):
                            # This is the smoothing step, take the average of every received gradient into a buffer
                            gradient_1_buffer[buffer_index] = rec_gradient_1
                            gradient_2_buffer[buffer_index] = rec_gradient_2

                            # move buffer to next position
                            if buffer_index < 5:
                                buffer_index += 1
                            else:
                                # reset the position if it will exceed index values
                                buffer_index = 0
                            
                            avg_1 = 0
                            avg_2 = 0
                            for i in range(len(gradient_1_buffer)):
                                avg_1 += gradient_1_buffer[i]
                                avg_2 += gradient_2_buffer[i]
                            #print(gradient_1_buffer)

                            s_1 = ((avg_1 + gradient_1)/(len(gradient_1_buffer)+1))-0.4
                            #print(s_1)
                            s_2 = ((avg_2 + gradient_2)/(len(gradient_2_buffer)+1))-0.4

                            # gradient_1 = ((gradient_1 + rec_gradient_1)/2)-0.5
                            # gradient_2 = ((gradient_2 + rec_gradient_2)/2)-0.5
                        # Relatively small search space, go through all possible (x,y) coordinate values
                        for x in range(16):
                            for y in range(16):
                                
                                #calculate distance from each seed robot using searching x and y vals
                                # d_j_i = sqrt( (x_i - x_j)^2 + (y_i - y_j)^2 )
                                d_j_1 = ((0.0 - x)**2 + (0.0 - y)**2)**0.5
                                d_j_2 = ((0.0 - x)**2 + (15.0 - y)**2)**0.5

                                # calculate distance error, hopcount values are best possible estimation for distance from each seed robot
                                calculated_position_error = ((d_j_1 - s_1)**2) + ((d_j_2 - s_2)**2)
                                
                                # minimize the position error calculation
                                if calculated_position_error < position_error:
                                    # if the calculated error is less than the current error, update value
                                    position_error = calculated_position_error
                                    # remember the the corresponding (x,y)
                                    x_coord = x
                                    y_coord = y

                        # Break the space up into blocks and have various on/off rules for making the N... simplest way to do it
                        if (x_coord < 5) or ((x_coord > 10)):
                            r = 100
                            g = 100
                            b = 100
                

                        elif (x_coord >= 8) and (x_coord < 11) and (y_coord <= 8) and (y_coord > 4):
                            r = 100
                            b = 100
                            g = 100

                        elif (x_coord > 4) and (x_coord < 8) and (y_coord <= 12) and (y_coord > 8):
                            r = 100
                            b = 100
                            g = 100

                        elif (x_coord >= 6) and (x_coord < 10) and (y_coord <= 10) and (y_coord > 6):
                            r = 100
                            b = 100
                            g = 100

                        elif (x_coord >= 8) and (x_coord < 11) and (y_coord <= 6) and (y_coord > 3):
                            r = 100
                            b = 100
                            g = 100

                        elif (x_coord > 4) and (x_coord < 10) and (y_coord > 12):
                            r = 0
                            b = 0
                            g = 0

                        elif (x_coord >= 3) and (x_coord < 10) and (y_coord < 5):
                            r = 0
                            b = 0
                            g = 0
                        
                        else:
                            r = 0
                            b = 0
                            g = 0


                    robot.set_led(r,g,b)
                    
                    
            # send an incremental value of gradients to the next robot
            send_gradient_1 = gradient_1 + 1
            send_gradient_2 = gradient_2 + 1 

            msg=struct.pack('ffff',x_coord,y_coord,send_gradient_1,send_gradient_2)    
            robot.send_msg(msg)  # send pose x,y in message