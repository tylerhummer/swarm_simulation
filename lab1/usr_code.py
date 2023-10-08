def usr(robot):
    import struct
    desired_distance = .3 #will vary from 0.3-0.5
    
    while True:
        if (robot.id == 0):
            # if we received a message, print out info in message
            msgs = robot.recv_msg()
            if len(msgs) > 0:
                pose_rxed = struct.unpack('ffi', msgs[0][:12])
                print('robot ', robot.id, ' received position ', pose_rxed[0], pose_rxed[1], ' from robot ', pose_rxed[2])

                #blink led if message is received  
                robot.set_led(100,0,0)
                robot.delay(10)
                robot.set_led(0,0,0)

            pose_t = robot.get_pose()
            # if there is a new postion sensor update, print out and transmit the info
            if pose_t:  # check pose is valid before using
                print('The x,y postion of robot ', robot.id, ' is ', pose_t[0], pose_t[1])
                robot.send_msg(struct.pack('ffi', pose_t[0], pose_t[1], robot.id))  # send pose x,y in message
         
         
        if (robot.id == 1):
            # if we received a message, print out info in message
            msgs = robot.recv_msg()
            if len(msgs) > 0:
                pose_rxed = struct.unpack('ffi', msgs[0][:12])
                print('robot ', robot.id, ' received position ', pose_rxed[0], pose_rxed[1], ' from robot ', pose_rxed[2])

                #blink led if message is received  
                robot.set_led(0,100,0)
                robot.delay(10)
                robot.set_led(0,0,0)

            pose_t = robot.get_pose()
            # if there is a new postion sensor update, print out and transmit the info
            if pose_t:  # check pose is valid before using
                print('The x,y postion of robot ', robot.id, ' is ', pose_t[0], pose_t[1])
                robot.send_msg(struct.pack('ffi', pose_t[0], pose_t[1], robot.id))  # send pose x,y in message

            #move wheels
            robot.set_vel(10,10)