def usr(robot):

    while True:

        robot.delay()
        id = robot.id

        if id == 3:
            robot.set_led(0,100,0)
            robot.send_msg("Set LED to (0,100,0)")
            robot.delay(1000)
            robot.set_led(100,0,0)
            robot.send_msg("Set LED to (100,0,0)")
            robot.delay(2000)
        elif id == 10:
            robot.set_led(100,100,0)
            robot.send_msg("Set LED to (100,100,0)")
            robot.delay(2000)
            robot.set_led(0,60,50)
            robot.send_msg("Set LED to (0,60,50)")
            robot.delay(2000)

        else:
            val = robot.recv_msg(clear=True)
            # print(val)
            if val[0] == "0Set LED to (100,100,0)":
                robot.set_led(100,100,0)
                robot.delay()
            elif val[0] == "0Set LED to (0,100,0)":
                robot.set_led(0,100,0)
                robot.delay()
            elif val[0] == "0Set LED to (100,0,0)":
                robot.set_led(100,0,0)
                robot.delay()
            elif val[0] == "0Set LED to (0,60,50)":
                robot.set_led(0,60,50)
                robot.delay()
            else:
                continue
        
