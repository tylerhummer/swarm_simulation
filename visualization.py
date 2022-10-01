# import pygame
import socket, sys, traceback
import pickle, time, json

from sympy import true

class bot_sim:
    def __init__(self, id, usr_led,clk,delay=0):
        self.id = id
        self.usr_led = usr_led
        self.pos_x = 3
        self.pos_y = 3
        self.clk = clk
        self.delay = delay

class visualization:

    def __init__(self):
        
        (width, height) = (1500, 1000)
        # self.screen = pygame.display.set_mode((width, height))
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((socket.gethostname(), 1245))
        self.set_vis_id()

        # pygame.display.flip()
    
    def set_vis_id(self):
        data = '0b101'
        self.client_socket.send(data.encode())
        print("Connected")
        data = self.client_socket.recv(1024)
        if data.decode() != str(bin(2)):
            print("Error in connecting to simulator server")
    
    # def update(self, robot_state, num_of_robot):
    #     for i in range(num_of_robot):
    #         robo = robot_state[i]
    #         # print(robo.usr_led)
    #         colour = robo.usr_led #green
    #         circle_x_y = (i*2, i*2)
    #         circle_radius = 8
    #         border_width = 1 #0 = filled circle
    #         pygame.draw.circle(self.screen, colour, circle_x_y, circle_radius, border_width)
    #     pygame.display.flip()

    def loop(self):

        while True:
            print("Waiting for client to receive")
            msg = self.client_socket.recv(4096)
            # msg = json.loads(msg)
            # print(msg)  
            data_send = '0b11'
            self.client_socket.send(data_send.encode())
            # gn = self.client_socket.recv(1024)
            print('loop')

def main():
    try:
        vis = visualization()
        vis.loop()
    except KeyboardInterrupt:
        print("Shutdown requested...exiting")
    except Exception:
        traceback.print_exc(file=sys.stdout)
    sys.exit(0)

if __name__ == "__main__":
    main()