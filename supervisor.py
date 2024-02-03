import pygame
import random
import math
from enum import Enum
import time

import DFS as planner

wScreen = 800
hScreen = 800

numGridX = 100
numGridY = 100
ofset = 0

sector = wScreen / numGridX
radius = sector
movePerFrame = sector
error = 2


class State(Enum):
    STOP = 1
    ASK = 2 #WAITING
    MOVE = 3


def distance(x1,y1, x2,y2):
    return math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

class Robot(object):
    def __init__(self,path,radius, color):
        self.radius = radius
        self.color = color
        self.path = path
        self.index = 1 #index allways represent for the point that the robots want to reach
        self.state = State.ASK

        (sNodex,sNodey) = self.path[0]
        self.locX = sNodex*sector #real location of the robot on the map
        self.locY = sNodey*sector

    def orienration(self):
        (gx,gy) = self.path[self.index]
        (cx,cy) = self.path[self.index-1]
        distance = math.sqrt((gx-cx)*(gx-cx)+(gy-cy)*(gy-cy))
        vx = ((gx-cx)*movePerFrame)/distance
        vy = ((gy-cy)*movePerFrame)/distance
        return (vx,vy)
    


    def move(self):
        if (self.state == State.MOVE):
            (vx,vy) = self.orienration()
            print(vx,vy)
            if (distance(self.locX, self.locY,self.locX + vx, self.locY+ vy) < error):
                self.state = State.ASK
            
            (gNodex,gNodey) = self.path[len(self.path)-1]
            if (distance(self.locX, self.locY,gNodex*sector, gNodey*sector) < error): # the case robot has reached the gooal
                 self.state = State.STOP
            else:
                self.locX = self.locX + vx
                self.locY = self.locY + vy

            

    def getState(self):
        return self.state
    def requestState(self):
        return self.path[self.index+1]
    def draw(self,screen):
        pygame.draw.circle(screen,self.color,(self.locX,self.locX),self.radius)



    def acceptMove(self):
        (gNodex,gNodey) = self.path[len(self.path)-1]
        if (distance(self.locX, self.locY,gNodex*sector, gNodey*sector) < error): # the case robot has reached the gooal
           self.State = State.STOP
        else:
            self.index = self.index + 1
            self.state = State.MOVE

    def currentNode(self):
        return self.path[self.index]


class Supervisor(object):
    def __init__(self,listOfRobots):
        self.listOfRobots = listOfRobots
        

    def checkAskingAction(self):
        occupiedNode = list() 
        for robot in self.listOfRobots:
            occupiedNode.append(robot.currentNode())
            print("Current node")
            print(occupiedNode)

        for robot in self.listOfRobots:
            if( robot.getState() == State.ASK):
                requestedNode = robot.requestState()
                if(requestedNode not in occupiedNode):
                    robot.acceptMove()
            
def generate_random_rgb():
    red = random.randint(0, 255)
    green = random.randint(0, 255)
    blue = random.randint(0, 255)
    return (red, green, blue)


def generatePaths(robotsTask, ox, oy):
    grid_size = 1  # Adjust as needed
    robot_radius = 1  # Adjust as needed
    dfs = planner.DepthFirstSearchPlanner(ox, oy, grid_size, robot_radius)
    
    paths = []
    # Use enumerate if you need the index for each robot
    for i, robot in enumerate(robotsTask):  # Added enumerate here
        start_x, start_y = robot["start"]
        goal_x, goal_y = robot["goal"]
        rx, ry = dfs.planning(start_x, start_y, goal_x, goal_y, i+1)
        paths.append(list(zip(rx, ry)))
    
    return paths



def main():
    run = True
    clock = pygame.time.Clock()
    
    num_robots = 2
    robotsTask = planner.generate_robot_info(num_robots)

    #obstacle
    ox, oy = [], []
    for i in range(0, 101):
        ox.append(i)
        oy.append(0.0)
    for i in range(0, 101):
        ox.append(100.0)
        oy.append(i)
    for i in range(0, 101):
        ox.append(i)
        oy.append(100.0)
    for i in range(0, 101):
        ox.append(0.0)
        oy.append(i)
        
    # #Obstacle    
    for i in range(60, 81):
        ox.append(i)
        oy.append(60.0)
    for i in range(20, 61):
        ox.append(80.0)
        oy.append(i)
    for i in range(20, 61):
        ox.append(60)
        oy.append(i)
    for i in range(60, 81):
        ox.append(i)
        oy.append(20)

    robots = list()
    for path in generatePaths(robotsTask,ox,oy):
         robots.append(Robot(path, radius, generate_random_rgb()))
    
    supervisor = Supervisor(robots)

    screen = pygame.display.set_mode((wScreen, hScreen))

    while run:
        time.sleep(0.3)  # Controls the speed of the simulation
        screen.fill((255, 255, 255))  # Fills the screen with white to clear old frames

        supervisor.checkAskingAction()

        for robot in robots:
            robot.move()
            robot.draw(screen)

        pygame.display.update()  # Updates the screen with what we've drawn
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

main()
