import pygame
import random
import math
from enum import Enum
import time

import DFS as planner

wScreen = 1000
hScreen = 1000

numGridX = 100
numGridY = 100
ofset = 0

sector = wScreen / numGridX
radius = sector * 0.8
movePerFrame = sector
error = 2


class State(Enum):
    STOP = 1
    ASK = 2 #WAITING
    MOVE = 3


def distance(x1,y1, x2,y2):
    return math.sqrt((x1-x2) **2 + (y1-y2) ** 2)

class Robot(object):
    def __init__(self, path_info, radius, color):
        self.radius = radius
        self.color = color
        self.path = path_info['path']  # Extract the path list from the path_info dictionary
        self.index = 0 # Index always represents the point that the robots want to reach
        self.state = State.ASK

        (sNodex, sNodey) = self.path[0]  # Now this accesses the first coordinate pair in the path list
        self.locX = sNodex * sector  # Real location of the robot on the map
        self.locY = sNodey * sector


    def orientation(self):
        if self.index < len(self.path) - 1:
            (gx, gy) = self.path[self.index + 1]
            (cx, cy) = self.path[self.index]
            distance = math.sqrt((gx - cx) ** 2 + (gy - cy) ** 2)
            vx = ((gx - cx) * movePerFrame) / distance
            vy = ((gy - cy) * movePerFrame) / distance
            return (vx, vy)
        else:
            return (0, 0)
    
    def move(self):
        if self.state == State.MOVE:
            (vx, vy) = self.orientation()
            if distance(self.locX, self.locY, self.locX + vx, self.locY + vy) < error:
                self.state = State.ASK

            (gNodex, gNodey) = self.path[-1]
            if distance(self.locX, self.locY, gNodex * sector, gNodey * sector) < error:
                self.state = State.STOP
            else:
                self.locX += vx
                self.locY += vy

    def getState(self):
        return self.state
    
    def requestState(self):
        if self.index < len(self.path) - 1:
            return self.path[self.index + 1]
        return self.path[self.index]
    
    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.locX), int(self.locY)), int(self.radius))

    def acceptMove(self):
        (gNodex, gNodey) = self.path[-1]
        if distance(self.locX, self.locY, gNodex * sector, gNodey * sector) < error:
            self.state = State.STOP
        else:
            self.index += 1
            self.state = State.MOVE

    def currentNode(self):
        return self.path[self.index]


class Supervisor(object):
    def __init__(self,listOfRobots):
        self.listOfRobots = listOfRobots
        

    def checkAskingAction(self):
        occupiedNode = []
        for robot in self.listOfRobots:
            if robot.getState() != State.STOP:  # Only consider non-stopped robots
                occupiedNode.append(robot.currentNode())

        for robot in self.listOfRobots:
            if robot.getState() == State.ASK:
                requestedNode = robot.requestState()
                if requestedNode not in occupiedNode:
                    robot.acceptMove()
                    occupiedNode.append(requestedNode)  # Update occupied nodes

def generate_random_rgb():
    return random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)

def generatePaths(robotsTask, ox, oy):
    grid_size = 1
    robot_radius = 1
    dfs = planner.DepthFirstSearchPlanner(ox, oy, grid_size, robot_radius)

    paths = []
    for i, robot in enumerate(robotsTask):
        start_x, start_y = robot["start"]
        goal_x, goal_y = robot["goal"]

        rx, ry = dfs.planning(start_x, start_y, goal_x, goal_y, i + 1)

        path_pairs = list(zip(rx, ry))[::-1]
        paths.append({'path': path_pairs, 'robot_id': i + 1})

        print(f"Robot {i + 1} Path: {path_pairs}")

    return paths

def main():
    pygame.init()
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

    robots = [Robot(path, radius, generate_random_rgb()) for path in generatePaths(robotsTask, ox, oy)]

    supervisor = Supervisor(robots)

    screen = pygame.display.set_mode((wScreen, hScreen))

    # Inside your main loop
    while run:
        time.sleep(0.1)  # Controls simulation speed
        screen.fill((255, 255, 255))  # Clear screen

        # Draw obstacles
        for x, y in zip(ox, oy):
            pygame.draw.rect(screen, (0, 0, 0), [x * sector, y * sector, sector, sector], 0)

        supervisor.checkAskingAction()

        # Move and draw each robot
        for robot in robots:
            robot.move()
            robot.draw(screen)

        pygame.display.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False


    pygame.quit()

main()
