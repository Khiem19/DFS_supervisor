import pygame
import random
import math
from enum import Enum
import time

import matplotlib.pyplot as plt
import random

show_animation = True
wScreen = 800
hScreen = 800
numGridX = 8
numGridY = 8
ofset = 0

sector = wScreen / numGridX
radius = sector/5
movePerFrame = sector / 10
error = 2

class DepthFirstSearchPlanner:

    def __init__(self, ox, oy, reso, rr):
        """
        Initialize grid map for Depth-First planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index, parent):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index
            self.parent = parent

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)
            
    def distance(self,node1, node2):
        rangeX = self.calc_grid_position(node1.x,self.minx) - self.calc_grid_position(node2.x,self.minx)
        rangeY = self.calc_grid_position(node1.y,self.miny) - self.calc_grid_position(node2.y,self.miny)
        return math.sqrt(rangeX*rangeX + rangeY*rangeY)
        
    def planning(self, sx, sy, gx, gy):
        """
        Depth First search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        nstart = self.Node(self.calc_xyindex(sx, self.minx),
                           self.calc_xyindex(sy, self.miny), 0.0, -1, None)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                          self.calc_xyindex(gy, self.miny), 0.0, -1, None)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart
        
        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            current = open_set.pop(list(open_set.keys())[-1])
            c_id = self.calc_grid_index(current)

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.minx),
                         self.calc_grid_position(current.y, self.miny), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event:
                                             [exit(0) if event.key == 'escape'
                                              else None])
                plt.pause(0.01)

            if current.x == ngoal.x and current.y == ngoal.y:
                print("Find goal")
                print("///////////////\n")
                ngoal.parent_index = current.parent_index
                ngoal.cost = current.cost
                break

            # expand_grid search grid based on motion model
            tmp = list()
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id, None)
                
                distance = self.distance(node,ngoal)
                tmp.append((distance,node))

            sortedList = sorted(tmp, key=lambda pair: pair[0], reverse = True)

            for (_,node) in sortedList:
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id not in closed_set:
                    open_set[n_id] = node
                    closed_set[n_id] = node
                    node.parent = current

        rx, ry = self.calc_final_path(ngoal, closed_set)
        return rx, ry

    def calc_final_path(self, ngoal, closedset):
        # generate final course
        rx, ry = [self.calc_grid_position(ngoal.x, self.minx)], [
            self.calc_grid_position(ngoal.y, self.miny)]
        n = closedset[ngoal.parent_index]
        while n is not None:
            rx.append(self.calc_grid_position(n.x, self.minx))
            ry.append(self.calc_grid_position(n.y, self.miny))
            n = n.parent

        return rx, ry

    def calc_grid_position(self, index, minp):
        """
        calc grid position

        :param index:
        :param minp:
        :return:
        """
        pos = index * self.reso + minp
        return pos

    def calc_xyindex(self, position, min_pos):
        return round((position - min_pos) / self.reso)

    def calc_grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.minx)
        py = self.calc_grid_position(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        if self.obmap[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))
        print("min_x:", self.minx)
        print("min_y:", self.miny)
        print("max_x:", self.maxx)
        print("max_y:", self.maxy)

        self.xwidth = round((self.maxx - self.minx) / self.reso)
        self.ywidth = round((self.maxy - self.miny) / self.reso)
        print("x_width:", self.xwidth)
        print("y_width:", self.ywidth)
        print("//////////////////////\n")

        # obstacle map generation
        self.obmap = [[False for _ in range(self.ywidth)]
                      for _ in range(self.xwidth)]
        for ix in range(self.xwidth):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(self.ywidth):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obmap[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1]]

        return motion

class State(Enum):
    STOP = 1
    ASK = 2
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

    def orientation(self):
        (gx,gy) = self.path[self.index]
        (cx,cy) = self.path[self.index-1]
        distance = math.sqrt((gx-cx)*(gx-cx)+(gy-cy)*(gy-cy))
        vx = ((gx-cx)*movePerFrame)/distance
        vy = ((gy-cy)*movePerFrame)/distance
        return (vx,vy)
    
    def move(self):
        if (self.state == State.MOVE):
            (vx,vy) = self.orientation()
            print(vx,vy)
            if (distance(self.locX, self.locY, self.locX + vx, self.locY + vy) < error):
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
        if self.index + 1 < len(self.path):
            return self.path[self.index + 1]
        else:
            return None  # or some other appropriate value

    def draw(self,screen):
        pygame.draw.circle(screen, self.color, (int(self.locX), int(self.locY)), self.radius)

    def acceptMove(self):
        (gNodex,gNodey) = self.path[len(self.path)-1]
        if (distance(self.locX, self.locY, gNodex * sector, gNodey * sector) < error):
            self.state = State.STOP

        if self.index + 1 < len(self.path):
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
                if(requestedNode is not None and requestedNode not in occupiedNode):
                    robot.acceptMove()

            
def generate_random_rgb():
    red = random.randint(0, 255)
    green = random.randint(0, 255)
    blue = random.randint(0, 255)
    return (red, green, blue)


def generatePaths(robotsTask):

    ox = [10]  # Initialize obstacle x-coordinates
    oy = [10]  # Initialize obstacle y-coordinates

    common_planner = DepthFirstSearchPlanner(ox, oy, movePerFrame, radius)
    initial_plans = []

    for i, robot in enumerate(robotsTask):
        start_x, start_y = robot["start"]
        job_x, job_y = robot["job"]
        goal_x, goal_y = robot["goal"]

        rx1, ry1 = common_planner.planning(start_x, start_y, job_x, job_y)
        rx1.reverse()
        ry1.reverse()
        rx2, ry2 = common_planner.planning(job_x, job_y, goal_x, goal_y)
        rx2.reverse()
        ry2.reverse()
        rx = rx1 + rx2[1:]
        ry = ry1 + ry2[1:]

        for j in range(i):
            path1_x, path1_y = initial_plans[j]
            for k, (x1, y1) in enumerate(zip(rx, ry)):
                for l, (x2, y2) in enumerate(zip(path1_x, path1_y)):
                    if x1 == x2 and y1 == y2:
                        ox.extend([x1, x1 + movePerFrame, x1 - movePerFrame, x1, x1])
                        oy.extend([y1, y1, y1, y1 + movePerFrame, y1 - movePerFrame])
                        common_planner.calc_obstacle_map(ox, oy)

        initial_plans.append((rx, ry))

    # Construct the paths in the desired format
    robotsPath = []
    for path in initial_plans:
        robotPath = [(x, y) for x, y in zip(path[0], path[1])]
        robotsPath.append(robotPath)

    return robotsPath


def main():
    run = True
    clock = pygame.time.Clock()
    
    # Setting up robots
    robotsTask = [
        {"start": (10.0, 10.0), "job": (10.0, 40.0), "goal": (50.0, 40.0)},
        {"start": (0.0, 20.0), "job": (40.0, 20.0), "goal": (40.0, 10.0)},
        {"start": (0.0, 20.0), "job": (40.0, 20.0), "goal": (40.0, 10.0)},
    ]

    robots = list()
    for path in generatePaths(robotsTask):
         robots.append(Robot(path,radius,generate_random_rgb()))
    
    supervisor = Supervisor(robots)

    screen = pygame.display.set_mode((wScreen, hScreen))


    while run:
        time.sleep(0.3)
        screen.fill((255,255,255))
        for i in range(numGridX):
            pygame.draw.line(screen, (255,0,0), (i*int(wScreen/numGridX)+ofset,0), (i*int(wScreen/numGridX)+ofset,hScreen)) 

        for i in range(numGridY):
            pygame.draw.line(screen, (255,0,0), (0,i*int(hScreen/numGridY)+ofset), (wScreen, i*int(hScreen/numGridY)+ofset)) 

        supervisor.checkAskingAction()

        for robot in robots:
            robot.move()
            robot.draw(screen)

        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            
main()