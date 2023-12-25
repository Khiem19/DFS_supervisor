import math
import time
import matplotlib.pyplot as plt
import random
import tracemalloc

show_animation = True


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

def generate_random_position():
    return (random.uniform(-40, 40), random.uniform(-40, 40))

def generate_robot_info(num_robots):
    robot_info = []
    for _ in range(num_robots):
        robot = {
            "start": generate_random_position(),
            "job": generate_random_position(),
            "goal": generate_random_position()
        }
        robot_info.append(robot)
    return robot_info

def main():
    print(__file__ + " start!!")

    num_robots = 3
    robot_info = generate_robot_info(num_robots)

    grid_size = 1.0  # [m]
    robot_radius = 1.0  # [m]

    ox, oy = [], []
    for i in range(-50, 51):
        ox.append(i)
        oy.append(-50.0)
    for i in range(-50, 51):
        ox.append(50.0)
        oy.append(i)
    for i in range(-50, 51):
        ox.append(i)
        oy.append(50.0)
    for i in range(-50, 51):
        ox.append(-50.0)
        oy.append(i)

    # for i in range(10, 40):
    #     ox.append(30.0)
    #     oy.append(i)    

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        for i, robot in enumerate(robot_info):
            start_x, start_y = robot["start"]
            job_x, job_y = robot["job"]
            goal_x, goal_y = robot["goal"]
            plt.plot(start_x, start_y, "og", label=f"Robot {i + 1} Start")
            plt.plot(job_x, job_y, "xr", label=f"Robot {i + 1} Job")
            plt.plot(goal_x, goal_y, "xb", label=f"Robot {i + 1} Goal")
        plt.grid(True)
        plt.axis("equal")

    # Initialize a single planner for all robots
    common_planner = DepthFirstSearchPlanner(ox, oy, grid_size, robot_radius)
    initial_plans = []

    for i, robot in enumerate(robot_info):
        # loop_start_time = time.time()
        start_x, start_y = robot["start"]
        job_x, job_y = robot["job"]
        goal_x, goal_y = robot["goal"]
        
        # Plan path from start to job
        rx1, ry1 = common_planner.planning(start_x, start_y, job_x, job_y)
        rx1.reverse()
        ry1.reverse()
        # Plan path from job to goal
        rx2, ry2 = common_planner.planning(job_x, job_y, goal_x, goal_y)
        rx2.reverse()
        ry2.reverse()
        # Combine the paths (excluding the duplicate job point)
        rx = rx1 + rx2[1:]  # Exclude the first point of rx2 to avoid duplication
        ry = ry1 + ry2[1:]  # Exclude the first point of ry2

        # Update obstacles with crossing points from the current robot's path
        for j in range(i):
            path1_x, path1_y = initial_plans[j]
            for k, (x1, y1) in enumerate(zip(rx, ry)):
                for l, (x2, y2) in enumerate(zip(path1_x, path1_y)):
                    if x1 == x2 and y1 == y2:
                        # Add original and surrounding points to obstacles
                        ox.extend([x1, x1 + grid_size, x1 - grid_size, x1, x1])
                        oy.extend([y1, y1, y1, y1 + grid_size, y1 - grid_size])
                        # Update the common obstacle map in the planner
                        common_planner.calc_obstacle_map(ox, oy)
        initial_plans.append((rx, ry))

    # Use plt.show() to keep the window open
    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        for i, (rx, ry) in enumerate(initial_plans):
            plt.plot(rx, ry, label=f"Robot {i + 1} Path")
        plt.legend()
        plt.pause(0.01)
        plt.show()


if __name__ == '__main__':
    main()
