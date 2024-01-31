import math
import matplotlib.pyplot as plt
import random

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
        
    def planning(self, sx, sy, gx, gy, robot_number):
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
                print(f"Robot {robot_number}: Find goal")
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
    while True:
        x = random.uniform(10, 90)
        y = random.uniform(10, 90)

        # Check if the point is outside the obstacle
        if not (55 <= x <= 85 and 15 <= y <= 65):
            return (x, y)


def generate_robot_info(num_robots):
    robot_info = []
    for _ in range(num_robots):
        robot = {
            "start": generate_random_position(),
            "goal": generate_random_position()
        }
        robot_info.append(robot)
    return robot_info

def main():
    colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']  # Add more colors if you have more than 7 robots

    print(__file__ + " start!!")

    # start and goal position
    num_robots = 10
    robot_info = generate_robot_info(num_robots)
    
    grid_size = 1.0  # [m]
    robot_radius = 1.0  # [m]

    # set obstacle positions
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
    

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        for i, robot in enumerate(robot_info):
            start_x, start_y = robot["start"]
            goal_x, goal_y = robot["goal"]
            plt.plot(start_x, start_y, "og", label=f"Robot {i + 1} Start")
            plt.plot(goal_x, goal_y, "xb", label=f"Robot {i + 1} Goal")
        plt.grid(True)
        plt.axis("equal")

    dfs = DepthFirstSearchPlanner(ox, oy, grid_size, robot_radius)
    initial_plans = []
    
    # rx, ry = dfs.planning(sx, sy, gx, gy)
    for i, robot in enumerate(robot_info):
        start_x, start_y = robot["start"]
        goal_x, goal_y = robot["goal"]
        
        rx, ry = dfs.planning(start_x, start_y, goal_x, goal_y, i+1)
        
        
        path_pairs = list(zip(rx, ry))[::-1]
        initial_plans.append({'path': path_pairs, 'robot_id': i+1})

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        for plan in initial_plans:
            robot_id = plan['robot_id']
            path_pairs = plan['path']
            color = colors[robot_id % len(colors)]
            
            # Unzip the path pairs for plotting
            rx, ry = zip(*path_pairs)
            
            plt.plot(rx, ry, color=color, linewidth=2, label=f"Robot {robot_id} Path")
            
            # Print path pairs
            print(f"Robot {robot_id} path: {path_pairs}\n")
            
        plt.legend()
        plt.pause(0.01)
        plt.show()


if __name__ == '__main__':
    main()