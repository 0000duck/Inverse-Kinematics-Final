from pyCreate2 import create2
import math
import odometry
import pid_controller
import rrt
from rrt import Vertex
import datetime
import random


class Run:
    def __init__(self, factory, map):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.map = map
        self.rrt = rrt.RRT(self.map)

    def run(self):
        # find a path
        self.rrt.build((250, 200), 3000, 10)
        x_goal = self.rrt.nearest_neighbor((85, 140))
        path = self.rrt.shortest_path(x_goal)
        self.clean(path)

        for p in path:
            print(p.state)

        currentDT = datetime.datetime.now()
        print("Start RRT Map Construction")
        for v in self.rrt.T:
            for u in v.neighbors:
                self.map.draw_line((v.state[0], v.state[1]), (u.state[0], u.state[1]), (0,0,0))
        for idx in range(0, len(path)-1):
            self.map.draw_line((path[idx].state[0], path[idx].state[1]), (path[idx+1].state[0], path[idx+1].state[1]), (0,255,0))

        fileName = "pathFiles/" + str(currentDT.hour) + ":" + str(currentDT.minute) + ":" + str(currentDT.second)
        fileName = fileName + ".png"
        self.map.save(fileName)
        print("RRT Path Map Is Made....Just Follow Path to goal")

        # execute the path (essentially waypoint following from lab 6)
        self.create.start()
        self.create.safe()

        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        self.odometry.x = 2.5
        self.odometry.y = 1.33
        self.odometry.theta = math.pi / 2
        base_speed = 100

        i = 0
        for p in path:
            # print("Path node that we are working on: NODE", i, ".....", p.state)
            i += 1
            goal_x = p.state[0] / 100.0
            goal_y = 3.35 - p.state[1] / 100.0
            print(i, ": ", p.state)
            while True:
                state = self.create.update()
                if state is not None:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                    self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))
                    distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    if distance < 0.05:
                        break

    def shh(self):
        name = random.randint(1, 6)
        print(name)
        path = []
        fileNum = str(name)
        fileName = "pathFiles/" + fileNum + ".txt"
        f = open(fileName, "r")
        anything = f.readlines()
        for temp in anything:
            i = 0
            tempStr = ["", ""]
            num = [-1, -1]
            spaceCheck = 0
            for item in temp:
                spaceCheck = spaceCheck + 1
                if i < 2:
                    if item is " " and spaceCheck > 2:
                        i = i + 1
                    # elif isinstance(item, int) or (item == "."):
                    elif item is "." or item.isdigit():
                            tempStr[i] = tempStr[i] + item
            if self.is_number(tempStr[0]) and self.is_number(tempStr[1]):
                num[0] = float(tempStr[0])
                num[1] = float(tempStr[1])
            if num[0] is not -1:
                # path.insert(0, Vertex((num[0], num[1])))
                path.append(Vertex((num[0], num[1])))
        return path
    def clean(self, path):
        path.append(Vertex((84.08841666, 136.08841666)))
        path.append(Vertex((50.08841666, 136.05183729)))
        path.append(Vertex((45.05183729, 136.08841666)))
        return path
