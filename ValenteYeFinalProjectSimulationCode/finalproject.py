import pyCreate2
import math
import odometry
import pid_controller
import lab8_map
import lab10_map
import lab10_solution as lab10
import particle_filter
import numpy as np
import Hand_Class


class Run:
    def __init__(self, factory):
        """Constructor.
        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        self.virtual_create = factory.create_virtual_create()
        self.odometry = odometry.Odometry()
        self.armClass = Hand_Class.Hand(factory)

        # For Simulation Testing
        self.p_map = lab8_map.Map("final_map.json")
        self.map = lab10_map.Map("final_map_config.png")

        # For physical Testing
        # self.p_map = lab8_map.Map("finalproject_map3.json")
        # self.map = lab10_map.Map("finalproject_map3_config.png")

        self.path = lab10.Run(factory, self.map)
        # TODO identify good PID controller gains
        self.pidTheta = pid_controller.PIDController(200, 0, 100, [-10, 10], [-50, 50], is_angle=True)
        # TODO identify good particle filter parameters
        self.pf = particle_filter.ParticleFilter(self.p_map, 1000, 0.06, 0.15, 0.2)
        self.joint_angles = np.zeros(7)

    def sleep(self, time_in_sec):
        """Sleeps for the specified amount of time while keeping odometry up-to-date
        Args:
            time_in_sec (float): time to sleep in seconds
        """
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            t = self.time.time()
            if start + time_in_sec <= t:
                break

    def go_to_angle(self, goal_theta):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        while math.fabs(math.atan2(
            math.sin(goal_theta - self.odometry.theta),
                math.cos(goal_theta - self.odometry.theta))) > 0.05:
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def forward(self):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        base_speed = 100
        distance = 0.5
        goal_x = self.odometry.x + math.cos(self.odometry.theta) * distance
        goal_y = self.odometry.y + math.sin(self.odometry.theta) * distance
        while True:
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))

            # stop if close enough to goal
            distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            if distance < 0.05:
                self.create.drive_direct(0, 0)
                break
            self.sleep(0.01)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def visualize(self):
        x, y, theta = self.pf.get_estimate()
        self.virtual_create.set_pose((x, y, 0.1), theta)
        data = []
        for particle in self.pf._particles:
            data.extend([particle.x, particle.y, 0.1, particle.theta])
        self.virtual_create.set_point_cloud(data)

    def run(self):
        self.create.start()
        self.create.safe()

        self.create.drive_direct(0, 0)
        print("Drive Direct (0,0) Done...")

        self.time.sleep(2)

        # request sensors
        self.create.start_stream([
            pyCreate2.Sensor.LeftEncoderCounts,
            pyCreate2.Sensor.RightEncoderCounts,
        ])
        print("Request Sensors")

        startXCord, startYCord, startThetaPos = self.pf.get_estimate()
        print("Location (X, Y, Theta):", startXCord, startYCord, startThetaPos)

        self.visualize()
        print("Starting Path.Run")
        self.path.run()
        print("Finished Path.Run")
        print("Starting armClass.putOnShelf")
        print("Might put on shelf 2 or 3 depending on code.")
        self.armClass.putOnShelf3()
        print("Put on shelf completed")
        print("Program finished....ending")

    def localize(self):
        found = False
        count = -1

        print("Start Localization")
        i = 0
        print("Entered while loop")
        while found is False:
            i += 1
            print("In", i, "th iteration of loop------------------------")
            if self.sonar.get_distance() < 0.70:  # GO LEFT
                self.go_to_angle(self.odometry.theta + (math.pi/2))
                print("GO LEFT")
                count = -1
            else:
                if count is 3:
                    curTheta = self.odometry.theta  # get current theta

                    self.go_to_angle(curTheta + 90)  # go 90 CW
                    self.pf.measure(self.sonar.get_distance(), 0)
                    self.visualize()

                    self.go_to_angle(curTheta - 90)  # go 90 CCW
                    self.pf.measure(self.sonar.get_distance(), 0)
                    self.visualize()

                    self.go_to_angle(curTheta)      # go to original location
                    self.pf.measure(self.sonar.get_distance(), 0)
                    self.visualize()
                    count = -1
                count += 1
                self.forward()
            self.servo.go_to(0)
            self.pf.measure(self.sonar.get_distance(), 0)
            self.visualize()
            print("One visualization")
            found = self.threshold()
        print("Out of while loop ")

    def threshold(self):
        xArray = []
        yArray = []
        thetaArray = []

        for p in self.pf._particles:
            xArray.append(p.x)
            yArray.append(p.y)
            thetaArray.append(p.theta)
        xVar = np.var(xArray, dtype=np.float)
        yVar = np.var(yArray, dtype=np.float)
        thetaLoc = np.var(yArray, dtype=np.float)

        if xVar <= 0.05 and yVar <= 0.05 and thetaLoc <= 0.05:
            return True
        else:
            return False
