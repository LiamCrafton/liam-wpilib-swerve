'''
I'm sure you can come up with a much better implementation of full auto than I could,
so rather than creating an auto file and creating/following trajectories in an organized
way, I'll make a basic trajectory directly in robotInit and follow it in autonomousPeriodic.
It's ugly but should get the method across.
For reading about the WPILIB trajectory stuff, see:
https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/trajectory-tutorial-overview.html and
https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html

It may also be good to look into adding a slew rate limiter to the joystick
https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
'''

import wpilib
import wpimath
import math
import json
import os
import navx
import driveTrain
import driverStation
from time import strftime, gmtime
import autonomous

class MyRobot(wpilib.TimedRobot):
	
	def robotInit(self):
		'''
		This function is called upon program startup and
		should be used for any initialization code.
		'''
		with open (f"{os.path.dirname(os.path.abspath(__file__))}/config.json", "r") as f1:
			self.config = json.load(f1)
		
		self.driveTrain = driveTrain.driveTrain(self.config)
		self.driverStation = driverStation.driverStation(self.config)
		self.autonomousMode = "smart" # Alternatively "dumb"
		trajectoryConfig = wpimath.trajectory.TrajectoryConfig(self.config["RobotDimensions"]["maxSpeed"], self.config["RobotDimensions"]["maxAcceleration"]) # In meters
		trajectoryConfig.setKinematics(self.driveTrain.kinematics)
		
		self.trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(
			wpimath.geometry.Pose2d(0, 0, wpimath.geometry.Rotation2d.fromDegrees(0)), # Starting position
			[wpimath.geometry.Translation2d(1,1), wpimath.geometry.Translation2d(2,-1)], # Pass through these points
			wpimath.geometry.Pose2d(3, 0, wpimath.geometry.Rotation2d.fromDegrees(0)), # Ending position
			trajectoryConfig)
		
		xController = wpilib.controller.PIDController(1, 0, 0)
		yController = wpilib.controller.PIDController(1, 0, 0)
		angleController = wpilib.controller.ProfiledPIDControllerRadians(1, 0, 0, wpimath.trajectory.TrapezoidProfileRadians.Constraints(math.pi, math.pi))
		angleController.enableContinuousInput(-1*math.pi, math.pi)
		self.swerveController = wpilib.controller.HolonomicDriveController(xController, yController, angleController)
	
	def autonomousInit(self):
		''' This function is run once each time the robot enters autonomous mode.'''
		self.driveTrain.brake()
		if self.autonomousMode == "dumb":
			pass # I know it's not anywhere near that dumb, but this is bare-bones :P
		elif self.autonomousMode == "smart":
			autoPlanName = wpilib.SmartDashboard.getString("Auto Plan", "default")
		self.Timer = wpilib.Timer()
	
	def autonomousPeriodic(self):
		''' This function is called periodically during autonomous.'''
		if self.autonomousMode == "dumb":
			pass # Actual recording code here
		elif self.autonomousMode == "smart":
			self.driveTrain.updateOdometry()
			if self.Timer.get() < self.config["matchSettings"]["autonomousTime"]:
				goal = self.trajectory.sample(self.Timer.get())
				adjustedSpeeds = self.swerveController.calculate(self.driveTrain.odometry.getPose(), goal, wpimath.geometry.Rotation2d.fromDegrees(90))
				'''
				I'm not totally sure how the angle/heading stuff interacts, but it would be easy to read about and test.
				The docs say "Because the heading dynamics are decoupled from translations, users can specify a custom heading
								that the drivetrain should point toward. This heading reference is profiled for smoothness."
				'''
				self.driveTrain.autoDrive(adjustedSpeeds)
			else:
				self.driveTrain.drive(0, 0, 0)
	
	def teleopInit(self):
		'''
		This function is run once each time the robot exits 
		autonomous mode to enter teleoperated mode.'''
		self.driveTrain.resetNavx() # Comment out for competition
		self.driveTrain.brake() # Comment out for competition
	
	def teleopPeriodic(self):
		''' This function is called periodically during operator control.'''
		switches = self.driverStation.checkSwitches()
		switches["driverX"], switches["driverY"], switches["driverZ"] = self.evaluateDeadzones(switches["driverX"], switches["driverY"], switches["driverZ"])
		self.switchActions(switches)
	
	def disabledInit(self):
		''' This function is run once each time the robot is disabled.'''
		self.driveTrain.coast()
	
	def switchActions(self, switchDict: dict):
		''' Acts on and calls commands based on inputs from multiple robot modes.'''
		xSpeed = -1*switchDict["driverX"]*self.config["RobotDimensions"]["maxSpeed"]
		ySpeed = -1*switchDict["driverY"]*self.config["RobotDimensions"]["maxSpeed"]
		rot = -1*switchDict["driverZ"]*self.config["RobotDimensions"]["maxAngularVelocity"]
		self.driveTrain.drive(xSpeed, ySpeed, rot)
		
		if switchDict["swapFieldOrient"]:
			self.driveTrain.fieldOrient = not self.driveTrain.fieldOrient # swaps field orient to its opposite value
			wpilib.SmartDashboard.putBoolean("Field Orient", self.driveTrain.fieldOrient)
	
	def evaluateDeadzones(self, x: float, y: float, z: float):
		if not (abs(x) > self.config["driverStation"]["joystickDeadZones"]["xDeadZone"]):
			x = 0
		if not (abs(y) > self.config["driverStation"]["joystickDeadZones"]["yDeadZone"]):
			y = 0
		if not (abs(z) > self.config["driverStation"]["joystickDeadZones"]["zDeadZone"]):
			z = 0
		return x, y, z
	
	def stopAll(self):
		''' Stops all functions of the robot.'''
		# Force all drive train motors to a hard 0
	
if __name__ == "__main__":
	wpilib.run(MyRobot)