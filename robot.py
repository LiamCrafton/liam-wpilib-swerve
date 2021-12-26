'''
Comments
'''

import wpilib
import json
import os
import navx
import driveTrain
import driverStation
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
	
	def autonomousInit(self):
		''' This function is run once each time the robot enters autonomous mode.'''
		self.driveTrain.brake()
		if self.autonomousMode == "dumb":
			pass # I know it's not nearly that dumb, but this is bare-bones :P
		elif self.autonomousMode == "smart":
			autoPlanName = wpilib.SmartDashboard.getString("Auto Plan", "default")
			# Auto implementation here
	
	def autonomousPeriodic(self):
		''' This function is called periodically during autonomous.'''
		if self.autonomousMode == "dumb":
			pass # Actual recording code here
		elif self.autonomousMode == "smart":
			pass # Auto implementation here
	
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