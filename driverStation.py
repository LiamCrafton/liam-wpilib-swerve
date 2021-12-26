import wpilib

class driverStation:
	def __init__(self, config: dict):
		self.driverStationUtil = wpilib.DriverStation.getInstance() # this doesnt work please fix --Fixed!
		''' 
		From old docs: The single DriverStation instance is created statically with the instance 
		static member variable, you should never create a DriverStation instance. 
		'''
		self.config = config
		self.checkDriverStationInputs()
		
	
	def checkDriverStationInputs(self):
		driverInputName = self.driverStationUtil.getJoystickName(0) # Driverstation input is on port 0
		auxiliaryInputName = self.driverStationUtil.getJoystickName(1) # Auxiliary input is on port 1
		if driverInputName == "Logitech Extreme 3D": # This string is correct given we are using this joystick
			self.driverInputType = "Joystick"
			self.driverInput = wpilib.Joystick(0)
		elif driverInputName == "Controller (Xbox One For Windows)": # This string may be wrong and needs to be updated once I can see what the correct output is.
			self.driverInputType = "XboxController"
			self.driverInput = wpilib.XboxController(0)
		else:
			self.driverStationUtil.reportWarning("Driver input is either unplugged or not set to USB0", True)
			self.driverInputType = "disconnected"
		if auxiliaryInputName != "Controller (Xbox One For Windows)":
			self.driverStationUtil.reportWarning("Auxiliary input is either unplugged or not set to USB1", True)
			'''
			The if statement is built into reportWarning, so you could add True like above or consolidate to
			self.driverStationUtil.reportWarning([message], auxiliaryInputName != "Controller (Xbox One For Windows)")
			'''
		self.auxiliaryInput = wpilib.XboxController(1)
	
	def checkSwitches(self):
		switchDict = {
			"driverX": 0.0,
			"driverY": 0.0,
			"driverZ": 0.0,
			"swapFieldOrient": False,
			"playEasterEgg": False,
			"fieldOrient": wpilib.SmartDashboard.getBoolean("fieldOrient", True),
			#"navxAngle": -1*navx.getAngle() + 90, # I moved navx around a bit, not happy with how I did it but you can fix it :)
			"resetDriveTrainEncoders": False
		}
		if self.driverInputType != "disconnected":
			if self.driverInputType == "XboxController":
				switchDict["driverX"] = self.driverInput.Axis.kLeftX
				switchDict["driverY"] = self.driverInput.Axis.kLeftY
				switchDict["driverZ"] = self.driverInput.Axis.kRightX # NOTE: Use this for turning
				'''
				NOTE: Alternatively use this which uses the triggers for turning which may seem weird but could be more accurate:
				switchDict["driverZ"] = self.driverInput.Axis.kRightTrigger - self.driverInput.Axis.kLeftTrigger
				
				ALternative alternative! Have a button that, when held, puts the robot into "slow mode" (maybe 50% x and y, 20% z?) for better accuracy
				'''
				
			else: # Joystick
				switchDict["driverX"] = self.driverInput.getX()
				switchDict["driverY"] = self.driverInput.getY()
				switchDict["driverZ"] = self.driverInput.getZ()
			# after this point is auxiliary code
			switchDict["swapFieldOrient"] = self.auxiliaryInput.getStartButtonReleased()
			switchDict["playEasterEgg"] = self.auxiliaryInput.getBButtonReleased()
			switchDict["resetDriveTrainEncoders"] = self.auxiliaryInput.getBackButtonReleased()
		else:
			self.checkDriverStationInputs()
		return switchDict