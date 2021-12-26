'''
The wpilib autonomous stuff works with their swerve setup so I had to switch normal driving to that too.
Using their functions makes it clean and optimized, and I highly recommend switching to it using this code and 
https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot as examples
'''

import wpilib
import wpimath
import math
import ctre
import json
import navx
import os
from ctre._ctre import AbsoluteSensorRange, SensorInitializationStrategy
import wpilib.controller
import swerveModule

class driveTrain:
	def __init__(self, config: dict):
		self.maxSpeed = config["RobotDimensions"]["maxSpeed"]
		self.maxAngularVelocity = config["RobotDimensions"]["maxAngularVelocity"]
		self.fieldOrient = bool(config["RobotDefaultSettings"]["fieldOrient"])
		
		self.navx = navx.AHRS.create_spi()
		
		inchesToMeters = .0254
		halfWidth = .5 * config["RobotDimensions"]["trackWidth"] * inchesToMeters
		halfLength = .5 * config["RobotDimensions"]["wheelBase"] * inchesToMeters
		
		frontLeftLocation = wpimath.geometry.Translation2d(halfWidth, halfLength)
		frontRightLocation = wpimath.geometry.Translation2d(halfWidth, -1*halfLength)
		rearLeftLocation = wpimath.geometry.Translation2d(-1*halfWidth, halfLength)
		rearRightLocation = wpimath.geometry.Translation2d(-1*halfWidth, -1*halfLength)
		
		fLConfig = config["SwerveModules"]["frontLeft"]
		fRConfig = config["SwerveModules"]["frontRight"]
		rLConfig = config["SwerveModules"]["rearLeft"]
		rRConfig = config["SwerveModules"]["rearRight"]
		
		self.frontLeft = swerveModule.swerveModule(fLConfig["motor_ID_1"], fLConfig["motor_ID_2"], fLConfig["encoder_ID"], fLConfig["encoderOffset"], "frontLeft")
		self.frontRight = swerveModule.swerveModule(fRConfig["motor_ID_1"], fRConfig["motor_ID_2"], fRConfig["encoder_ID"], fRConfig["encoderOffset"], "frontRight")
		self.rearLeft = swerveModule.swerveModule(rLConfig["motor_ID_1"], rLConfig["motor_ID_2"], rLConfig["encoder_ID"], rLConfig["encoderOffset"], "rearLeft")
		self.rearRight = swerveModule.swerveModule(rRConfig["motor_ID_1"], rRConfig["motor_ID_2"], rRConfig["encoder_ID"], rRConfig["encoderOffset"], "rearRight")
		self.frontLeft.initMotorEncoder() # Why not do this at the end of the swerveModule init?
		self.frontRight.initMotorEncoder()
		self.rearLeft.initMotorEncoder()
		self.rearRight.initMotorEncoder()
		
		self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(frontLeftLocation, frontRightLocation, rearLeftLocation, rearRightLocation)
		self.odometry = wpimath.kinematics.SwerveDrive4Odometry(self.kinematics, self.getPoseRadians())
		
		self.navx.reset()
	
	def getPoseRadians(self):
		''' Gets the robot's heading, translates to radians, and translates to a 2D rotation (the preferred kinematics input) '''
		degreesToRadians = math.pi/180
		return wpimath.geometry.Rotation2d((-1*self.navx.getAngle()+90)*degreesToRadians)
	
	def resetNavx(self):
		self.navx.reset()
	
	'''
	xSpeed: Speed of the robot in the x direction (forward).
	ySpeed: Speed of the robot in the y direction (sideways).
	rot: Angular rate of the robot.
	'''
	def drive(self, xSpeed: float, ySpeed: float, rot: float):
		if self.fieldOrient:
			swerveModuleStates = self.kinematics.toSwerveModuleStates(wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, self.getPoseRadians()))
		else:
			swerveModuleStates = self.kinematics.toSwerveModuleStates(wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot))
		swerveModuleStates = wpimath.kinematics.SwerveDrive4Kinematics.normalizeWheelSpeeds(swerveModuleStates, self.maxSpeed)
		self.frontLeft.setState(swerveModuleStates[0])
		self.frontRight.setState(swerveModuleStates[1])
		self.rearLeft.setState(swerveModuleStates[2])
		self.rearRight.setState(swerveModuleStates[3])
	
	def autoDrive(self, chassisSpeeds):
		''' This could be consolidated with drive() pretty easily.'''
		swerveModuleStates = self.kinematics.toSwerveModuleStates(chassisSpeeds)
		swerveModuleStates = wpimath.kinematics.SwerveDrive4Kinematics.normalizeWheelSpeeds(swerveModuleStates, self.maxSpeed)
		
		self.frontLeft.setState(swerveModuleStates[0])
		self.frontRight.setState(swerveModuleStates[1])
		self.rearLeft.setState(swerveModuleStates[2])
		self.rearRight.setState(swerveModuleStates[3])
	
	def updateOdometry(self):
		self.odometry.update(self.getPoseRadians(), self.frontLeft.getState(), self.frontRight.getState(), self.rearLeft.getState(), self.rearRight.getState())
	
	def brake(self):
		self.frontLeft.setNeutralMode(ctre.NeutralMode.Brake)
		self.frontRight.setNeutralMode(ctre.NeutralMode.Brake)
		self.rearLeft.setNeutralMode(ctre.NeutralMode.Brake)
		self.rearRight.setNeutralMode(ctre.NeutralMode.Brake)
	
	def coast(self):
		self.frontLeft.setNeutralMode(ctre.NeutralMode.Coast)
		self.frontRight.setNeutralMode(ctre.NeutralMode.Coast)
		self.rearLeft.setNeutralMode(ctre.NeutralMode.Coast)
		self.rearRight.setNeutralMode(ctre.NeutralMode.Coast)