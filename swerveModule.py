'''
I used the Talon's built-in velocity control for simplicity and greater reliability than calculating & setting a current.
Configuration information can be found at:
	https://docs.ctre-phoenix.com/en/stable/ch13_MC.html
	https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#sensor-preparation
and this example may be helpful:
	https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/Java%20Talon%20FX%20(Falcon%20500)/VelocityClosedLoop/src/main/java/frc/robot
'''

import wpilib
import wpimath
import math
import ctre
import json
import os
import navx
from ctre._ctre import AbsoluteSensorRange, SensorInitializationStrategy
import wpilib.controller

class swerveModule:
	def __init__(self, driveID: int, turnID: int, absoluteID: int, absoluteOffset: float, moduleName: str):
		self.wheelDiameter = .1016 # Meters
		self.CPR = 2048
		
		maxAngularVelocity = math.pi # Radians per second
		maxAngularAcceleration = 2*math.pi # Radians per second squared
		
		kPTurn = .1
		kITurn = 0
		kDTurn = 0.5
		kPDrive = 1
		kIDrive = 0
		kDDrive = 0
		kFDrive = .108
		
		self.CPR = 2048
		self.turningGearRatio = 12.8 # The steering motor gear ratio
		self.drivingGearRatio = 8.14 # The driving motor gear ratio
		self.moduleName = moduleName
		self.absoluteOffset = absoluteOffset
		
		self.driveMotor = ctre.TalonFX(driveID)
		self.turnMotor = ctre.TalonFX(turnID)
		
		self.driveMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, 0)
		self.turnMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, 0)
		
		self.driveMotor.config_kP(0, kPDrive, 0)
		self.driveMotor.config_kI(0, kIDrive, 0)
		self.driveMotor.config_kD(0, kDDrive, 0)
		self.driveMotor.config_kF(0, kFDrive, 0)
		
		self.absoluteEncoder = ctre.CANCoder(absoluteID)
		self.absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition)
		self.absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
		self.absoluteEncoder.configMagnetOffset(self.absoluteOffset)
		
		''' Set up to control the turn motor directly by current, but the old positional PID works so you could just use that instead.'''
		self.turnController = wpilib.controller.ProfiledPIDController(kPTurn, kITurn, kDTurn,
			wpimath.trajectory.TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration))
		self.turnController.setTolerance(0.005) # Radians, approximately .3 degrees
		self.turnFeedForward = wpimath.controller.SimpleMotorFeedforward(1, 0.5) # You will need to do drive train characterization to get the actual kS and kV values
		
		self.turnController.enableContinuousInput(-math.pi, math.pi)
	
	def wheelPosition(self):
		''' Returns wheel position in radians from -pi to pi.'''
		wheelPositionRadians = ((self.turnMotor.getSelectedSensorPosition(0) % (self.CPR*self.turningGearRatio)) * 2 * math.pi/(self.CPR*self.turningGearRatio))
		if wheelPositionRadians > math.pi:
			wheelPositionRadians -= 2*math.pi
		return wheelPositionRadians
	
	def getState(self):
		''' I believe the Talon works in '''
		speedMetersPerSecond = self.driveMotor.getSelectedSensorVelocity(0) * 10 * self.wheelDiameter * math.pi /self.CPR # Talon works with velocity in ticks/100ms
		return wpimath.kinematics.SwerveModuleState(speedMetersPerSecond, wpimath.geometry.Rotation2d(self.wheelPosition())) 
	
	def setState(self, desiredState):
		state = wpimath.kinematics.SwerveModuleState.optimize(desiredState, wpimath.geometry.Rotation2d(self.wheelPosition())) # Update with actual encoder code
		
		velocity = state.speed * self.CPR / (10 * self.wheelDiameter * math.pi) # Converting from m/s to ticks/100ms
		turnOutput = self.turnController.calculate(self.wheelPosition(), state.angle.radians())
		turnFeedForward = self.turnFeedForward.calculate(self.turnController.getSetpoint().velocity)
		
		self.driveMotor.set(ctre.TalonFXControlMode.Velocity, velocity)
		#I'm not sure how to scale these units so I'll leave (careful) testing of that to you. Or you could use the direct position PID
		self.turnMotor.set(ctre.TalonFXControlMode.Current, turnOutput + turnFeedForward)
	
	def initMotorEncoder(self):
		''' Called to set the turn encoder zero based off of absolute offset and position.'''
		self.turnMotor.setSelectedSensorPosition(self.absoluteEncoder.getAbsolutePosition() * self.CPR * self.turningGearRatio / 360)
	
	def setNeutralMode(self, mode):
		self.driveMotor.setNeutralMode(mode)
		self.turnMotor.setNeutralMode(mode)