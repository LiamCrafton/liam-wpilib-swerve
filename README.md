Note: This is untested.

Example code adapting Joe's work (https://github.com/jserra99/rapid-react-robot-2022) using WPILIB swerve kinematics and odometry.
Before running any complex PID stuff, make sure you carefully follow the characterization in Phoenix Tuner (for wheel velocity) and from WPILIB (for other controllers)...and have plenty of open space when you finally touch the ground.

Little things: I added abs() to the deadzone check and fixed the DriverStation issue.

I left full dependencies on everything to be explicit (and due to poor planning). Between that, random constants not in the dictionary, and keeping auto in robot.py, the example is messy but should get the ideas across.
