package org.usfirst.frc.team5254.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive myRobot = new RobotDrive(0, 1);
	Joystick stick = new Joystick(0);
	Timer timer = new Timer();
	Encoder encoder = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		timer.reset();
		timer.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
		Waypoint[] points = new Waypoint[] {
				new Waypoint(-4, -1, Pathfinder.d2r(-45)),
				new Waypoint(-2, -2, 0),
				new Waypoint(0, 0, 0)
		};
		
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
		Trajectory trajectory = Pathfinder.generate(points, config);
		TankModifier modifier = new TankModifier(trajectory).modify(0.5);

		EncoderFollower left = new EncoderFollower(modifier.getLeftTrajectory());
		EncoderFollower right = new EncoderFollower(modifier.getRightTrajectory());
		
		left.configureEncoder(encoder.get(), 256, 4);
		left.configurePIDVA(1.0, 0.0, 0.0, 1 / 1, 0);
		
		double output = left.calculate(encoder.get());
		
		double l = left.calculate(encoder.get());
		double r = right.calculate(encoder.get());
		
		double gro = 0.0; // "insert gyro code here"
		double desired_heading = Pathfinder.r2d(left.getHeading());
		
		double angle_diffenece = Pathfinder.boundHalfDegrees(desired_heading - gro);
		double turn = 0.8 * (-1.0/80.0) * angle_diffenece;
		
		// setLeftMotors(l + turn);
		// setRightMotors(r + turn);
		
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		myRobot.arcadeDrive(stick);
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
