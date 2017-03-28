package org.usfirst.frc.team2077.season2017.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	
//	public static double P = .15;
//	public static double I = .0005;
//	public static double F = .12;

	public static final String ADVANCE_LENGTH_MULTIPLIER_KEY = "Auto-Targeting Advance Length Multiplier";
	public static final String ROTATION_MULTIPLIER_KEY = "Auto-Targeting Rotate Multiplier";
	public static final String PLANE_TO_PEG_DISTANCE_KEY = "Plane-to-Peg distance";
	public static final String SLOW_CLIMBER_TIME_KEY = "Slow Climber Time";
	
	public static final double ADVANCE_LENGTH_MULTIPLIER_DEFAULT = 1.0;
	public static final double ROTATION_MULTIPLIER_DEFAULT = 0.5;
	public static final double PLANE_TO_PEG_DISTANCE_DEFAULT = 8.0;
	public static final double SLOW_CLIMBER_TIME_DEFAULT = 0.5;
	
}
