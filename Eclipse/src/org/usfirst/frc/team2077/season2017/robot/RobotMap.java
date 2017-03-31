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
	
	public static enum RobotPlatform {
		RP_ECLIPSE, RP_PIZZA
	}
	
//	public static double P = .15;
//	public static double I = .0005;
//	public static double F = .12;
	
	public static final RobotPlatform ROBOT_PLATFORM = RobotPlatform.RP_PIZZA;//RP_ECLIPSE;

	public static final String ADVANCE_LENGTH_MULTIPLIER_KEY = "Auto-Targeting Advance Length Multiplier";
	public static final String ROTATION_MULTIPLIER_KEY = "Auto-Targeting Rotate Multiplier";
	public static final String PLANE_TO_PEG_DISTANCE_KEY = "Plane-to-Peg distance";
	public static final String SLOW_CLIMBER_TIME_KEY = "Slow Climber Time";
	
	public static final double ADVANCE_LENGTH_MULTIPLIER_DEFAULT = 1.0;
	public static final double ROTATION_MULTIPLIER_DEFAULT = 0.5;
	public static final double PLANE_TO_PEG_DISTANCE_DEFAULT = 8.0;
	public static final double SLOW_CLIMBER_TIME_DEFAULT = 0.5;

	public static final String HEADING_P_KEY = "Heading PID - P";
	public static final String HEADING_I_KEY = "Heading PID - I";
	public static final String HEADING_D_KEY = "Heading PID - D";
	public static final String HEADING_F_KEY = "Heading PID - F";
	
	public static final double HEADING_DEFAULT_P = 0.01;
	public static final double HEADING_DEFAULT_I = 0.0;
	public static final double HEADING_DEFAULT_D = 0.001;
	public static final double HEADING_DEFAULT_F = 0.0;

	public static final String T2C_P_KEY = "T2C PID - P";
	public static final String T2C_I_KEY = "T2C PID - I";
	public static final String T2C_D_KEY = "T2C PID - D";
	public static final String T2C_F_KEY = "T2C PID - F";

	public static final double T2C_DEFAULT_P = 0.0;
	public static final double T2C_DEFAULT_I = 0.0;
	public static final double T2C_DEFAULT_D = 0.0;
	public static final double T2C_DEFAULT_F = 0.0;

	public static final String DIST_P_KEY = "Distance PID - P";
	public static final String DIST_I_KEY = "Distance PID - I";
	public static final String DIST_D_KEY = "Distance PID - D";
	public static final String DIST_F_KEY = "Distance PID - F";

	public static final double DIST_DEFAULT_P = 0.0;
	public static final double DIST_DEFAULT_I = 0.0;
	public static final double DIST_DEFAULT_D = 0.0;
	public static final double DIST_DEFAULT_F = 0.0;
	
}
