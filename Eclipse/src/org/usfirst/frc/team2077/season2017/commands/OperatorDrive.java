
package org.usfirst.frc.team2077.season2017.commands;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.*;

import org.usfirst.frc.team2077.season2017.math.*;
import org.usfirst.frc.team2077.season2017.robot.OperatorInterface;
import org.usfirst.frc.team2077.season2017.robot.Robot;

/*
units

wheel speed
    distance units
        robot dimension units
    rotation units
        degrees/second
        radians/second
        encoder counts / time unit (100ms?)
    fraction of full speed

robot translation speed
    distance units
        robot dimension units
    fraction of full speed

robot rotation speed
    distance units
        robot dimension units
    rotation units
        degrees/second
        radians/second
    fraction of full speed
*/

public class OperatorDrive extends Command {
	public static int toggle;
	public static boolean fieldRelative;
	
	public OperatorDrive() {
		requires(Robot.drive_);
		toggle = 1;
	}

	@Override
	protected void initialize() {
	}

	@Override
	protected void execute() {

		// Controls are for a Logitech Extreme 3D or similar joystick.

//		// Scale the x, y, and twist input based on the setting of the throttle control.
//		double scaleMin = 0.2;
//		double scale = Robot.operatorInterface_.joystick_.getThrottle() / -2 * (1 - scaleMin) + scaleMin + (1 - scaleMin) / 2;
		double scale = 1.0;

		// Apply deadband, exponential curve, and scaling to the raw control inputs
		CompassVector translation = new CompassVector(adjustInputSensitivity(-Robot.operatorInterface_.getNorthSouth(), .2, 2.5, scale),
		                                              adjustInputSensitivity( Robot.operatorInterface_.getEastWest(), .2, 2.5, scale));
		double rotation = adjustInputSensitivity(Robot.operatorInterface_.getRotation(), .5, 2.5, scale);
		rotation /= 1.75;

		// Optional field-relative translation
		double heading = fieldRelative ? Robot.gyro_.getAngle() : 0;
		translation = translation.rotate(-heading);

		// Set acceleration/deceleration limit using the throttle control. Should be at or below the CoF of the wheels on the floor.
		//double gMin = 0.1;
		//double gMax = 1.0;
		double gLimit = 0.6; //Robot.operatorInterface_.joystick_.getThrottle() / -2 * (gMax - gMin) + gMin + (gMax - gMin) / 2;
		Robot.drive_.setAcceleration(gLimit);

		Robot.drive_.setRelativeVelocity(toggle*translation.north_, toggle*translation.east_, rotation);
	}

	/**
	 * Condition control axis input to improve driveability. Each axis has a center dead band in which the output for
	 * that axis is always zero. Outside the dead band the output increases exponentially from zero to 1*scale or -1*scale.
	 * @param input
	 * @param deadBand
	 * @param exponent
	 * @param scale
	 * @return
	 */
	protected double adjustInputSensitivity(double input, double deadBand, double exponent, double scale) {
		return Math.pow(Math.max(0, Math.abs(input) - deadBand) / (1 - deadBand), exponent) * (input > 0 ? scale : -scale);
	}
	
	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {
		Robot.drive_.halt();
	}

	@Override
	protected void interrupted() {
		Robot.drive_.halt();
	}
}
