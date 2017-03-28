package org.usfirst.frc.team2077.season2017.subsystems;

import org.usfirst.frc.team2077.season2017.robot.AngleSensor;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.PIDController;

public interface MecanumHardware {
	
	/** @return Track width between wheel center lines, in inches. */
	double getTrackWidth();

	/** @return Wheel base between front and rear axles, in inches. */
	double getWheelBase();

	/** @return Wheel radius, in inches. */
	double getWheelRadius();

	/** @return Nominal top speed, in inches per second. */
	double getMaximumSpeed();

	/** @return Encoder counts per revolution. */
	int getEncoderCountsPerRevolution();

	/**
	 * Direction of wheel rotation relative to motor drive voltage.
	 * Positive values indicate forward with positive motor voltage,
	 * negative values indicate the opposite.
	 * @return Array of size 4, in the order NE, SE, SW, NW.
	 */
	int[] getWheelDirections();

	/**
	 * Motor controllers.
	 * @return Array of size 4, in the order NE, SE, SW, NW.
	 */
	CANTalon[] getMotorControllers();
	
	AngleSensor getAngleSensor();

	PIDController getHeadingController();
}
