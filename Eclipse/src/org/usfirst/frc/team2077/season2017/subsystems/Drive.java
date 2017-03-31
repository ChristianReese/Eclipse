package org.usfirst.frc.team2077.season2017.subsystems;

import edu.wpi.first.wpilibj.command.*;

public abstract class Drive extends Subsystem {
	
	private double acceleration_ = Double.POSITIVE_INFINITY;
	
	/**
	 * @return Set point values for {n/s , e/w, rotation} velocity vector.
	 */
	public abstract double[] getVelocity();
	
	/**
	 * Update set point values for {n/s , e/w, rotation} velocity vector,
	 * with adjustments to translation components to effect rotation about a specified point.
	 * @param ns North/south translation velocity component in distance units per second.
	 * @param ew East/west translation velocity component in distance units per second.
	 * @param rotation Rotation speed in units of wheel distance per second about the robot center.
	 * @param rotationCenter Center of rotation relative to the robot center.
	 */
	public abstract void setVelocity(double ns, double ew, double rotation, double[] rotationCenter);
	
	/**
	 * Update set point values for {n/s , e/w, rotation} velocity vector.
	 * @param ns North/south translation velocity component in distance units per second.
	 * @param ew East/west translation velocity component in distance units per second.
	 * @param rotation Rotation speed in units of wheel distance per second about the robot center.
	 */
	public void setVelocity(double ns, double ew, double rotation) {
		setVelocity(ns, ew, rotation, new double[] {0, 0});
	}
	
	/**
	 * Update set points for {n/s, e/w} velocity vector.
	 * Rotation speed/angle set settings are unchanged.
	 * @param ns North/south translation velocity component in distance units per second.
	 * @param ew East/west translation velocity component in distance units per second.
	 */
	public abstract void setTranslationVelocity(double ns, double ew);
	
	/**
	 * Update set point for rotation speed.
	 * This will adjust translation velocity settings iff rotation is about other than the robot center.
	 * @param rotation Rotation speed in units of wheel distance per second about the robot center.
	 * @param rotationCenter Center of rotation relative to the robot center.
	 */
	public abstract void setRotationVelocity(double rotation, double[] rotationCenter);
	
	/**
	 * @param rotation Set point for rotation speed about the robot center.
	 */
	public void setRotationVelocity(double rotation) {
		setRotationVelocity(rotation, new double[] {0, 0});
	}
	
	private double heading_ = 0;
	
	/**
	 * @return Heading set point in degrees from initial heading.
	 */
	public double getHeading() {
		return heading_;
	}
	
	/**
	 * @param heading Set point in degrees from initial heading.
	 */
	public void setHeading(double heading) {
		heading_ = heading;
	}

	/**
	 * @param ns North/south translation velocity component as a fraction of maximum wheel speed.
	 * @param ew East/west translation velocity component as a fraction of maximum wheel speed.
	 * @param rotation Rotation speed as a fraction of maximum wheel speed.
	 * @param rotationCenter Center of rotation relative to the robot center.
	 */
	public abstract void setRelativeVelocity(double ns, double ew, double rotation, double[] rotationCenter);

	/**
	 * @param ns North/south translation velocity component as a fraction of maximum wheel speed.
	 * @param ew East/west translation velocity component as a fraction of maximum wheel speed.
	 * @param rotation Rotation speed as a fraction of maximum wheel speed.
	 */
	public void setRelativeVelocity(double ns, double ew, double rotation) {
		setRelativeVelocity(ns, ew, rotation, new double[] {0, 0});
	}
	
	/**
	 * @param ns North/south translation velocity component as a fraction of maximum wheel speed.
	 * @param ew East/west translation velocity component as a fraction of maximum wheel speed.
	 */
	public abstract void setRelativeTranslationVelocity(double ns, double ew);
		
	/**
	 * @param rotation Rotation speed as a fraction of maximum wheel speed.
	 * @param rotationCenter Center of rotation relative to the robot center.
	 */
	public abstract void setRelativeRotationVelocity(double rotation, double[] rotationCenter);
	
	/**
	 * @param rotation Rotation speed as a fraction of maximum wheel speed.
	 */
	public void setRelativeRotationVelocity(double rotation) {
		setRelativeRotationVelocity(rotation, new double[] {0, 0});
	}
		
	/**
	 * Set all wheels to zero speed immediately.
	 */
	public void halt() {
		setVelocity(0, 0, 0);
	}
	
	/**
	 * @return Software acceleration limit as a fraction of G.
	 */
	public double getAcceleration() {
		return acceleration_;
	}

	/**
	 * @param acceleration Software acceleration limit as a fraction of G.
	 */
	public void setAcceleration(double acceleration) {
		acceleration_ = acceleration;
	}
	
	/**
	 * Get the "odometer" reading.
	 * @return 
	 */
	public abstract double[] getDistanceFromZero();
	
	/**
	 * Zero the "odometer".
	 * @return 
	 */
	public abstract void zeroDistance();
	
	/**
	 * Distance required to smoothly stop by proportionally slowing all wheels to zero.
	 * @return Translation and rotation from current position at which which all wheels would be stopped.
	 */
	public abstract double[] getDistanceToStop();
	
	/**
	 * Convert distance units traveled by a wheel contact point along a circle about
	 * the robot center to degrees of rotation.
	 * @param distance
	 * @return
	 */
	public abstract double robotRotationDistanceToDegrees(double distance);
	
	/**
	 * Convert degrees of rotation to distance units traveled by a wheel contact
	 * point along a circle about the robot center.
	 * @param degrees
	 * @return
	 */
	public abstract double robotRotationDegreesToDistance(double degrees);
	
	/**
	 * Re-set the PID values for heading adjustment (to use for tuning without having to reset the robot).
	 * @param p P-value
	 * @param i I-value
	 * @param d D-value
	 * @param f F-value
	 */
	public abstract void setHeadingPIDValues( double p, double i, double d, double f );
}
