package org.usfirst.frc.team2077.season2017.subsystems;

import org.usfirst.frc.team2077.season2017.commands.OperatorDrive;
import org.usfirst.frc.team2077.season2017.math.MecanumMath;
import org.usfirst.frc.team2077.season2017.robot.AngleSensor;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 *
 * @author 2077
 */
public class MecanumDrive extends Drive {
	
	private static final double G_IN_INCHES_PER_SECOND_2 = 386.1; // acceleration of gravity in inches/second/second for conversion
    
	private final MecanumMath math_;
	private final double rotationSpeedFactor_;
    private final double encoderVelocityG_; // G in encoder velocity units
    private final double wheelSpeedMax_;
    private final double encoderVelocityMax_; // Maximum wheel speed in encoder velocity units

    private final CANTalon[] motors_;
	private final int[] directions_;
	private final AngleSensor angleSensor_;
    private final PIDController headingPID_;
	
	private final NetworkTable networkTable_;
	
	public MecanumDrive(MecanumHardware hardware) {

		double wheelSpeedFactor = hardware.getEncoderCountsPerRevolution() / 10 / (2 * Math.PI); // controller speed units are clicks/100ms, convert to radians/second
		double robotSpeedFactor = 1; // will work everywhere with length units (e.g, inches) per second, so no conversion factor
		double trackWidth = hardware.getTrackWidth();
		double wheelBase = hardware.getWheelBase();
		double wheelRadius = hardware.getWheelRadius();
		double trackCircumference = Math.PI * Math.sqrt(wheelBase*wheelBase + trackWidth*trackWidth);
		rotationSpeedFactor_ = trackCircumference / (2 * Math.PI); // length units per second of wheel contact points about the center of rotation, convert to radians/second
		math_ = new MecanumMath(wheelBase, trackWidth, wheelRadius, wheelSpeedFactor, robotSpeedFactor, rotationSpeedFactor_);
		wheelSpeedMax_ = hardware.getMaximumSpeed();
		encoderVelocityMax_ = wheelSpeedMax_ / (2*Math.PI*wheelRadius) * (hardware.getEncoderCountsPerRevolution()/10);
		encoderVelocityG_ = G_IN_INCHES_PER_SECOND_2 * encoderVelocityMax_ / wheelSpeedMax_; // assumes wheelSpeedMax is in inches/second
		directions_ = hardware.getWheelDirections();
		motors_ = hardware.getMotorControllers();
		angleSensor_ = hardware.getAngleSensor();
		headingPID_ = hardware.getHeadingController();

		System.out.println("Maximum wheel speed in encoder velocity units:" + encoderVelocityMax_);
		System.out.println("G in encoder velocity units per second:" + encoderVelocityG_);
		
		networkTable_ = NetworkTable.getTable("MecanumDrive");
	}


	@Override
	public double[] getVelocity() {
		return math_.forward(getWheelSpeedsSet());
	}
	
	private double[] getWheelSpeedsSet() {
		double[] w = {w_[0], w_[1], w_[2], w_[3]};
		return signCorrectWheels(w);
	}

	private double[] getWheelSpeedsPID() {
		double[] w = {motors_[0].get(), motors_[1].get(), motors_[2].get(), motors_[3].get()};
		return signCorrectWheels(w);
	}

	private double[] getWheelSpeedsMeasured() {
		double[] w = {motors_[0].getEncVelocity(), motors_[1].getEncVelocity(), motors_[2].getEncVelocity(), motors_[3].getEncVelocity()};
		return signCorrectWheels(w);
	}
	
	private double[] signCorrectWheels(double[] w) {
		double[] ws = {
			w[0] * Math.signum(directions_[0]), w[1] * Math.signum(directions_[1]),
			w[2] * Math.signum(directions_[2]), w[3] * Math.signum(directions_[3])
		};
		return ws;
	}

	/**
	 * Rescale wheel speeds to fit within a specified range. This method is intended to ensure that the full range of
	 * possible wheel speeds is used for any combination of N/S and E/W translation and rotation, as opposed to the
	 * "pure" mecanum math, which maintains(?) the same actual robot speed range regardless of direction.
	 * @param w Wheel speeds.
	 * @param wMax Maximum wheel speed as computed by pure mecanum math..
	 * @param input Current input value.
	 * @param inputMax Maximum input value.
	 * @return Adjusted wheel speeds.
	 */
	private double[] normalizeWheelSpeeds(double[] w, double wMax, double input, double inputMax) {

		double maxAbsW = 0;
		for (int i = 0; i < 4; i++) {
			maxAbsW = Math.max(Math.abs(w[i]), maxAbsW);
		}
		double[] n = new double[4];
		for (int i = 0; i < 4; i++) {
			n[i] = maxAbsW > 0 ? wMax * (w[i] / maxAbsW) * (input / inputMax) : 0;
		}
		return n;
	}
	
	private double[] w_ = {0, 0, 0, 0};
	
	private double time_ = Timer.getFPGATimestamp();
	
	private void printWheelSpeeds() {
		
		// last set
		double[] s = getWheelSpeedsSet();
		for (int i = 0; i < 4; i++) {
			System.out.print(" " + s[i]);
		}
		System.out.print("    \t");
		// computed by PID
		double[] p = getWheelSpeedsPID();
		for (int i = 0; i < 4; i++) {
			System.out.print(" " + p[i]);
		}
		System.out.print("    \t");
		// measured by encoders
		double[] a = getWheelSpeedsMeasured();
		for (int i = 0; i < 4; i++) {
			System.out.print(" " + a[i]);
		}
		System.out.println();
	}

	// portion of last iterations rotation input that should be applied to heading set point
	private double lastRotation_;

	// index for telemetry
	private static int i_ = 0;
	
	/**
	 * Compute wheel speeds and apply them to the motor controllers.
	 * Merges caller input velocities with rotation adjustments from the heading PID.
	 * If any wheel speeds or accelerations exceed limits, scale them all down proportionally until they don't.
	 * @param ns North/south translation velocity component in inches per second.
	 * @param ew East/west translation velocity component in inches per second.
	 * @param rotation Rotation speed in units of wheel inches per second about the robot center.
	 * @param rotationCenter Center of rotation relative to the robot center.
	 */
	@Override
	public void setVelocity(double ns, double ew, double rotation, double[] rotationCenter) {

		// make sure the heading PID is running
		// TODO: move elsewhere?
		if (!headingPID_.isEnabled()) {
			setHeading(angleSensor_.gyro_.getAngle());
			headingPID_.enable();
		}

		
		double time = Timer.getFPGATimestamp();
		double timeDelta = time - time_;
		time_ = time;

		//printWheelSpeeds();

		// apply any pending heading set point affecting rotation component since last iteration
		setHeading(getHeading() + robotRotationDistanceToDegrees(timeDelta * lastRotation_));

		
		
		
		
		// adjust velocity components for non-center rotation
		double[] v = math_.forward(math_.inverse(new double[] {ns, ew, rotation}, rotationCenter));
		ns = v[0];
		ew = v[1];
		double callerRotation = v[2];
		double pidRotation = wheelSpeedMax_ * headingPID_.get();
		double totalRotation = callerRotation + pidRotation;
		
		
		//System.out.println("\tCR:" + callerRotation + "\tPR:" + pidRotation + "\tHEADING:" + getHeading() + "\tGYRO:" + angleSensor_.gyro_.getAngle());

		// post data for external display/logging
		networkTable_.putNumberArray("Mecanum Velocity", new double[] {i_++, ns, ew, callerRotation}); // TODO: add pidRotation
		
		// target wheel speeds prior to any limiting
		double[] w = math_.inverse(new double[] {ns, ew, totalRotation});
		
		double wMaxDelta =  timeDelta * getAcceleration() * encoderVelocityG_;
		
		double wMax = encoderVelocityMax_;

		// scale to limit maximum wheel speed
		double maxAbsW = 0;
		for (int i = 0; i < 4; i++) {
			maxAbsW = Math.max(Math.abs(w[i]), maxAbsW);
		}
		double scale = Math.max(1,  maxAbsW / wMax);
		for (int i = 0; i < 4; i++) {
			w[i] /= scale;
		}
		
		// scale to limit maximum wheel acceleration
		double[] lastW = getWheelSpeedsSet();
		double[] deltaW = {w[0]-lastW[0],  w[1]-lastW[1],  w[2]-lastW[2],  w[3]-lastW[3]};
		double maxAbsDeltaW = 0;
		for (int i = 0; i < 4; i++) {
			maxAbsDeltaW = Math.max(Math.abs(deltaW[i]), maxAbsDeltaW);
		}
		double scaleDelta = Math.max(1,  maxAbsDeltaW / wMaxDelta);
		for (int i = 0; i < 4; i++) {
			w[i] = lastW[i] + (deltaW[i] / scaleDelta);
		}
		
		// final velocity set points, after scaling to limit wheel speed/acceleration
		v = math_.forward(w);
		
		// sign correction for motor direction
		w = signCorrectWheels(w);
		
		// apply wheel speeds
		for (int i = 0; i < 4; i++) {
			motors_[i].set(w_[i] = w[i]);
		}

		
		
		// figure out how these updates affect the heading set point
		double totalRotationScaled = v[2];
		double callerRotationScaled;
		double pidRotationScaled;
		// decide how much of the scaled rotation is attributed to caller input and how much to PID correction
		// PID correction is based on the current heading set point and doesn't change it
		// caller rotation, in addition to being directly applied, also alters the heading set point
		// this might be doable via feed-forward in a PIDF arrangement, but that's out of scope for now
		if (Math.signum(totalRotationScaled - callerRotation) == Math.signum(pidRotation)) {
			// all caller rotation can be applied, with some left for PID
			callerRotationScaled = callerRotation;
			pidRotationScaled = totalRotationScaled - callerRotation;
		}
		else {
			// caller gets it all, PID will have to wait
			callerRotationScaled = totalRotationScaled;
			pidRotationScaled = 0;
		}
		// stash the heading affecting rotation component for application over next iteration's time delta
		lastRotation_ = callerRotationScaled;
	}
	
	@Override
	public void setRelativeVelocity(double ns, double ew, double rotation, double[] rotationCenter) {
		
		ns *= wheelSpeedMax_;
		ew *= wheelSpeedMax_;
		rotation *= wheelSpeedMax_;
		
		double[] w = math_.inverse(new double[] {ns, ew, rotation}, rotationCenter);
		w = normalizeWheelSpeeds(w, encoderVelocityMax_, Math.max(Math.max(Math.abs(ns), Math.abs(ew)), Math.abs(rotation)), wheelSpeedMax_);
		double[] v = math_.forward(w);
		
		setVelocity(v[0], v[1], v[2], rotationCenter);
	}
	
	@Override
	public void setRelativeTranslationVelocity(double ns, double ew) {
		
		ns *= wheelSpeedMax_;
		ew *= wheelSpeedMax_;
		
		double[] w = math_.inverse(new double[] {ns, ew, 0});
		w = normalizeWheelSpeeds(w, encoderVelocityMax_, Math.max(Math.abs(ns), Math.abs(ew)), wheelSpeedMax_);
		double[] v = math_.forward(w);
		
		setTranslationVelocity(v[0], v[1]);
	}

	@Override
	public void setRelativeRotationVelocity(double rotation, double[] rotationCenter) {
		
		rotation *= wheelSpeedMax_;
		
		setRotationVelocity(rotation, rotationCenter);
	}

	@Override
	public void setTranslationVelocity(double ns, double ew) {
		double[] v = getVelocity();
		setVelocity(ns, ew, v[2]);
	}

	@Override
	public void setRotationVelocity(double rotation) {
		double[] v = getVelocity();
		setVelocity(v[0], v[1], rotation);
	}

	@Override
	public void setRotationVelocity(double rotation, double[] rotationCenter) {
		double[] velocity = getVelocity();
		double[] vr = math_.forward(math_.inverse(new double[] {0, 0, rotation}, rotationCenter));
		setVelocity(velocity[0] + vr[0], velocity[1] + vr[1], rotation);
	}
	
	@Override
	public void setHeading(double heading) {
		super.setHeading(heading);
		headingPID_.setSetpoint(heading);
	}
	
	@Override
	public void halt() {
		for (int i = 0; i < 4; i++) {
			motors_[i].set(w_[i] = 0);
		}
		time_ = Timer.getFPGATimestamp();
		
    	headingPID_.disable();
	}

	@Override
	public void zeroDistance() {
		for (int i = 0; i < 4; i++) {
			motors_[i].setEncPosition(0);
		}
	}
	
	@Override
	public double[] getDistanceFromZero() {
		double[] p = {motors_[0].getEncPosition()/10, motors_[1].getEncPosition()/10, motors_[2].getEncPosition()/10, motors_[3].getEncPosition()/10};
		p = signCorrectWheels(p);
		
		
		//////////////////////// !!!!!!!!!!!!!!!!!!!!! 2017 competition robot ONLY !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		for(int i = 0; i < 4; i++) {
			p[i] *= -1;
		}
		//////////////////////// !!!!!!!!!!!!!!!!!!!!! 2017 competition robot ONLY !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		
		
		
		
		double[] d = math_.forward(p);
		//System.out.println("getDistanceFromZero:" + p[0] + " " + p[1] + " " + p[2] + " " + p[3] + "\t\t" + d[0] + " " + d[1] + " " + d[2]);
		return d;
	}
	
	@Override
	public double[] getDistanceToStop() {

		double[] w = getWheelSpeedsSet();
		//double[] w = getWheelSpeedsActual();
		double wMax = 0;
		for (int i = 0; i < 4; i++) {
			wMax = Math.max(Math.abs(w[i]), wMax);
		}
		if (wMax > 0) {
			double dMax = wMax*wMax/(2*getAcceleration()*encoderVelocityG_);
			for (int i = 0; i < 4; i++) {
				w[i] *= dMax / wMax;
			}
		}
		return math_.forward(w);
	}

	@Override
	public double robotRotationDistanceToDegrees(double distance) {
		return Math.toDegrees(distance / rotationSpeedFactor_);
	}

	@Override
	public double robotRotationDegreesToDistance(double degrees) {
		return Math.toRadians(degrees) * rotationSpeedFactor_;
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new OperatorDrive());
	}
}
