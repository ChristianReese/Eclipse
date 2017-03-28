package org.usfirst.frc.team2077.season2017.commands;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

import org.usfirst.frc.team2077.season2017.math.*;
import org.usfirst.frc.team2077.season2017.robot.Robot;

/**
 *
 */
public class MoveRelative extends Command {

	private final double g_;
	protected double[][] segments_;
	private int segmentIndex_ = -1;

	private double[] segmentDistance_ = {0, 0, 0};
	private double[] relativeVelocity_ = {0, 0, 0};
	
	private final double DISTANCE_TOLERANCE = .1;
	private final double NEAR_STOP_VELOCITY = .1;
	private final boolean DEBUG_PRINT = false;
	
	/**
	 * @param segments Each segment is {inches ns, inches ew, degrees rotation}, (robot-relative).
	 */
	public MoveRelative(double g, double[]... segments) {
		requires(Robot.drive_);
		g_ = g;
		segments_ = segments;
	}

	/**
	 * initializeSegments must be called in the initializer of a child class, before MoveRelative.initialize() is executed.
	 * Otherwise this command will do nothing.
	 */
	public MoveRelative(double g) {
		requires(Robot.drive_);
		g_ = g;
		segments_ = null;
	}
	
	protected void initializeSegments(double[]... segments) {
		if ( segments_ == null )
		{
			segments_ = segments;
		}
		else
		{
			System.err.println("Error in MoveRelative: Attempted to initialize an already-initialized segments_ array!");
		}
	}
	
    @Override
    protected void initialize() {
    	Robot.drive_.setAcceleration(g_);
    	segmentIndex_ = -1;
    	if(DEBUG_PRINT)
    		System.out.println("\ninitialize:" + segmentIndex_);
    	startNextSegment();
    }
    
    private double zeroAngle_ = 0;
    private double zeroTime_;
    
    private void startNextSegment() {
    	if(DEBUG_PRINT)
    		System.out.println("\nstartNextSegment:" + segmentIndex_);
    	Robot.drive_.halt();
    	Robot.drive_.zeroDistance();
    	zeroTime_ = Timer.getFPGATimestamp();
    	zeroAngle_ = Robot.gyro_.getAngle();
    	
    	if (segments_ == null)
    	{
    		segmentIndex_ = -1;
    		return;
    	}
    	
    	if (++segmentIndex_ >= segments_.length) {
    		segmentIndex_ = -1;
    		return;
    	}
    	segmentDistance_ = new double[] {segments_[segmentIndex_][0], segments_[segmentIndex_][1], Robot.drive_.robotRotationDegreesToDistance(segments_[segmentIndex_][2])};
    	if(DEBUG_PRINT)
    		System.out.println("segment distance:" + segmentDistance_[0] + " " + segmentDistance_[1] + " " + segmentDistance_[2]);
    }

    @Override
    protected void execute() {
    	if (segmentIndex_ < 0) {
    		return;
    	}
    	if (Timer.getFPGATimestamp() - zeroTime_ < .2) {
    		return;
    	}
    	double[] distanceTraveled = Robot.drive_.getDistanceFromZero();
    	if (true) { // measure rotation with gyro instead of mecanum math
    		distanceTraveled[2] = Robot.drive_.robotRotationDegreesToDistance(Robot.gyro_.getAngle() - zeroAngle_);
    	}
    	double[] distanceRemaining = {segmentDistance_[0]-distanceTraveled[0], segmentDistance_[1]-distanceTraveled[1], segmentDistance_[2]-distanceTraveled[2]};
    	if(DEBUG_PRINT)
    		System.out.println("remaining distance:" + distanceRemaining[0] + " " + distanceRemaining[1] + " " + distanceRemaining[2]);
    	if(DEBUG_PRINT)
    		System.out.println("Remaining:" + distanceRemaining[2] + "\tTraveled:" + distanceTraveled[2] + "\tHeading:" + Robot.drive_.getHeading());
    	double maxDistance = 0;
    	for (int i = 0; i < 3; i++) {
    		if (Math.signum(distanceRemaining[i]) != Math.signum(segmentDistance_[i])) {
    			distanceRemaining[i] = 0;
    		}
    		maxDistance = Math.max(maxDistance, Math.abs(distanceRemaining[i]));
    	}
    	if (maxDistance < DISTANCE_TOLERANCE) {
    		startNextSegment();
    		return;
    	}
     	double[] distanceToStop = Robot.drive_.getDistanceToStop();
    	for (int i = 0; i < 3; i++) {
    		if (Math.abs(distanceRemaining[i]) < DISTANCE_TOLERANCE) {
    			relativeVelocity_[i] = 0;
    		}
    		else if (Math.abs(distanceRemaining[i]) <= Math.abs(distanceToStop[i])) {
    			relativeVelocity_[i] = Math.signum(distanceRemaining[i]) * NEAR_STOP_VELOCITY;
    		}
    		else {
    			relativeVelocity_[i] = distanceRemaining[i] / maxDistance;
    		}
    	}
    	Robot.drive_.setRelativeVelocity(relativeVelocity_[0], relativeVelocity_[1], relativeVelocity_[2]);
    }

    @Override
    protected boolean isFinished() {
    	return segmentIndex_ < 0 && Timer.getFPGATimestamp() - zeroTime_ >= .2;
    }

    @Override
    protected void end() {
    	if(DEBUG_PRINT)
    		System.out.println("\nend:" + segmentIndex_);
    	segmentIndex_ = -1;
    	Robot.drive_.setRelativeVelocity(0,0,0);
    }

    @Override
    protected void interrupted() {
    	if(DEBUG_PRINT)
    		System.out.println("\ninterrupted:" + segmentIndex_);
    	segmentIndex_ = -1;
    	Robot.drive_.setRelativeVelocity(0,0,0);
    }
}
