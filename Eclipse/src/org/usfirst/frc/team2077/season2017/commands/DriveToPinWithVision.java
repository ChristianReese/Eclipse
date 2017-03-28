package org.usfirst.frc.team2077.season2017.commands;

import org.usfirst.frc.team2077.season2017.robot.Robot;
import org.usfirst.frc.team2077.season2017.vision.measurement.TargetTransform;

import edu.wpi.first.wpilibj.command.Command;

// This code was branched from CenterToTargetSnapshot in Eclipse_2017-03-18-1751
public class DriveToPinWithVision extends Command {

	private TargetTransform previousTargetTransform = null;
	private TargetTransform currentTargetTransform = null;

	/**
	 * It is essential that the robot not stop short of the pin, so the drive distance will
	 * be deliberately set to a point beyond its assumed location, so the robot will move forward
	 * until it either detects the pin with the GearSetter touch plate or wheelspins against the wall/lift.
	 * OVERSHOOT_DISTANCE is the amount added to the best estimate of the target point to create
	 * a phantom aim point beyond the wall to which to decelerate.
	 */
	private static final double OVERSHOOT_DISTANCE = 18;
	/** give up if we get this close to stopping point without hitting pin */
	private static final double DISTANCE_TOLERANCE = 2; // 
	private double distanceToStoppingPoint;
	private double speedLimit;
	private double gLimit;

	/**
	 * This command is designed primarily for autonomous. It may be possible to use it in teleop by setting
	 * distanceToPin to an overlong upper bound, and speedLimit to a speed at which we're willing to slam
	 * into the wall/lift. What acceleration limit can be supported depends on vision tracking capability
	 * and drive heading PID tuning.
	 * @param nominalDistanceToPin Best estimate of the forward distance in inches between the start position
	 * and where the robot is as far onto the lift pin as it will go. It is assumed that the pin is more or less
	 * straight in front of the robot and that the vision targets will probably be detected after driving some
	 * distance forward.
	 * @param speedLimit Upper limit on robot speed, in inches/second.
	 * @param gLimit Upper limit on acceleration/deceleration, relative to G. (try 0.15)
	 */
    public DriveToPinWithVision(double nominalDistanceToPin, double speedLimit, double gLimit) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drive_);
        // planned stop point; may be recalculated if/when vision data is available
        this.distanceToStoppingPoint = nominalDistanceToPin + OVERSHOOT_DISTANCE;
        // nominal robot speed; may be recalculated if vision data changes confidence level
        this.speedLimit = speedLimit;
        // acceleration limit, normally should not change during run
        this.gLimit = gLimit;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drive_.zeroDistance();
    	Robot.drive_.setAcceleration(gLimit);
    	Robot.drive_.setHeading(Robot.gyro_.getAngle());
    	Robot.drive_.setVelocity(speedLimit, 0, 0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	// if vision tracking data is available use it
    	previousTargetTransform = currentTargetTransform;
    	currentTargetTransform = Robot.vision_.getLatestGearLiftTargetTransform();
    	TargetTransform newTT = getNewImageFound();
    	if (newTT != null){
    		Robot.drive_.setHeading(Robot.gyro_.getAngle() + newTT.getRobotAngleToTarget());
    		if (Robot.instance_.isOperatorControl()) { // for autonomous the initial distance estimate should be better than what vision can provide
        		// update distanceToStoppingPoint based on new distance data
        		Robot.drive_.zeroDistance();
        		distanceToStoppingPoint = newTT.getRobotToTargetDistance() + OVERSHOOT_DISTANCE;
        		// TODO: speedLimit = something faster if we're more confident now
    		}
    	}
    	
    	// drive forward, braking if near target
    	Robot.drive_.setVelocity(Robot.drive_.getDistanceToStop()[0] < distanceToStoppingPoint ? speedLimit : 0, 0, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Robot.gearSetter.isOnPin() // success
    			|| Math.abs(Robot.drive_.getDistanceFromZero()[0] - distanceToStoppingPoint) < DISTANCE_TOLERANCE; // failure
    	// TODO: if heading adjustments are large it's possible the forward distanceFromZero value may not converge
    	//       this seems unlikely in autonomous, and if it happens worst-case is that the robot just spins against the missed
    	//       gear lift until teleop begins - IF THIS COMMAND IS USED IN TELEOP THERE MUST BE A WAY FOR THE DRIVER TO INTERRUPT IT
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive_.setVelocity(0, 0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
    
    protected TargetTransform getNewImageFound() {
    	
    	if ( currentTargetTransform == null )
    	{
    		return null;
    	}
    	
    	if ( !currentTargetTransform.targetFound() )
    	{
    		return null;
    	}
    	
    	if ( previousTargetTransform == null )
    	{
    		//Robot.vision_.takeTargetSnapshot( tt );
			return currentTargetTransform;
    	}
    	
    	if ( !currentTargetTransform.equals(previousTargetTransform) )
    	{
			//Robot.vision_.takeTargetSnapshot( tt );
			return currentTargetTransform;
    	}
    	
    	return null;
    }
    
    protected double forwardSpeedMultiplier()
    {
    	return 1.0 - Math.min(1.0, Math.max(0.0, Math.abs(Robot.gyro_.getAngle() - Robot.drive_.getHeading()) / 2.0));
    }
}
