package org.usfirst.frc.team2077.season2017.commands;

import org.usfirst.frc.team2077.season2017.robot.Robot;
import org.usfirst.frc.team2077.season2017.vision.measurement.TargetTransform;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class FindAndTakeTargetSnapshot extends Command {
	
	private double trackerFPS_;
	
	private TargetTransform previousTargetTransform;

    public FindAndTakeTargetSnapshot(double trackerFPS) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.vision_);
        
        trackerFPS_ = trackerFPS;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(0.0);
    	
    	Robot.vision_.setTargetSnapshot(null);
    	
    	previousTargetTransform = Robot.vision_.getLatestGearLiftTargetTransform();
    	Robot.vision_.startGearLiftTracker(trackerFPS_);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	TargetTransform tt = Robot.vision_.getLatestGearLiftTargetTransform();
    	
    	if ( !isTimedOut() )
    	{
    		return false;
    	}
    	
    	if ( tt == null )
    	{
    		return false;
    	}
    	
    	if ( !tt.targetFound() )
    	{
    		return false;
    	}
    	
    	if ( previousTargetTransform == null )
    	{
    		Robot.vision_.takeTargetSnapshot( tt );
			return true;
    	}
    	
    	if ( !tt.equals(previousTargetTransform) )
    	{
			Robot.vision_.takeTargetSnapshot( tt );
			return true;
    	}
    	
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.vision_.stopGearLiftTracker();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.vision_.stopGearLiftTracker();
    }
}
