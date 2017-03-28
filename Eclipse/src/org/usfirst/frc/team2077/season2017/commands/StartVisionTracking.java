package org.usfirst.frc.team2077.season2017.commands;

import org.usfirst.frc.team2077.season2017.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class StartVisionTracking extends Command {
	
	private double fps_;

    public StartVisionTracking(double fps) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.vision_);
        fps_ = fps;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.vision_.startGearLiftTracker(fps_);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
