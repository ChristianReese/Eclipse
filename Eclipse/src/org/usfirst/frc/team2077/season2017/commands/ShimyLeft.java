package org.usfirst.frc.team2077.season2017.commands;

import org.usfirst.frc.team2077.season2017.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class ShimyLeft extends TimedCommand {

    public ShimyLeft() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super(.333);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(.1);
    	Robot.drive_.halt();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drive_.setRelativeVelocity(-1, 0, 0);;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(isTimedOut())
    	{
    		return true;
    	}
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive_.halt();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drive_.halt();
    }
}
