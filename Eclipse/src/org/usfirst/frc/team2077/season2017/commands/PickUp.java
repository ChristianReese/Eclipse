package org.usfirst.frc.team2077.season2017.commands;

import org.usfirst.frc.team2077.season2017.robot.OperatorInterface;
import org.usfirst.frc.team2077.season2017.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PickUp extends Command {

    public PickUp() {
    	requires(Robot.ballCollector);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.ballCollector.ballCollector.set(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.ballCollector.PickUp();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(!OperatorInterface.pickBallButton.get())
    	{
    		return true;
    	}
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.ballCollector.ballCollector.set(0);

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.ballCollector.ballCollector.set(0);

    }
}
