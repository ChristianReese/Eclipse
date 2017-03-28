package org.usfirst.frc.team2077.season2017.autonomous;

import org.usfirst.frc.team2077.season2017.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TurnLeft extends Command {

    public TurnLeft() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drive_);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drive_.halt();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drive_.setRelativeRotationVelocity(-.1);
    	SmartDashboard.putDouble("Robot Angle", Robot.gyro_.getAngle());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(60 - Math.abs(Robot.gyro_.getAngle()) < 1)
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
