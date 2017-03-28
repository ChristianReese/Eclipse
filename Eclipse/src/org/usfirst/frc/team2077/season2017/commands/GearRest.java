package org.usfirst.frc.team2077.season2017.commands;

import org.usfirst.frc.team2077.season2017.robot.OperatorInterface;
import org.usfirst.frc.team2077.season2017.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class GearRest extends Command {

    public GearRest() {
    	requires(Robot.gearSetter);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.gearSetter.initial();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.gearSetter.rest();
    	SmartDashboard.putDouble("Gear Setter", Robot.gearSetter.getGearSetterEncoderDistance());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(Robot.gearSetter.getGearSetterEncoderDistance() - 
    			(int)(Robot.gearSetter.getGearSetterEncoderDistance()/470) * 470 <= 2)
    	{
    		return true;
    	}
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.gearSetter.initial();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.gearSetter.initial();

    }
}
