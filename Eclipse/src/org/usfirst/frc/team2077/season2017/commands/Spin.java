package org.usfirst.frc.team2077.season2017.commands;

import org.usfirst.frc.team2077.season2017.robot.Robot;
import org.usfirst.frc.team2077.season2017.subsystems.Climber;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Spin extends Command {
	
	double spinMultiplier;
	
	public Spin(double v) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
		spinMultiplier = v;
    	requires(Robot.climber);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.climber.noSpin();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute(){
    	
    	if (Robot.climber.checkCurrent()) {
    		return;
    	}
    	
     	Robot.climber.spin(spinMultiplier);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (Robot.climber.checkCurrent()) {
    		return true;
    	}
        /*if(Climber.climberStopper.get())
        {
        	return true;
        }*/
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.climber.noSpin();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.climber.noSpin();
    }
}
