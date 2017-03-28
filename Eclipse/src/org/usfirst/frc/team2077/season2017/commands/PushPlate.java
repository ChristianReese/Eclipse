package org.usfirst.frc.team2077.season2017.commands;

import org.usfirst.frc.team2077.season2017.robot.Robot;
import org.usfirst.frc.team2077.season2017.robot.RobotMap;
import org.usfirst.frc.team2077.season2017.subsystems.Climber;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class PushPlate extends TimedCommand {
	

    public PushPlate() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super(.5);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Preferences prefs = Preferences.getInstance();

		setTimeout(.5/*prefs.getDouble(RobotMap.SLOW_CLIMBER_TIME_KEY, RobotMap.SLOW_CLIMBER_TIME_DEFAULT)*/);
    	Robot.climber.NoSpin();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Climber.checkCurrent()) {
    		return;
    	}
    	
    	Robot.climber.PushPlate();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished(){
    	
    	if (Climber.checkCurrent()) {
        	return true;
        }
        
        if(isTimedOut())
    	{
    		return true;
    	}        
        
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.climber.NoSpin();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.climber.NoSpin();

    }
}
