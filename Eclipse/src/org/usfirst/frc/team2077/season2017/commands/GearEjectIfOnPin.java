package org.usfirst.frc.team2077.season2017.commands;

import org.usfirst.frc.team2077.season2017.robot.OperatorInterface;
import org.usfirst.frc.team2077.season2017.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// This code was branched from GearRotate in Eclipse_2017-03-18-1751
// It does nothing unless isOnPin() returns true.
public class GearEjectIfOnPin extends Command {
	
	double startTime;

    public GearEjectIfOnPin() {
    	requires(Robot.gearSetter);
    	requires(Robot.drive_);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.gearSetter.initial();
    	startTime = Timer.getFPGATimestamp();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if ((Robot.gearSetter.isOnPin() && !Robot.isTeleop) || (Robot.isTeleop && Robot.gearSetter.isOnPin() && Robot.operatorInterface_.gearRotation.get())) {
        	Robot.gearSetter.Set(1);
        	SmartDashboard.putDouble("Gear Setter", Robot.gearSetter.gearSetterEnc.getDistance());
    	}
    	if(!Robot.isTeleop)
    		Robot.drive_.setVelocity(-12, 0,  0);   
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if ((Timer.getFPGATimestamp() - startTime < 1.0) || (Robot.isTeleop && Robot.operatorInterface_.gearRotation.get())) {
    		return false;
    	}
    	return true;
//    	
//    	
//    	if (!Robot.gearSetter.isOnPin()) {
//    		return true;
//    	}
//    	if((!OperatorInterface.gearRotation.get() && Robot.isTeleop) || 
//    			(Robot.gearSetter.gearSetterEnc.getDistance() > 485 && !Robot.isTeleop))
//    	{
//    		return true;
//    	}
//        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.gearSetter.initial();
    	Robot.drive_.setVelocity(0, 0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.gearSetter.initial();
    	Robot.drive_.setVelocity(0, 0, 0);

    }
}
