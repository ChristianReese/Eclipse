package org.usfirst.frc.team2077.season2017.commands;

import org.usfirst.frc.team2077.season2017.commands.MoveRelative;
import org.usfirst.frc.team2077.season2017.robot.Robot;
import org.usfirst.frc.team2077.season2017.robot.RobotMap;
import org.usfirst.frc.team2077.season2017.subsystems.GearSetter;
import org.usfirst.frc.team2077.season2017.vision.measurement.TargetTransform;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;

public class CenterToTargetSnapshot extends Command {

	private TargetTransform previousTargetTransform = null;
	private TargetTransform currentTargetTransform = null;

    public CenterToTargetSnapshot() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drive_);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//Robot.vision_.setTargetSnapshot(null);
    	
    	//previousTargetTransform = Robot.vision_.getLatestGearLiftTargetTransform();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	previousTargetTransform = currentTargetTransform;
    	currentTargetTransform = Robot.vision_.getLatestGearLiftTargetTransform();
    	
    	TargetTransform newTT = getNewImageFound();
    	
    	if (newTT != null){
    		Robot.drive_.setHeading(Robot.gyro_.getAngle() + newTT.getRobotAngleToTarget());
    		// todo: compute ETA
    	}
    	
    	//Robot.drive_.setRelativeRotationVelocity(0);
    	
    	//Robot.drive_.setRelativeTranslationVelocity(0.2, 0.0);
    	
    	Robot.drive_.setRelativeVelocity(0.1, 0.0, 0.0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
// pin detect    	boolean superResult = super.isFinished() || (!GearSetter.gearFlapLeft.get() && !GearSetter.gearFlapRight.get());
// check eta
    	
    	
    	
    	
    	return false;//Math.abs(Robot.gyro_.getAngle() - Robot.drive_.getHeading()) < 0.5; 
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
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
