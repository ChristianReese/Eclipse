package org.usfirst.frc.team2077.season2017.commands;

import org.usfirst.frc.team2077.season2017.commands.MoveRelative;
import org.usfirst.frc.team2077.season2017.robot.Robot;
import org.usfirst.frc.team2077.season2017.robot.RobotMap;
import org.usfirst.frc.team2077.season2017.vision.measurement.TargetTransform;

import edu.wpi.first.wpilibj.Preferences;

public class CenterToTargetSnapshot_old extends MoveRelative {

    public CenterToTargetSnapshot_old(double g) {
        super(g);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	TargetTransform tt = Robot.vision_.getTargetSnapshot();
    	Preferences prefs = Preferences.getInstance();
    	
    	if ( tt != null )
    	{
    		if (tt.targetFound())
    		{
    			double strafeLength = 0.0;
    			double rotateAmount = tt.getRobotAngleToTarget();
    			double rotateAmountMultiplier = prefs.getDouble(RobotMap.ROTATION_MULTIPLIER_KEY, RobotMap.ROTATION_MULTIPLIER_DEFAULT);
    			
    			System.out.println("Before - Robot gyro: " + Robot.gyro_.getAngle() + " DriveTrain Gyro: "
    								+ Robot.drive_.getHeading() + " Amount to Rotate: " + rotateAmount);
    			
    			if (Math.abs(tt.getRobotAngleToTarget()) > 0.5){
    				initializeSegments(new double[] {0.0, strafeLength, rotateAmount * rotateAmountMultiplier});
    			}else{
    				initializeSegments(new double[] {0.0, strafeLength, 0.0});
    			}
    		}
    	}
    	
    	super.initialize();
    }

    @Override
    protected boolean isFinished() {
    	boolean superResult = super.isFinished();
    	
//    	if ( superResult )
//    	{
//    		segments_ = null;
//    	}
    	
    	return superResult;
    }

    @Override
    protected void end() {
    	super.end();
    	segments_ = null;
    	System.out.println("End - Robot gyro: " + Robot.gyro_.getAngle() + " DriveTrain Gyro: "
				+ Robot.drive_.getHeading());
    }

    @Override
    protected void interrupted() {
    	super.interrupted();
    	segments_ = null;
    	System.out.println("Interrupt - Robot gyro: " + Robot.gyro_.getAngle() + " DriveTrain Gyro: "
				+ Robot.drive_.getHeading());
    }
}
