package org.usfirst.frc.team2077.season2017.commands;

import org.usfirst.frc.team2077.season2017.commands.MoveRelative;
import org.usfirst.frc.team2077.season2017.robot.Robot;
import org.usfirst.frc.team2077.season2017.robot.RobotMap;
import org.usfirst.frc.team2077.season2017.subsystems.GearSetter;
import org.usfirst.frc.team2077.season2017.vision.measurement.TargetTransform;

import edu.wpi.first.wpilibj.Preferences;

public class AdvanceTowardTargetSnapshot extends MoveRelative {

    public AdvanceTowardTargetSnapshot(double g) {
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
    			double advanceLength = tt.getRobotToTargetDistance()
    					- prefs.getDouble(RobotMap.PLANE_TO_PEG_DISTANCE_KEY, RobotMap.PLANE_TO_PEG_DISTANCE_DEFAULT);
    			double advanceMultiplier = prefs.getDouble(RobotMap.ADVANCE_LENGTH_MULTIPLIER_KEY, 
    													   RobotMap.ADVANCE_LENGTH_MULTIPLIER_DEFAULT);
    			
            	initializeSegments(new double[] {advanceLength * advanceMultiplier, 0.0, 0.0});
    		}
    	}
    	
    	super.initialize();
    }

    @Override
    protected boolean isFinished() {
    	boolean superResult = super.isFinished() || (!GearSetter.gearFlapLeft.get() && !GearSetter.gearFlapRight.get());
    	
//    	if (superResult)
//    	{
//    		segments_ = null;
//    	}
    	
    	return superResult;
    }

    @Override
    protected void end() {
    	super.end();
    	segments_ = null;
    }

    @Override
    protected void interrupted() {
    	super.interrupted();
    	segments_ = null;
    }
}
