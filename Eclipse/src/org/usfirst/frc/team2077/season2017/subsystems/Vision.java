package org.usfirst.frc.team2077.season2017.subsystems;

import org.usfirst.frc.team2077.season2017.vision.measurement.TargetTransform;
import org.usfirst.frc.team2077.season2017.vision.trackers.VisionTracker;
import org.usfirst.frc.team2077.season2017.vision.trackers.VisionTrackers;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Vision extends Subsystem {
	
	private VisionTracker gearLiftTracker;
	
	private TargetTransform targetSnapshot_;
	
	public Vision( String camera )
	{
		gearLiftTracker = VisionTrackers.createGearLiftTracker(camera);
		targetSnapshot_ = null;
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void startGearLiftTracker( double fps )
    {
    	if ( !gearLiftTracker.isRunning() )
    	{
        	gearLiftTracker.start( fps );
    	}
    }
    
    public void stopGearLiftTracker()
    {
    	if ( gearLiftTracker.isRunning() )
    	{
        	gearLiftTracker.stop();
    	}
    }
    
    public TargetTransform getLatestGearLiftTargetTransform()
    {
    	return gearLiftTracker.getLatestTargetTransform();
    }
    
    public void takeTargetSnapshot( TargetTransform targetSnapshot )
    {
    	targetSnapshot_ = targetSnapshot;
    }
    
    public TargetTransform getTargetSnapshot()
    {
    	return targetSnapshot_;
    }
    
    public void setTargetSnapshot( TargetTransform tt )
    {
    	targetSnapshot_ = tt;
    }
}

