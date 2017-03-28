package org.usfirst.frc.team2077.season2017.subsystems;

import org.usfirst.frc.team2077.season2017.robot.RobotMap;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class BallCollector extends Subsystem {
	
	private Talon ballCollector;
	
	public BallCollector() {
		super();
		
		ballCollector = null;
		
		if ( RobotMap.ROBOT_PLATFORM == RobotMap.RobotPlatform.RP_ECLIPSE )
		{
			ballCollector = new Talon(2);
		}
	}
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void setBallCollectorSpeed( double speed )
    {
    	if ( ballCollector != null )
    	{
    		ballCollector.set( speed );
    	}
    }
    
    public void pickUp()
    {
    	Preferences prefs = Preferences.getInstance();

    	setBallCollectorSpeed(prefs.getDouble("Ball Collector Speed", -.5));
    }
    
    public void drop()
    {
    	setBallCollectorSpeed(.25);
    }
}

