package org.usfirst.frc.team2077.season2017.subsystems;

import org.usfirst.frc.team2077.season2017.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GearSetter extends Subsystem {
	private Talon gearSetter;
	private Encoder gearSetterEnc;
	private DigitalInput gearFlapLeft;
	private DigitalInput gearFlapRight;
	
	private static final boolean GEAR_FLAP_TRIGGEREDPUSHED_STATE = false; // true if limits switches wired NO, false if wired NC

	public GearSetter() {
		super();

		gearSetter = null;
		gearSetterEnc = null;
		gearFlapLeft = null;
		gearFlapRight = null;

		if ( RobotMap.ROBOT_PLATFORM == RobotMap.RobotPlatform.RP_ECLIPSE )
		{
			gearSetter = new Talon(1);
			gearSetterEnc = new Encoder(0,1);
			gearFlapLeft = new DigitalInput(8);
			gearFlapRight = new DigitalInput(7);
		}
	}
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void set()
    {
    	if ( gearSetter != null )
    	{
        	gearSetter.set(1);
    	}
    }
    
    public void rest()
    {
    	if ( gearSetter != null )
    	{
    		gearSetter.set(-.2);
    	}
    }
    
    public void initial()
    {
    	if ( gearSetter != null )
    	{
    		gearSetter.set(0);
    	}
    }
    
    /** @return true iff the pin detector plate is pushed back */
    public boolean isOnPin() {
    	
    	if ( gearFlapLeft != null )
    	{
	    	// TODO: Is this return statement a bug?
	    	return gearFlapLeft.get()==GEAR_FLAP_TRIGGEREDPUSHED_STATE || gearFlapLeft.get()==GEAR_FLAP_TRIGGEREDPUSHED_STATE;
    	}
    	
    	return false;
    }
    
    public void resetGearSetterEncoder()
    {
    	if ( gearSetterEnc != null )
    	{
        	gearSetterEnc.reset();
    	}
    }
    
    public double getGearSetterEncoderDistance()
    {
    	if ( gearSetterEnc != null )
    	{
    		return gearSetterEnc.getDistance();
    	}
    	
    	return 0.0;
    }
    
    public boolean getGearFlapLeftValue()
    {
    	if ( gearFlapLeft != null ) 
    	{
    		return gearFlapLeft.get();
    	}
    	
    	return !GEAR_FLAP_TRIGGEREDPUSHED_STATE;
    }
    
    public boolean getGearFlapRightValue()
    {
    	if ( gearFlapRight != null ) 
    	{
    		return gearFlapRight.get();
    	}
    	
    	return !GEAR_FLAP_TRIGGEREDPUSHED_STATE;
    }
}