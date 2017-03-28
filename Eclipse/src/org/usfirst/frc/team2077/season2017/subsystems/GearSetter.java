package org.usfirst.frc.team2077.season2017.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GearSetter extends Subsystem {
	public static Talon gearSetter = new Talon(1);
	public static Encoder gearSetterEnc = new Encoder(0,1);
	public static DigitalInput gearFlapLeft = new DigitalInput(8);
	public static DigitalInput gearFlapRight = new DigitalInput(7);
	private static final boolean GEAR_FLAP_TRIGGEREDPUSHED_STATE = false; // true if limits switches wired NO, false if wired NC
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void Set(double speed)
    {
    	gearSetter.set(speed);
    }
    
    public void initial(){
    	gearSetter.set(0);
    }
    
    /** @return true iff the pin detector plate is pushed back */
    public boolean isOnPin() {
    	return gearFlapLeft.get()==GEAR_FLAP_TRIGGEREDPUSHED_STATE || gearFlapRight.get()==GEAR_FLAP_TRIGGEREDPUSHED_STATE;
    }
}