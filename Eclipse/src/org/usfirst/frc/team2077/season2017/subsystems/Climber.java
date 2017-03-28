package org.usfirst.frc.team2077.season2017.subsystems;

import org.usfirst.frc.team2077.season2017.robot.Robot;
import org.usfirst.frc.team2077.season2017.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {
	private Talon spinningDevice;
	private DigitalInput climberStopper;
	
	private PowerDistributionPanel pdp = new PowerDistributionPanel(0); // TODO: check CAN ID
	private int pdpCircuit = 4;
	private double currentLimit = 30;
	private boolean limitExceeded = false; // fail-safe hack in case limit switch is missed
	private int sampleCount = 32;
	private double sampleSum = 0;
	private int sampleIndex = 0;
	private double[] sample = new double[sampleCount];
	
	public Climber() {
		super();

		spinningDevice = null;
		climberStopper = null;

		if ( RobotMap.ROBOT_PLATFORM == RobotMap.RobotPlatform.RP_ECLIPSE )
		{
			spinningDevice = new Talon(0);
			climberStopper = new DigitalInput(2);
		}
	}

	/**
	* @return true if climber current limit exceeded
	 */
	public boolean checkCurrent() {
		if (limitExceeded) {
			return true;
		}
		sampleSum -= sample[sampleIndex];
		sample[sampleIndex] = pdp.getCurrent(pdpCircuit);
		sampleSum += sample[sampleIndex];
		sampleIndex++;
		sampleIndex = sampleIndex % sampleCount;
		limitExceeded = sampleSum/sampleCount > currentLimit;
		System.out.println("CLIMBER CURRENT:" + (sampleSum/sampleCount));
		if (limitExceeded) {
			System.out.println("CLIMBER CURRENT LIMIT EXCEEDED:" + (sampleSum/sampleCount));
		}
		return limitExceeded;
	}

	// Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void noSpin()
    {
    	if ( spinningDevice != null )
    	{
        	spinningDevice.set(0);
    	}
    }
    
    public void spin(double spinMultiplier)
    {
    	if ( spinningDevice != null )
    	{
	    	Preferences prefs = Preferences.getInstance();
	
			spinningDevice.set(prefs.getDouble("Top Climber Speed", -1)*spinMultiplier);
    	}
    }
    
    public void pushPlate()
    {
    	if ( spinningDevice != null )
    	{
	    	Preferences prefs = Preferences.getInstance();
	    	
	    	//prefs.getDouble("Push Plate Climber Speed", -1)
	    	
			spinningDevice.set(-.5);
    	}
    }
}

