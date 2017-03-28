package org.usfirst.frc.team2077.season2017.subsystems;

import org.usfirst.frc.team2077.season2017.robot.Robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {
	public static Talon spinningDevice = new Talon(0);
	public static DigitalInput climberStopper = new DigitalInput(2);
	
	private static PowerDistributionPanel pdp = new PowerDistributionPanel(0); // TODO: check CAN ID
	private static int pdpCircuit = 4;
	private static double currentLimit = 30;
	private static boolean limitExceeded = false; // fail-safe hack in case limit switch is missed
	private static int sampleCount = 32;
	private static double sampleSum = 0;
	private static int sampleIndex = 0;
	private static double[] sample = new double[sampleCount];
	
	/**
	* @return true if climber current limit exceeded
	 */
	public static boolean checkCurrent() {
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
    
    public static void NoSpin()
    {
    	spinningDevice.set(0);
    }
    
    public static void Spin(double spinMultiplier)
    {
    	Preferences prefs = Preferences.getInstance();

		spinningDevice.set(prefs.getDouble("Top Climber Speed", -1)*spinMultiplier);
    }
    
    public static void PushPlate()
    {
    	Preferences prefs = Preferences.getInstance();
    	
    	//prefs.getDouble("Push Plate Climber Speed", -1)
    	
		spinningDevice.set(-.5);
    }
}

