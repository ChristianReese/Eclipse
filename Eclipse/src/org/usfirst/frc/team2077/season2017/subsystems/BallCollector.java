package org.usfirst.frc.team2077.season2017.subsystems;

import org.usfirst.frc.team2077.season2017.commands.PickUp;
import org.usfirst.frc.team2077.season2017.robot.Robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class BallCollector extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public static Talon ballCollector = new Talon(2);
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public static void PickUp()
    {
    	Preferences prefs = Preferences.getInstance();

		ballCollector.set(prefs.getDouble("Ball Collector Speed", -.5));
    }
    
    public static void Drop()
    {
		ballCollector.set(.25);
    }
}

