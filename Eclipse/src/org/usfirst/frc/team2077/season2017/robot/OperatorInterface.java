package org.usfirst.frc.team2077.season2017.robot;

import org.usfirst.frc.team2077.season2017.commands.CenterToTargetSnapshot;
import org.usfirst.frc.team2077.season2017.commands.Climb;
import org.usfirst.frc.team2077.season2017.commands.Drop;
import org.usfirst.frc.team2077.season2017.commands.FieldRelativeSwitch;
import org.usfirst.frc.team2077.season2017.commands.GearRest;
import org.usfirst.frc.team2077.season2017.commands.GearRotate;
import org.usfirst.frc.team2077.season2017.commands.MoveRelative;
import org.usfirst.frc.team2077.season2017.commands.PickUp;
import org.usfirst.frc.team2077.season2017.commands.ShimyLeft;
import org.usfirst.frc.team2077.season2017.commands.ShimyRight;
import org.usfirst.frc.team2077.season2017.commands.Spin;
import org.usfirst.frc.team2077.season2017.commands.SwitchDirection;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
*
*/
public class OperatorInterface {
    
	public static XboxController xbox_ = new XboxController(0);
	public static Joystick joystick_ = new Joystick(1);
	
	public static Button pickBallButton = new JoystickButton(joystick_,1),
			dropBallButton = new JoystickButton(joystick_,19),
			gearRotation = new JoystickButton(joystick_, 9),
			spinLift = new JoystickButton(joystick_,3),
			slowSpinLift = new JoystickButton(joystick_,21),
			smallTurn = new JoystickButton(xbox_,1),
			fieldRelativeChange = new JoystickButton(xbox_,2),
			backupGearButton = new JoystickButton(xbox_,3),
			switchDirectionButton = new JoystickButton(xbox_, 4),
			shimyLeftButton = new JoystickButton(xbox_, 5),
			shimyRightButton = new JoystickButton(xbox_, 6);
	
    public OperatorInterface()
    {
    	//double a = .15;
    	// test code
        //(new JoystickButton(joystick_,  1)).whenPressed(new VisualDrive);
    	//findAndTravelToTargetButton.whileHeld(new CenterToTargetSnapshot());//(new FindAndTravelToTarget(5.0,0.15));
    	switchDirectionButton.whenPressed(new SwitchDirection());
    	fieldRelativeChange.whenPressed(new FieldRelativeSwitch());
    	shimyLeftButton.whenPressed(new ShimyLeft()); 
    	shimyRightButton.whenPressed(new ShimyRight());
    	smallTurn.whenPressed(new MoveRelative (0.15, new double[]{0,0,15}));

		pickBallButton.whenPressed(new PickUp());
		dropBallButton.whenPressed(new Drop());
		
		gearRotation.whileHeld(new GearRotate());
		gearRotation.whenReleased(new GearRest());
		
		backupGearButton.whileHeld(new GearRotate());
		backupGearButton.whenReleased(new GearRest());
		
		spinLift.whileHeld(new Spin(1));
		slowSpinLift.whileHeld(new Spin(.5));
		//spinLift.whenPressed(new Climb(1));
		//slowSpinLift.whenPressed(new Climb(.5));
		//spinLift.whileHeld(new Climb(1));
		//slowSpinLift.whileHeld(new Climb(.5));
        
    }
    
    public double getNorthSouth()
    {
    	return xbox_.getY(Hand.kLeft);
    }
    
    public double getEastWest()
    {
    	return xbox_.getX(Hand.kLeft);
    }
    
    public double getRotation()
    {
    	return xbox_.getX(Hand.kRight);
    }
}

 