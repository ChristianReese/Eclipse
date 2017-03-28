package org.usfirst.frc.team2077.season2017.robot;

import org.usfirst.frc.team2077.season2017.autonomous.AutoCenterPos;
import org.usfirst.frc.team2077.season2017.autonomous.AutoLeftPos;
import org.usfirst.frc.team2077.season2017.autonomous.AutoRightPos;
import org.usfirst.frc.team2077.season2017.subsystems.BallCollector;
import org.usfirst.frc.team2077.season2017.subsystems.Climber;
import org.usfirst.frc.team2077.season2017.subsystems.Drive;
import org.usfirst.frc.team2077.season2017.subsystems.Eclipse;
import org.usfirst.frc.team2077.season2017.subsystems.GearSetter;
import org.usfirst.frc.team2077.season2017.subsystems.MecanumDrive;
import org.usfirst.frc.team2077.season2017.subsystems.MecanumHardware;
import org.usfirst.frc.team2077.season2017.subsystems.Vision;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static Robot instance_ = null;

	// Subsystems
	public static Climber climber;
	public static GearSetter gearSetter;
	public static BallCollector ballCollector;
	public static Vision vision_;
	public static Drive drive_;
	
	public static MecanumHardware hardware_;
	public static OperatorInterface operatorInterface_;
	public static Gyro gyro_;

	
    public static PIDController headingPID_;
    double headingStraight_ = 0;
    double headingCorrection_ = 0;
    
    public static double P,I,F,speedMultipler, Ramp;
    public static boolean isTeleop; 
    SendableChooser autoChooser; 
    Command autoCommand;
	
    
    public Robot() {
		if (instance_ != null) {
			throw new RuntimeException("ERROR:" + this.getClass().getName() + ".instance_ already initialized.");
		}
		instance_ = this;
	}

	@Override
	public void robotInit() {

		Preferences prefs = Preferences.getInstance();

		if(!prefs.containsKey(RobotMap.ADVANCE_LENGTH_MULTIPLIER_KEY))
			prefs.putDouble(RobotMap.ADVANCE_LENGTH_MULTIPLIER_KEY, RobotMap.ADVANCE_LENGTH_MULTIPLIER_DEFAULT);
		if(!prefs.containsKey(RobotMap.ROTATION_MULTIPLIER_KEY))
			prefs.putDouble(RobotMap.ROTATION_MULTIPLIER_KEY, RobotMap.ROTATION_MULTIPLIER_DEFAULT);
		if(!prefs.containsKey(RobotMap.PLANE_TO_PEG_DISTANCE_KEY))
			prefs.putDouble(RobotMap.PLANE_TO_PEG_DISTANCE_KEY, RobotMap.PLANE_TO_PEG_DISTANCE_DEFAULT);
		if(!prefs.containsKey(RobotMap.SLOW_CLIMBER_TIME_KEY))
			prefs.putDouble(RobotMap.SLOW_CLIMBER_TIME_KEY, RobotMap.SLOW_CLIMBER_TIME_DEFAULT);

		//hardware_ = new PizzaBot();
		hardware_ = new Eclipse();
		
		climber = new Climber();
		gearSetter = new GearSetter();
		ballCollector = new BallCollector();
		vision_ = new Vision("10.20.77.14");
		drive_ = new MecanumDrive(hardware_);
		
		gyro_ = hardware_.getAngleSensor().gyro_;
		operatorInterface_ = new OperatorInterface();
    		
    		Robot.gearSetter.gearSetterEnc.reset();
        	SmartDashboard.putDouble("Gear Setter", Robot.gearSetter.gearSetterEnc.getDistance());
        	
    		isTeleop = isOperatorControl(); 
	
    		Robot.gearSetter.gearSetterEnc.reset();
        	SmartDashboard.putDouble("Gear Setter", Robot.gearSetter.gearSetterEnc.getDistance());
        	
    		autoChooser = new SendableChooser();
    		
    		autoChooser.addDefault("Don't Run Autonomous", null);
    		autoChooser.addObject("Left Position", new AutoLeftPos());
    		autoChooser.addObject("Center Position", new AutoCenterPos());
    		autoChooser.addObject("Right Position", new AutoRightPos());
    		
    		SmartDashboard.putData("Autonomus", autoChooser);
    		
			//Robot.driveTrain.robotDrive.setInvertedMotor(MotorType.kFrontRight, true);
	    	//Robot.driveTrain.robotDrive.setInvertedMotor(MotorType.kRearRight, true);    				
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		vision_.startGearLiftTracker(15);
		gyro_.reset();
		autoCommand = (Command) autoChooser.getSelected();
		if(autoCommand != null)
		{
			autoCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();	
	}
	
	public void teleopInit(){
		vision_.startGearLiftTracker(15); // TODO: find out if disabledInit gets called between auto and teleop.
		//vision_.stopGearLiftTracker();
		if (autoCommand != null) {
			autoCommand.cancel();
		}
	}
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		
	}
	
	@Override
	public void disabledInit()
	{
		vision_.stopGearLiftTracker();
	}
}

