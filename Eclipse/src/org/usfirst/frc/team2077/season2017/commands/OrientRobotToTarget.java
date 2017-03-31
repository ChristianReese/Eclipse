package org.usfirst.frc.team2077.season2017.commands;

import org.usfirst.frc.team2077.season2017.commands.MoveRelative;
import org.usfirst.frc.team2077.season2017.math.CompassVector;
import org.usfirst.frc.team2077.season2017.robot.Robot;
import org.usfirst.frc.team2077.season2017.robot.RobotMap;
import org.usfirst.frc.team2077.season2017.subsystems.GearSetter;
import org.usfirst.frc.team2077.season2017.vision.measurement.TargetTransform;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;

public class OrientRobotToTarget extends Command {

	private TargetTransform previousTargetTransform = null;
	private TargetTransform currentTargetTransform = null;

	private PIDController dist_pid;
	private PIDController t2c_pid;

    public OrientRobotToTarget() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drive_);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	Preferences prefs = Preferences.getInstance();
    	
    	//Robot.vision_.setTargetSnapshot(null);
    	
    	//previousTargetTransform = Robot.vision_.getLatestGearLiftTargetTransform();
    	
		dist_pid = new PIDController(
				prefs.getDouble(RobotMap.DIST_P_KEY, RobotMap.DIST_DEFAULT_P), // P
				prefs.getDouble(RobotMap.DIST_I_KEY, RobotMap.DIST_DEFAULT_I), // I
				prefs.getDouble(RobotMap.DIST_D_KEY, RobotMap.DIST_DEFAULT_D), // D
				prefs.getDouble(RobotMap.DIST_F_KEY, RobotMap.DIST_DEFAULT_F), // F
					
					new PIDSource() {
						@Override
						public void setPIDSourceType(PIDSourceType pidSource) {
						}
						@Override
						public PIDSourceType getPIDSourceType() {
							return PIDSourceType.kDisplacement;
						}
						@Override
						public double pidGet() {
							return 0.0;
						}
					},
					new PIDOutput() { // dummy output, PID will be read directly
						@Override
						public void pidWrite(double value) {
						}
					}
				);

		t2c_pid = new PIDController(
				prefs.getDouble(RobotMap.T2C_P_KEY, RobotMap.T2C_DEFAULT_P), // P
				prefs.getDouble(RobotMap.T2C_I_KEY, RobotMap.T2C_DEFAULT_I), // I
				prefs.getDouble(RobotMap.T2C_D_KEY, RobotMap.T2C_DEFAULT_D), // D
				prefs.getDouble(RobotMap.T2C_F_KEY, RobotMap.T2C_DEFAULT_F), // F
					
					new PIDSource() {
						@Override
						public void setPIDSourceType(PIDSourceType pidSource) {
						}
						@Override
						public PIDSourceType getPIDSourceType() {
							return PIDSourceType.kDisplacement;
						}
						@Override
						public double pidGet() {
							return 0.0;
						}
					},
					new PIDOutput() { // dummy output, PID will be read directly
						@Override
						public void pidWrite(double value) {
						}
					}
				);
		
		Robot.drive_.setHeadingPIDValues(
				prefs.getDouble(RobotMap.HEADING_P_KEY, RobotMap.HEADING_DEFAULT_P), // P
				prefs.getDouble(RobotMap.HEADING_I_KEY, RobotMap.HEADING_DEFAULT_I), // I
				prefs.getDouble(RobotMap.HEADING_D_KEY, RobotMap.HEADING_DEFAULT_D), // D
				prefs.getDouble(RobotMap.HEADING_F_KEY, RobotMap.HEADING_DEFAULT_F) // F
				);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	final double MAX_TRANSLATE_SPEED = 70.0;
    	
    	CompassVector translation = null;
    	
    	TargetTransform newTT;
    	
    	previousTargetTransform = currentTargetTransform;
    	currentTargetTransform = Robot.vision_.getLatestGearLiftTargetTransform();
    	
    	newTT = getNewImageFound();
    	
    	if (newTT != null){
    		Robot.drive_.setHeading(Robot.gyro_.getAngle() + newTT.getRobotAngleToTarget());
    		// todo: compute ETA
    	}
    	
    	if ( ( dist_pid != null ) && ( t2c_pid != null ) )
    	{
    		if ( !dist_pid.isEnabled() ) 
    		{
    			dist_pid.enable();
    		}
    		
    		if ( !t2c_pid.isEnabled() ) 
    		{
    			t2c_pid.enable();
    		}
    	}
    	
    	if ( ( currentTargetTransform != null ) && ( dist_pid != null ) && ( t2c_pid != null ) )
    	{
    		dist_pid.setSetpoint( currentTargetTransform.getRobotToTargetDistance() );
    		t2c_pid.setSetpoint( currentTargetTransform.getTargetAngleToRobot() );

    		translation = new CompassVector( 
    				Math.max( -MAX_TRANSLATE_SPEED, 
    						Math.min( MAX_TRANSLATE_SPEED, 
    								t2c_pid.get() * MAX_TRANSLATE_SPEED ) ), 
    				Math.max( -MAX_TRANSLATE_SPEED, 
    						Math.min( MAX_TRANSLATE_SPEED, 
    								dist_pid.get() * MAX_TRANSLATE_SPEED ) ) );
    	}
    	else
    	{
    		translation = new CompassVector( 0.0, 0.0 );
    	}
    	
    	if ( translation != null )
    	{
	    	//Robot.drive_.setRelativeRotationVelocity(0);
	    	
	    	//Robot.drive_.setRelativeTranslationVelocity(0.2, 0.0);
	    	
	    	Robot.drive_.setVelocity(translation.north_, translation.east_, 0.0);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
// pin detect    	boolean superResult = super.isFinished() || (!GearSetter.gearFlapLeft.get() && !GearSetter.gearFlapRight.get());
// check eta
    	
    	
    	
    	
    	return false;//Math.abs(Robot.gyro_.getAngle() - Robot.drive_.getHeading()) < 0.5; 
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    	if ( dist_pid != null )
    	{
    		dist_pid.disable();
    	}
    	
    	if ( t2c_pid != null )
    	{
    		t2c_pid.disable();
    	}
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
    
    protected TargetTransform getNewImageFound() {
    	
    	if ( currentTargetTransform == null )
    	{
    		return null;
    	}
    	
    	if ( !currentTargetTransform.targetFound() )
    	{
    		return null;
    	}
    	
    	if ( previousTargetTransform == null )
    	{
    		//Robot.vision_.takeTargetSnapshot( tt );
			return currentTargetTransform;
    	}
    	
    	if ( !currentTargetTransform.equals(previousTargetTransform) )
    	{
			//Robot.vision_.takeTargetSnapshot( tt );
			return currentTargetTransform;
    	}
    	
    	return null;
    }
    
    protected double forwardSpeedMultiplier()
    {
    	return 1.0 - Math.min(1.0, Math.max(0.0, Math.abs(Robot.gyro_.getAngle() - Robot.drive_.getHeading()) / 2.0));
    }
}
