package org.usfirst.frc.team2077.season2017.subsystems;


import org.usfirst.frc.team2077.season2017.robot.AngleSensor;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Preferences;

public class PizzaBot implements MecanumHardware {

    public static Preferences preferences_ = Preferences.getInstance();
    
    private static final double WHEEL_BASE = 17.5;
    private static final double TRACK_WIDTH = 18.0;
    private static final double WHEEL_RADIUS = 3.0;
    private static final double MAXIMUM_SPEED = 70;
    private static final int ENCODER_COUNTS_PER_REVOLUTION = 8192;
    private static final int[] WHEEL_DIRECTIONS = {1, 1 ,-1, -1};
    private static final boolean INVERTING_GEARBOX = false;
    
    private static final double motorKP = preferences_.getDouble("PizzaBot_kPmotor", 0.6);
    private static final double motorKI = preferences_.getDouble("PizzaBot_kImotor", 0.001);
    private static final double motorKD = preferences_.getDouble("PizzaBot_kDmotor", 0.0);
    private static final double motorKF = preferences_.getDouble("PizzaBot_kFmotor", 0.39);

    private static final double angleKP = preferences_.getDouble("PizzaBot_kPangle", 0.01);
    private static final double angleKI = preferences_.getDouble("PizzaBot_kIangle", 0.0);
    private static final double angleKD = preferences_.getDouble("PizzaBot_kDangle", 0.001);
    private static final double angleKF = preferences_.getDouble("PizzaBot_kFangle", 0.0);

    private final CANTalon[] motors_ = new CANTalon[4];
    private final AngleSensor angleSensor_ = new AngleSensor();
    private final PIDController headingPID_;

    public PizzaBot() {
		for (int i = 0; i < 4; i++) {
			motors_[i] = new CANTalon(i+1);
			motors_[i].setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
			motors_[i].reverseSensor(INVERTING_GEARBOX);
			motors_[i].setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
			motors_[i].setPID(motorKP, motorKI, motorKD, motorKF,
					300, // clear I accumulator if err exceeds (encoder units)
					60, // maximum ramp rate (volts/sec)
					0);
			motors_[i].enableBrakeMode(true);
			motors_[i].changeControlMode(CANTalon.TalonControlMode.Speed);
		}
		headingPID_ = new PIDController(
			angleKP, angleKI, angleKD, angleKF,
			angleSensor_.headingPIDSource_,
			new PIDOutput() { // dummy output, PID will be read directly
				@Override
				public void pidWrite(double value) {
				}
			}
		);
	}

	@Override
	public double getTrackWidth() {
		return TRACK_WIDTH;
	}

	@Override
	public double getWheelBase() {
		return WHEEL_BASE;
	}

	@Override
	public double getWheelRadius() {
		return WHEEL_RADIUS;
	}

	@Override
	public double getMaximumSpeed() {
		return MAXIMUM_SPEED;
	}

	@Override
	public int getEncoderCountsPerRevolution() {
		return ENCODER_COUNTS_PER_REVOLUTION;
	}

	@Override
	public int[] getWheelDirections() {
		return WHEEL_DIRECTIONS;
	}

	@Override
	public CANTalon[] getMotorControllers() {
		return motors_;
	}

	@Override
	public AngleSensor getAngleSensor() {
		return angleSensor_;
	}

	@Override
	public PIDController getHeadingController() {
		return headingPID_;
	}
}
