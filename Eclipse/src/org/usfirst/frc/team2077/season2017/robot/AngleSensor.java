package org.usfirst.frc.team2077.season2017.robot;

import java.lang.reflect.Field;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class AngleSensor {

	// heading angle measurement devices, we expect at least one to be available
	public final AHRS navX_; // preferred, use if available
	public final ADXRS450_Gyro adxGyro_; // use if no NavX
	public final AnalogGyro analogGyro_; // fallback, use if no other choice

	// interface wrappers for preferred angle measurement device
	public final PIDSource headingPIDSource_;
	public final Gyro gyro_;

	public AngleSensor() {

		AHRS navX = new AHRS(SPI.Port.kMXP, (byte)200);
		navX.setPIDSourceType(PIDSourceType.kDisplacement);
		if ( ( RobotMap.ROBOT_PLATFORM == RobotMap.RobotPlatform.RP_PIZZA ) && !navX.isConnected() ) {
		//if (!navX.isConnected()) { // no NavX hardware
			System.out.println("NavX not detected.");
			navX = null;
		}
		navX_ = navX;
		System.out.println("NavX:" + navX_);

		ADXRS450_Gyro adxGyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
		adxGyro.setPIDSourceType(PIDSourceType.kDisplacement);
		try { // no official way to detect ADXRS450_Gyro initialization failure, so try to do it the sneaky way
			Field m_spi = ADXRS450_Gyro.class.getDeclaredField("m_spi");
			m_spi.setAccessible(true);
			if (m_spi.get(adxGyro) == null) { // this means the ADXRS450_Gyro failed to initialize, e.g. hardware not present
				System.out.println("ADXRS450 gyro not detected.");
				adxGyro = null;
			}
		} catch (Exception e) {
			System.out.println("Warning: ADXRS450_Gyro error check failed, object may or may not be usable.");
		}
		adxGyro_ = adxGyro;
		System.out.println("ADXRS450 Gyro:" + adxGyro_);

		analogGyro_ = new AnalogGyro(0);
		analogGyro_.setPIDSourceType(PIDSourceType.kDisplacement);
		// no reliable way to detect presence or absence of analog gyro // TODO: runtime disabling of gyro-dependent code elsewhere?
		System.out.println("Analog Gyro:" + analogGyro_);
		
		if (navX_ != null) {
			// NavX PIDSource is the -180 to 180 "yaw"; we want "angle" which is continuous beyond 360, as in the Gyro interface
			headingPIDSource_ = new PIDSource() {
				@Override
				public void setPIDSourceType(PIDSourceType pidSource) {
				}
				@Override
				public PIDSourceType getPIDSourceType() {
					return PIDSourceType.kDisplacement;
				}
				@Override
				public double pidGet() {
					return navX_.getAngle();
				}
			};
			// NavX (this version at least) doesn't implement the Gyro interface
			gyro_ = new Gyro() {
				@Override
				public double getAngle() {
					return navX_.getAngle();
				}
				@Override
				public double getRate() {
					return navX_.getRate();
				}
				@Override
				public void reset() {
					while (navX_.isCalibrating());
					navX_.setAngleAdjustment(navX_.getAngleAdjustment() - navX_.getAngle());
				}
				@Override
				public void calibrate() {
					System.out.println("Gyro.calibrate() is unsupported for NavX.");
				}
				@Override
				public void free() {
					System.out.println("Gyro.free() is unsupported for NavX.");
				}
			};
		}
		else if (adxGyro_ != null) {
			headingPIDSource_ = adxGyro_;
			gyro_ = adxGyro_;
		}
		else {
			System.out.println("Warning: AnalogGyro(0) will be used for heading angle measurement. Code may fail if no gyro hardware is connected to analog input 0.");
			headingPIDSource_ = analogGyro_;
			gyro_ = analogGyro_;
		}
		gyro_.reset();
	}
}
