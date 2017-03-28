package org.usfirst.frc.team2077.season2017.vision.measurement;

import org.opencv.core.Mat;

public class TargetTransform 
{
	private double targetAngleToRobot;
	private double robotAngleToTarget;
	private double robotToTargetDistance;

	private double acquisitionTime;
	private double timestamp;
	private Mat debugMat;
	
	private boolean targetFound;
	
	/**
	 * @param targetAngleToRobot The angle CW from target-forward to the camera, in degrees.
	 * @param robotAngleToTarget The angle CW from camera-forward to the target, in degrees.
	 * @param robotToTargetDistance The distance between the robot and target, in inches.
	 */
	public TargetTransform(double targetAngleToRobot, double robotAngleToTarget, double robotToTargetDistance, 
			double acquisitionTime, double timestamp, boolean targetFound, Mat debugMat) {
		this.targetAngleToRobot = targetAngleToRobot;
		this.robotAngleToTarget = robotAngleToTarget;
		this.robotToTargetDistance = robotToTargetDistance;
		this.acquisitionTime = acquisitionTime;
		this.timestamp = timestamp;
		this.debugMat = debugMat;
		this.targetFound = targetFound;
	}

	public TargetTransform(double targetAngleToRobot, double robotAngleToTarget, double robotToTargetDistance, 
			double acquisitionTime, double timestamp, boolean targetFound) {
		this(targetAngleToRobot, robotAngleToTarget, robotToTargetDistance, acquisitionTime, timestamp, targetFound, null);
	}

	/**
	 * @return The angle CW from target-forward to the camera, in degrees.
	 */
	public double getTargetAngleToRobot() {
		return targetAngleToRobot;
	}

	/**
	 * @return The angle CW from camera-forward to the target, in degrees.
	 */
	public double getRobotAngleToTarget() {
		return robotAngleToTarget;
	}

	/**
	 * @return The distance between the robot and target, in inches.
	 */
	public double getRobotToTargetDistance() {
		return robotToTargetDistance;
	}

	/**
	 * @return the time it took to acquire this frame, in seconds.
	 */
	public double getAcquisitionTime() {
		return acquisitionTime;
	}

	/**
	 * @return the timestamp of the calculation, in seconds.
	 */
	public double getTimestamp() {
		return timestamp;
	}
	
	/** Optional, may return null. */
	public Mat getDebugMat() {
		return debugMat;
	}
	
	public boolean targetFound() {
		return targetFound;
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return "TargetTransform [targetAngleToRobot=" + targetAngleToRobot + ", robotAngleToTarget="
				+ robotAngleToTarget + ", robotToTargetDistance=" + robotToTargetDistance + ", acquisitionTime="
				+ acquisitionTime + ", timestamp=" + timestamp + ", debugMat=" + debugMat + "]";
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		long temp;
		temp = Double.doubleToLongBits(acquisitionTime);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		temp = Double.doubleToLongBits(robotAngleToTarget);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		temp = Double.doubleToLongBits(robotToTargetDistance);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		temp = Double.doubleToLongBits(targetAngleToRobot);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		result = prime * result + (targetFound ? 1231 : 1237);
		temp = Double.doubleToLongBits(timestamp);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		TargetTransform other = (TargetTransform) obj;
		if (Double.doubleToLongBits(acquisitionTime) != Double.doubleToLongBits(other.acquisitionTime))
			return false;
		if (Double.doubleToLongBits(robotAngleToTarget) != Double.doubleToLongBits(other.robotAngleToTarget))
			return false;
		if (Double.doubleToLongBits(robotToTargetDistance) != Double.doubleToLongBits(other.robotToTargetDistance))
			return false;
		if (Double.doubleToLongBits(targetAngleToRobot) != Double.doubleToLongBits(other.targetAngleToRobot))
			return false;
		if (targetFound != other.targetFound)
			return false;
		if (Double.doubleToLongBits(timestamp) != Double.doubleToLongBits(other.timestamp))
			return false;
		return true;
	}
	
}
