package org.usfirst.frc.team2077.season2017.vision.trackers;

import java.util.concurrent.TimeUnit;

import org.usfirst.frc.team2077.season2017.vision.measurement.TargetTransform;

public interface VisionTracker 
{
	void start(double framesPerSecond);
	
	void stop();
	
	boolean isRunning();
	
//    /**
//     * @return The current rate of execution of this vision tracker, in frames per second.
//     * 0.0 if {@link #isRunning() isRunning()} is false.
//     */
//	public double getCurrentFPS();
	
//    /**
//     * @return The target rate of execution of this vision tracker, in frames per second, according to what
//     * was set in the last call to {@link #scheduleAtFixedRate(long, long, TimeUnit) scheduleAtFixedRate()}.
//     */
//	public double getTargetFPS();
	
	/**
	 * @return Null if no target transforms have been captured, otherwise the latest target transform obtained 
	 * {@link #getIterationsSinceLastTargetTracked() getIterationsSinceLastTargetTracked()} iterations ago.
	 */
	public TargetTransform getLatestTargetTransform();
	
//	/**
//	 * @return The number of iterations (scheduled executions) since the last target was detected.
//	 * -1 if no targets have been detected yet.
//	 */
//	public long getIterationsSinceLastTargetTracked();
}
