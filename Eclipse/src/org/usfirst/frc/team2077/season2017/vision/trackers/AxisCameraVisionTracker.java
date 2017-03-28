package org.usfirst.frc.team2077.season2017.vision.trackers;

import java.net.MalformedURLException;
import java.net.URL;
import java.util.Observable;
import java.util.Observer;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;

import org.opencv.core.Mat;
import org.usfirst.frc.team2077.season2017.video.MJPG;
import org.usfirst.frc.team2077.season2017.vision.measurement.TargetTransform;

public abstract class AxisCameraVisionTracker extends Observable implements VisionTracker {
	
	private final String cameraIP_;
	private double framesPerSecond_;
	private MJPG mjpg_ = null;
	private AtomicBoolean cameraFrameReady_;
	private Timer timer_;
	private TargetTransform targetTransform_ = null;
	private boolean isRunning_ = false;

	public AxisCameraVisionTracker(String cameraIP) {
		cameraIP_ = cameraIP;
	}

	@Override
	public void start(double framesPerSecond) {
		
		if (mjpg_ == null || framesPerSecond != framesPerSecond_) {
			framesPerSecond_ = framesPerSecond;
			try {
				// TODO: use Axis CGI to set camera FPS just a bit faster than tracker FPS
				mjpg_ = new MJPG(new URL("http://" + cameraIP_ + "/mjpg/video.mjpg"));
			}
			catch (MalformedURLException e) {
				e.printStackTrace();
				return;
			}
		}
		cameraFrameReady_ = new AtomicBoolean(false);
		mjpg_.addObserver(new Observer() {
			@Override
			public void update(Observable o, Object arg) {
				cameraFrameReady_.set(true);
			}
		});
		timer_ = new Timer(AxisCameraVisionTracker.class.getName(), true);
		timer_.scheduleAtFixedRate(
				new TimerTask() {
					public void run() {
						//System.out.println("RUN:" + Thread.currentThread().getName() + " " + cameraFrameReady_);
						if (cameraFrameReady_.compareAndSet(true, false)) {
							targetTransform_ = computeTargetTransform(mjpg_.getMat());
							//System.out.println(targetTransform_.getTimestamp() + ": " + targetTransform_.getRobotAngleToTarget());
							setChanged();
							notifyObservers();
						}
					}
				}, 0L, (long)(1000L/framesPerSecond_));
		mjpg_.start();
		isRunning_ = true;
	}
	
	protected abstract TargetTransform computeTargetTransform(Mat mat);
	
	@Override
	public void stop() {
		if (mjpg_ != null) {
			mjpg_.stop();
		}
		if (timer_ != null) {
			timer_.cancel();
		}
		isRunning_ = false;
	}

	@Override
	public boolean isRunning()
	{
		return isRunning_;
	}

	@Override
	public TargetTransform getLatestTargetTransform() {
		return targetTransform_;
	}
}
