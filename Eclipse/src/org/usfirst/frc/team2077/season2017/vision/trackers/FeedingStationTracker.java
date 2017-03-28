package org.usfirst.frc.team2077.season2017.vision.trackers;

import org.opencv.core.Mat;
import org.usfirst.frc.team2077.season2017.vision.measurement.TargetTransform;

class FeedingStationTracker extends AxisCameraVisionTracker
{

	public FeedingStationTracker(String cameraIP) {
		super(cameraIP);
		// TODO Auto-generated constructor stub
	}

	@Override
	protected TargetTransform computeTargetTransform(Mat mat) {
		// TODO Auto-generated method stub
		return null;
	}
}
