package org.usfirst.frc.team2077.season2017.vision.test;

import java.awt.*;
import java.awt.image.*;
import java.net.*;
import javax.swing.*;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team2077.season2017.video.MJPG;
import org.usfirst.frc.team2077.season2017.vision.measurement.TargetTransform;
import org.usfirst.frc.team2077.season2017.vision.trackers.AxisCameraVisionTracker;
import org.usfirst.frc.team2077.season2017.vision.trackers.VisionTracker;
import org.usfirst.frc.team2077.season2017.vision.trackers.VisionTrackers;

import java.util.*;

/** Test code for VisionTracker. Will not run on the robot. */
public class VisionTrackerPanel extends JPanel {

	private final VisionTracker visionTracker_;
	private Mat mat_;
	private BufferedImage image_;

	public VisionTrackerPanel(String cameraIP) {
		//mjpg_ = new OpenCVSandBox(url);
		visionTracker_ = VisionTrackers.createGearLiftTracker(cameraIP);
		((AxisCameraVisionTracker)visionTracker_).addObserver(new Observer() {
			@Override
			public void update(Observable o, Object arg) {
				TargetTransform targetTransform = visionTracker_.getLatestTargetTransform();
				//System.out.println(targetTransform);
				mat_ = targetTransform.getDebugMat();
				if (mat_ != null) {
					image_ = new BufferedImage(mat_.cols(), mat_.rows(), BufferedImage.TYPE_3BYTE_BGR);
					byte[] imageBytes = ((DataBufferByte)image_.getRaster().getDataBuffer()).getData();
					mat_.get(0,  0, imageBytes);
				}
				repaint();
			}
		});
		visionTracker_.start(5);
		image_ = new BufferedImage(640, 480, BufferedImage.TYPE_3BYTE_BGR);
	}

	@Override
	public Dimension getPreferredSize() {
		Insets insets = getInsets();
		return new Dimension(image_.getWidth() + insets.left + insets.right, image_.getHeight() + insets.top + insets.bottom);
	}

	@Override
	public void paintComponent(Graphics graphics) {
		Insets insets = getInsets();
		((Graphics2D) graphics).drawImage(image_, null, insets.left, insets.right);
	}

	public static void main(String[] argv) throws Exception {
		if (argv.length < 1) {
			throw new RuntimeException("Try \"java " + VisionTrackerPanel.class.getName() + " http://<axis_camera_ip>/mjpg/video.mjpg\".");
		}
		JFrame frame = new JFrame(argv[0]);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		JPanel content = new JPanel(new GridLayout(0, Math.min(argv.length, 2)));
		frame.setContentPane(content);
		for (String arg : argv) {
			content.add(new VisionTrackerPanel(arg));
		}
		frame.pack();
		frame.setVisible(true);
	}
}
