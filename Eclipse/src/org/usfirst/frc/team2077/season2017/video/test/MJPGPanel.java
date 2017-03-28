package org.usfirst.frc.team2077.season2017.video.test;

import java.awt.*;
import java.awt.image.*;
import java.net.*;
import javax.swing.*;

import org.usfirst.frc.team2077.season2017.video.MJPG;

import java.util.*;

/** Test code for MJPG. Will not run on the robot. */
public class MJPGPanel extends JPanel {

	private final MJPG mjpg_;
	private BufferedImage image_;

	public MJPGPanel(URL url) {
		//mjpg_ = new OpenCVSandBox(url);
		mjpg_ = new MJPG(url);
		mjpg_.addObserver(new Observer() {
			@Override
			public void update(Observable o, Object arg) {
				image_ = mjpg_.getImage();
				repaint();
			}
		});
		mjpg_.start();
		image_ = mjpg_.getImage();
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
			throw new RuntimeException("Try \"java " + MJPGPanel.class.getName() + " http://<axis_camera_ip>/mjpg/video.mjpg\".");
		}
		JFrame frame = new JFrame(argv[0]);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		JPanel content = new JPanel(new GridLayout(0, Math.min(argv.length, 2)));
		frame.setContentPane(content);
		for (String arg : argv) {
			content.add(new MJPGPanel(new URL(arg)));
		}
		frame.pack();
		frame.setVisible(true);
	}
}
