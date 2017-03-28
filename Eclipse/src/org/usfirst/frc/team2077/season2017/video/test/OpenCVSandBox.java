package org.usfirst.frc.team2077.season2017.video.test;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.net.URL;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team2077.season2017.video.MJPG;

/** Various OpenCV tests and experiments. */
public class OpenCVSandBox extends MJPG {

	private Mat template_ = null;;

	public OpenCVSandBox(URL url) {
		super(url);
	}

	@Override
	protected void processImage(BufferedImage image) {

		// get the raw image buffer into OpenCV Mat
		byte[] imageBytes = ((DataBufferByte)image.getRaster().getDataBuffer()).getData();
		Mat mat = new Mat(image.getHeight(), image.getWidth(), CvType.CV_8UC3);
		mat.put(0, 0, imageBytes);

		processImage(mat); // OpenCV test code 

		// write the processed image back to the BufferedImage
		mat.get(0,  0, imageBytes);
		mat.release();
	}

	@Override
	protected void processImage(Mat mat) {

		processImage1(mat); // OpenCV test code 
	}

	public void track(Mat mat, int x, int y, int width, int height) {
		template_ = mat.submat(y, y+height, x, x+width);
	}

	protected void processImage7(Mat mat) {

		if (template_ == null) {
			track(mat, 300, 0, 40, 480);
		}

		if (template_ != null) {
			Mat match = new Mat();
			Imgproc.matchTemplate(mat, template_, match, Imgproc.TM_SQDIFF_NORMED);

			Core.MinMaxLocResult mm = Core.minMaxLoc(match);
			//System.out.println("" +  mm.minLoc.x + " " +  mm.minLoc.y);
			Imgproc.rectangle(mat, mm.minLoc, new Point(mm.minLoc.x+40, mm.minLoc.y+40), new Scalar(0, 255, 0), 4);


			//Imgproc.cvtColor(match, mat, Imgproc.COLOR_GRAY2BGR);
		}
	}

	protected void processImage6(Mat mat) {

		Mat hsv = new Mat();
		Imgproc.cvtColor(mat, hsv, Imgproc.COLOR_BGR2HSV);
		List<Mat> h_s_v = new LinkedList<>(Arrays.asList(new Mat[]{new Mat(), new Mat(), new Mat()}));
		Core.split(hsv,  h_s_v);
		Mat h = h_s_v.get(0);
		Imgproc.equalizeHist(h, h);

		//Mat gray = new Mat();
		//Imgproc.cvtColor(mat, gray, Imgproc.COLOR_BGR2GRAY);


		Mat x = new Mat();
		Core.multiply(h,  h_s_v.get(2),  x, 1./180);
		Core.multiply(x,  h_s_v.get(1),  x, 1./255);
		Imgproc.equalizeHist(x, x);
		Imgproc.GaussianBlur(x, x, new Size(21, 21), 0);

		Imgproc.Canny(x, x, 115, 120);
		Imgproc.GaussianBlur(x, x, new Size(5, 5), 0);

		Imgproc.cvtColor(x, mat, Imgproc.COLOR_GRAY2BGR);


		Mat lines = new Mat();
		Imgproc.HoughLines(x, lines, 1, Math.PI/180., 50);
		for(int r = 0; r < lines.height() && r < 8; r++) {
			for(int c = 0; c < lines.width(); c++) {
				double[] line = lines.get(r, c);
				double rho = line[0];
				double theta = line[1];
				double a = Math.cos(theta);
				double b = Math.sin(theta);
				int x0 = (int)Math.round(a*rho);
				int y0 = (int)Math.round(b*rho);
				int x1 = (int)Math.round(x0 + 1000*(-b));
				int y1 = (int)Math.round(y0 + 1000*(a));
				int x2 = (int)Math.round(x0 - 1000*(-b));
				int y2 = (int)Math.round(y0 - 1000*(a));
				Imgproc.line(mat, new Point(x1,y1), new Point(x2,y2), new Scalar(0, 255, 0));
				//System.out.println("R:" + r + " C:" + c + " LINE:" + line[0] + "," + line[1]);
			}
		}


	}

	protected void processImage5(Mat mat) {

		Mat hsv = new Mat();
		Imgproc.cvtColor(mat, hsv, Imgproc.COLOR_BGR2HSV);
		Mat mask = new Mat();
		Imgproc.GaussianBlur(mat, mat, new Size(21, 21), 0);

		//Core.inRange(hsv, new Scalar(0, 0, 0), new Scalar(9, 255, 255), mask); // red
		//Core.inRange(hsv, new Scalar(10, 0, 0), new Scalar(19, 255, 255), mask); // orange
		//Core.inRange(hsv, new Scalar(20, 0, 0), new Scalar(29, 255, 255), mask); // yellow
		//Core.inRange(hsv, new Scalar(30, 0, 0), new Scalar(39, 255, 255), mask); // 
		//Core.inRange(hsv, new Scalar(40, 0, 0), new Scalar(49, 255, 255), mask); // 
		//Core.inRange(hsv, new Scalar(50, 0, 0), new Scalar(59, 255, 255), mask); // green
		//Core.inRange(hsv, new Scalar(60, 0, 0), new Scalar(69, 255, 255), mask); // 
		//Core.inRange(hsv, new Scalar(70, 0, 0), new Scalar(79, 255, 255), mask); // 
		//Core.inRange(hsv, new Scalar(80, 0, 0), new Scalar(89, 255, 255), mask); // 
		//Core.inRange(hsv, new Scalar(90, 0, 0), new Scalar(99, 255, 255), mask); // 
		//Core.inRange(hsv, new Scalar(100, 0, 0), new Scalar(109, 255, 255), mask); // blue
		//Core.inRange(hsv, new Scalar(110, 0, 0), new Scalar(119, 255, 255), mask); //
		//Core.inRange(hsv, new Scalar(120, 0, 0), new Scalar(129, 255, 255), mask); // 
		//Core.inRange(hsv, new Scalar(130, 0, 0), new Scalar(139, 255, 255), mask); // 
		//Core.inRange(hsv, new Scalar(140, 0, 0), new Scalar(149, 255, 255), mask); // 
		//Core.inRange(hsv, new Scalar(150, 0, 0), new Scalar(159, 255, 255), mask); // 
		//Core.inRange(hsv, new Scalar(160, 0, 0), new Scalar(169, 255, 255), mask); // none
		//Core.inRange(hsv, new Scalar(170, 0, 0), new Scalar(179, 255, 255), mask); // purple/pink

		Imgproc.cvtColor(hsv, mat, Imgproc.COLOR_HSV2BGR);
	}

	protected void processImage4(Mat mat) {

		Imgproc.GaussianBlur(mat, mat, new Size(5, 5), 0);

		Mat gray = new Mat();
		Imgproc.cvtColor(mat, gray, Imgproc.COLOR_BGR2GRAY);
		Imgproc.equalizeHist(gray, gray);
		Imgproc.threshold(gray, gray, 190, 255, Imgproc.THRESH_BINARY);
		Mat edges = new Mat();
		Imgproc.Canny(gray, edges, 115, 120);
		Mat lines = new Mat();
		Imgproc.HoughLinesP(edges, lines, 1, Math.PI/180., 64, 40, 10);
		//Imgproc.cvtColor(edges, mat, Imgproc.COLOR_GRAY2BGR);

		for(int r = 0; r < lines.height(); r++) {
			for(int c = 0; c < lines.width(); c++) {
				double[] line = lines.get(r, c);
				int x1 = (int)Math.round(line[0]);
				int y1 = (int)Math.round(line[1]);
				int x2 = (int)Math.round(line[2]);
				int y2 = (int)Math.round(line[3]);
				Imgproc.line(mat, new Point(x1,y1), new Point(x2,y2), new Scalar(255, 255, 255));

				//System.out.println("R:" + r + " C:" + c + " LINE:" + line[0] + "," + line[1]);
			}
		}
	}

	protected void processImage3(Mat mat) {

		Imgproc.GaussianBlur(mat, mat, new Size(5, 5), 0);

		Mat gray = new Mat();
		Imgproc.cvtColor(mat, gray, Imgproc.COLOR_BGR2GRAY);
		Imgproc.equalizeHist(gray, gray);
		//Imgproc.threshold(gray, gray, 190, 255, Imgproc.THRESH_BINARY);
		Mat edges = new Mat();
		Imgproc.Canny(gray, edges, 115, 120);
		Mat lines = new Mat();
		Imgproc.HoughLines(edges, lines, 1, Math.PI/180., 150);
		Imgproc.cvtColor(edges, mat, Imgproc.COLOR_GRAY2BGR);

		for(int r = 0; r < lines.height(); r++) {
			for(int c = 0; c < lines.width(); c++) {
				double[] line = lines.get(r, c);
				double rho = line[0];
				double theta = line[1];
				double a = Math.cos(theta);
				double b = Math.sin(theta);
				int x0 = (int)Math.round(a*rho);
				int y0 = (int)Math.round(b*rho);
				int x1 = (int)Math.round(x0 + 1000*(-b));
				int y1 = (int)Math.round(y0 + 1000*(a));
				int x2 = (int)Math.round(x0 - 1000*(-b));
				int y2 = (int)Math.round(y0 - 1000*(a));
				Imgproc.line(mat, new Point(x1,y1), new Point(x2,y2), new Scalar(0, 255, 0));

				//System.out.println("R:" + r + " C:" + c + " LINE:" + line[0] + "," + line[1]);
			}
		}
	}

	protected void processImage3a(Mat mat) {

		Imgproc.GaussianBlur(mat, mat, new Size(5, 5), 0);

		Mat gray = new Mat();
		Imgproc.cvtColor(mat, gray, Imgproc.COLOR_BGR2GRAY);
		Imgproc.equalizeHist(gray, gray);
		Imgproc.threshold(gray, gray, 190, 255, Imgproc.THRESH_BINARY);
		Mat edges = new Mat();
		Imgproc.Canny(gray, edges, 115, 120);
		Mat lines = new Mat();
		Imgproc.HoughLines(edges, lines, 1, Math.PI/180., 96);
		//Imgproc.cvtColor(edges, mat, Imgproc.COLOR_GRAY2BGR);

		for(int r = 0; r < lines.height(); r++) {
			for(int c = 0; c < lines.width(); c++) {
				double[] line = lines.get(r, c);
				double rho = line[0];
				double theta = line[1];
				double a = Math.cos(theta);
				double b = Math.sin(theta);
				int x0 = (int)Math.round(a*rho);
				int y0 = (int)Math.round(b*rho);
				int x1 = (int)Math.round(x0 + 1000*(-b));
				int y1 = (int)Math.round(y0 + 1000*(a));
				int x2 = (int)Math.round(x0 - 1000*(-b));
				int y2 = (int)Math.round(y0 - 1000*(a));
				Imgproc.line(mat, new Point(x1,y1), new Point(x2,y2), new Scalar(0, 255, 0));

				//System.out.println("R:" + r + " C:" + c + " LINE:" + line[0] + "," + line[1]);
			}
		}
	}

	protected void processImage2(Mat mat) {

		Imgproc.GaussianBlur(mat, mat, new Size(5, 5), 0);

		Mat gray = new Mat();
		Imgproc.cvtColor(mat, gray, Imgproc.COLOR_BGR2GRAY);
		Imgproc.equalizeHist(gray, gray);
		Imgproc.threshold(gray, gray, 190, 255, Imgproc.THRESH_BINARY);

		Mat edges = new Mat();
		Imgproc.Canny(gray, edges, 115, 120);
		Imgproc.cvtColor(edges, mat, Imgproc.COLOR_GRAY2BGR);

		//Imgproc.GaussianBlur(mat, mat, new Size(5, 5), 0);

		//Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2YUV);
	}

	protected void processImage1(Mat mat) {

		Imgproc.drawMarker(mat, new Point(100,100), new Scalar(0, 0, 255));

		Imgproc.blur(mat, mat, new Size(3, 3));

		Mat gray = new Mat();
		Imgproc.cvtColor(mat, gray, Imgproc.COLOR_BGR2GRAY);
		Imgproc.equalizeHist(gray, gray);
		Imgproc.threshold(gray, gray, 220, 255, Imgproc.THRESH_BINARY);

		Mat edges = new Mat();
		Imgproc.Canny(gray, edges, 115, 120);
		Imgproc.cvtColor(edges, mat, Imgproc.COLOR_GRAY2RGB);

		Imgproc.blur(mat, mat, new Size(5, 5));
	}
}
