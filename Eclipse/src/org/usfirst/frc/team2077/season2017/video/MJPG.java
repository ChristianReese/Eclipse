package org.usfirst.frc.team2077.season2017.video;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.InputStream;
import java.io.StringWriter;
import java.net.URL;
import java.util.Arrays;
import java.util.Observable;

import javax.imageio.ImageIO;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;

/** Connect to an MJPG stream via a URL. Tested for various Axis IP cameras. */
public class MJPG extends Observable {
	
	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
	
	private final URL url_;
	private final Runnable reader_;
	private Thread thread_ = null;
	private byte[][] buffer_ = new byte[2][0];
	private int[] bufferSize_ = {0, 0};
	private Object[] bufferLock_ = {new Object(), new Object()};
	private int frameNumber_ = -1;
	private int imageNumber_ = -1;
	private int matNumber_ = -1;
	private BufferedImage image_ = null;
	private Mat mat_ = new Mat();
	
	public MJPG(URL url) {
		url_ = url;
		reader_ = new Runnable() {
			public void run() {
				try {
					InputStream input = url_.openStream();
					StringWriter headerWriter = new StringWriter();
					String header;
					boolean haveJpgHeader = false;
					for (int b = input.read(); b != -1 && Thread.currentThread() == thread_; b = input.read()) {
						if (!haveJpgHeader || b != 255) {
							headerWriter.write(b);
							haveJpgHeader = haveJpgHeader || headerWriter.toString().indexOf("Content-Type: image/jpeg") >= 0;
						}
						else {
							header = headerWriter.toString();
							headerWriter = new StringWriter();
							haveJpgHeader = false;

							// read content length from header
							int indexOfContentLength = header.indexOf("Content-Length:"); // CASE?
							int valueStartPos = indexOfContentLength + "Content-Length:".length();
							int indexOfEOL = header.indexOf('\n', indexOfContentLength);
							String lengthValStr = header.substring(valueStartPos, indexOfEOL).trim();
							int contentLength = Integer.parseInt(lengthValStr);
							
							// read bytes into next buffer
							int frameNumber = frameNumber_ + 1;
							int i = frameNumber % 2;
							synchronized(bufferLock_[i]) {
								if (buffer_[i].length < contentLength) {
									buffer_[i] = new byte[contentLength * 3 / 2];
								}
								buffer_[i][0] = (byte)b;
								for (bufferSize_[i] = 1; bufferSize_[i] < contentLength;) {
									bufferSize_[i] += input.read(buffer_[i], bufferSize_[i], contentLength - bufferSize_[i]);
								}
							}
							frameNumber_ = frameNumber;
							//System.out.println("FRAME:" + frameNumber_);
							setChanged();
							notifyObservers();
						}
					}
				}
				catch (Exception ex) {
					ex.printStackTrace();
				}
			}
		};
	}

	public final void start() {
		thread_ = new Thread(reader_, MJPG.class.getName()+ "@" + url_);
		thread_.setDaemon(true);
		thread_.start();
		//System.out.println("STARTED:" + thread_.getName());
		// TODO: The following hangs if the camera feed is unavailable - MUST FIX.
		//while(!hasChanged()); // don't return until an image is available to guarantee getImage/getMat will always work
	}

	public final void stop() {
		thread_ = null;
	}

	/** Get the latest image (unsupported on RoboRio). */
	public BufferedImage getImage() {
		int frameNumber = frameNumber_;
		if (imageNumber_ < frameNumber) {
			int i = frameNumber % 2;
			synchronized(bufferLock_[i]) {
				try {
					imageNumber_ = frameNumber;
					image_ = ImageIO.read(ImageIO.createImageInputStream(new ByteArrayInputStream(buffer_[i], 0, bufferSize_[i])));
				}
				catch (Exception ex) {
					ex.printStackTrace();
				}
			}
			processImage(image_);
		}
		return image_;
	}

	/** Get the latest image. */
	public Mat getMat() {
		int frameNumber = frameNumber_;
		if (matNumber_ < frameNumber) {
			int i = frameNumber % 2;
			synchronized(bufferLock_[i]) {
				matNumber_ = frameNumber;
				Mat jpg = new MatOfByte(Arrays.copyOf(buffer_[i], bufferSize_[i]));
				mat_.release();
				mat_ = Imgcodecs.imdecode(jpg, Imgcodecs.CV_LOAD_IMAGE_COLOR);
				jpg.release();
			}
			processImage(mat_);
		}
		return mat_;
	}

	/** In-place processing of the image pixel data. */
	protected void processImage(BufferedImage image) {
	}

	/** In-place processing of the image pixel data. */
	protected void processImage(Mat mat) {
	}
}
