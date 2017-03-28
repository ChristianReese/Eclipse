package org.usfirst.frc.team2077.season2017.vision.trackers;

import java.net.URL;
import java.text.DecimalFormat;
import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team2077.season2017.video.MJPG;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearLiftTargetTracking 
{
	
	static final String AXIS_NAME = "axis-cam-vision";
	static final String AXIS_IP_STR = "10.20.76.14";
	static final String CAM_STREAM_NAME = "Rectangle";
	static final String CAM_SERVER_NAME = "serve_Rectangle";

	static final String SHOW_THRESHOLD_STR = "DB/Button 0";//"Show Threshold";
	static final String MIN_1_VAL_STR = "DB/Slider 0";//"min1Val";
	static final String MAX_1_VAL_STR = "DB/Slider 1";//"max1Val";
	static final String MIN_2_VAL_STR = "DB/Slider 2";//"min2Val";
	static final String MAX_2_VAL_STR = "DB/Slider 3";//"max2Val";
	static final String MIN_3_VAL_STR = "min3Val";
	static final String MAX_3_VAL_STR = "max3Val";

	static final int CAM_WIDTH = 640;
	static final int CAM_HEIGHT = 480;
	
	static final double DASH_SLIDER_MULTIPLIER = 51.0; // Dash value times this = actual 0-255 value.
	
	static final boolean SHOW_THRESHOLD_DEFAULT = true;
	static final double THRESHOLD_DEFAULT_MIN1 = 0.0 / DASH_SLIDER_MULTIPLIER;
	static final double THRESHOLD_DEFAULT_MAX1 = 127.5 / DASH_SLIDER_MULTIPLIER;
	static final double THRESHOLD_DEFAULT_MIN2 = 204.0 / DASH_SLIDER_MULTIPLIER;
	static final double THRESHOLD_DEFAULT_MAX2 = 255.0 / DASH_SLIDER_MULTIPLIER;
	static final double THRESHOLD_DEFAULT_MIN3 = 0.0 / DASH_SLIDER_MULTIPLIER;
	static final double THRESHOLD_DEFAULT_MAX3 = 136.68 / DASH_SLIDER_MULTIPLIER;
	
	static Thread visionThread = null;
	
	
	static double time_ = Timer.getFPGATimestamp();
	static int frameCount_ = 0;
	private static void checkFPS()
	{
		frameCount_++;
		double time = Timer.getFPGATimestamp();
		if (time - time_ > 5) {
			System.out.println(frameCount_/ (time - time_));
			time_ = time;
			frameCount_ = 0;
		}
	}
	
	public static void stop()
	{
		visionThread = null;
	}
	public static void startMJPG()
	{
		System.out.println("~~~~~~~~~~~~~~~GearLiftTargetTracking.start()~~~~~~~~~~~~~~~");
		
		SmartDashboard.putBoolean(SHOW_THRESHOLD_STR, SHOW_THRESHOLD_DEFAULT);
		SmartDashboard.putNumber( MIN_1_VAL_STR, THRESHOLD_DEFAULT_MIN1 );
		SmartDashboard.putNumber( MAX_1_VAL_STR, THRESHOLD_DEFAULT_MAX1 );
		SmartDashboard.putNumber( MIN_2_VAL_STR, THRESHOLD_DEFAULT_MIN2 );
		SmartDashboard.putNumber( MAX_2_VAL_STR, THRESHOLD_DEFAULT_MAX2 );
		SmartDashboard.putNumber( MIN_3_VAL_STR, THRESHOLD_DEFAULT_MIN3 );
		SmartDashboard.putNumber( MAX_3_VAL_STR, THRESHOLD_DEFAULT_MAX3 );
		
		visionThread = new Thread() {
			
			MJPG mjpg_ = null;
			
			public void run()
			{
				try {
					mjpg_ = new MJPG(new URL("http://10.20.76.14/mjpg/video.mjpg"));
					mjpg_.start();

					while ( !Thread.interrupted() && ( visionThread == this ) ) 
					{
						boolean showThreshold = SmartDashboard.getBoolean(SHOW_THRESHOLD_STR, SHOW_THRESHOLD_DEFAULT);
						double min1Val = SmartDashboard.getNumber(MIN_1_VAL_STR, THRESHOLD_DEFAULT_MIN1) * DASH_SLIDER_MULTIPLIER;
						double max1Val = SmartDashboard.getNumber(MAX_1_VAL_STR, THRESHOLD_DEFAULT_MAX1) * DASH_SLIDER_MULTIPLIER;
						double min2Val = SmartDashboard.getNumber(MIN_2_VAL_STR, THRESHOLD_DEFAULT_MIN2) * DASH_SLIDER_MULTIPLIER;
						double max2Val = SmartDashboard.getNumber(MAX_2_VAL_STR, THRESHOLD_DEFAULT_MAX2) * DASH_SLIDER_MULTIPLIER;
						double min3Val = SmartDashboard.getNumber(MIN_3_VAL_STR, THRESHOLD_DEFAULT_MIN3) * DASH_SLIDER_MULTIPLIER;
						double max3Val = SmartDashboard.getNumber(MAX_3_VAL_STR, THRESHOLD_DEFAULT_MAX3) * DASH_SLIDER_MULTIPLIER;

						Mat mat = mjpg_.getMat();
						GearLiftTargetTracking.processMat( mat, CAM_WIDTH, CAM_HEIGHT, 
								min1Val, max1Val, min2Val, max2Val, min3Val, max3Val, showThreshold );
						
						checkFPS();
					}
				}
				catch(Exception ex) {
					ex.printStackTrace();
				}
				
				mjpg_.stop();
			}
		};
		
		visionThread.setDaemon(true);
		
		visionThread.start();
	}
	
	public static void startCS()
	{
		System.out.println("~~~~~~~~~~~~~~~GearLiftTargetTracking.start()~~~~~~~~~~~~~~~");
		
		SmartDashboard.putBoolean(SHOW_THRESHOLD_STR, SHOW_THRESHOLD_DEFAULT);
		SmartDashboard.putNumber( MIN_1_VAL_STR, THRESHOLD_DEFAULT_MIN1 );
		SmartDashboard.putNumber( MAX_1_VAL_STR, THRESHOLD_DEFAULT_MAX1 );
		SmartDashboard.putNumber( MIN_2_VAL_STR, THRESHOLD_DEFAULT_MIN2 );
		SmartDashboard.putNumber( MAX_2_VAL_STR, THRESHOLD_DEFAULT_MAX2 );
		SmartDashboard.putNumber( MIN_3_VAL_STR, THRESHOLD_DEFAULT_MIN3 );
		SmartDashboard.putNumber( MAX_3_VAL_STR, THRESHOLD_DEFAULT_MAX3 );
		
		visionThread = new Thread() {
			
			public void run()
			{
				
				// Get the Axis camera from CameraServer
				AxisCamera camera = CameraServer.getInstance().addAxisCamera(AXIS_NAME, AXIS_IP_STR);
				// Set the resolution
				camera.setResolution(CAM_WIDTH, CAM_HEIGHT);
				
				System.out.println("~~~~~~~~~~~~~~~GearLiftTargetTracking.run() 1~~~~~~~~~~~~~~~");
	
				// Get a CvSink. This will capture Mats from the camera
				CvSink cvSink = CameraServer.getInstance().getVideo();
				System.out.println("~~~~~~~~~~~~~~~GearLiftTargetTracking.run() 2~~~~~~~~~~~~~~~");
				
				// Setup a CvSource. This will send images back to the Dashboard
				//CvSource outputStream = CameraServer.getInstance().putVideo(CAM_STREAM_NAME, CAM_WIDTH, CAM_HEIGHT);
				System.out.println("~~~~~~~~~~~~~~~GearLiftTargetTracking.run() 3~~~~~~~~~~~~~~~");
	
				// Mats are very memory expensive. Lets reuse this Mat.
				Mat mat = new Mat();
	
				// This cannot be 'true'. The program will never exit if it is. This
				// lets the robot stop this thread when restarting robot code or
				// deploying.
				while ( !Thread.interrupted() && ( visionThread == this ) ) 
				{
					boolean showThreshold = SmartDashboard.getBoolean(SHOW_THRESHOLD_STR, SHOW_THRESHOLD_DEFAULT);
					double min1Val = SmartDashboard.getNumber(MIN_1_VAL_STR, THRESHOLD_DEFAULT_MIN1) * DASH_SLIDER_MULTIPLIER;
					double max1Val = SmartDashboard.getNumber(MAX_1_VAL_STR, THRESHOLD_DEFAULT_MAX1) * DASH_SLIDER_MULTIPLIER;
					double min2Val = SmartDashboard.getNumber(MIN_2_VAL_STR, THRESHOLD_DEFAULT_MIN2) * DASH_SLIDER_MULTIPLIER;
					double max2Val = SmartDashboard.getNumber(MAX_2_VAL_STR, THRESHOLD_DEFAULT_MAX2) * DASH_SLIDER_MULTIPLIER;
					double min3Val = SmartDashboard.getNumber(MIN_3_VAL_STR, THRESHOLD_DEFAULT_MIN3) * DASH_SLIDER_MULTIPLIER;
					double max3Val = SmartDashboard.getNumber(MAX_3_VAL_STR, THRESHOLD_DEFAULT_MAX3) * DASH_SLIDER_MULTIPLIER;
					
					// Tell the CvSink to grab a frame from the camera and put it
					// in the source mat.  If there is an error notify the output.
					if (cvSink.grabFrame(mat) == 0) {
						// Send the output the error.
						//outputStream.notifyError(cvSink.getError());
						// skip the rest of the current iteration
						continue;
					}
					
					GearLiftTargetTracking.processMat( mat, CAM_WIDTH, CAM_HEIGHT, 
							min1Val, max1Val, min2Val, max2Val, min3Val, max3Val, showThreshold );
					
					// Give the output stream a new image to display
					//outputStream.putFrame(mat);
					
					mat.release();
					
					checkFPS();
				}
				
				System.out.println("~~~~~~~~~~~~~~~TRYING TO CLOSE CAMERA CONNECTION~~~~~~~~~~~~~~~");
				camera.free();
				CameraServer.getInstance().removeCamera(AXIS_NAME);
				CameraServer.getInstance().removeServer(CAM_SERVER_NAME);
				System.out.println("~~~~~~~~~~~~~~~FINISHED TRYING TO CLOSE CAMERA CONNECTION~~~~~~~~~~~~~~~");
			}
		};
		
		visionThread.setDaemon(true);
		
		visionThread.start();
	}
	
	
	
	
	public static final double CAM_FRUSTUM_HORIZONTAL_ANGLE_DEG = 47.0;
	public static final double CAM_FRUSTUM_VERTICAL_ANGLE_DEG = 35.0;
	public static final double CAM_FRUSTUM_HORIZONTAL_ANGLE_TAN
				= Math.tan( Math.toRadians( CAM_FRUSTUM_HORIZONTAL_ANGLE_DEG ) );//1.07236871;
	public static final double CAM_FRUSTUM_VERTICAL_ANGLE_TAN
				= Math.tan( Math.toRadians( CAM_FRUSTUM_VERTICAL_ANGLE_DEG ) );
	public static final double GEAR_TARGET_HEIGHT_DIMENSION_INCHES = 5.0;

	public static final double TEXT_X_OFFSET = 3.0;
	public static final double TEXT_Y_OFFSET = 10.0;
	public static final double TEXT_FONT_SCALE = 4.0;
	public static final int TEXT_THICKNESS = 4;
	
	// Hack-ish multipliers:
	public static final double DISTANCE_MULTIPLIER = 27.0 / 10.5; // actual / calculated
	public static final double T2C_MULTIPLIER = 45.0 / 13.25; // actual / calculated
	public static final double C2T_MULTIPLIER = -1.0;
	
	public static final double GEAR_TARGET_HEIGHT_INCHES = 5.0;
	
	public static Mat processMat( Mat toProcess, 
			double cameraWidth, double cameraHeight, 
			double min1, double max1, 
			double min2, double max2, 
			double min3, double max3, 
			boolean drawContours )
	{
		//long prevTickCount = Core.getTickCount();
		//double elapsedTime = 0.0;
		
		final double CAMERA_DIAGONAL = Math.sqrt( cameraWidth*cameraWidth + cameraHeight*cameraHeight );
		
		Mat thresholdMat = new Mat();
		Mat bridgedThreshold = null;
		
		double bridgeValue = CAMERA_DIAGONAL / 80.0;
		
		Core.inRange(toProcess, new Scalar(min1, min2, min3), new Scalar(max1, max2, max3), thresholdMat);
		bridgedThreshold = bridgeOperation( thresholdMat, new Size( bridgeValue, bridgeValue ) );
		
		thresholdMat.release();
		
		ArrayList<Polygon> polygons = trackPolygons( toProcess, bridgedThreshold, CAMERA_DIAGONAL, drawContours );	
		
		if ( !drawContours )
		{
			ArrayList< CollinearLine > collinearLines = findCollinearLines( polygons );					
			TargetCandidate targetCandidate = getTargetCandidate( collinearLines, CAMERA_DIAGONAL );
			
			if ( targetCandidate != null )
			{
				double targetFwdToCameraAngle = calculateTargetFwdToCameraAngle(targetCandidate);
				double cameraFwdToTargetAngle = calculateCameraFwdToTargetAngle(targetCandidate,
						cameraHeight, false);
				double cameraToTargetDistance = calculateCameraToTargetDistance(targetCandidate,
						cameraWidth, true);
				
				targetCandidate.draw( toProcess );
				
				printTargetFwdToCameraAngle( targetFwdToCameraAngle, toProcess );
				printCameraFwdToTargetAngle( cameraFwdToTargetAngle, toProcess );
				printCameraToTargetDistance( cameraToTargetDistance, toProcess );
				
				//System.out.println( "Angle difference: " + targetCandidate.calculateAngleDifference() + " degrees" );
			}
		}

		//elapsedTime = (double)( Core.getTickCount() - prevTickCount ) * 1000.0 / Core.getTickFrequency();
		//System.out.print("time: ");
		//System.out.printf("%.5f", elapsedTime);
		//System.out.println(" ms");
		
		bridgedThreshold.release();

		return toProcess;
	}
	
	private static double calculateTargetFwdToCameraAngle(TargetCandidate targetCandidate)
	{
		if ( targetCandidate == null )
		{
			return 0.0;
		}
		
		return ( T2C_MULTIPLIER * targetCandidate.calculateAngleDifference() );
	}
	
	private static double calculateCameraFwdToTargetAngle(TargetCandidate targetCandidate,
			double cameraPixelLength, boolean usingHorizontalAxis)
	{
		if ( ( targetCandidate == null ) || ( cameraPixelLength < 0.1 ) )
		{
			return 0.0;
		}
		
		double camFrustumAngle = usingHorizontalAxis ? CAM_FRUSTUM_HORIZONTAL_ANGLE_DEG
						: CAM_FRUSTUM_VERTICAL_ANGLE_DEG;
		
		double targetAxisCoord = usingHorizontalAxis ? targetCandidate.getCenterPoint().x
				: targetCandidate.getCenterPoint().y;
		
		double targetCoordFromCenter = targetAxisCoord - ( cameraPixelLength / 2.0 );
		
		return ( C2T_MULTIPLIER * targetCoordFromCenter * camFrustumAngle / cameraPixelLength );
	}
	
	private static double calculateCameraToTargetDistance(TargetCandidate targetCandidate,
			double cameraPixelLength, boolean usingHorizontalAxis)
	{
		if ( ( targetCandidate == null ) || ( cameraPixelLength < 0.1 ) )
		{
			return 0.0;
		}
		
		double camFrustumAngleTan = usingHorizontalAxis ? CAM_FRUSTUM_HORIZONTAL_ANGLE_TAN
				: CAM_FRUSTUM_VERTICAL_ANGLE_TAN;
		
		double targetHeightAverage = 
				( targetCandidate.getLargestSideLength() + targetCandidate.getSmallestSideLength() ) / 2.0;
		
		if ( ( targetHeightAverage < 0.1 ) || ( camFrustumAngleTan < 0.1 ) )
		{
			return 0.0;
		}
		
		return ( DISTANCE_MULTIPLIER * GEAR_TARGET_HEIGHT_INCHES * cameraPixelLength
				/ ( 2.0 * targetHeightAverage * camFrustumAngleTan ) );
	}
	
	/**
	 * @param angle in degrees.
	 * @param output print to this.
	 */
	private static void printTargetFwdToCameraAngle( double angle, Mat output )
	{
		Imgproc.putText(output, "T2C:" + new DecimalFormat("#.0").format(angle)+"d", 
				new Point( TEXT_X_OFFSET, TEXT_Y_OFFSET + output.height() / 12.0), 
				Core.FONT_HERSHEY_PLAIN, TEXT_FONT_SCALE, new Scalar( Utility.white ), TEXT_THICKNESS );
	}
	
	/**
	 * @param angle in degrees.
	 * @param output print to this.
	 */
	private static void printCameraFwdToTargetAngle( double angle, Mat output )
	{
		Imgproc.putText(output, "C2T:" + new DecimalFormat("#.0").format(angle)+"d", 
				new Point( TEXT_X_OFFSET, TEXT_Y_OFFSET + output.height() * 11.0 / 12.0), 
				Core.FONT_HERSHEY_PLAIN, TEXT_FONT_SCALE, new Scalar( Utility.white ), TEXT_THICKNESS );
	}
	
	/**
	 * @param distance in inches.
	 * @param output print to this.
	 */
	private static void printCameraToTargetDistance( double distance, Mat output )
	{
		Imgproc.putText(output, "DST:" + new DecimalFormat("#.0").format(distance), 
				new Point( TEXT_X_OFFSET + output.width() / 1.8, TEXT_Y_OFFSET + output.height() * 11.0 / 12.0), 
				Core.FONT_HERSHEY_PLAIN, TEXT_FONT_SCALE, new Scalar( Utility.white ), TEXT_THICKNESS );
	}
	
	private static Mat bridgeOperation( Mat inThreshold, Size bridgeAmount )
	{
		final double ERODE_EXTENSION = 2.0;
		
		Size erodeAmount = new Size( bridgeAmount.width + ERODE_EXTENSION, bridgeAmount.height + ERODE_EXTENSION );
		
		Mat dilateResult = new Mat();
		Mat erodeResult = new Mat();
		Mat dilateKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, bridgeAmount);
		Mat erodeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, erodeAmount);
		
		Imgproc.dilate( inThreshold, dilateResult, dilateKernel );
		Imgproc.erode( dilateResult, erodeResult, erodeKernel );

		dilateKernel.release();
		erodeKernel.release();
		dilateResult.release();
		
		return erodeResult;
	}
	
	private static ArrayList<CollinearLine> findCollinearLines( ArrayList<Polygon> polygons )
	{
		ArrayList<CollinearLine> result = new ArrayList<>();
		
		for ( int i = 0; i < polygons.size(); ++i )
		{
			for ( int j = ( i + 1 ); j < polygons.size(); ++j )
			{
				findCollinearLines( polygons.get( i ), polygons.get( j ), result );
			}
		}
		
		return result;
	}
	
	private static TargetCandidate getTargetCandidate( ArrayList<CollinearLine> collinearLines,
			double cameraDiagonal )
	{
		TargetCandidate bestCandidate = null;
		
		for ( int i = 0; i < collinearLines.size(); ++i )
		{
			for ( int j = ( i + 1 ); j < collinearLines.size(); ++j )
			{
				TargetCandidate candidate = TargetCandidate.generateTargetCandidate( 
						collinearLines.get( i ), collinearLines.get( j ), cameraDiagonal, true );
				
				if ( candidate != null )
				{
					if ( bestCandidate == null )
					{
						bestCandidate = candidate;
					}
					else if ( bestCandidate.getScore() < candidate.getScore() )
					{
						bestCandidate = candidate;
					}
				}
			}
		}
		
		if ( bestCandidate != null )
		{
			if ( bestCandidate.getScore() < TargetCandidate.MIN_SCORE )
			{
				return null;
			}
		}
		
		return bestCandidate;
	}
	
	private static void findCollinearLines( Polygon polygon1, Polygon polygon2, ArrayList<CollinearLine> result )
	{
		for ( LineSegment ls1 : polygon1 )
		{
			for ( LineSegment ls2 : polygon2 )
			{
				if ( ls1.isCollinearWith( ls2 ) )
				{
					result.add( CollinearLine.createCollinearLine( ls1, ls2, polygon1, polygon2 ) );
				}
			}
		}
	}
	
	private static ArrayList<Polygon> trackPolygons(Mat camFrame, Mat hlsMask, double cameraDiagonal, boolean drawContours)
	{
		final double MIN_ARC_LENGTH = ( cameraDiagonal / 8.0 );
		
		ArrayList<Polygon> polygons = new ArrayList<>();
		ArrayList<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchy = new Mat();
        Scalar color = new Scalar( Utility.red );
        int i = 0;
		
		Imgproc.findContours(hlsMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		
		for ( MatOfPoint contour : contours )
		{
			MatOfPoint2f contour2f = new MatOfPoint2f( contour.toArray() );		
			MatOfPoint2f approxCurve = new MatOfPoint2f();
			double arcLength = Imgproc.arcLength(contour2f, true);
			
			if ( arcLength >= MIN_ARC_LENGTH )
			{
				if ( drawContours )
				{
					Imgproc.drawContours( camFrame, contours, i, color, 2, 8, hierarchy, 0, new Point() );
				}
				
				Imgproc.approxPolyDP(contour2f, approxCurve, 0.04 * arcLength, true);
				
				polygons.add( Polygon.createPolygon( approxCurve.toList() ) );
			}
			
			++i;
			
			contour.release();
			contour2f.release();
			approxCurve.release();
		}
		
		hierarchy.release();
		
		return polygons;
	}

}
