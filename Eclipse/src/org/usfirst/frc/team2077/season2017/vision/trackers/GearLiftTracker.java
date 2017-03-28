package org.usfirst.frc.team2077.season2017.vision.trackers;

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
import org.usfirst.frc.team2077.season2017.vision.measurement.TargetTransform;

class GearLiftTracker extends AxisCameraVisionTracker
{
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
	
	final double THRESHOLD_DEFAULT_MIN1 = 0.0;
	final double THRESHOLD_DEFAULT_MAX1 = 127.5;
	final double THRESHOLD_DEFAULT_MIN2 = 204.0;
	final double THRESHOLD_DEFAULT_MAX2 = 255.0;
	final double THRESHOLD_DEFAULT_MIN3 = 0.0;
	final double THRESHOLD_DEFAULT_MAX3 = 136.68;

	public GearLiftTracker(String cameraIP) {
		super(cameraIP);
	}
	
	private int i_;

	@Override
	protected TargetTransform computeTargetTransform(Mat mat) {

		final int CAM_WIDTH = 640;
		final int CAM_HEIGHT = 480;

		final double THRESHOLD_MIN = 200.00;
		final double THRESHOLD_MAX = 255.00;
		
		final double CAMERA_DIAGONAL = Math.sqrt( CAM_WIDTH*CAM_WIDTH + CAM_HEIGHT*CAM_HEIGHT );
		
		final boolean DEBUG_DRAW = true;
		
		//Mat small = new Mat();
		Mat thresholdMat = new Mat();
		
		double targetAngleToRobot = 0.0;
		double robotAngleToTarget = 0.0;
		double robotToTargetDistance = 0.0;
		double acquisitionTime = 0.0;
		double timestamp = ++i_;
		boolean targetFound = false;
		
		long prevTickCount = Core.getTickCount();
		
		//applyFiltering( mat );

		//Imgproc.resize(mat, small, new Size(16, 12), .5, .5, Imgproc.INTER_AREA);
		//Imgproc.resize(small, mat, new Size(CAM_WIDTH, CAM_HEIGHT), .5, .5, Imgproc.INTER_CUBIC);
		
		//small.release();
		
		//autoWindow(mat);
		
		//Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);
		//Imgproc.equalizeHist(mat, mat);
		//Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);
		
		///////////////////
		
//		Core.inRange(mat, 
//				new Scalar(THRESHOLD_MIN,THRESHOLD_MIN,THRESHOLD_MIN), 
//				new Scalar(THRESHOLD_MAX,THRESHOLD_MAX,THRESHOLD_MAX), thresholdMat);
		
		Core.inRange(mat, 
				new Scalar(THRESHOLD_DEFAULT_MIN1,THRESHOLD_DEFAULT_MIN2,THRESHOLD_DEFAULT_MIN3), 
				new Scalar(THRESHOLD_DEFAULT_MAX1,THRESHOLD_DEFAULT_MAX2,THRESHOLD_DEFAULT_MAX3), thresholdMat);
		
		
		ArrayList<Polygon> polygons = trackPolygons( mat, thresholdMat, CAMERA_DIAGONAL, DEBUG_DRAW );	
		
		ArrayList< CollinearLine > collinearLines = findCollinearLines( polygons );					
		TargetCandidate targetCandidate = getTargetCandidate( collinearLines, CAMERA_DIAGONAL );
		
		if ( targetCandidate != null )
		{
			targetFound = true;
			
			targetAngleToRobot = calculateTargetFwdToCameraAngle(targetCandidate);
			robotAngleToTarget = calculateCameraFwdToTargetAngle(targetCandidate,
					CAM_HEIGHT, false);
			robotToTargetDistance = calculateCameraToTargetDistance(targetCandidate,
					CAM_WIDTH, true);
			
			if ( DEBUG_DRAW )
			{
				targetCandidate.draw( mat );
				
				printTargetFwdToCameraAngle( targetAngleToRobot, mat );
				printCameraFwdToTargetAngle( robotAngleToTarget, mat );
				printCameraToTargetDistance( robotToTargetDistance, mat );
			}
		}
		
		thresholdMat.release();
		
		///////////////////

		acquisitionTime = (double)( Core.getTickCount() - prevTickCount ) * 1000.0 / Core.getTickFrequency();

		return new TargetTransform(targetAngleToRobot, robotAngleToTarget, robotToTargetDistance, 
				acquisitionTime, timestamp, targetFound, mat);
	}
	
	private static void applyFiltering( Mat mat )
	{
		byte[] bgr = new byte[3];
		for (int r = 0; r < mat.rows(); r++) {
			for (int c = 0; c < mat.cols(); c++) {
				mat.get(r,  c, bgr);
				int blue = 0x000000FF & bgr[0];
				int green = 0x000000FF & bgr[1];
				int red = 0x000000FF & bgr[2];
				bgr[0] = bgr[1] = bgr[2] = (byte)Math.min(255, Math.max(0, (2*green - red - blue)));
				mat.put(r,  c, bgr);
			}
		}
	}
	
	private static void autoWindow( Mat mat)
	{
		int max = 0;
		int min = 255;
		
		byte[] bgr = new byte[3];
		for (int r = 0; r < mat.rows(); r++) {
			for (int c = 0; c < mat.cols(); c++) {
				mat.get(r,  c, bgr);

				int pixelIntensity = 0x000000FF & bgr[0];
				
				max = Math.max(max, pixelIntensity);
				min = Math.min(min, pixelIntensity);
			}
		}
		
		for (int r = 0; r < mat.rows(); r++) {
			for (int c = 0; c < mat.cols(); c++) {
				mat.get(r,  c, bgr);

				int pixelIntensity = 0x000000FF & bgr[0];
				
				pixelIntensity = (pixelIntensity - min)*(255/(max-min));
				bgr[0] = bgr[1] = bgr[2] = (byte)pixelIntensity;
				
				mat.put(r, c, bgr);
			}
		}
		
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
				Core.FONT_HERSHEY_PLAIN, TEXT_FONT_SCALE, new Scalar( Utility.green ), TEXT_THICKNESS );
	}
	
	/**
	 * @param angle in degrees.
	 * @param output print to this.
	 */
	private static void printCameraFwdToTargetAngle( double angle, Mat output )
	{
		Imgproc.putText(output, "C2T:" + new DecimalFormat("#.0").format(angle)+"d", 
				new Point( TEXT_X_OFFSET, TEXT_Y_OFFSET + output.height() * 11.0 / 12.0), 
				Core.FONT_HERSHEY_PLAIN, TEXT_FONT_SCALE, new Scalar( Utility.green ), TEXT_THICKNESS );
	}
	
	/**
	 * @param distance in inches.
	 * @param output print to this.
	 */
	private static void printCameraToTargetDistance( double distance, Mat output )
	{
		Imgproc.putText(output, "DST:" + new DecimalFormat("#.0").format(distance), 
				new Point( TEXT_X_OFFSET + output.width() / 1.8, TEXT_Y_OFFSET + output.height() * 11.0 / 12.0), 
				Core.FONT_HERSHEY_PLAIN, TEXT_FONT_SCALE, new Scalar( Utility.green ), TEXT_THICKNESS );
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
