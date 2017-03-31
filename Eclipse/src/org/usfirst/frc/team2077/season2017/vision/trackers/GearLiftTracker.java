package org.usfirst.frc.team2077.season2017.vision.trackers;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team2077.season2017.vision.measurement.TargetTransform;
import org.usfirst.frc.team2077.season2017.vision.trackers.CollinearLine;
import org.usfirst.frc.team2077.season2017.vision.trackers.LineSegment;
import org.usfirst.frc.team2077.season2017.vision.trackers.Polygon;
import org.usfirst.frc.team2077.season2017.vision.trackers.TargetCandidate;
import org.usfirst.frc.team2077.season2017.vision.trackers.Utility;

class GearLiftTracker extends AxisCameraVisionTracker
{
	public static final boolean CAMERA_RUNNING_SIDEWAYS = false; // true on Eclipse @ MKE
	
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

		TargetCandidate targetCandidate = trackTarget( mat, thresholdMat, CAMERA_DIAGONAL, DEBUG_DRAW, DEBUG_DRAW );
		
		if ( targetCandidate != null )
		{
			targetAngleToRobot = calculateTargetFwdToCameraAngle(targetCandidate);
			robotAngleToTarget = calculateCameraFwdToTargetAngle(targetCandidate,
					CAMERA_RUNNING_SIDEWAYS ? CAM_HEIGHT : CAM_WIDTH, !CAMERA_RUNNING_SIDEWAYS);
			robotToTargetDistance = calculateCameraToTargetDistance(targetCandidate,
					CAM_WIDTH, CAM_HEIGHT, CAMERA_RUNNING_SIDEWAYS != targetCandidate.hasBothRectangles());
			
			targetFound = true;
			
			if ( DEBUG_DRAW )
			{
				targetCandidate.draw( mat );
				
				printTargetFwdToCameraAngle( targetAngleToRobot, mat );
				printCameraFwdToTargetAngle( robotAngleToTarget, mat );
				printCameraToTargetDistance( robotToTargetDistance, mat );
			}			
			
			//System.out.println( "Angle difference: " + targetCandidate.calculateAngleDifference() + " degrees" );
		}

		//elapsedTime = (double)( Core.getTickCount() - prevTickCount ) * 1000.0 / Core.getTickFrequency();
		//System.out.print("time: ");
		//System.out.printf("%.5f", elapsedTime);
		//System.out.println(" ms");
		
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
		
		double signMultiplier = ( usingHorizontalAxis ? -1.0 : 1.0 );
		
		return ( C2T_MULTIPLIER * signMultiplier * targetCoordFromCenter * camFrustumAngle / cameraPixelLength );
	}
	
	private static double calculateCameraToTargetDistance(TargetCandidate targetCandidate,
			double cameraPixelWidth, double cameraPixelHeight, boolean usingHorizontalAxis)
	{
		if ( ( targetCandidate == null ) || ( cameraPixelWidth < 0.1 ) || ( cameraPixelHeight < 0.1 ) )
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
		
		return ( DISTANCE_MULTIPLIER * ( usingHorizontalAxis ? cameraPixelWidth : cameraPixelHeight )
				/ ( 2.0 * targetCandidate.getNormalizedScale() * camFrustumAngleTan ) );
	}
	
	/**
	 * @param angle in degrees.
	 * @param output print to this.
	 */
	private static void printTargetFwdToCameraAngle( double angle, Mat output )
	{
		Imgproc.putText(output, "T2C:" + new DecimalFormat("00.0").format(angle)+"d", 
				new Point( TEXT_X_OFFSET, TEXT_Y_OFFSET + output.height() / 12.0), 
				Core.FONT_HERSHEY_PLAIN, TEXT_FONT_SCALE, new Scalar( Utility.white ), TEXT_THICKNESS );
	}
	
	/**
	 * @param angle in degrees.
	 * @param output print to this.
	 */
	private static void printCameraFwdToTargetAngle( double angle, Mat output )
	{
		Imgproc.putText(output, "C2T:" + new DecimalFormat("0.0").format(angle)+"d", 
				new Point( TEXT_X_OFFSET, TEXT_Y_OFFSET + output.height() * 11.0 / 12.0), 
				Core.FONT_HERSHEY_PLAIN, TEXT_FONT_SCALE, new Scalar( Utility.white ), TEXT_THICKNESS );
	}
	
	/**
	 * @param distance in inches.
	 * @param output print to this.
	 */
	private static void printCameraToTargetDistance( double distance, Mat output )
	{
		Imgproc.putText(output, "DST:" + new DecimalFormat("00.0").format(distance), 
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
	
	private static TargetCandidate evaluateTargetCandidate( TargetCandidate currentCandidate, 
			TargetCandidate bestCandidate, double minimumScore )
	{
		if ( currentCandidate != null )
		{
			if ( currentCandidate.getScore() >= minimumScore )
			{
				if ( bestCandidate == null )
				{
					return currentCandidate;
				}
				else if ( bestCandidate.getScore() < currentCandidate.getScore() )
				{
					return currentCandidate;
				}
			}
		}
		
		return bestCandidate;
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
						collinearLines.get( i ), collinearLines.get( j ), cameraDiagonal, CAMERA_RUNNING_SIDEWAYS );
				
				bestCandidate = evaluateTargetCandidate( candidate, bestCandidate, TargetCandidate.MIN_SCORE );
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
	
	/**
	 * @param basePoints Base polygon approximation (lower epsilon value, higher precision/detail)
	 * @param overlapPoints Simplified, overlapping polygon approximation (higher epsilon value, lower precision/detail)
	 * @param indices Output list of indices to overlapping points in the base polygon.
	 */
	private static int[] findOverlapIndices( Point[] basePoints, Point[] overlapPoints )
	{
		List<Integer> indices = new ArrayList<>();
		int[] result;
		int i = 0;
		
		for ( Point overlapPoint : overlapPoints )
		{
			int bestIndex = -1;
			double smallestDistance = 0.0;
			
			for ( int j = 0; j < basePoints.length; ++j )
			{
				double currentDistance = Utility.getPointsDistance(basePoints[ j ], overlapPoint);
				
				if ( ( bestIndex < 0 ) || ( currentDistance < smallestDistance ) )
				{
					bestIndex = j;
					smallestDistance = currentDistance;
				}
			}
			
			if ( bestIndex >= 0 )
			{
				indices.add( bestIndex );
			}
		}
		
		result = new int[indices.size()];
		
		for ( Integer idx : indices )
		{
			result[i++] = idx;
		}
		
		return result;
	}
	
	private static TargetCandidate trackTarget(Mat camFrame, Mat hlsMask, double cameraDiagonal, 
			boolean drawContours, boolean drawPolygons)
	{
		TargetCandidate targetCandidate = null;

		ArrayList<Polygon> polygons = new ArrayList<>();	
		ArrayList<Integer> polyIndices = new ArrayList<>();
		ArrayList<Double> arcLengths = new ArrayList<>();
		
		ArrayList<Polygon> polygonsToDraw = null;

		ArrayList<MatOfPoint> contours = new ArrayList<>();	
		Mat hierarchy = new Mat();
		
		Imgproc.findContours(hlsMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		
		targetCandidate = trackFullTarget( camFrame, contours, hierarchy, polygons, polyIndices, arcLengths, 
				cameraDiagonal / 8.0, cameraDiagonal, drawContours );
		
		if ( targetCandidate == null )
		{
			if ( drawPolygons )
			{
				polygonsToDraw = new ArrayList<>();
			}
			
			targetCandidate = trackRectangleTarget( camFrame, contours, polygons, polyIndices, arcLengths, polygonsToDraw );
		}
		else
		{
			polygonsToDraw = polygons;
			
			for ( MatOfPoint contour : contours )
			{
				contour.release();
			}
			
			contours.clear();
		}
		
		hierarchy.release();
		
		if ( drawPolygons )
		{
			for ( Polygon poly : polygonsToDraw )
			{
				poly.draw( camFrame );
			}
		}
		
		return targetCandidate;
	}
	
	private static TargetCandidate trackFullTarget( Mat camFrame, ArrayList<MatOfPoint> contours, Mat hierarchy, 
			ArrayList<Polygon> polygons, ArrayList<Integer> polyIndices, ArrayList<Double> arcLengths,
			double minimumArcLength, double cameraDiagonal, boolean drawContours )
	{
        int contourIndex = 0;
		ArrayList< CollinearLine > collinearLines = null;
        
		for ( MatOfPoint contour : contours )
		{
			MatOfPoint2f contour2f = new MatOfPoint2f( contour.toArray() );		
			MatOfPoint2f approxCurve = new MatOfPoint2f();
			double arcLength = Imgproc.arcLength(contour2f, true);
			
			if ( arcLength >= minimumArcLength )
			{
				if ( drawContours )
				{
					Imgproc.drawContours( camFrame, contours, contourIndex, new Scalar( Utility.red ), 2, 8, hierarchy, 0, new Point() );
				}
				
				Imgproc.approxPolyDP(contour2f, approxCurve, 0.04 * arcLength, true);
				
				polygons.add( new Polygon( approxCurve.toList() ) );
				polyIndices.add( contourIndex );
				arcLengths.add( arcLength );
			}
			else
			{
				contour.release();
			}
			
			++contourIndex;
			
			contour2f.release();
			approxCurve.release();
		}
		
		collinearLines = findCollinearLines( polygons );		
		return getTargetCandidate( collinearLines, cameraDiagonal );
	}
	
	private static TargetCandidate trackRectangleTarget( Mat camFrame, ArrayList<MatOfPoint> contours, ArrayList<Polygon> polygons, 
			ArrayList<Integer> polyIndices, ArrayList<Double> arcLengths, ArrayList<Polygon> polygonsToDraw )
	{
		TargetCandidate bestTargetCandidate = null;
		
		for ( int i = 0; ( i < polygons.size() ) && ( i < polyIndices.size() ) && ( i < arcLengths.size() ); ++i )
		{
			MatOfPoint contour = contours.get(polyIndices.get(i));
			MatOfPoint2f contour2f = new MatOfPoint2f( contour.toArray() );
			
			MatOfInt hullIndices = new MatOfInt();
			MatOfPoint2f approxCurve = new MatOfPoint2f();
			MatOfPoint overlapPolygon;
			
			int[] overlapIndicesArray = null;
			Point[] basePolygonArray;
			Point[] overlapPolygonArray;
			
			Polygon newPolygon;
			
			Imgproc.approxPolyDP(contour2f, approxCurve, 0.01/*0.00875/*0.0075*/ * arcLengths.get(i), true); // Base	
			basePolygonArray = approxCurve.toArray();
			
			overlapPolygonArray = polygons.get(i).toPointsArray();
			overlapPolygon = new MatOfPoint( overlapPolygonArray );
			
			Imgproc.convexHull(overlapPolygon, hullIndices);
			overlapIndicesArray = findOverlapIndices(basePolygonArray, overlapPolygonArray);
			
			if ( overlapIndicesArray.length >= 4 )
			{
				newPolygon = Polygon.constructPotentialTargetRectangle( basePolygonArray, overlapPolygonArray, 
						hullIndices.toArray(), overlapIndicesArray, camFrame );
				
				if ( newPolygon != null )
				{
					TargetCandidate currentCandidate = TargetCandidate.generateTargetCandidate( 
							newPolygon, CAMERA_RUNNING_SIDEWAYS );
					
					bestTargetCandidate = evaluateTargetCandidate( currentCandidate, bestTargetCandidate, TargetCandidate.MIN_SCORE );
					
					if ( polygonsToDraw != null )
					{
						polygonsToDraw.add( newPolygon );
					}
				}
			}
			
			overlapPolygon.release();
			approxCurve.release();
			hullIndices.release();			
			contour2f.release();
			contour.release();
		}
		
		return bestTargetCandidate;
	}
}
