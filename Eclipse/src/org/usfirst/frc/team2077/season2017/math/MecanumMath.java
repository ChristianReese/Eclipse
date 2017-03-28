package org.usfirst.frc.team2077.season2017.math;

import java.util.Random;

/***
 * An implementation of the mecanum drivetrain inverse and forward kinematics described in
 * <a href="http://www.chiefdelphi.com/media/papers/download/2722"><i>Kinematic Analysis of Four-Wheel Mecanum Vehicle</i></a>.
 * <p>
 * The core matrix algebra is implemented in the static methods:
 * <dl>
 *  <dt>{@link #inverse(double[],double[][],double)}</dt>
 *  <dd><p style="margin-left: 40px">Implements the inverse kinematic equation <b>[&#x03A9;] = (1/r)[R][V]</b>,
 *  which calculates the the individual wheel speeds necessary to produce a specified robot motion.
 *  This calculation is the central function of a basic drive control program that converts user input to motor control.</p></dd>
 *  <dt>{@link #forward(double[],double[][],double)}</dt>
 *  <dd><p style="margin-left: 40px">Implements the forward kinematic equation <b>[V] = [F][&#x03A9;](r)</b>,
 *  which calculates the robot motion to be expected from a set of individual wheel speeds.
 *  This calculation is not generally necessary for basic robot control, but may be useful for more advanced operations.
 *  Unlike the inverse equation, the forward calculation represents an overdetermined system,
 *  where most combinations of wheel speeds are inconsistent. Inconsistent wheel speeds in practice mean
 *  wheel slippage, motor stalling, and generally erratic behavior, the more inconsistent the worse.
 *  This forward calculation produces a least-squares best fit.</p></dd>
 *  <dt>{@link #createInverseMatrix(double,double)}, {@link #createInverseMatrix(double,double,double[])}</dt>
 *  <dd><p style="margin-left: 40px">Convenience methods for initializing the inverse kinematic matrix <b>[R]</b>.</p></dd>
 *  <dt>{@link #createForwardMatrix(double,double)}</dt>
 *  <dd><p style="margin-left: 40px">Convenience method for initializing the forward kinematic matrix <b>[F]</b>.</p></dd>
 * </dl>
 * <p>
 * Instance objects wrap the core static methods with specific robot geometry and optional unit
 * conversion factors. Robot geometry, including dimensions, wheel size, and center of rotation is set through
 * the constructor methods, and the <b>[R]</b> and <b>[F]</b> matrices are internally managed. Application code may then
 * call the simpler {@link #inverse(double[])} and {@link #forward(double[])} methods with the current motion (<b>[V]</b>)
 * or wheel speed (<b>[&#x03A9;]</b>) vectors. Some of the constructors also take conversion factors to automatically
 * convert input and output values for these methods from and to other units more convenient to the calling code. 
 * <div style="font-size: smaller">
 * Note: This implementation departs slightly from the notational conventions used in the paper above:
 * <ul>
 * <li>"North" and "East" are used instead of X and Y.</li>
 * <li>Orientation is mirror-imaged: Y (East) is positive-right, and rotation is positive-clockwise.
 * <li>Wheel numbers are also mirror imaged, so the matrix math itself is unchanged.</li>
 * </ul>
 * </div>
 * @author 2077
 */
public class MecanumMath {

    /***
     * Solve the inverse kinematic equation <b>[&#x03A9;] = (1/r)[R][V]</b>.
     * @param V Robot motion velocity vector <b>[V]</b>: [north/south translation (units/second), east/west translation, rotation (radians/second)].
     * @param R A 4x3 inverse kinematic matrix <b>[R]</b>. See {@link #createInverseMatrix}.
     * @param r wheel radius, in the length units used to construct <b>[R]</b>.
     * @return The wheel angular velocity vector <b>[&#x03A9;]</b>: {NE, SE, SW, NW} in radians/second.
     */
     public static double[] inverse(double[] V, double[][] R, double r) {
         double[] O = new double[4];
         for (int i = 0; i < 4; i++) {
             for (int j = 0; j < 3; j++) {
                 O[i] += (V[j] * R[i][j]) / r;
             }
         }
         return O;
     }
     
    /***
     * Solve the forward kinematic equation <b>[V] = [F][&#x03A9;](r)</b>.
     * Where <b>[&#x03A9;]</b> has internal inconsistencies a best-fit value will be generated.
     * @param O A wheel angular velocity vector <b>[&#x03A9;]</b>: [NE, SE, SW, NW] in radians/second.
     * @param F A 3x4 forward kinematic matrix <b>[F]</b>. See {@link #createForwardMatrix}.
     * @param r Wheel radius, in the length units used to construct <b>[F]</b>.
     * @return The robot motion velocity vector <b>[V]</b>: [north/south translation (units/second), east/west translation, rotation (radians/second)].
     */
     public static double[] forward(double[] O, double[][] F, double r) {
         double[] V = new double[3];
         for (int i = 0; i < 3; i++) {
             for (int j = 0; j < 4; j++) {
                 V[i] += (O[j] * F[i][j]) * r;
             }
         }
         return V;
     }
     
    /**
     * Construct the inverse matrix <b>[R]</b> for a rectangular robot with an arbitrary center of rotation.
     * @param length Distance between wheel contact points in the N/S direction, in any length unit.
     * @param width Distance between wheel contact points in the E/W direction, in the same unit.
     * @param rotationCenter {N,E} coordinates of the robot's center of rotation relative to its geometric center.
     * @return A 4x3 inverse kinematic matrix <b>[R]</b> for use by {@link #inverse}.
     */
    public static double[][] createInverseMatrix(double length, double width, double[] rotationCenter) {

        double[][] m = {
            { length/2 - rotationCenter[0], width/2 - rotationCenter[1]},
            {-length/2 - rotationCenter[0], width/2 - rotationCenter[1]},
            {-length/2 - rotationCenter[0],-width/2 - rotationCenter[1]},
            { length/2 - rotationCenter[0],-width/2 - rotationCenter[1]}
        };
        return new double[][] {
            {1,-1,-m[0][0]-m[0][1]},
            {1, 1, m[1][0]-m[1][1]},
            {1,-1,-m[2][0]-m[2][1]},
            {1, 1, m[3][0]-m[3][1]}
        };
    }
     
    /**
     * Construct the inverse matrix <b>[R]</b> for a rectangular robot whose center of rotation is its geometric center.
     * This is a convenience function wrapping {@link #createInverseMatrix(double,double,double[])}.
     * @param length Distance between wheel contact points in the N/S direction, in any length unit.
     * @param width Distance between wheel contact points in the E/W direction, in the same unit.
     * @return A 4x3 inverse kinematic matrix <b>[R]</b> for use by {@link #inverse}.
     */
    public static double[][] createInverseMatrix(double length, double width) {
        return createInverseMatrix(length, width, new double[] {0,0});
    }
     
    /**
     * Construct the forward matrix <b>[F]</b> for a rectangular robot.
     * @param length Distance between wheel contact points in the N/S direction, in any length unit.
     * @param width Distance between wheel contact points in the E/W direction, in the same unit.
     * @return A 3x4 forward kinematic matrix <b>[F]</b> for use by {@link #forward}.
     */
    public static double[][] createForwardMatrix(double length, double width) {
        double K = length/2 + width/2;
        return new double[][] {
            { 1/4.,    1/4.,    1/4.,    1/4.   },
            {-1/4.,    1/4.,   -1/4.,    1/4.   },
            {-1/(4*K),-1/(4*K), 1/(4*K), 1/(4*K)}
        };
    }

    private final double[][] reverseMatrix_;
    private final double[][] forwardMatrix_;
    private final double length_;
    private final double width_;
    private final double wheelRadius_;
    private final double wheelSpeedFactor_;
    private final double robotSpeedFactor_;
    private final double rotationSpeedFactor_;
    
    /**
     * Initialize the robot context for inverse and forward kinematics solutions.
     * This includes the robot geometry and associated <b>[R]</b> and <b>[F]</b> matrices,
     * as well as optional conversion factors to allow input and output values to be input and
     * returned in units convenient to the calling code.
     * @param length Distance between wheel contact points in the N/S direction, in any length unit.
     * @param width Distance between wheel contact points in the E/W direction, in the same unit.
     * @param wheelRadius Radius of each wheel, in the same unit.
     * @param wheelSpeedFactor Multiplier to convert wheel rotational velocities in radians/second to user units.
     * @param robotSpeedFactor Multiplier to convert robot translation speeds in length units/second to user units.
     * @param rotationSpeedFactor Multiplier to convert robot rotational vbelocities in radians/second to user units.
     */
     public MecanumMath(double length, double width, double wheelRadius, double wheelSpeedFactor, double robotSpeedFactor, double rotationSpeedFactor) {
    	length_ = length;
    	width_ = width;
        reverseMatrix_ = createInverseMatrix(length, width);
        forwardMatrix_ = createForwardMatrix(length, width);
        wheelRadius_ = wheelRadius;
        wheelSpeedFactor_ = wheelSpeedFactor;
        robotSpeedFactor_ = robotSpeedFactor;
        rotationSpeedFactor_ = rotationSpeedFactor;
    }
    
    /**
     * This is a convenience constructor wrapping {@link #MecanumMath(double,double,double,double,double,double)}.
     * Calls to {@link #inverse(double[])} or {@link #forward(double[])} will use robot translation speeds in units/second
     * using the same length units as length, width, and wheel radius, with robot rotation and wheel speeds in radians/second.
     * @param length Distance between wheel contact points in the N/S direction, in any length unit.
     * @param width Distance between wheel contact points in the E/W direction, in the same unit.
     * @param wheelRadius Radius of each wheel, in the same unit.
     */
    public MecanumMath(double length, double width, double wheelRadius) {
        this(length, width, wheelRadius, 1, 1, 1);
    }

    /***
     * Solve the inverse kinematic equation <b>[&#x03A9;] = (1/r)[R][V]</b>.
     * This method wraps the static {@link #inverse(double[],double[][],double)} method, passing the internal
     * <b>[R]</b> matrix and wheel radius initialized in the constructor, and converting input and output
     * values from and to user units if conversion factors were specified.
     * @param userV Robot motion velocity vector <b>[V]</b>: [north/south translation, east/west translation, rotation], in user units.
     * @return The wheel speed vector <b>[&#x03A9;]</b>: {NE, SE, SW, NW} in user units.
     */
    public final double[] inverse(double[] userV) {
    	return inverse(userV, new double[] {0, 0});
    }

    /***
     * Solve the inverse kinematic equation <b>[&#x03A9;] = (1/r)[R][V]</b> for rotation about an arbitrary point.
     * This method wraps the static {@link #inverse(double[],double[][],double)} method, passing the internal
     * <b>[R]</b> matrix and wheel radius initialized in the constructor, and converting input and output
     * values from and to user units if conversion factors were specified.
     * @param userV Robot motion velocity vector <b>[V]</b>: [north/south translation, east/west translation, rotation], in user units.
     * @param rotationCenter {N,E} coordinates of the center of rotation relative to the robot's geometric center.
     * @return The wheel speed vector <b>[&#x03A9;]</b>: {NE, SE, SW, NW} in user units.
     */
    public final double[] inverse(double[] userV, double[] rotationCenter) {
        double[] v = { fromUserRobotSpeed(userV[0]), fromUserRobotSpeed(userV[1]), fromUserRotationSpeed(userV[2]) };
    	double[][] reverseMatrix = rotationCenter[0]==0 && rotationCenter[1]==0 ? reverseMatrix_ : createInverseMatrix(length_, width_, rotationCenter);
        double[] o = inverse(v, reverseMatrix, wheelRadius_);
        return new double[] { toUserWheelSpeed(o[0]), toUserWheelSpeed(o[1]), toUserWheelSpeed(o[2]), toUserWheelSpeed(o[3]) };
    }

    /***
     * Solve the forward kinematic equation <b>[V] = [F][&#x03A9;](r)</b>.
     * This method wraps the static {@link #forward(double[],double[][],double)} method, passing the internal
     * <b>[F]</b> matrix and wheel radius initialized in the constructor, and converting input and output
     * values from and to user units if conversion factors were specified.
     * @param userO A wheel speed vector <b>[&#x03A9;]</b>: [NE, SE, SW, NW] in user units.
     * @return [3] robot motion velocities {north/east translation, rotation} in user units.
     */
    public final double[] forward(double[] userO) {
        double[] o = { fromUserWheelSpeed(userO[0]), fromUserWheelSpeed(userO[1]), fromUserWheelSpeed(userO[2]), fromUserWheelSpeed(userO[3]) };
        double[] v = forward(o, forwardMatrix_, wheelRadius_);
        return new double[] { toUserRobotSpeed(v[0]), toUserRobotSpeed(v[1]), toUserRotationSpeed(v[2]) };
    }
    
    /***
     * Convert a wheel rotation speed from user units to radians/second
     * by dividing the input value by the conversion factor passed to the
     * constructor {@link #MecanumMath(double,double,double,double[],double,double,double)}.
     * This method exists mainly for internal use by {@link #forward(double[])}
     * and should not normally be called from application code.
     * @param userWheelSpeed An individual wheel speed in user units
     * as input to {@link #forward(double[])}.
     * @return Wheel rotational velicity in radians/second
     * as expected by {@link #forward(double[],double[][],double)}.
     */
    protected double fromUserWheelSpeed(double userWheelSpeed) {
        return userWheelSpeed / wheelSpeedFactor_;
    }

    /***
     * Convert a robot translation speed from user units to units/second
     * (where units/second is in the length/width/radius units passed to the constructor)
     * by dividing the input value by the conversion factor passed to the
     * constructor {@link #MecanumMath(double,double,double,double[],double,double,double)}.
     * This method exists mainly for internal use by {@link #inverse(double[])}
     * and should not normally be called from application code.
     * @param userRobotSpeed A N/S or E/W robot translation velocity component in user units
     * as input to {@link #inverse(double[])}.
     * @return Velocity component in length units/second
     * as expected by {@link #inverse(double[],double[][],double)}.
     */
    protected double fromUserRobotSpeed(double userRobotSpeed) {
        return userRobotSpeed / robotSpeedFactor_;
    }

    /***
     * Convert a robot rotation speed from user units to radians/second
     * by dividing the input value by the conversion factor passed to the
     * constructor {@link #MecanumMath(double,double,double,double[],double,double,double)}.
     * This method exists mainly for internal use by {@link #inverse(double[])}
     * and should not normally be called from application code.
     * @param userRotationSpeed A robot rotational velocity in user units
     * as input to {@link #inverse(double[])}.
     * @return Rotational velocity (radians/second)
     * as expected by {@link #inverse(double[],double[][],double)}.
     */
    protected double fromUserRotationSpeed(double userRotationSpeed) {
        return userRotationSpeed / rotationSpeedFactor_;
    }

    /***
     * Convert a wheel rotation speed from radians/second to user units
     * by multiplying the input value by the conversion factor passed to the
     * constructor {@link #MecanumMath(double,double,double,double[],double,double,double)}.
     * This method exists mainly for internal use by {@link #inverse(double[])}
     * and should not normally be called from application code.
     * @param radiansPerSecond An individual wheel rotational velicity in radians/second
     * as returned by {@link #inverse(double[],double[][],double)}.
     * @return Speed in user units as returned by {@link #inverse(double[])}.
     */
    protected double toUserWheelSpeed(double radiansPerSecond) {
        return radiansPerSecond * wheelSpeedFactor_;
    }

    /***
     * Convert a robot translation speed from units/second
     * (where units/second is in the length/width/radius units passed to the constructor) to user units
     * by multiplying the input value by the conversion factor passed to the
     * constructor {@link #MecanumMath(double,double,double,double[],double,double,double)}.
     * This method exists mainly for internal use by {@link #forward(double[])}
     * and should not normally be called from application code.
     * @param unitsPerSecond A N/S or E/W robot translation velocity component in in units/second
     * as returned by {@link #forward(double[],double[][],double)}.
     * @return Speed in user units as returned by {@link #forward(double[])}.
     */
    protected double toUserRobotSpeed(double unitsPerSecond) {
        return unitsPerSecond * robotSpeedFactor_;
    }

    /***
     * Convert a robot rotation speed from radians/second to user units
     * by multiplying the input value by the conversion factor passed to the
     * constructor {@link #MecanumMath(double,double,double,double[],double,double,double)}.
     * This method exists mainly for internal use by {@link #forward(double[])}
     * and should not normally be called from application code.
     * @param radiansPerSecond A robot rotational velocity in radians/second
     * as returned by {@link #forward(double[],double[][],double)}.
     * @return Speed in user units as returned by {@link #forward(double[])}.
     */
    protected double toUserRotationSpeed(double radiansPerSecond) {
        return radiansPerSecond * rotationSpeedFactor_;
    }

    /**
     * Test code.
     * @param argv Not used.
     */
    public static void main(String[] argv) {
        double[][] R = createInverseMatrix(17.5, 18.125);
        double[][] F = createForwardMatrix(17.5, 18.125);
        double r = 3;
        double[] V;
        double[] O;
      
        System.out.println();
        V = new double[] {6*Math.PI, 0, 0};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = inverse(V, R, r);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = forward(O, F, r);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
       
        System.out.println();
        V = new double[] {6*Math.PI*Math.sqrt(2)/2, 6*Math.PI*Math.sqrt(2)/2, 0};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = inverse(V, R, r);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = forward(O, F, r);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
     
        System.out.println();
        V = new double[] {0, 6*Math.PI, 0};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = inverse(V, R, r);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = forward(O, F, r);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
      
        System.out.println();
        V = new double[] {0, 0, Math.PI/2};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = inverse(V, R, r);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = forward(O, F, r);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        
        System.out.println();
        MecanumMath mecanum = new MecanumMath(17.5, 18.125, 3, 3, 1, 180/Math.PI); // 17-1/4" x 18-1/8", 6" diameter, inches/second, inches/second, degrees/second

        System.out.println();
        V = new double[] {120, 0, 0};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = mecanum.inverse(V);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = mecanum.forward(O);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
       
        System.out.println();
        V = new double[] {120*Math.sqrt(2)/2, 120*Math.sqrt(2)/2, 0};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = mecanum.inverse(V);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = mecanum.forward(O);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
     
        System.out.println();
        V = new double[] {0, 120, 0};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = mecanum.inverse(V);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = mecanum.forward(O);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
      
        System.out.println();
        V = new double[] {0, 0, 90};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = mecanum.inverse(V);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = mecanum.forward(O);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        
        // confirm the legality of interpolating wheel speeds
        System.out.println();
        double length = 17.375;
        double width = 18.0;
        double wheelRadius = 3;
		double wheelSpeedFactor = 8192 / 10 / (2 * Math.PI); // Talon SRX speed units here are clicks/100ms, this converts to radians/second
		double robotSpeedFactor = 1; // will work everywhere with length units (e.g, inches) per second, so no conversion factor
		double circumference = Math.PI * Math.sqrt(length*length + width*width);
		double rotationSpeedFactor = circumference / (2 * Math.PI); // length units per second of wheel contact points about the center of rotation, this converts to radians/second
		MecanumMath math = new MecanumMath(length, width, wheelRadius, wheelSpeedFactor, robotSpeedFactor, rotationSpeedFactor);
		double[][] vRandom = new double[10][];
		Random random = new Random();
		for (int i = 0; i < 10; i++) {
			vRandom[i] = new double[] {2*random.nextInt(36)-36, 2*random.nextInt(36)-36, 2*random.nextInt(36)-36};
		}
		for (int i = 0; i < 10; i++) {
			double[] wI = math.inverse(vRandom[i]); // non-conflicting wheel speeds
			for (int j = 0; j < 10; j++) {
				double[] wJ = math.inverse(vRandom[j]); // non-conflicting wheel speeds
				double[] wIJ = {(3*wI[0]+2*wJ[0])/5, (3*wI[1]+2*wJ[1])/5, (3*wI[2]+2*wJ[2])/5, (3*wI[3]+2*wJ[3])/5};
				double[] wIJ2 = math.inverse(math.forward(wIJ)); // if wIJ2 equals wIJ, wIJ is also non-conflicting
				System.out.println(
						(int)Math.round(wIJ[0]) + "/" + (int)Math.round(wIJ2[0]) + "\t" +
						(int)Math.round(wIJ[1]) + "/" + (int)Math.round(wIJ2[1]) + "\t" +
						(int)Math.round(wIJ[2]) + "/" + (int)Math.round(wIJ2[2]) + "\t" +
						(int)Math.round(wIJ[3]) + "/" + (int)Math.round(wIJ2[3]));
			}
		}
    }
}
