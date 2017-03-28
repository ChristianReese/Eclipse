package org.usfirst.frc.team2077.season2017.math;

/**
 * Immutable vector in a "compass heading" frame of reference, using north and
 * east component values. Direction 0 is north, by convention the "front" of the
 * robot drive train, with angles increasing clockwise: 90 degrees is east, 180
 * degrees is south, etc. It is designed to represent both positions relative to
 * a fixed reference such as wheel contact points relative to the robot drive
 * train center, and direction-only or magnitude/heading vectors such as wheel
 * orientation and drive vectors. Individual components may be accessed by their
 * respective fields but not changed.
 * @author 2077
 */
public class CompassVector {

	public final double north_;
    public final double east_;
 
    /**
     * North/east component Vector.
     * @param north Magnitude parallel to the north(+)/south(-) axis.
     * @param east  Magnitude parallel to the east(+)/west(-) axis.
     */
    public CompassVector(double north, double east) {
        north_ = north;
        east_ = east;
    }

    /**
     * Direction (unit magnitude) vector.
     * @param heading In degrees clockwise from north.
     */
    public CompassVector(double heading) {
        double radians = Math.toRadians(heading);
        north_ = Math.cos(radians);
        east_ = Math.sin(radians);
    }

    /**
     * Magnitude.
     * @return <code>sqrt(north_*north_ + east_*east_)</code>.
     */
    public double magnitude() {
        return Math.sqrt(north_*north_ + east_*east_);
    }

    /**
     * Unit vector.
     * @return Normalized heading vector.
     */
    public CompassVector unit() {
    	double m = magnitude();
    	return new CompassVector(north_ / m, east_ / m);
    }

    /**
     * New vector of the same heading with magnituded scaled.
     * @param magnitude
     * @return <code>{ north_ * magnitude, east_ * magnitude }</code>.
     */
    public CompassVector scale(double magnitude) {
    	return new CompassVector(north_ * magnitude, east_ * magnitude);
    }

    /**
     * Sum of this vector and another.
     * @param other Other vector.
     * @return <code>{ north_ + other.north_, east_ + other.east_ }</code>.
     */
    public CompassVector plus(CompassVector other) {
    	return new CompassVector(north_ + other.north_, east_ + other.east_);
    }

    /**
     * Difference between this vector and another.
     * @param other Other vector.
     * @return <code>{ north_ - other.north_, east_ - other.east_ }</code>.
     */
    public CompassVector minus(CompassVector other) {
    	return new CompassVector(north_ - other.north_, east_ - other.east_);
    }
 
    /**
     * Dot product with another vector. Typically used to compute the component
     * of an overall drive vector that lies along a unit direction vector.
     * @param other Other vector.
     * @return <code>north_ * other.north_ + east_ * other.east_</code>.
     */
    public double dot(CompassVector other) {
        return north_ * other.north_ + east_ * other.east_;
    }

    /**
     * 
     * @param angle
     * @return
     */
    public CompassVector rotate(double angle) {
    	double radians = Math.toRadians(angle);
    	double sin = Math.sin(radians);
    	double cos = Math.cos(radians);
    	return new CompassVector(north_ * cos - east_ * sin, north_ * sin + east_ * cos);
    }

    @Override
    public String toString() {
    	return "[N:" + north_ + ",E:" + east_ + "]";
    }
}
