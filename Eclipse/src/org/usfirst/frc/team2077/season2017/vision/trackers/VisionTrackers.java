package org.usfirst.frc.team2077.season2017.vision.trackers;

/**
 * Use this to create specific types of vision trackers. One *cannot*
 * construct the specific classes directly, as they are package-private.
 * There is no need to instantiate this class, as all methods are static.
 * @author Christian Reese
 *
 */
public abstract class VisionTrackers 
{
	/**
	 * Constructs a new gear lift tracker. Does not execute it.
	 * @return A vision tracker to track the closest gear lift.
	 */
	public static VisionTracker createGearLiftTracker(String camera)
	{
        return new GearLiftTracker(camera);
    }
	
	/**
	 * Constructs a new feeding station tracker. Does not execute it.
	 * @return A vision tracker to track the closest feeding station.
	 */
	public static VisionTracker createFeedingStationTracker(String camera)
	{
        return new FeedingStationTracker(camera);
    }
	
	// Other kinds of trackers possibly following...
}
