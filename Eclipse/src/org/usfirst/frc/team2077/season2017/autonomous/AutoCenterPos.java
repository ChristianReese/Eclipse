package org.usfirst.frc.team2077.season2017.autonomous;

import org.usfirst.frc.team2077.season2017.commands.DriveToPinWithVision;
import org.usfirst.frc.team2077.season2017.commands.GearEjectIfOnPin;
import org.usfirst.frc.team2077.season2017.commands.GearRest;
import org.usfirst.frc.team2077.season2017.commands.MoveRelative;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoCenterPos extends CommandGroup {

    public AutoCenterPos() {

    	addSequential(new DriveToPinWithVision(78, 24, .15));
    	addSequential(new GearEjectIfOnPin());
    	addSequential(new GearRest());
    	addSequential(new MoveRelative(.15, new double[]{-18, 0, 0}));
    	//addSequential(new MoveRelative(.15, new double[]{0, 0, 90})); // shouldn't do unless we're sure gear unloaded
    }
}
