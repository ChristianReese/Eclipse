package org.usfirst.frc.team2077.season2017.autonomous;

import org.usfirst.frc.team2077.season2017.commands.DriveToPinWithVision;
import org.usfirst.frc.team2077.season2017.commands.GearEjectIfOnPin;
import org.usfirst.frc.team2077.season2017.commands.GearRest;
import org.usfirst.frc.team2077.season2017.commands.GearRotate;
import org.usfirst.frc.team2077.season2017.commands.MoveRelative;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoRightPos extends CommandGroup {

    public AutoRightPos() {

    	//addSequential(new MoveRelative(0.15, new double[]{71, 0, 0}));
    	addSequential(new MoveRelative(0.15, new double[]{77, 0, 0}));
    	addSequential(new MoveRelative(0.15, new double[]{0, 0, -60}));
    	//addSequential(new DriveToPinWithVision(68, 24, 0.15));
    	addSequential(new DriveToPinWithVision(80, 24, 0.15));
    	//addSequential(new GearRotate());
    	addSequential(new GearEjectIfOnPin());
    	addSequential(new GearRest());
    	addSequential(new MoveRelative(0.15, new double[]{-36, 0, 0}));
    	//addSequential(new MoveRelative(0.15, new double[]{0, 0, 60})); // shouldn't do unless we're sure gear unloaded
    }
}
