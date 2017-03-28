package org.usfirst.frc.team2077.season2017.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

//This code was branched from SpinClimb in Eclipse_2017-03-18-1751
public class Climb extends CommandGroup {

    public Climb(double speed) {

    	addSequential(new Spin(speed));
    	addSequential(new PushPlate());
    }
}
