package org.usfirst.frc.team2783.robot.subsystems;

import org.usfirst.frc.team2783.robot.RobotMap;
import org.usfirst.frc.team2783.robot.commands.DriveVictor;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class OneVictor extends Subsystem {
	
    public VictorSP victor;

    public OneVictor() {
    	victor = new VictorSP(RobotMap.VICTOR);
    }
    
    public void initDefaultCommand() {
        setDefaultCommand(new DriveVictor());
    }
    
    public void driveVictor(double speed){
    	victor.set(speed);
    }
}

