package org.usfirst.frc.team2783.robot.commands;

import org.usfirst.frc.team2783.robot.OI;
import org.usfirst.frc.team2783.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SwerveDrive extends Command {

    public SwerveDrive() {
    	requires(Robot.swerveBase);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Double throttleValue = OI.xBoxController.getRawAxis(3) - OI.xBoxController.getRawAxis(2);
    	
    	Double FBValue = -(OI.xBoxController.getRawAxis(1))*throttleValue;
    	Double RLValue = OI.xBoxController.getRawAxis(0)*throttleValue;
    	Double rotValue = OI.xBoxController.getRawAxis(4)*throttleValue;
    	
    	Robot.swerveBase.swerveDrive(FBValue, RLValue, rotValue);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

}
