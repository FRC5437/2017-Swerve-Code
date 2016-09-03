package org.usfirst.frc.team2783.robot;

import org.usfirst.frc.team2783.robot.commands.SwerveDrive;
import org.usfirst.frc.team2783.robot.commands.SwerveTankDrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public static Joystick xBoxController = new Joystick(RobotMap.XBOX_CONTROLLER_ID);
	
	Button forwardThrottleControl = new JoystickButton(xBoxController, 3);
	Button backwardThrottleControl = new JoystickButton(xBoxController, 4);
	
	//In order: L Stick Y Axis, L Stick X Axis, R Stick X Axis
	Button swerveFB = new JoystickButton(xBoxController, 1);
	Button swerveRL = new JoystickButton(xBoxController, 0);
	Button swerveRot = new JoystickButton(xBoxController, 4);
	
	public OI() {
		
		forwardThrottleControl.whileActive(new SwerveTankDrive());
		backwardThrottleControl.whileActive(new SwerveTankDrive());
		
		//swerveFB.whileActive(new SwerveDrive());
		//swerveRL.whileActive(new SwerveDrive());
		//swerveRot.whileActive(new SwerveDrive());
	}
	
}
