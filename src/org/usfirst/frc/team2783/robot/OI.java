package org.usfirst.frc.team2783.robot;

import org.usfirst.frc.team2783.robot.commands.DriveVictor;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public static Joystick xBoxController = new Joystick(RobotMap.XBOX_CONTROLLER_ID);
	
	//In order: L Stick Y Axis, L Stick X Axis, R Stick X Axis
	Button driveVictor = new JoystickButton(xBoxController, 1);
	
	public OI() {
		driveVictor.whileActive(new DriveVictor());
	}
	
}
