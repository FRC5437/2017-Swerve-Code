package org.usfirst.frc.team2783.robot.subsystems;

import org.usfirst.frc.team2783.robot.RobotMap;
import org.usfirst.frc.team2783.robot.commands.SwerveDrive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class SwerveDriveBase extends Subsystem {
	private CANTalon frontRightWheel;
	private CANTalon frontRightSwivel;
	
	private CANTalon frontLeftWheel;
	private CANTalon frontLeftSwivel;
	
	private CANTalon rearRightWheel;
	private CANTalon rearRightSwivel;
	
	private CANTalon rearLeftWheel;
	private CANTalon rearLeftSwivel;
	
	private AHRS navSensor;
	
	private Encoder frontRightEnc;
	private Encoder frontLeftEnc;
	private Encoder rearRightEnc;
	private Encoder rearLeftEnc;
	
	private PIDController frontRightPID;
	private PIDController frontLeftPID;
	private PIDController rearRightPID;
	private PIDController rearLeftPID;
	
    public SwerveDriveBase() {
    	super();
    	
    	//TODO: Correct DInput
    	frontRightEnc = new Encoder(new DigitalInput(0), new DigitalInput(1));
    	frontLeftEnc = new Encoder(new DigitalInput(2), new DigitalInput(3));
    	rearRightEnc = new Encoder(new DigitalInput(4), new DigitalInput(5));
    	rearLeftEnc = new Encoder(new DigitalInput(6), new DigitalInput(7));
    	
    	frontRightPID = new PIDController(0.1, 0.01, 0, frontRightEnc, frontRightSwivel);
    	frontLeftPID = new PIDController(0.1, 0.01, 0, frontLeftEnc, frontLeftSwivel);
    	rearRightPID = new PIDController(0.1, 0.01, 0, rearRightEnc, rearRightSwivel);
    	rearLeftPID = new PIDController(0.1, 0.01, 0, rearLeftEnc, rearLeftSwivel);
    	
    	try {
	         navSensor = new AHRS(SPI.Port.kMXP);
	     } catch (RuntimeException ex ) {
	         DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
	     }
    	
    	frontRightWheel = new CANTalon(RobotMap.FRONT_RIGHT_WHEEL);
    	frontRightSwivel = new CANTalon(RobotMap.FRONT_RIGHT_SWIVEL);
    	
    	frontLeftWheel = new CANTalon(RobotMap.FRONT_LEFT_WHEEL);
    	frontLeftSwivel = new CANTalon(RobotMap.FRONT_LEFT_SWIVEL);
    	
    	rearRightWheel = new CANTalon(RobotMap.REAR_RIGHT_WHEEL);
    	rearRightSwivel = new CANTalon(RobotMap.REAR_RIGHT_SWIVEL);
    	
    	rearLeftWheel = new CANTalon(RobotMap.REAR_LEFT_WHEEL);
    	rearLeftSwivel = new CANTalon(RobotMap.REAR_LEFT_SWIVEL);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new SwerveDrive());
    }
    
    public void tankDrive(double leftValue, double rightValue) {
    	if (DriverStation.getInstance().isFMSAttached() && DriverStation.getInstance().getMatchTime() < 4) {
    		setBrake(true);
    	} else {
    		setBrake(false);
    	}
    	
    	frontRightWheel.set(rightValue);
    	rearRightWheel.set(rightValue);
    	
    	frontLeftWheel.set(leftValue);
    	rearLeftWheel.set(leftValue);
    }
    
    public void swerveDrive(double FBMotion, double RLMotion, double rotMotion) {
    	FBMotion = (FBMotion*(Math.sin(getNavSensor().getAngle()))) + (RLMotion*(Math.cos(getNavSensor().getAngle())));
    	RLMotion = -(FBMotion*(Math.cos(getNavSensor().getAngle()))) + (RLMotion*(Math.sin(getNavSensor().getAngle())));
    	
    	double L = 2.0;
    	double W = 2.0;
    	double R = Math.sqrt((L*L) + (W*W));
    	
    	double A = RLMotion - rotMotion*(L/R);
    	double B = RLMotion + rotMotion*(L/R);
    	double C = FBMotion - rotMotion*(W/R);
    	double D = FBMotion + rotMotion*(W/R);
    	
    	double frontRightWheelSpeed = Math.sqrt((B*B) + (C*C));
    	double frontLeftWheelSpeed = Math.sqrt((B*B) + (D*D));
    	double rearLeftWheelSpeed = Math.sqrt((A*A) + (D*D));
    	double rearRightWheelSpeed = Math.sqrt((A*A) + (C*C));
    	
    	double t = 180/Math.PI;
    	
    	double frontRightAngle = Math.atan2(B, C)*t;
    	double frontLeftAngle = Math.atan2(B, D)*t;
    	double rearLeftAngle = Math.atan2(A, C)*t;
    	double rearRightAngle = Math.atan2(A, C)*t;
    	 
    	double max = frontRightWheelSpeed;
    	if(max < frontLeftWheelSpeed) {max = frontLeftWheelSpeed;}
    	if(max < rearLeftWheelSpeed) {max = rearLeftWheelSpeed;}
    	if(max < rearRightWheelSpeed) {max = rearRightWheelSpeed;}
    	//I'm so sorry Jake
    	
    	if(max > 1) {
    		frontRightWheelSpeed /= max;
    		frontLeftWheelSpeed /= max;
    		rearLeftWheelSpeed /= max;
    		rearRightWheelSpeed /= max;
    	}
    	
    	frontRightWheel.set(frontRightWheelSpeed);
    	frontLeftWheel.set(frontLeftWheelSpeed);
    	rearLeftWheel.set(rearLeftWheelSpeed);
    	rearRightWheel.set(rearRightWheelSpeed);
    	
    	setFrontRightAngle(frontRightAngle);
    	setFrontLeftAngle(frontLeftAngle);
    	setRearLeftAngle(rearLeftAngle);
    	setRearRightAngle(rearRightAngle);
    	
    }
    
    public void setBrake(boolean brake) {
    	frontRightWheel.enableBrakeMode(brake);
    	frontRightSwivel.enableBrakeMode(brake);
    	
    	frontLeftWheel.enableBrakeMode(brake);
    	frontLeftSwivel.enableBrakeMode(brake);
    	
    	rearRightWheel.enableBrakeMode(brake);
    	rearRightSwivel.enableBrakeMode(brake);
    	
    	rearLeftWheel.enableBrakeMode(brake);
    	rearLeftSwivel.enableBrakeMode(brake);
    }
    
    public AHRS getNavSensor() {
    	if (navSensor != null) {
    		return navSensor;
    	} else {
    		return null;
    	}
    }
    
    public Double getFrontRightEncPercent() {
    	return Math.abs(frontRightEnc.getDistance() / 100.0);
    }
    
    public Double getFrontLeftEncPercent() {
    	return Math.abs(frontLeftEnc.getDistance() / 100.0);
    }
    
    public Double getRearRightEncPercent() {
    	return Math.abs(rearRightEnc.getDistance() / 100.0);
    }
    
    public Double getRearLeftEncPercent() {
    	return Math.abs(rearLeftEnc.getDistance() / 100.0);
    }
    
    public double getFrontRightAngle(){
		if (frontRightEnc != null) {
			return (getFrontRightEncPercent());
		} else {
			return -1.0;
		}
	}
    
    public double getFrontLeftAngle(){
		if (frontLeftEnc != null) {
			return (getFrontLeftEncPercent());
		} else {
			return -1.0;
		}
	}
    
    public double getRearRightAngle(){
		if (rearRightEnc != null) {
			return (getRearRightEncPercent());
		} else {
			return -1.0;
		}
	}
    
    public double getRearLeftAngle(){
		if (rearLeftEnc != null) {
			return (getRearLeftEncPercent());
		} else {
			return -1.0;
		}
	}
    
    public void setFrontRightAngle(double angle) {
    	frontRightPID.enable();
    	frontRightPID.setSetpoint(angle);
    }
    
    public void setFrontLeftAngle(double angle) {
    	frontLeftPID.enable();
    	frontLeftPID.setSetpoint(angle);
    }
    
    public void setRearRightAngle(double angle) {
    	rearRightPID.enable();
    	rearRightPID.setSetpoint(angle);
    }
    
    public void setRearLeftAngle(double angle) {
    	rearLeftPID.enable();
    	rearLeftPID.setSetpoint(angle);
    }
    
    public void frontRightDrive(double value) {
    	frontRightWheel.set(value);
    }
    
    public void frontRightTwist(double value) {
    	frontRightSwivel.set(value);
    }
    
    public void frontLeftDrive(double value) {
    	frontLeftWheel.set(value);
    }
    
    public void frontLeftTwist(double value) {
    	frontLeftSwivel.set(value);
    }
    
    public void rearRightDrive(double value) {
    	rearRightWheel.set(value);
    }
    
    public void rearRightTwist(double value) {
    	rearRightSwivel.set(value);
    }
    
    public void rearLeftDrive(double value) {
    	rearLeftWheel.set(value);
    }
    
    public void rearLeftTwift(double value) {
    	rearLeftSwivel.set(value);
    }
    
}

