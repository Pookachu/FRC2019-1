/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6132.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();

	
	//Initialize SpeedControllers to control motors.
	SpeedController motor0 = new Talon(0);
	SpeedController motor1 = new Talon(1);
	SpeedController motor2 = new Talon(2);
	SpeedController motor3 = new Talon(3);
	//Create Mecanum Drive Object (From WPILIB)
	MecanumDrive mecanum = new MecanumDrive(motor0, motor1, motor2, motor3);
	
	//initialize Xbox Controller
	Joystick Xbox = new Joystick(0);
	
	//initialize Gyro
	AnalogGyro Gyro = new AnalogGyro(0);
	
	//Define Variables and their defaults
	double SpeedModifier = 0.5;

	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
	}
	

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		System.out.println("Autonomous has started.");
		m_autoSelected = m_chooser.getSelected();
		// m_autoSelected = SmartDashboard.getString("Auto Selector",
		// 		kDefaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
	}


	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case kCustomAuto:
				// Put custom auto code here
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
		}
	}

	@Override
	public void teleopInit() {
		System.out.println("Teleop has started. SpeedModifier is set to " + SpeedModifier);
	}
	
	@Override
	public void teleopPeriodic() {
		//Drive robot with WPILIB Mecanum. If rotate does not work, try changing the third axis from 3 (Triggers) to 1 (Left Stick X Axis)
		mecanum.driveCartesian(Xbox.getRawAxis(4)*SpeedModifier, Xbox.getRawAxis(5)*SpeedModifier, Xbox.getRawAxis(3)*SpeedModifier);
		//Try the following line for field-relative drive:
		//mecanum.driveCartesian(Xbox.getRawAxis(4)*SpeedModifier, Xbox.getRawAxis(5)*SpeedModifier, Xbox.getRawAxis(3)*SpeedModifier, Gyro.getAngle());
		
		if (Xbox.getRawButton(5)) {
			SpeedModifier = SpeedModifier - 0.1;
			if (SpeedModifier <= 0) {
				SpeedModifier = 0;
				System.out.println("SpeedModifier is at Zero.");
			} else {
				System.out.println("SpeedModifier set to " +  SpeedModifier);
			}
			Timer.delay(0.3);
		}
		
		if (Xbox.getRawButton(6)) {
			SpeedModifier = SpeedModifier + 0.1;
			if (SpeedModifier >= 1) {
				SpeedModifier = 1;
				System.out.println("SpeedModifier is at One.");
			} else {
				System.out.println("SpeedModifier set to " + SpeedModifier);
			}
			Timer.delay(0.3);
		}
	}

	@Override
	public void testPeriodic() {
	}
}
