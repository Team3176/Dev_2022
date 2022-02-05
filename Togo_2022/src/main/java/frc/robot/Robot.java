// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with tank
 * steering and an Xbox controller.
 */
public class Robot extends TimedRobot {
  
  private final VictorSP leftFrontMotor = new VictorSP(Constants.VICTOR_LEFT_FRONT_PWM_CH);
  private final VictorSP leftBackMotor = new VictorSP(Constants.VICTOR_LEFT_BACK_PWM_CH);
  private final MotorControllerGroup leftMotorGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);

  private final VictorSP rightFrontMotor = new VictorSP(Constants.VICTOR_RIGHT_FRONT_PWM_CH);
  private final VictorSP rightBackMotor = new VictorSP(Constants.VICTOR_RIGHT_BACK_PWM_CH);
  private final MotorControllerGroup rightMotorGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  private final XboxController m_driverController = new XboxController(Constants.XBOX_CONTROLLER_PORT);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    leftMotorGroup.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with tank drive.
    // That means that the Y axis of the left stick moves the left side
    // of the robot forward and backward, and the Y axis of the right stick
    // moves the right side of the robot forward and backward.

    double leftYValue = m_driverController.getLeftY();
    double rightYValue = m_driverController.getRightY();

    System.out.println(leftYValue + ", " + rightYValue);

    if (Math.abs(leftYValue) < 0.05) { leftYValue = 0.0;}
    if (Math.abs(rightYValue) < 0.05) { rightYValue = 0.0;}

    m_robotDrive.tankDrive(-leftYValue, -rightYValue);
  }
}
