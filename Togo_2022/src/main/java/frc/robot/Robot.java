// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with tank
 * steering and an Xbox controller.
 */
public class Robot extends TimedRobot {

  private Drivetrain m_Drivetrain;
  private Controller m_Controller;

  @Override
  public void robotInit() {
    m_Drivetrain = Drivetrain.getInstance();
    m_Controller = Controller.getInstance();
  }

  @Override
  public void teleopPeriodic() {

    /*
    // - - - Select the method of driving - - -
    m_Drivetrain.driveTank(m_Controller.getXboxLeftY(), m_Controller.getXboxRightY());
    m_Drivetrain.driveTank(m_Controller.getLeftStickY(), m_Controller.getRightStickY());
    m_Drivetrain.driveArcade(m_Controller.getXboxLeftY(), m_Controller.getXboxRightX());
    m_Drivetrain.driveArcade(m_Controller.getLeftStickY(), m_Controller.getRightStickX());
    */

    /*
    #################### DIRECTIONS ####################

    To control, set the Drive Mode and Controller Type in Shuffleboard.
    For Drive Mode, enter 0 or 1. 0 = tank drive and 1 = arcade drive.
    For Controller Type, enter 0 or 1. 0 = joystick control and 1 = Xbox controller control.

    To scale the input of the controller, adjust the value of Constants.kScalingConstantIndex. This number is the index of the
    desired scaling constant from the Constants.kScalingConstants list. A scaling constant of 1 makes the controller give
    linear input from 0 to 1. A scaling constant between 0 and 1 makes the joystick curve concave up, (lower response at higher
    inputs) and a scaling constant above 1 makes the joystick curve concave down (higher response at lower inputs). (When using
    a scaling constant, 0 input always equals 0 output and full input always equals full output. The curve in between is what
    changes.)

    A useful Shuffleboard layout for monitoring a few inputs from Togo is saved in the Togo_2022 folder.

    CURRENT BUGS:
    1) Sometimes, the Drive Mode number does not get put to Shuffleboard for some reason. If it doesn't, redeploy the code until
      it does.
    2) When in arcade drive, the robot's motors instantly flip the sign of their velocity if the sign of the power input changes
    (left joystick Y). This is to ensure the robot behaves like a car would when driving backwards and steering using arcade
    drive. I've tried to make a ramp so the motors don't change speed so quickly, but it doesn't work and its call in the
    Drivetrain.driveArcade method has been commented out for now.
    */

    m_Drivetrain.driveModeCheck();
    m_Controller.controllerTypeCheck();

    double leftY = 0;
    double rightY = 0;
    double rightX = 0;

    if (m_Controller.getControllerType() == 0) {
      leftY = -m_Controller.getLeftStickY();
      rightY = -m_Controller.getRightStickY();
      rightX = m_Controller.getRightStickX();
    } else if (m_Controller.getControllerType() == 1) {
      leftY = -m_Controller.getXboxLeftY();
      rightY = -m_Controller.getXboxRightY();
      rightX = m_Controller.getXboxRightX();
    }

    if (m_Drivetrain.getDriveMode() == 0) {
      m_Drivetrain.driveTank(leftY, rightY);
    } else if (m_Drivetrain.getDriveMode() == 1) {
      m_Drivetrain.driveArcade(leftY, rightX);
    }

    /* FIX THESE SO THEY CAN ACCESS NUMBERS IN OTHER FILES (write getter methods maybe?)
    SmartDashboard.putNumber("leftInput%", -100 * leftYValue);
    SmartDashboard.putNumber("leftOutput%", -100 * scaleInput(leftYValue));
    SmartDashboard.putNumber("rightInput%", -100 * rightYValue);
    SmartDashboard.putNumber("rightOutput%", -100 * scaleInput(rightYValue));

    SmartDashboard.putNumber("scalingConstant", scalingConstants[scalingConstantIndex]);
    */

  }
}
