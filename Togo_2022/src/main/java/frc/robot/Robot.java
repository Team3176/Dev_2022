// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with tank
 * steering and an Xbox controller.
 */
public class Robot extends TimedRobot {

  // for easy switching between scaling constants in Shuffleboard
  private final double[] scalingConstants = {0.25, 0.5, 1.0, 2.0, 3.0};
  private int scalingConstantIndex = 0;
  
  private final VictorSP leftFrontMotor = new VictorSP(Constants.VICTOR_LEFT_FRONT_PWM_CH);
  private final VictorSP leftBackMotor = new VictorSP(Constants.VICTOR_LEFT_BACK_PWM_CH);
  private final MotorControllerGroup leftMotorGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);

  private final VictorSP rightFrontMotor = new VictorSP(Constants.VICTOR_RIGHT_FRONT_PWM_CH);
  private final VictorSP rightBackMotor = new VictorSP(Constants.VICTOR_RIGHT_BACK_PWM_CH);
  private final MotorControllerGroup rightMotorGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  // with Xbox controller
  private final XboxController m_driverController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
  /*
  // with joysticks
  private final Joystick leftJoystick = new Joystick(Constants.JOYSTICK_LEFT_PORT);
  private final Joystick rightJoystick = new Joystick(Constants.JOYSTICK_RIGHT_PORT);
  */

  /**
   * Increments up and down the scalingConstants list when testing several constants quickly with
   * Shuffleboard using the A (increment down) and Y (increment up) buttons on an Xbox controller
   * @author Jared Brown
   */
  public void switchScalingConstant()
  {
    // with Xbox controller
    if (m_driverController.getYButton() && scalingConstantIndex < scalingConstants.length - 1)
    {
      scalingConstantIndex++;
    } else if (m_driverController.getAButton() && scalingConstantIndex > 0)
    {
      scalingConstantIndex--;
    }
  }

  public double scaleInput(double input)
  {
    // "a" is the scaling constant

    // - - - - -
    // Equation 1: f(x) = b(x) + a(x^3)
    // Recommended: 0 <= a <= 1
    // double output = ((1 - Constants.kScalingConstant) * input) + (Constants.kScalingConstant * Math.pow(input, 3));

    // - - - - -
    // Equation 2: f(x) = x^(1/c) ... because this function does weird stuff with negative input, it will run negative input like a
    // positive input and then flip it at the end
    // Recommended: 0.25 <= a <= 5 ... Valid: a > 0
    if (input >= 0) {
      // double output = Math.pow(input, 1 / Constants.kScalingConstant);
      // for Shuffleboard testing
      double output = Math.pow(input, 1 / this.scalingConstants[scalingConstantIndex]);
      return output;
    }
    // if negative
    // double output = -Math.pow(input * -1, 1 / Constants.kScalingConstant);
    // for Shuffleboard testing
    double output = -Math.pow(input * -1, 1 / this.scalingConstants[scalingConstantIndex]);
    return output;

  }


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

    switchScalingConstant();

    // with Xbox controller
    double leftYValue = m_driverController.getLeftY();
    double rightYValue = m_driverController.getRightY();
    /*
    // with joysticks
    double leftYValue = leftJoystick.getY();
    double rightYValue = rightJoystick.getY();
    */


    if (Math.abs(leftYValue) < Constants.kDeadbandValue) { leftYValue = 0.0;}
    if (Math.abs(rightYValue) < Constants.kDeadbandValue) { rightYValue = 0.0;}

    m_robotDrive.tankDrive(-scaleInput(leftYValue), -scaleInput(rightYValue));

    SmartDashboard.putNumber("leftInput", -leftYValue);
    SmartDashboard.putNumber("leftOutput", -scaleInput(leftYValue));
    SmartDashboard.putNumber("rightInput", -rightYValue);
    SmartDashboard.putNumber("rightOutput", -scaleInput(rightYValue));

    SmartDashboard.putNumber("scalingConstant", scalingConstants[scalingConstantIndex]);
  }
}

/*
Of the scaling constants I tested {0.25, 0.5, 1.0, 2.0, 3.0}...
  - The Logitech controller did best with constants 0.5 and 1.0
  - The Xbox controller did beest with constants 1.0 and 2.0
*/