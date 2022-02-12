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
  public final double[] scalingConstants = {0.25, 0.33, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0};
  private int scalingConstantIndex = 4;
  
  private final VictorSP leftFrontMotor = new VictorSP(Constants.VICTOR_LEFT_FRONT_PWM_CH);
  private final VictorSP leftBackMotor = new VictorSP(Constants.VICTOR_LEFT_BACK_PWM_CH);
  private final MotorControllerGroup leftMotorGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);

  private final VictorSP rightFrontMotor = new VictorSP(Constants.VICTOR_RIGHT_FRONT_PWM_CH);
  private final VictorSP rightBackMotor = new VictorSP(Constants.VICTOR_RIGHT_BACK_PWM_CH);
  private final MotorControllerGroup rightMotorGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  private final XboxController m_driverController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
  private final Joystick leftJoystick = new Joystick(Constants.JOYSTICK_LEFT_PORT);
  private final Joystick rightJoystick = new Joystick(Constants.JOYSTICK_RIGHT_PORT);

  private double leftYValue;
  private double rightYValue;
  private double rightXValue;

  private boolean wasButtonPressed = false;

  /**
   * Increments up and down the scalingConstants list when testing several constants quickly with
   * Shuffleboard using the A (increment down) and Y (increment up) buttons on an Xbox controller
   * @author Jared Brown
   */
  public void switchScalingConstantXbox()
  {
    if (wasButtonPressed) 
    {
      if (!m_driverController.getAButton() && !m_driverController.getYButton()) 
      {
        wasButtonPressed = false;
      }
    } else if (m_driverController.getYButton() && scalingConstantIndex < scalingConstants.length - 1) {

      scalingConstantIndex++;
      wasButtonPressed = true;
    } else if (m_driverController.getAButton() && scalingConstantIndex > 0)
    {
      scalingConstantIndex--;
      wasButtonPressed = true;
    }
  }

  public void switchScalingConstantJoystick()
  {
    if (wasButtonPressed) 
    {
      if (!leftJoystick.getTrigger() && !rightJoystick.getTrigger()) 
      {
        wasButtonPressed = false;
      }
    } else if (rightJoystick.getTrigger() && scalingConstantIndex < scalingConstants.length - 1) {

      scalingConstantIndex++;
      wasButtonPressed = true;
    } else if (leftJoystick.getTrigger() && scalingConstantIndex > 0)
    {
      scalingConstantIndex--;
      wasButtonPressed = true;
    }
  }


  /**
   * Scales the input from the controller/joystick based on the equation being used and the scaling constant selected
   * @author Jared Brown
   * @param input
   * @return output
   */
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

    this.leftYValue = 0.0;
    this.rightYValue = 0.0;
    this.rightXValue = 0.0;
  }

  @Override
  public void teleopPeriodic() {
    // Drive with tank drive.
    // That means that the Y axis of the left stick moves the left side
    // of the robot forward and backward, and the Y axis of the right stick
    // moves the right side of the robot forward and backward.
    
    /* * * * *
    Two modes: joystick control and Xbox controller control. Comment out whichever block is not being used.
    */


    // Joysticks
    this.leftYValue = leftJoystick.getY();
    this.rightYValue = rightJoystick.getY();
    this.rightXValue = rightJoystick.getX();
    switchScalingConstantJoystick();
    
    /*
    // Xbox controller
    this.leftYValue = m_driverController.getLeftY();
    this.rightYValue = m_driverController.getRightY();
    this.rightXValue = m_driverController.getRightX();
    switchScalingConstantXbox();
    */

    if (Math.abs(leftYValue) < Constants.kDeadbandValue) { leftYValue = 0.0;}
    if (Math.abs(rightYValue) < Constants.kDeadbandValue) { rightYValue = 0.0;}
 
    m_robotDrive.tankDrive(-scaleInput(leftYValue), -scaleInput(rightYValue));

    /* * * Arcade drive!
    m_robotDrive.arcadeDrive(-scaleInput(leftYValue), -scaleInput(rightXValue));
    */

    SmartDashboard.putNumber("leftInput%", -100 * leftYValue);
    SmartDashboard.putNumber("leftOutput%", -100 * scaleInput(leftYValue));
    SmartDashboard.putNumber("rightInput%", -100 * rightYValue);
    SmartDashboard.putNumber("rightOutput%", -100 * scaleInput(rightYValue));

    SmartDashboard.putNumber("scalingConstant", scalingConstants[scalingConstantIndex]);
  }
}

/*
Scaling constant testing on the cart: {0.25, 0.33, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0}
  - Xbox controller X2 was best at 2.0 and 3.0 and deadband 0.05
  - Xbox controller X1 was best at 0.75 and 1.0 and deadband 0.05
  - Xbox controller X3 was best at 0.75 and deadband 0.05
  - Joysticks J-R2 and J-R3 set were best at 0.75, 1.0, and 1.5 and deadband 0.20
  - Joysticks J-N1 and J-N2 were best at 1.0 and deadband 0.10
*/