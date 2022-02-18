// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/** Add your docs here. */
public class Drivetrain {

    private static Drivetrain m_Drivetrain = new Drivetrain();
    private Controller m_Controller;
    private VictorSP leftFrontMotor;
    private VictorSP leftBackMotor;
    private MotorControllerGroup leftMotorGroup;
    private VictorSP rightFrontMotor;
    private VictorSP rightBackMotor;
    private MotorControllerGroup rightMotorGroup;
    private DifferentialDrive robotDrive;

    public Drivetrain()
    {
        leftFrontMotor = new VictorSP(Constants.VICTOR_LEFT_FRONT_PWM_CH);
        leftBackMotor = new VictorSP(Constants.VICTOR_LEFT_BACK_PWM_CH);
        leftMotorGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
        rightFrontMotor = new VictorSP(Constants.VICTOR_RIGHT_FRONT_PWM_CH);
        rightBackMotor = new VictorSP(Constants.VICTOR_RIGHT_BACK_PWM_CH);
        rightMotorGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);
        robotDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

        leftMotorGroup.setInverted(true);

        m_Controller = Controller.getInstance();
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
      double output = Math.pow(input, 1 / Constants.kScalingConstants[m_Controller.getScalingConstantIndex()]);

      return output;
    }

    // if negative
    // double output = -Math.pow(input * -1, 1 / Constants.kScalingConstant);

    // for Shuffleboard testing
    double output = -Math.pow(input * -1, 1 / Constants.kScalingConstants[m_Controller.getScalingConstantIndex()]);

    return output;
  }

  public void driveTank(double leftY, double rightY)
  {
    if (Math.abs(leftY) < Constants.kDeadbandValue) { leftY = 0.0;}
    if (Math.abs(rightY) < Constants.kDeadbandValue) { rightY = 0.0;}
    robotDrive.tankDrive(-scaleInput(leftY), -scaleInput(rightY));
  }

  public void driveArcade(double powerY, double steerX)
  {
    if (Math.abs(powerY) < Constants.kDeadbandValue) { powerY = 0.0;}
    if (Math.abs(steerX) < Constants.kDeadbandValue) { steerX = 0.0;}
    robotDrive.tankDrive(-scaleInput(powerY), -scaleInput(steerX));
  }

  public static Drivetrain getInstance() { return m_Drivetrain; }

}
