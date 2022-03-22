// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // Method of driving. 0 = tank drive, 1 = arcade drive
    private int driveMode;

    private double previousPower;
    private int rampIterationsLeft;
    private int rampIterationsToUse;

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

        // Method of driving. 0 = tank drive, 1 = arcade drive
        this.driveMode = 0;
        SmartDashboard.putNumber(Constants.kDriveModeNameSB, this.driveMode);

        this.previousPower = 0;
        this.rampIterationsLeft = 0;
        this.rampIterationsToUse = Constants.kRampIterationsToUse;
    }

    public void driveModeCheck() {
      this.driveMode = (int) SmartDashboard.getNumber(Constants.kDriveModeNameSB, 0);
    }
    
    public int getDriveMode() { return this.driveMode; }

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

  private double backwardArcadeSteerRamp(double newPower, double steerX)
  {
    if ((previousPower >= 0 && newPower < 0) || (previousPower < 0 && newPower >= 0)) {
      this.rampIterationsLeft = Constants.kRampIterationsToUse;
      System.out.println("Ramping started ############################################");
    }
    
    if (this.rampIterationsLeft > 0) {
      steerX = steerX + ((-(2 * steerX)) * ((rampIterationsToUse - rampIterationsLeft) / rampIterationsToUse));
      System.out.println(steerX + " + [" + (-(2 * steerX)) + " * ( (" + rampIterationsToUse + " - " + rampIterationsLeft + " ) / " + rampIterationsToUse + " )]");
      System.out.println("Ramps left: " + this.rampIterationsLeft + " -- steerX = " + steerX + " -- #################");
      this.rampIterationsLeft--;
    }
    return steerX;
  }

  private double scaleForTurbo(double power)
  {
    SmartDashboard.putBoolean("LeftTrigger", m_Controller.getLeftStickTrigger());
    if (m_Controller.getLeftStickTrigger()) {
      return power;
    }
    return power * Constants.kMaxPercentWithoutTurbo;
  }

  /**
   * Drives with robot with tank drive.
   * @param leftY Power on left stick
   * @param rightY Power on right stick
   * @author Jared Brown
   */
  public void driveTank(double leftY, double rightY)
  {
    leftY = this.scaleForTurbo(leftY);
    rightY = this.scaleForTurbo(rightY);
    if (Math.abs(leftY) < Constants.kDeadbandValue) { leftY = 0.0;}
    if (Math.abs(rightY) < Constants.kDeadbandValue) { rightY = 0.0;}
    SmartDashboard.putNumber("leftStickScaled", scaleInput(leftY));
    SmartDashboard.putNumber("rightStickScaled", scaleInput(rightY));
    robotDrive.tankDrive(scaleInput(leftY), scaleInput(rightY));
  }

  /**
   * Drives with robot with arcade drive.
   * @param powerY Power on left stick
   * @param steerX Steering on right stick
   * @author Jared Brown
   */
  public void driveArcade(double powerY, double steerX)
  {
    // steerX = this.backwardArcadeSteerRamp(powerY, steerX);
    // this.previousPower = powerY;

    powerY = this.scaleForTurbo(powerY);
    if (Math.abs(powerY) < Constants.kDeadbandValue) { powerY = 0.0;}
    if (Math.abs(steerX) < Constants.kDeadbandValue) { steerX = 0.0;}
    /* if (this.rampIterationsLeft == 0) { */
      if (powerY < 0) { steerX *= -1; }
    /* } */
    SmartDashboard.putNumber("leftStickScaled", scaleInput(powerY));
    SmartDashboard.putNumber("rightStickScaled", scaleInput(steerX));
    if (Math.abs(steerX) <= 1) {
      robotDrive.arcadeDrive(scaleInput(powerY), scaleInput(steerX));
    }
  }

  public static Drivetrain getInstance() { return m_Drivetrain; }

}
