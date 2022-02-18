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

  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private Controller m_Controller = Controller.getInstance();

  @Override
  public void robotInit() {

  }

  @Override
  public void teleopPeriodic() {

    // - - - Select the method of driving - - -
    m_Drivetrain.driveTank(m_Controller.getXboxLeftY(), m_Controller.getXboxRightY());
    m_Drivetrain.driveTank(m_Controller.getLeftStickY(), m_Controller.getRightStickY());
    m_Drivetrain.driveArcade(m_Controller.getXboxLeftY(), m_Controller.getXboxRightX());
    m_Drivetrain.driveArcade(m_Controller.getLeftStickY(), m_Controller.getRightStickX());


    /* FIX THESE SO THEY CAN ACCESS NUMBERS IN OTHER FILES (write getter methods maybe?)
    SmartDashboard.putNumber("leftInput%", -100 * leftYValue);
    SmartDashboard.putNumber("leftOutput%", -100 * scaleInput(leftYValue));
    SmartDashboard.putNumber("rightInput%", -100 * rightYValue);
    SmartDashboard.putNumber("rightOutput%", -100 * scaleInput(rightYValue));

    SmartDashboard.putNumber("scalingConstant", scalingConstants[scalingConstantIndex]);
    */

  }
}
