// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/** Add your docs here. */
public class Drivetrain {

    private final VictorSP leftFrontMotor;
    private final VictorSP leftBackMotor;
    private final MotorControllerGroup leftMotorGroup;
    private final VictorSP rightFrontMotor;
    private final VictorSP rightBackMotor;
    private final MotorControllerGroup rightMotorGroup;
  
    private final DifferentialDrive robotDrive;

    public Drivetrain()
    {
        leftFrontMotor = new VictorSP(Constants.VICTOR_LEFT_FRONT_PWM_CH);
        leftBackMotor = new VictorSP(Constants.VICTOR_LEFT_BACK_PWM_CH);
        leftMotorGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
        rightFrontMotor = new VictorSP(Constants.VICTOR_RIGHT_FRONT_PWM_CH);
        rightBackMotor = new VictorSP(Constants.VICTOR_RIGHT_BACK_PWM_CH);
        rightMotorGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);
    
        robotDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    }

}
