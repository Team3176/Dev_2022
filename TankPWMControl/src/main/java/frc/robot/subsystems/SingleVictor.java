// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SingleVictor extends SubsystemBase {
  /** Creates a new SingleVictor. */

  private static SingleVictor m_SingleVictor = new SingleVictor();
  private VictorSP victorMotor;

  public SingleVictor() {

    victorMotor = new VictorSP(Constants.SINGLE_VICTOR_PWM_CH);
    SmartDashboard.putNumber("SingleVictorPercent", 0.0);
    System.out.println("Constructor run");
    
  }

  public void percentOutput(double percent)
  {
    if (percent >= -1 && percent <= 1) {
      victorMotor.set(percent);
    }
  }

  public void shuffleboardPercentOutput()
  {
    double percent = SmartDashboard.getNumber("SingleVictorPercent", 0.0);
    if (percent >= -1.0 && percent <= 1.0) {
      victorMotor.set(percent);
    }
    System.out.println("run");
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public static SingleVictor getInstance()
  {
    return m_SingleVictor;


  }



}
