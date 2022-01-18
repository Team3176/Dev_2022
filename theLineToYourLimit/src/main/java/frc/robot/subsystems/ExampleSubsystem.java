// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  TalonFX eagle_motor;
  DigitalInput nintendo_switch;
  DigitalInput line_break;
  static ExampleSubsystem instance = new ExampleSubsystem();
  public static ExampleSubsystem getInstance() {
    return instance;
  } 
  public ExampleSubsystem() {
    eagle_motor = new TalonFX(1);
    nintendo_switch = new DigitalInput(1);
    line_break = new DigitalInput(3);
  }
  public void spin() {
    eagle_motor.set(TalonFXControlMode.PercentOutput, 80);    
  }
  public void no_spin() {
    eagle_motor.set(TalonFXControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if(nintendo_switch.get()) {
    //   no_spin();
    // } else {
    //   spin();
    // }
    if(line_break.get()) {
      no_spin();
    } else {
      spin();
    }
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
