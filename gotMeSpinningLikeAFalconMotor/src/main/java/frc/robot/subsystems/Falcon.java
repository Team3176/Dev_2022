// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.function.FloatConsumer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Falcon extends SubsystemBase {
  private static Falcon instance = new Falcon();
  TalonSRX mytalon = new TalonSRX(1);

  /** Creates a new ExampleSubsystem. */
  public Falcon() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  public void runAt80Percent() {
    mytalon.set(ControlMode.PercentOutput, 80);
  }
  
  public void runAt0Percent() {
    mytalon.set(ControlMode.PercentOutput, 0);
  }
 


  public static Falcon getInstance() {
    return instance;
  }

}
