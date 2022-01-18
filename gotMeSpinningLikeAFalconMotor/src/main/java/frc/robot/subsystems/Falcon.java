// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.function.FloatConsumer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Falcon extends SubsystemBase {
  private static Falcon instance = new Falcon();
  TalonFX mytalon = new TalonFX(1);
  double kP = 0.1;
  double kI = 0.001;
  double kD = 5;
  /** Creates a new ExampleSubsystem. */
  public Falcon() {
  
mytalon.configFactoryDefault();

mytalon.configNeutralDeadband(0.001);

mytalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

mytalon.configNominalOutputForward(0, 30);
mytalon.configNominalOutputReverse(0, 30);
mytalon.configPeakOutputForward(1, 30);
mytalon.configPeakOutputReverse(-1, 30);

mytalon.config_kP(0, kP,30);
mytalon.config_kI(0, kI,30);
mytalon.config_kD(0, kD,30);
  }
  public void velocityControl(double speed){

  mytalon.set(TalonFXControlMode.Velocity, speed);
    System.out.println("FalconInit");
  }


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
    System.out.println("Falcon80");
  }
  
  public void runAt0Percent() {
    mytalon.set(ControlMode.PercentOutput, 0);
    System.out.println("Falcon0");
  }
 


  public static Falcon getInstance() {
    return instance;
  }

}
