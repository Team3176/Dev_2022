// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SingleVictor;

public class VictorShuffleboardTest extends CommandBase {
  /** Creates a new VictorShuffleboardTest. */

  private SingleVictor m_SingleVictor = SingleVictor.getInstance();

  public VictorShuffleboardTest() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_SingleVictor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_SingleVictor.shuffleboardPercentOutput();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
