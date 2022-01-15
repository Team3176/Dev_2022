// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.RunFalcon;
import frc.robot.commands.StopFalcon;
import frc.robot.commands.Velocity;
import frc.robot.subsystems.Falcon;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Falcon m_Falcon = new Falcon();

  //private final RunFalcon m_autoCommand = new RunFalcon(m_Falcon);

  //private XboxController m_Controller;

  private Controller m_Controller;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_Controller = Controller.getInstance();

    // Configure the button bindings
    m_Falcon.getInstance();

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_Controller.getGoButton().whenActive(new RunFalcon());
    //m_Controller.getGoButton().whenPressed(new RunFalcon());
    m_Controller.getStopButton().whenActive(new StopFalcon());
    m_Controller.getVelocityButton().whenActive(new Velocity());
    //m_Controller.getAButton();
    //if(getAButton()){

    //}
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
/*  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
  }
  */
}
