// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Controller {

    private final XboxController m_driverController;
    private final Joystick leftJoystick;
    private final Joystick rightJoystick;
  
    private double leftYValue;
    private double rightYValue;
    private double rightXValue;
  
    // for easy switching between scaling constants in Shuffleboard
    private boolean wasButtonPressed;
    private final double[] scalingConstants = {0.25, 0.33, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0};
    private int scalingConstantIndex;

    public Controller()
    {
        m_driverController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
        leftJoystick = new Joystick(Constants.JOYSTICK_LEFT_PORT);
        rightJoystick = new Joystick(Constants.JOYSTICK_RIGHT_PORT);
      
        leftYValue = 0.0;
        rightYValue = 0.0;
        rightXValue = 0.0;
      
        // for easy switching between scaling constants in Shuffleboard
        wasButtonPressed = false;
        scalingConstantIndex = 4;
    }

}
