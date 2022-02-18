// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Controller {

    private static Controller m_Controller = new Controller();
    private final XboxController m_driverController;
    private final Joystick leftJoystick;
    private final Joystick rightJoystick;
  
    private double leftYValue;
    private double rightYValue;
    private double rightXValue;
  
    // for easy switching between scaling constants in Shuffleboard
    private boolean wasButtonPressed;
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


    public double getLeftStickY() { return leftJoystick.getY(); }
    public double getRightStickY() { return rightJoystick.getY(); }
    public double getRightStickX() { return rightJoystick.getX(); }

    public double getXboxLeftY() { return m_driverController.getLeftY(); }
    public double getXboxRightY() { return m_driverController.getRightY(); }
    public double getXboxRightX() { return m_driverController.getRightX(); }

    public double getScalingConstant() { return Constants.kScalingConstants[scalingConstantIndex]; }


    /**
   * Increments up and down the scalingConstants list when testing several constants quickly with
   * Shuffleboard using the A (increment down) and Y (increment up) buttons on an Xbox controller
   * @author Jared Brown
   */
  public void switchScalingConstantXbox()
  {
    if (wasButtonPressed) 
    {
      if (!m_driverController.getAButton() && !m_driverController.getYButton()) 
      {
        wasButtonPressed = false;
      }
    } else if (m_driverController.getYButton() && scalingConstantIndex < Constants.kScalingConstants.length - 1) {

      scalingConstantIndex++;
      wasButtonPressed = true;
    } else if (m_driverController.getAButton() && scalingConstantIndex > 0)
    {
      scalingConstantIndex--;
      wasButtonPressed = true;
    }
  }

  public void switchScalingConstantJoystick()
  {
    if (wasButtonPressed) 
    {
      if (!leftJoystick.getTrigger() && !rightJoystick.getTrigger()) 
      {
        wasButtonPressed = false;
      }
    } else if (rightJoystick.getTrigger() && scalingConstantIndex < Constants.kScalingConstants.length - 1) {

      scalingConstantIndex++;
      wasButtonPressed = true;
    } else if (leftJoystick.getTrigger() && scalingConstantIndex > 0)
    {
      scalingConstantIndex--;
      wasButtonPressed = true;
    }
  }


  public int getScalingConstantIndex() { return scalingConstantIndex; }

  public static Controller getInstance() { return m_Controller; }

}
