package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.util.*;

public class Controller {
    private static Controller instance = new Controller();
    public static Controller getInstance() {return instance;}

    /* Controllers */

    
    private final XboxController op;     
   
 

    private final Trigger motorGoButton;
    private final Trigger motorStopButton;
    private final Trigger motorVelocity;
  

    public Controller() {

        // Define control sticks: Translation stick, Rotation stick, and XboxController(aka "op")
      
        op = new XboxController(2);

        motorGoButton = new JoystickButton(op, Button.kA.value);
        motorStopButton = new JoystickButton(op, Button.kB.value);
        motorVelocity = new JoystickButton(op, Button.kX.value);
    }


    public Trigger getGoButton() {return motorGoButton;}
    public Trigger getStopButton() {return motorStopButton;}
    public Trigger getVelocityButton() {return motorVelocity;}
}
