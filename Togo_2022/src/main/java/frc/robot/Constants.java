// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Constants {

    public static final int VICTOR_LEFT_FRONT_PWM_CH = 0;
    public static final int VICTOR_LEFT_BACK_PWM_CH = 1;
    public static final int VICTOR_RIGHT_FRONT_PWM_CH = 2;
    public static final int VICTOR_RIGHT_BACK_PWM_CH = 3;

    public static final int XBOX_CONTROLLER_PORT = 0;
    public static final int JOYSTICK_LEFT_PORT = 0;
    public static final int JOYSTICK_RIGHT_PORT = 1;

    // Check each equation's valid and recommended values list
    public static final double kScalingConstant = 1.0;
    public static final double kDeadbandValue = 0.05;
    public static final double[] kScalingConstants = {0.25, 0.33, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0};
    public static final int kScalingConstantIndex = 5;
    public static final double kMaxPercentWithoutTurbo = 0.75;

    public static final String kDriveModeNameSB = "Drive Mode";
    public static final String kControllerTypeNameSB = "Controller Type";

    public static final int kRampIterationsToUse = 50;
}

/*
Scaling constant testing on the cart: {0.25, 0.33, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0}
  - Xbox controller X2 was best at 2.0 and 3.0 and deadband 0.05
  - Xbox controller X1 was best at 0.75 and 1.0 and deadband 0.05
  - Xbox controller X3 was best at 0.75 and deadband 0.05
  - Joysticks J-R2 and J-R3 set were best at 0.75, 1.0, and 1.5 and deadband 0.20
  - Joysticks J-N1 and J-N2 were best at 1.0 and deadband 0.10
*/
