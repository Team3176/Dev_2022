/*
 * Copyright (C) Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
// package org.photonlib.examples.aimandrange;
package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    // Change this to match the name of your camera
    // PhotonCamera camera = new PhotonCamera("HD_USB_Camera");
    PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    // PID constants should be tuned per robot
    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
    double pitchFilterCoef = 0.1; // Coefficient for filter to smooth out jumpy data from the vision system
    double pitchValue = 0.0; // Output of the filter used for driving autonomously

    // final double ANGULAR_P = 0.1;
    final double ANGULAR_P = 0.01;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0.005, ANGULAR_D);
    double yawFilterCoef = 0.1; // Coefficient for filter to smooth out jumpy data from the vision system
    double yawValue = 0.0; // Output of the filter used for steering autonomously

    XboxController xboxController = new XboxController(0);

    // Drive motors
    PWMVictorSPX leftMotor = new PWMVictorSPX(0);
    PWMVictorSPX rightMotor = new PWMVictorSPX(1);
    DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

    @Override
    public void teleopInit() {
        rightMotor.setInverted(true); // The right motor on a Romi is backwards so we need to invert it.
    }

    @Override
    public void teleopPeriodic() {
        double forwardSpeed;
        double rotationSpeed;

        if (xboxController.getAButton()) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = camera.getLatestResult();

            if (result.hasTargets()) {
                // First calculate range
                // pitchValue = pitchFilterCoef * result.getBestTarget().getPitch() + (1.0 - pitchFilterCoef) * pitchValue;
                // double range =
                //         PhotonUtils.calculateDistanceToTargetMeters(
                //                 CAMERA_HEIGHT_METERS,
                //                 TARGET_HEIGHT_METERS,
                //                 CAMERA_PITCH_RADIANS,
                //                 Units.degreesToRadians(pitchValue));

                // // Use this range as the measurement we give to the PID controller.
                // // -1.0 required to ensure positive PID controller effort _increases_ range
                // forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
                forwardSpeed = -0.4; // The camera is mounted on the back of the Romi so backward is forward

                // Also calculate angular power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw
                yawValue = yawFilterCoef * (1.0 * result.getBestTarget().getYaw()) + (1.0 - yawFilterCoef) * yawValue;
                rotationSpeed = -turnController.calculate(yawValue, 0);
                } else {
                // If we have no targets, stay still.
                forwardSpeed = 0.0;
                rotationSpeed = 0.0;
                yawValue = 0.0;
                pitchValue = 0.0;
            }
        } else {
            // Manual Driver Mode
            forwardSpeed = xboxController.getRightY(); // We are driving the Romi backwards so invert from normal
            rotationSpeed = xboxController.getLeftX();
            // Do controller deadband manually
            if (Math.abs(forwardSpeed) < 0.1)
            {
                forwardSpeed = 0.0;
            }
            if (Math.abs(rotationSpeed) < 0.1)
            {
                rotationSpeed = 0.0;
            }
            yawValue = 0.0;
            pitchValue = 0.0;
        }

        // Use our forward/turn speeds to control the drivetrain
        // drive.arcadeDrive(forwardSpeed, rotationSpeed);
        drive.arcadeDrive(forwardSpeed, rotationSpeed, false);
    }
}