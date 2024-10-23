// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

// these are all the constants that never change for example the motor IDs
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants{
    public static final double MAX_SPEED = 4.0;


    public static final double DRIVE_GEAR_RATIO = 6.75; // How many rotation dose the Drive motor take for the wheel to spin once
    public static final double GEAR_RATIO = 21.39;

    public static final double WHEEL_DIAMETER = 0.1016;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // used for odometry position in meters

    public static final int PIGEON_ID = 16;

    public static final int FRONT_LEFT_DRIVE = 2;
    public static final int FRONT_LEFT_STEER = 1;
    public static final int FRONT_LEFT_CANCODER = 10;
    public static final double FRONT_LEFT_OFFSET = 315.0; 

    public static final int FRONT_RIGHT_DRIVE = 3;
    public static final int FRONT_RIGHT_STEER = 4; 
    public static final int FRONT_RIGHT_CANCODER = 12;
    public static final double FRONT_RIGHT_OFFSET = 135.0;

    public static final int BACK_LEFT_DRIVE = 8;
    public static final int BACK_LEFT_STEER = 7;
    public static final int BACK_LEFT_CANCODER = 9;
    public static final double BACK_LEFT_OFFSET = 20.0;

    public static final int BACK_RIGHT_DRIVE = 9;
    public static final int BACK_RIGHT_STEER = 10;
    public static final int BACK_RIGHT_CANCODER = 11;
    public static final double BACK_RIGHT_OFFSET = 180.0;

    public static final HolonomicPathFollowerConfig AUTO_CONFIG = new HolonomicPathFollowerConfig(
      new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
      new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
      4.5, // Max module speed, in m/s for Autonomus
      0.4, // Drive base radius in meters. Distance from robot center to furthest module.
      new ReplanningConfig() // Default path replanning config. See the API for the options here
    );

  }
}
