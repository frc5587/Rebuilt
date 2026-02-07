// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.math.Vector3;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class DrivebaseConstants {
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED  = Units.feetToMeters(14.5);

    public static final double WHEEL_LOCK_TIME = 10; //seconds
  }

  public static class OperatorConstants {
    public static final double DEADBAND = 0.1;
  }

  public static class ShooterConstants {
    public static final double SHOOTER_HIGH_SPEED = 300.0;
    public static final double SHOOTER_LOW_SPEED = 60.0;
    public static final double HIGH_DUTY_CYCLE = 0.3;
    public static final double LOW_DUTY_CYCLE = -0.3;

    // Robot constants
    public static final double PITCH = 1.2217304764;
    public static final double SHOTS_PER_SECOND = 2;
    public static final double SHOT_SPEED_CONVERSION_FACTOR = 300;
    public static final Vector3 SHOOTER_POSITION = new Vector3(0.2,0,0.4);
    public static final double MAX_SHOT_SPEED = 20;
    public static final double MIN_SHOOT_SPEED = 1;
    public static final int SEARCH_DEPTH = 10;
    public static final double GRAVITY = 9.81;
  }
}
