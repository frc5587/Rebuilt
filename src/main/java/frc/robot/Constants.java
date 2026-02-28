// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.UnaryOperator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.math.Vector3;
import swervelib.math.Matter;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Normal
  public static class DrivebaseConstants {
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double WHEEL_LOCK_TIME = 10; //seconds

    public static final double MAX_SPEED  = Units.feetToMeters(8);
    public static final double MAX_SPIN_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;
    public static final double MAX_SPIN_ACCEL = 4 * Math.PI;
    public static final double HEADING_DEADBAND = 0.25;
    public static final ProfiledPIDController HEADING_CONTROLLER = new ProfiledPIDController(10,0,0.1,new Constraints(MAX_SPIN_SPEED_RADIANS_PER_SECOND, MAX_SPIN_ACCEL));

    public static final double LOOKAHEAD = 0.1;
    public static final double SHOOT_WHILE_MOVING_SPEED  = Units.feetToMeters(4);
    public static final double SHOOT_WHILE_MOVE_ACCEL_LIMIT = 10;
    public static final ProfiledPIDController SHOOT_WHILE_MOVE_HEADING_CONTROLLER = new ProfiledPIDController(25,0,0,new Constraints(MAX_SPIN_SPEED_RADIANS_PER_SECOND, MAX_SPIN_ACCEL));
  }

  public static class ShooterConstants {
    public static final int MOTOR_ID = 30;
    public static final UnaryOperator<SmartMotorControllerConfig> APPLY_SMC_CONFIG = (SmartMotorControllerConfig config) -> {
      return config.withControlMode(ControlMode.CLOSED_LOOP)
                   .withClosedLoopController(0.05, 0, 0)
                   .withSimClosedLoopController(1, 0, 0)
                   .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                   .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                   .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
                   .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                   .withMotorInverted(false)
                   .withIdleMode(MotorMode.COAST)
                   .withStatorCurrentLimit(Amps.of(40));
    };
    public static final UnaryOperator<FlyWheelConfig> APPLY_FLYWHEEL_CONFIG = (FlyWheelConfig config) -> {
      return config.withDiameter(Inches.of(4))
                   .withMass(Pounds.of(2.5))
                   .withUpperSoftLimit(RPM.of(1000))
                   .withTelemetry("Shooter", TelemetryVerbosity.HIGH);
    };

    // Aiming math
    public static final double PITCH = 1.2217304764;
    public static final double LOOKAHEAD = 0.1;
    public static final double SHOTS_PER_SECOND = 2;
    public static final double TIME_BETWEEN_LOG_TIMESTAMPS = 0.055;
    public static final double SHOT_SPEED_CONVERSION_FACTOR = 187.978279242;
    public static final Vector3 SHOOTER_POSITION = new Vector3(0.2,0,0.4);
    public static final Vector3 BLUE_ALLIANCE_GOAL = new Vector3(4.625626,4.0346315,1.8288);
    public static final Vector3 RED_ALLIANCE_GOAL = new Vector3(11.915426,4.0346315,1.8288);
    public static final Vector3 getGoal(Alliance alliance) {
      if (alliance == Alliance.Red) {
        return RED_ALLIANCE_GOAL;
      }
      return BLUE_ALLIANCE_GOAL;
    }
    public static final int SEARCH_DEPTH = 5;
    public static final double GRAVITY = 9.81;
  }

  public static class ArmConstants {
    public static final int LEFT_MOTOR_ID = 20;
    public static final int RIGHT_MOTOR_ID = 21;
    public static final Angle TOP_ANGLE = Degrees.of(100.);
    public static final Angle BOTTOM_ANGLE = Degrees.of(-5.);
    public static final Angle ZERO_ANGLE = Degrees.of(0.);
    public static final UnaryOperator<SmartMotorControllerConfig> APPLY_SMC_CONFIG = (SmartMotorControllerConfig config) -> {
      return config.withControlMode(ControlMode.CLOSED_LOOP)
                   .withClosedLoopController(5, 0, 0)
                   .withSimClosedLoopController(1, 0, 0)
                   .withFeedforward(new ArmFeedforward(0,0.3, 0))
                   .withSimFeedforward(new ArmFeedforward(0, 0, 0))
                   .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
                   .withGearing(new MechanismGearing(GearBox.fromReductionStages(5.,24./18.)))
                   .withMotorInverted(false)
                   .withIdleMode(MotorMode.BRAKE)
                   .withStatorCurrentLimit(Amps.of(40))

                   .withClosedLoopRampRate(Seconds.of(0.25))
                   .withOpenLoopRampRate(Seconds.of(0.25));
    };
    public static final UnaryOperator<ArmConfig> APPLY_ARM_CONFIG = (ArmConfig config) -> {
      return config.withSoftLimits(BOTTOM_ANGLE,TOP_ANGLE)
                   .withHardLimit(BOTTOM_ANGLE,TOP_ANGLE)
                   .withLength(Inches.of(15.81))
                   .withMass(Pounds.of(2))
                   .withTelemetry("Arm", TelemetryVerbosity.HIGH)
                   .withStartingPosition(TOP_ANGLE);
    };
  }

  public static class IntakeConstants {
    public static final double DUTY_CYCLE = 0.5;
    public static final int MOTOR_ID = 22;
    public static final UnaryOperator<SmartMotorControllerConfig> APPLY_SMC_CONFIG = (SmartMotorControllerConfig config) -> {
      return config.withControlMode(ControlMode.OPEN_LOOP)
             .withTelemetry("IntakeMotor", TelemetryVerbosity.LOW)
             .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
             .withMotorInverted(false)
             .withIdleMode(MotorMode.COAST)
             .withStatorCurrentLimit(Amps.of(20));  
    };
    
    
    public static
     final UnaryOperator<FlyWheelConfig> APPLY_INTAKE_CONFIG = (FlyWheelConfig config) -> {
      return config.withDiameter(Inches.of(2))
                   .withMass(Pounds.of(1))
                   .withTelemetry("Intake", TelemetryVerbosity.LOW);
    };
  }
  
  public static class IndexerConstants {
    public static final int MOTOR_ID = 23;
    public static final UnaryOperator<SmartMotorControllerConfig> APPLY_SMC_CONFIG = (SmartMotorControllerConfig config) -> {
      return config.withControlMode(ControlMode.OPEN_LOOP)
             .withTelemetry("IndexerMotor", TelemetryVerbosity.LOW)
             .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
             .withMotorInverted(false)
             .withIdleMode(MotorMode.COAST)

             .withStatorCurrentLimit(Amps.of(20));  
    };
    public static final UnaryOperator<FlyWheelConfig> APPLY_INDEXER_CONFIG = (FlyWheelConfig config) -> {
      return config.withDiameter(Inches.of(1))
                   .withMass(Pounds.of(1))
    
                   .withTelemetry("Indexer", TelemetryVerbosity.LOW);
    };
  }

  public static class ClimbConstants {
    public static final Angle UP_ANGLE = Rotations.of(77);
    public static final int MOTOR_ID = 40;
    public static final int LIMIT_SWITCH_ID = 0; //TODO set actual id
    public static final UnaryOperator<SmartMotorControllerConfig> APPLY_SMC_CONFIG = (SmartMotorControllerConfig config) -> {
      return config.withControlMode(ControlMode.CLOSED_LOOP)
                   .withClosedLoopController(0.25, 0, 0)
                   .withSimClosedLoopController(1, 0, 0)
                   .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                   .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                   .withTelemetry("ClimbMotor", TelemetryVerbosity.HIGH)
                   .withGearing(new MechanismGearing(GearBox.fromReductionStages(5,4,4)))
                   .withMotorInverted(false)
                   .withIdleMode(MotorMode.BRAKE)
                   .withStatorCurrentLimit(Amps.of(40))
                   .withSoftLimit(Rotations.of(0), Rotations.of(77.0));

    };
  }

  public static class OperatorConstants {
    public static final double DEADBAND = 0.1;
  }
}