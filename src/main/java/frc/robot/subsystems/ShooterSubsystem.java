// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;


public class ShooterSubsystem extends SubsystemBase {
  private SmartMotorControllerConfig smcConfig = ShooterConstants.APPLY_SMC_CONFIG.apply(new SmartMotorControllerConfig(this));
  private SparkMax spark = new SparkMax(ShooterConstants.MOTOR_ID, MotorType.kBrushless);
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);
  private final FlyWheelConfig shooterConfig = ShooterConstants.APPLY_FLYWHEEL_CONFIG.apply(new FlyWheelConfig(sparkSmartMotorController));
  private FlyWheel shooter = new FlyWheel(shooterConfig);
  private StructArrayPublisher<Pose3d> fuelPosePublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyPoseArray", Pose3d.struct)
    .publish();

  /**
   * Gets the current velocity of the shooter.
   * 
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {return shooter.getSpeed();}

  /**
   * Set the shooter velocity.
   * 
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setAngularVelocity(Supplier<AngularVelocity> speed) {return shooter.setSpeed(speed);}

  /**
   * Converts ball m/s to RPM and sets the flywheel velocity.
   * 
   * @param speed Ball speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setBallVelocity(Supplier<LinearVelocity> speed) {
    return shooter.setSpeed(RPM.of(speed.get().in(MetersPerSecond) * ShooterConstants.SHOT_SPEED_CONVERSION_FACTOR));
  }

  /**
   * Sets the shooter velocity to zero.
   * 
   * @return {@link edu.wpi.first.wpilibj2.command.runCommand}
   */
  public Command stop() {
    return setAngularVelocity(() -> RPM.of(0));
  }

  /**
   * Set the dutycycle of the shooter.
   * 
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {return shooter.set(dutyCycle);}

  /**
   * Run sysId on the {@link Shooter}
   */
  public Command sysID() {
    return shooter.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }


  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooter.updateTelemetry();
    SmartDashboard.putNumber("flywheel velocity", sparkSmartMotorController.getMechanismVelocity().in(RPM));
    if (sparkSmartMotorController.getMechanismSetpointVelocity().isPresent()) {
      SmartDashboard.putNumber("flywheel setpoint", sparkSmartMotorController.getMechanismSetpointVelocity().get().in(RPM));
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooter.simIterate();

    Pose3d[] fuelPoses = SimulatedArena.getInstance()
                                       .getGamePiecesArrayByType("Fuel");       
    fuelPosePublisher.accept(fuelPoses);
  }
}
