// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;


public class ShooterSubsystem extends SubsystemBase {
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(1, 0, 0)
    .withSimClosedLoopController(0.2, 0.1, 0)
    .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
    .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
    .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
    .withMotorInverted(false)
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(40));

    private SparkMax spark = new SparkMax(4, MotorType.kBrushless);

    private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(sparkSmartMotorController)
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(1))
    .withUpperSoftLimit(RPM.of(1000))
    .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

    private FlyWheel shooter = new FlyWheel(shooterConfig);

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
    public Command setVelocity(AngularVelocity speed) {return shooter.setSpeed(speed);}

    /**
     * Set the shooter velocity to the high value.
     * 
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setHighVelocity() {
      return setVelocity(RPM.of(ShooterConstants.SHOOTER_HIGH_SPEED));
    }

    /**
     * Set the shooter velocity to the low value.
     * 
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setLowVelocity() {
      return setVelocity(RPM.of(ShooterConstants.SHOOTER_LOW_SPEED));
    }

    /**
     * Sets the shooter velocity to zero.
     * 
     * @return {@link edu.wpi.first.wpilibj2.command.runCommand}
     */
    public Command setZeroVelocity() {
      return setVelocity(RPM.of(0));
    }

    /**
     * Set the dutycycle of the shooter.
     * 
     * @param dutyCycle DutyCycle to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command set(double dutyCycle) {return shooter.set(dutyCycle);}

    /**
     * Set the dutycycle of the shooter to the high value.
     * 
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setHigh() {
      return set(ShooterConstants.HIGH_DUTY_CYCLE);
    }

    /**
     * Set the dutycycle of the shooter to the high value.
     * 
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setLow() {
      return set(ShooterConstants.LOW_DUTY_CYCLE);
    }

    /**
     * Set the dutycycle of the shooter to zero.
     * 
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setZero() {
      return set(0);
    }


  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooter.simIterate();
  }
}
