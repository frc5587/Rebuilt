// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
    private SparkMax spark = new SparkMax(ShooterConstants.FLYWHEEL_ID, MotorType.kBrushless);
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
    public Command setVelocity(Supplier<AngularVelocity> speed) {return shooter.setSpeed(speed);}

    /**
     * Sets the shooter velocity to zero.
     * 
     * @return {@link edu.wpi.first.wpilibj2.command.runCommand}
     */
    public Command stop() {
      return setVelocity(() -> RPM.of(0));
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
    // try {
    //   SmartDashboard.putNumber("flywheel setpoint", sparkSmartMotorController.get().get().in(RPM));
    // }
    // catch(Exception e) {
    //   SmartDashboard.putNumber("flywheel setpoint", 0.);
    // }

    Pose3d[] fuelPoses = SimulatedArena.getInstance()
            .getGamePiecesArrayByType("Fuel");
    fuelPosePublisher.accept(fuelPoses);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooter.simIterate();
  }
}
