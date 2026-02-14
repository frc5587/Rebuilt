package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final SparkMax motor = new SparkMax(IntakeConstants.INTAKE_ID, MotorType.kBrushless);
  public static SparkMaxConfig motorConfig = new SparkMaxConfig();

  public Intake() {
    super();

    motorConfig.inverted(IntakeConstants.INTAKE_INVERTED);
    motorConfig.smartCurrentLimit(IntakeConstants.INTAKE_STALL_LIMIT, IntakeConstants.INTAKE_FREE_LIMIT); // maybe
    motorConfig.idleMode(IdleMode.kBrake);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void stop() {
    motor.set(0);
  }

  public void forward() {
    motor.set(IntakeConstants.INTAKE_SPEED);
  }

  public void backward() {
    motor.set(-IntakeConstants.INTAKE_REVERSE_SPEED);
  }
}