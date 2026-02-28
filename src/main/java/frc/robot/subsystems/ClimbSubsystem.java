package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ShooterConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class ClimbSubsystem extends SubsystemBase {
  private SmartMotorControllerConfig smcConfig = ShooterConstants.APPLY_SMC_CONFIG.apply(new SmartMotorControllerConfig(this));
  private SparkMax spark = new SparkMax(ClimbConstants.MOTOR_ID, MotorType.kBrushless);
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);
  private final boolean hasLimitSwitch = false;
  private DigitalInput limitSwitch = new DigitalInput(ClimbConstants.LIMIT_SWITCH_ID);

  public ClimbSubsystem() {
    sparkSmartMotorController.setEncoderPosition(Radians.of(0));
  }

  /**
    * Sets the dutycycle of the intake.
    * @param dutyCycle Dutycycle to set.
    * @return A command to set the dutycycle.
    */
  public Command set(double dutyCycle) {
      return Commands.run(() -> sparkSmartMotorController.setDutyCycle(dutyCycle));
  }
  
  public Command forceResetClimb() {
    return Commands.run(() -> sparkSmartMotorController.setVoltage(Volts.of(0.)));
  }

  public Command setAngularPosition(Angle angle) {
    return Commands.run(() -> sparkSmartMotorController.setPosition(angle));
  }
  
  public Command setLinearPosition(Distance distance) {
    return Commands.run(() -> sparkSmartMotorController.setPosition(distance));
  }

  public boolean getLimitSwitch() throws Exception {
    if (!hasLimitSwitch) {
      throw new Exception("No limit switch");
    }
    return limitSwitch.get();
  }

  public boolean hasLimitSwitch() {
    return hasLimitSwitch;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("climb position", sparkSmartMotorController.getMechanismPosition().in(Rotations));
    if (sparkSmartMotorController.getMechanismPositionSetpoint().isPresent()) {
      SmartDashboard.putNumber("climb setpoint", sparkSmartMotorController.getMechanismPositionSetpoint().get().in(Rotations));
    }

    sparkSmartMotorController.getDutyCycle();
    if (hasLimitSwitch  &&  limitSwitch.get()) {
      sparkSmartMotorController.setEncoderPosition(Radians.of(0.));
    }
  }
}
