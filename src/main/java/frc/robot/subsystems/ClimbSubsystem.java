package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class ClimbSubsystem extends SubsystemBase {
  private SparkMax leftSpark = new SparkMax(ClimbConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private SparkMax rightSpark = new SparkMax(ClimbConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
  private SmartMotorControllerConfig leftSMCConfig = ClimbConstants.APPLY_SMC_CONFIG
      .apply(new SmartMotorControllerConfig(this).withFollowers(Pair.of(rightSpark, true)));

  private SmartMotorController climbSMC = new SparkWrapper(leftSpark, DCMotor.getNEO(1), leftSMCConfig);

  public ClimbSubsystem() {
    climbSMC.setEncoderPosition(Radians.of(0));
    SmartDashboard.putBoolean("climb resetencoder", false);
  }

  /**
   * Sets the dutycycle of the intake.
   * 
   * @param dutyCycle Dutycycle to set.
   * @return A command to set the dutycycle.
   */
  public Command set(double dutyCycle) {
    return Commands.run(() -> climbSMC.setDutyCycle(dutyCycle));
  }

  public Command forceResetClimb() {
    return Commands.run(() -> climbSMC.setVoltage(Volts.of(0.)));
  }

  public Command setAngularPosition(Angle angle) {
    return Commands.run(() -> climbSMC.setPosition(angle));
  }

  public Command setLinearPosition(Distance distance) {
    return Commands.run(() -> climbSMC.setPosition(distance));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("climb position", climbSMC.getMechanismPosition().in(Rotations));
    if (climbSMC.getMechanismPositionSetpoint().isPresent()) {
      SmartDashboard.putNumber("climb setpoint", climbSMC.getMechanismPositionSetpoint().get().in(Rotations));
    }

    if (SmartDashboard.getBoolean("climb resetencoder", false)) {
      climbSMC.setEncoderPosition(ClimbConstants.DOWN_ANGLE);
    }
    SmartDashboard.putBoolean("climb resetencoder", false);
  }
}
