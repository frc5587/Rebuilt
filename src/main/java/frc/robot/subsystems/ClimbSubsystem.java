package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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
  private SmartMotorController sparkSMC = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  public ClimbSubsystem() {
    sparkSMC.setEncoderPosition(Radians.of(0));
  }

   /**
     * Sets the dutycycle of the intake.
     * @param dutyCycle Dutycycle to set.
     * @return A command to set the dutycycle.
     */
  public Command set(double dutyCycle) {
      return Commands.run(() -> sparkSMC.setDutyCycle(dutyCycle));
  }
  
  public Command forceResetClimb() {
    return Commands.run(() -> sparkSMC.setVoltage(Volts.of(0.)));
  }

  public Command setAngularPosition(Angle angle) {
    return Commands.run(() -> sparkSMC.setPosition(angle));
  }
  
  public Command setLinearPosition(Distance distance) {
    return Commands.run(() -> sparkSMC.setPosition(distance));
  }
}
