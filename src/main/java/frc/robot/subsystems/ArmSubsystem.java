package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class ArmSubsystem extends SubsystemBase {
  private SmartMotorControllerConfig leftSMCConfig = ArmConstants.APPLY_SMC_CONFIG
      .apply(new SmartMotorControllerConfig(this));

  private SmartMotorControllerConfig rightSMCConfig = ArmConstants.APPLY_SMC_CONFIG
      .apply(new SmartMotorControllerConfig(this));
  private SparkMax leftSpark = new SparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private SparkMax rightSpark = new SparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
  private SmartMotorController lSparkSmartMotorController = new SparkWrapper(leftSpark, DCMotor.getNEO(1),
      leftSMCConfig);
  private SmartMotorController rSparkSmartMotorController = new SparkWrapper(rightSpark, DCMotor.getNEO(1),
      rightSMCConfig.withMotorInverted(true)
                    .withLooselyCoupledFollowers(lSparkSmartMotorController));

  private ArmConfig rightArmConfig = ArmConstants.APPLY_ARM_CONFIG.apply(new ArmConfig(rSparkSmartMotorController));
  private ArmConfig leftArmCfg = ArmConstants.APPLY_ARM_CONFIG.apply(new ArmConfig(lSparkSmartMotorController));
  private Arm arm = new Arm(rightArmConfig);

  private Angle lastAngle = ArmConstants.BOTTOM_ANGLE;

  public ArmSubsystem() {
    CommandScheduler.getInstance().schedule(arm.setAngle(ArmConstants.TOP_ANGLE));
    leftArmCfg.applyConfig();
    SmartDashboard.putBoolean("top arm resetencoders", false);
    SmartDashboard.putBoolean("bottom arm resetencoders", false);
  }

  /**
   * Set the angle of the arm
   * 
   * @param angle Angle to go to.
   * @return A command.
   */
  public Command setAngle(Angle angle) {
    return arm.setAngle(angle).alongWith(Commands.runOnce(() -> lastAngle = angle));
  }
  /**
   * Gets the current angle of the arm
   * 
   * 
   * @return current {@link Angle} of the arm.
   */
  public Angle getAngle() {
    return arm.getMotorController().getMechanismPosition();
  }

  public Angle getLastSetpoint() {
    return lastAngle;
  }

  /**
   * Move the arm up and down.
   * 
   * @param dutycycle [-1, 1] speed to set the arm to.
   */
  public Command set(double dutycycle) {
    return Commands.run(() -> {lSparkSmartMotorController.setDutyCycle(dutycycle); rSparkSmartMotorController.setDutyCycle(dutycycle);});
  }

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() {
    return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  /*
   * Resets angle
   */
  public Command resetAngle(Angle _angle) {
    return run(() -> {
      lSparkSmartMotorController.setEncoderPosition(_angle);
      rSparkSmartMotorController.setEncoderPosition(_angle);
    });
  }

  @Override
  public void periodic() {
    arm.updateTelemetry();
    SmartDashboard.putNumber("right arm position", rSparkSmartMotorController.getMechanismPosition().in(Degrees));
    if (rSparkSmartMotorController.getMechanismPositionSetpoint().isPresent()) {
      SmartDashboard.putNumber("right arm setpoint", rSparkSmartMotorController.getMechanismPositionSetpoint().get().in(Degrees));
    }
    SmartDashboard.putNumber("right dutycycle", rSparkSmartMotorController.getDutyCycle());

    SmartDashboard.putNumber("left arm position", lSparkSmartMotorController.getMechanismPosition().in(Degrees));
    if (rSparkSmartMotorController.getMechanismPositionSetpoint().isPresent()) {
      SmartDashboard.putNumber("left arm setpoint", lSparkSmartMotorController.getMechanismPositionSetpoint().get().in(Degrees));
    }
    SmartDashboard.putNumber("left dutycycle", lSparkSmartMotorController.getDutyCycle());

    if (SmartDashboard.getBoolean("top arm resetencoders", false)) {
      resetAngle(ArmConstants.TOP_ANGLE);
    }
    SmartDashboard.putBoolean("top arm resetencoders", false);

    if (SmartDashboard.getBoolean("bottom arm resetencoders", false)) {
      resetAngle(ArmConstants.BOTTOM_ANGLE);
    }
    SmartDashboard.putBoolean("bottom arm resetencoders", false);
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }
}
