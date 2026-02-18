package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeSubsystem extends SubsystemBase {
    
    private SmartMotorControllerConfig smcConfig = IntakeConstants.APPLY_SMC_CONFIG.apply(new SmartMotorControllerConfig(this));

    private SparkMax spark = new SparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);

    private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

    private FlyWheelConfig flywheelCfg = IntakeConstants.APPLY_INTAKE_CONFIG.apply(new FlyWheelConfig(sparkSmartMotorController));

    private FlyWheel intake = new FlyWheel(flywheelCfg);

    public IntakeSubsystem() {}

    /**
     * Gets the velocity of the intake.
     * @return Intake velocity.
     */
    public AngularVelocity getVelocity() {
        return intake.getSpeed();
    }

    /**
     * Sets the velocity of the intake.
     * @param speed Speed to set the intake.
     * @return A command to set the speed.
     */
    public Command setVelocity(AngularVelocity speed) {
        return intake.setSpeed(speed);
    }

    /**
     * Sets the dutycycle of the intake.
     * @param dutyCycle Dutycycle to set.
     * @return A command to set the dutycycle.
     */
    public Command set(double dutyCycle) {
        return intake.set(dutyCycle);
    }

    /**
     * Stops the intake.
     * @return A command to stop the intake.
     */
    public Command stop() {
        return intake.set(0);
    }

    @Override
    public void periodic() {
        intake.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        intake.simIterate();
    }
}
