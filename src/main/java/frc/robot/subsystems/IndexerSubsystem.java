package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class IndexerSubsystem extends SubsystemBase{
    private SmartMotorControllerConfig smcConfig = IndexerConstants.APPLY_SMC_CONFIG.apply(new SmartMotorControllerConfig(this));

    private SparkMax spark = new SparkMax(IndexerConstants.MOTOR_ID, MotorType.kBrushless);

    private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

    private FlyWheelConfig flywheelCfg = IndexerConstants.APPLY_INDEXER_CONFIG.apply(new FlyWheelConfig(sparkSmartMotorController));

    private FlyWheel indexer = new FlyWheel(flywheelCfg);

    public IndexerSubsystem() {}

    /**
     * Gets the velocity of the indexer.
     * @return Indexer velocity.
     */
    public AngularVelocity getVelocity() {
        return indexer.getSpeed();
    }

    /**
     * Sets the velocity of the indexer.
     * @param speed Speed to set the indexer.
     * @return A command to set the speed.
     */
    public Command setVelocity(AngularVelocity speed) {
        return indexer.setSpeed(speed);
    }

    /**
     * Sets the dutycycle of the indexer.
     * @param dutyCycle Dutycycle to set.
     * @return A command to set the dutycycle.
     */
    public Command set(double dutyCycle) {
        return indexer.set(dutyCycle);
    }

    /**
     * Stops the indexer.
     * @return A command to stop the indexer.
     */
    public Command stop() {
        return indexer.set(0);
    }

    @Override
    public void periodic() {
        indexer.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        indexer.simIterate();
    }
}
