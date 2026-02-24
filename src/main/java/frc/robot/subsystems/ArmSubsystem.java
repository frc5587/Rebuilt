package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class ArmSubsystem extends SubsystemBase {
    
    private SmartMotorControllerConfig smcConfig = ArmConstants.APPLY_SMC_CONFIG.apply(new SmartMotorControllerConfig(this));

    private SparkMax leftspark = new SparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

    private SparkMax rightspark = new SparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    private SmartMotorController lSparkSmartMotorController = new SparkWrapper(leftspark, DCMotor.getNEO(1), smcConfig);

    private SmartMotorController rSparkSmartMotorController = new SparkWrapper(rightspark, DCMotor.getNEO(1), smcConfig.withLooselyCoupledFollowers(lSparkSmartMotorController));

    private ArmConfig armCfg = ArmConstants.APPLY_ARM_CONFIG.apply(new ArmConfig(rSparkSmartMotorController));
                                   

    private Arm arm = new Arm(armCfg);
    
    public ArmSubsystem() {
        lSparkSmartMotorController.setEncoderPosition(ArmConstants.TOP_ANGLE);
    }

    /**
     * Set the angle of the arm
     * @param angle Angle to go to.
     * @return A command.
     */
    public Command setAngle(Angle angle) {
        return arm.setAngle(angle);
    }

    /**
     * Move the arm up and down.
     * @param dutycycle [-1, 1] speed to set the arm to.
     */
    public Command set(double dutycycle) {
        return arm.set(dutycycle);
    }

    /**
     * Run sysId on the {@link Arm}
     */
    public Command sysId() {
        return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
    }

    @Override
    public void periodic() {
        arm.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        arm.simIterate();
    }
}
