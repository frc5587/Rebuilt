package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ArmSubsystem extends SubsystemBase {
    
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
                                                       .withControlMode(ControlMode.CLOSED_LOOP)
                                                       .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
                                                       .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
                                                       .withFeedforward(new ArmFeedforward(0, 0, 0))
                                                       .withSimFeedforward(new ArmFeedforward(0, 0, 0))
                                                       .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
                                                       .withGearing(new MechanismGearing(GearBox.fromReductionStages(3,4)))//Can also use .withGearing(ratio)
                                                       .withMotorInverted(false)
                                                       .withIdleMode(MotorMode.BRAKE)
                                                       .withStatorCurrentLimit(Amps.of(40))
                                                       .withClosedLoopRampRate(Seconds.of(0.25))
                                                       .withOpenLoopRampRate(Seconds.of(0.25));

    private SparkMax spark = new SparkMax(4, MotorType.kBrushless);

    private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

    private ArmConfig armCfg = new ArmConfig(sparkSmartMotorController)
                                   .withSoftLimits(Degrees.of(-20), Degrees.of(10))
                                   .withHardLimit(Degrees.of(-30), Degrees.of(40))
                                   .withStartingPosition(Degrees.of(-5))
                                   .withLength(Inches.of(15.81))
                                   .withMass(Pounds.of(2))
                                   .withTelemetry("Arm", TelemetryVerbosity.HIGH);

    private Arm arm = new Arm(armCfg);
    
    public ArmSubsystem() {}

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
