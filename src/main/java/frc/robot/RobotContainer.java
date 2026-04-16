// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import frc.robot.Constants.DrivebaseConstants;
import frc.robot.math.Vector3;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDController;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final SwerveSubsystem swerve = TunerConstants.createDrivetrain();
  private final IndexerSubsystem indexer = new IndexerSubsystem();

  private final LEDController ledController = new LEDController();

  private Trigger shooterSpunUp = new Trigger(() -> shooterAtGoal());

  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(DrivebaseConstants.MAX_SPEED * 0.1) // Add a 10% deadband
      .withMaxAbsRotationalRate(DrivebaseConstants.MAX_SPIN_SPEED_RADIANS_PER_SECOND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withHeadingPID(DrivebaseConstants.HEADING_CONTROLLER.getP(), DrivebaseConstants.HEADING_CONTROLLER.getI(),
          DrivebaseConstants.HEADING_CONTROLLER.getD());
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  Supplier<Vector3> position = () -> {
    Pose2d position = swerve.getState().Pose;
    return new Vector3(position.getX(), position.getY(), 0);
  };
  DoubleSupplier heading = () -> swerve.getState().RawHeading.getRadians();
  Supplier<Vector3> velocity = () -> {
    ChassisSpeeds velocity = swerve.getState().Speeds;
    velocity = ChassisSpeeds.fromRobotRelativeSpeeds(velocity, swerve.getState().Pose.getRotation());
    Vector3 input = new Vector3(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, 0);
    return input;
  };
  DoubleSupplier angularVelocity = () -> swerve.getState().Speeds.omegaRadiansPerSecond;
  Supplier<Vector3> inputVelocity = () -> Vector3.getOrigin();
  DoubleSupplier inputAngularVelocity = () -> 0.;

  private Rotation2d lastHeading;

  private final PowerDistribution pdh = new PowerDistribution();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // PLEASE DON'T SET DEFAULT COMMANDS UP HERE!! USE TELEOPINIT() AT BOTTOM OF
    // FILE
    indexer.setDefaultCommand(indexer.stop());

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    pdh.setSwitchableChannel(true);
    pdh.clearStickyFaults();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /**
     * ** Driver Controls **
     * Left Stick: Drive (field oriented)
     * Right Stick: Rotate robot (heading control)
     * Start: Zero robot gyro (use if field oriented feels off)
     * 
     * ** Operator Controls **
     * Left Trigger: Spin up shooter (to manual speed)
     * Right Trigger: Spin indexer (if shooter up to speed)
     * Start: Indexer Override
     * 
     * POV Up: Reverse shooter
     * POV Down: Reverse shooter and indexer
     */

    // Zero Gyro
    driver.start().onTrue((Commands.runOnce(swerve::seedFieldCentric))
        .alongWith(Commands.runOnce(() -> {
          lastHeading = Rotation2d.kZero;
        })));

    // Forward overrides
    operator.leftTrigger().whileTrue(shooter.useManualSpeed());
    operator.rightTrigger().and(shooterSpunUp).whileTrue(indexer.start());
    operator.start().whileTrue(indexer.start());

    // Reverse overrides
    operator.povUp().whileTrue(shooter.set(-0.3));
    operator.povDown().whileTrue(indexer.set(-1.).alongWith(shooter.set(-0.3)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  public void setMotorBrake(boolean isBrake) {
    if (isBrake) {
      swerve.applyRequest(() -> brake);
    }
  }

  public void teleopInit() {
    shooter.setDefaultCommand(shooter.idle());
    indexer.setDefaultCommand(indexer.stop());
    lastHeading = swerve.getState().Pose.getRotation();
    // Swerve
    swerve.setDefaultCommand(swerve.applyRequest(() -> {
      if (Math.hypot(driver.getRightX(),
          driver.getRightY()) > DrivebaseConstants.HEADING_DEADBAND) {
        lastHeading = new Rotation2d(-driver.getRightY(), -driver.getRightX());
      }
      return driveFacingAngle.withVelocityX(-driver.getLeftY() * DrivebaseConstants.MAX_SPEED) // Drive
                                                                                                         // forward with
                                                                                                         // negative Y
                                                                                                         // (forward)
          .withVelocityY(-driver.getLeftX() * DrivebaseConstants.MAX_SPEED) // Drive left with negative X
                                                                                      // (left)
          .withTargetDirection(lastHeading); // Point in joystick direction
    }));
  }
    public LEDController getLEDController() {
      return ledController;
    }

    public boolean shooterAtGoal() {
      return shooter.atGoal().getAsBoolean();
    }
    public boolean shooterUsingNonDefaultCommand() {
      if (shooter.getCurrentCommand() != shooter.getDefaultCommand()) {
        return true;
      } else {return false;}
    }
}


