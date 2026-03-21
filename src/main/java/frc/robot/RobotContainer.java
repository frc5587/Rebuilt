// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AimTowardsGoal;
import frc.robot.commands.LoadBalls;
import frc.robot.math.Vector3;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDController;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final SwerveSubsystem drivebase = TunerConstants.createDrivetrain();
  private final ArmSubsystem arm = new ArmSubsystem((rumbleMagnitude) -> {
            driver.getHID().setRumble(RumbleType.kBothRumble, rumbleMagnitude);
            operator.getHID().setRumble(RumbleType.kBothRumble, rumbleMagnitude);});
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final ClimbSubsystem climb = new ClimbSubsystem();
  private final LEDController ledController = new LEDController();

  private Trigger armUp = new Trigger(() -> arm.getCurrentSetpoint() == ArmConstants.TOP_ANGLE);

  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(DrivebaseConstants.MAX_SPEED * 0.1) // Add a 10% deadband
      .withMaxAbsRotationalRate(DrivebaseConstants.MAX_SPIN_SPEED_RADIANS_PER_SECOND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withHeadingPID(DrivebaseConstants.HEADING_CONTROLLER.getP(), DrivebaseConstants.HEADING_CONTROLLER.getI(),
          DrivebaseConstants.HEADING_CONTROLLER.getD());
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private AimTowardsGoal aimingCommand = null;
  Supplier<Vector3> position = () -> {
    Pose2d position = drivebase.getState().Pose;
    return new Vector3(position.getX(), position.getY(), 0);
  };
  DoubleSupplier heading = () -> drivebase.getState().RawHeading.getRadians();
  Supplier<Vector3> velocity = () -> {
    ChassisSpeeds velocity = drivebase.getState().Speeds;
    velocity = ChassisSpeeds.fromRobotRelativeSpeeds(velocity, drivebase.getState().Pose.getRotation());
    Vector3 input = new Vector3(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, 0);
    return input;
  };
  DoubleSupplier angularVelocity = () -> drivebase.getState().Speeds.omegaRadiansPerSecond;
  Supplier<Vector3> inputVelocity = () -> Vector3.getOrigin();
  DoubleSupplier inputAngularVelocity = () -> 0.;

  private Rotation2d lastHeading;

  private final PowerDistribution pdh = new PowerDistribution();

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  @SuppressWarnings("removal")
  public RobotContainer() {
    // PLEASE DON'T SET DEFAULT COMMANDS UP HERE!! USE TELEOPINIT() AT BOTTOM OF
    // FILE
    indexer.setDefaultCommand(indexer.set(0.));

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    NamedCommands.registerCommand("Arm Up", Commands.runOnce(() -> arm.setAngle(ArmConstants.TOP_ANGLE).schedule()));
    NamedCommands.registerCommand("Arm Down", Commands.runOnce(() -> arm.setAngle(ArmConstants.BOTTOM_ANGLE).schedule()));

    NamedCommands.registerCommand("Intake Forward", Commands.runOnce(() -> intake.set(IntakeConstants.DUTY_CYCLE).schedule())
                                                         .alongWith(Commands.runOnce(() -> arm.set(-0.3).schedule())));
    NamedCommands.registerCommand("Intake Stop", Commands.runOnce(() -> intake.set(0).schedule()));

    NamedCommands.registerCommand("Shoot Preload",
        new SequentialCommandGroup(
            Commands.runOnce(() -> shooter.setBallVelocity(() -> MetersPerSecond.of(6.80873631)).schedule()),
            new LoadBalls(arm, shooter, indexer, intake).raceWith(Commands.waitSeconds(5.)),
            Commands.runOnce(() -> shooter.set(0.).schedule())));

    NamedCommands.registerCommand("Shoot Hopper",
        new SequentialCommandGroup(
            Commands.runOnce(() -> shooter.setBallVelocity(() -> MetersPerSecond.of(6.80873631)).schedule()),
            new LoadBalls(arm, shooter, indexer, intake).raceWith(Commands.waitSeconds(10.)),
            Commands.runOnce(() -> shooter.set(0.).schedule())));
    NamedCommands.registerCommand("Climb Up", climb.setAngularPosition(ClimbConstants.UP_ANGLE));
    NamedCommands.registerCommand("Climb Down", climb.setAngularPosition(ClimbConstants.DOWN_ANGLE));

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

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
     * Driver Controls
     * Left Stick: Drive (field oriented)
     * Right Stick: Rotate robot (heading control)
     * 
     * Start: Zero robot gyro (use if field oriented feels off)
     * 
     * Right Bumper: Together indexer
     * 
     * X: Shoot while moving swerve
     * Y: Point forward and arm down (for trench)
     * A: Point in drive direction and intake (for intaking)
     * Left Bumper: Alternate shoot on the move control
     * 
     * Operator Controls
     * Right Bumper: Together indexer
     * Left Bumper: Override indexer
     * 
     * Right Trigger: Arm down + intake forward
     * Left Trigger: Arm up
     * B: Arm angled (for shooting)
     * 
     * X: Shooter override (constant speed)
     * 
     * Y: Climb top
     * A: Climb bottom
     * 
     * POV Up: 100% intake speed
     * POV Right: Reverse indexer and shooter
     * POV Down: Reverse intake
     * 
     * Start: reset arm position to top
     */

    // Driver

    // Zero Gyro
    driver.start().onTrue((Commands.runOnce(drivebase::seedFieldCentric))
        .alongWith(Commands.runOnce(() -> {
          lastHeading = Rotation2d.fromDegrees(0.);
        })));
    // Zero Odmometry
    driver.back().onTrue(Commands.runOnce(() -> {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        drivebase.resetPose(DrivebaseConstants.BLUE_ALLIANCE_MIDDLE_HUB);
      } else if (DriverStation.getAlliance().get() == Alliance.Red) {
        drivebase.resetPose(DrivebaseConstants.RED_ALLIANCE_MIDDLE_HUB);
      }
    }));

    // Sim stuff
    // driver.leftTrigger().onTrue(Commands.runOnce(() -> {
    //   if (aimingCommand != null) {
    //     aimingCommand.getMath().logSim();
    //   }
    // }));
    // driver.rightTrigger().onTrue(Commands.runOnce(() -> {
    //   if (aimingCommand != null) {
    //     aimingCommand.getMath().resetSim();
    //   }
    // }));
    driver.rightTrigger().whileTrue(shooter.useManualSpeed());
    driver.rightBumper().whileTrue(shooter.setAngularVelocity(() -> RPM.of(3200)));
    driver.leftTrigger().whileTrue(shooter.setAngularVelocity(() -> RPM.of(2900)));
    driver.leftBumper().whileTrue(shooter.setAngularVelocity(() -> RPM.of(3000)));

    // Main controlls
    driver.x().onTrue(Commands.runOnce(() -> {
      if (aimingCommand != null && aimingCommand.isScheduled()) {
        aimingCommand.cancel();
      }
      aimingCommand = new AimTowardsGoal(() -> {
        Vector3 goalVector = Vector3
            .normalize(Vector3.subtract(ShooterConstants.getGoal(DriverStation.getAlliance().get()),
                new Vector3(drivebase.getState().Pose.getX(), drivebase.getState().Pose.getY(), 0)).get2D());
        double goalAngle = Vector3.getCounterclockwiseAngle(goalVector);
        Vector3 input = new Vector3(driver.getLeftY() * DrivebaseConstants.SHOOT_WHILE_MOVING_SPEED,
            driver.getLeftX() * DrivebaseConstants.SHOOT_WHILE_MOVING_SPEED, 0);
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          input = Vector3.scale(input, -1.);
        }
        input = Vector3.rotate(input, Vector3.getOrigin(), goalAngle);
        return input;
      },
          shooter,
          drivebase,
          ShooterConstants.getGoal(DriverStation.getAlliance().get()));
      CommandScheduler.getInstance().schedule(aimingCommand);
    }))
        .onFalse(Commands.runOnce(() -> aimingCommand.cancel())
            .alongWith(Commands.runOnce(() -> {
              lastHeading = drivebase.getState().Pose.getRotation();
            })));
    driver.y().whileTrue(arm.setAngle(ArmConstants.BOTTOM_ANGLE)
        .alongWith(drivebase.applyRequest(
            () -> driveFacingAngle.withVelocityX(-driver.getLeftY() * DrivebaseConstants.MAX_SPEED) // Drive
                                                                                                              // forward
                                                                                                              // with
                                                                                                              // negative
                                                                                                              // Y
                                                                                                              // (forward)
                .withVelocityY(-driver.getLeftX() * DrivebaseConstants.MAX_SPEED)
                .withTargetDirection(Rotation2d.kZero))));
    // driverController.a().whileTrue(arm.setAngle(ArmConstants.BOTTOM_ANGLE)
    // .alongWith(intake.set(IntakeConstants.DUTY_CYCLE))
    // .alongWith(drivebase.applyRequest(() -> {
    // if (Math.hypot(driverController.getLeftY(), driverController.getLeftX()) >
    // DrivebaseConstants.INTAKE_HEADING_DEADBAND) {
    // lastHeading = new Rotation2d(-driverController.getLeftY(),
    // -driverController.getLeftX());
    // }
    // return driveFacingAngle.withVelocityX(-driverController.getLeftY() *
    // DrivebaseConstants.MAX_SPEED) // Drive forward with negative Y (forward)
    // .withVelocityY(-driverController.getLeftX() * DrivebaseConstants.MAX_SPEED)
    // // Drive left with negative X (left)
    // .withTargetDirection(lastHeading); // Drive counterclockwise with negative X
    // (left)
    // })));
    driver.a().whileTrue(arm.setAngle(ArmConstants.TOP_ANGLE)
        .alongWith(drivebase.applyRequest(
            () -> driveFacingAngle.withVelocityX(-driver.getLeftY() * DrivebaseConstants.MAX_SPEED) // Drive
                                                                                                              // forward
                                                                                                              // with
                                                                                                              // negative
                                                                                                              // Y
                                                                                                              // (forward)
                .withVelocityY(-driver.getLeftX() * DrivebaseConstants.MAX_SPEED)
                .withTargetDirection(Rotation2d.fromDegrees(45.)))));
    // driver.leftBumper().onTrue(Commands.runOnce(() -> {
    //   if (aimingCommand != null && aimingCommand.isScheduled()) {
    //     aimingCommand.cancel();
    //   }
    //   aimingCommand = new AimTowardsGoal(
    //       () -> new Vector3(-driver.getLeftY() * DrivebaseConstants.SHOOT_WHILE_MOVING_SPEED,
    //           -driver.getLeftX() * DrivebaseConstants.SHOOT_WHILE_MOVING_SPEED, 0),
    //       shooter,
    //       drivebase,
    //       ShooterConstants.getGoal(DriverStation.getAlliance().get()));
    //   CommandScheduler.getInstance().schedule(aimingCommand);
    // }))
    //     .onFalse(Commands.runOnce(() -> aimingCommand.cancel()));

    // Indexer
    // driverController.rightBumper().whileTrue(Commands.run(() ->
    // {driverAllowIndexing = true;}))
    // .onFalse(Commands.run(() -> {driverAllowIndexing = false;}));

    // Operator

    // Arm
    operator.rightTrigger().whileTrue(
        intake.set(IntakeConstants.DUTY_CYCLE)
        .alongWith(arm.setAngle(ArmConstants.BOTTOM_ANGLE).until(() -> !(arm.getAngle().in(Degrees) > ArmConstants.BOTTOM_ANGLE.in(Degrees) + 20))
        .andThen(arm.set(-.3))))
        .onFalse(arm.set(0.));
    operator.leftTrigger().whileTrue(arm.setAngle(ArmConstants.TOP_ANGLE));

    // Indexer
    operator.rightBumper().whileTrue(Commands.runOnce(() -> {
        if (shooter.atGoal()) {
          CommandScheduler.getInstance().schedule(indexer.set(IndexerConstants.DUTY_CYCLE));
        }}))
        .onFalse(indexer.set(0));

    operator.leftBumper().whileTrue(indexer.set(IndexerConstants.DUTY_CYCLE));

    // Silly shooter override (set this to shoot from an easy to drive to position,
    // like in front of the hub)
    operator.x().whileTrue(new LoadBalls(arm, shooter, indexer, intake));
    operator.b().whileTrue(shooter.useManualSpeed());

    // Climb
    operator.y().whileTrue(climb.set(ClimbConstants.SLOW_DUTYCYCLE)).onFalse(climb.set(0.));
    operator.a().whileTrue(climb.set(-ClimbConstants.SLOW_DUTYCYCLE)).onFalse(climb.set(0.));
    operator.povUp().whileTrue(climb.set(1.0)).onFalse(climb.set(0.));

    // Utils
    operator.povRight().whileTrue(indexer.set(-0.5).alongWith(shooter.set(-0.3)));
    operator.povLeft().whileTrue(intake.set(-1.));
    operator.povLeft().whileTrue(arm.set(1.))
      .onFalse(arm.setAngle(arm.getLastSetpoint()));
    // operatorController.povDown().whileTrue(Commands.runOnce(() -> arm.setAngularVelocity(ArmConstants.ARM_VELOCITY_WIGGLE)))
    //     .onFalse(Commands.runOnce(() -> arm.setAngularVelocity(RotationsPerSecond.of(0))));

    // Reset Arm Gyro
    operator.start().onTrue(Commands.runOnce(() -> arm.resetAngle(ArmConstants.BOTTOM_ANGLE)));


    // Triggers
    armUp.whileTrue(shooter.stop()
          .alongWith(indexer.stop()))
          .onFalse(shooter.set(ShooterConstants.IDLE_DUTYCYCLE));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean isBrake) {
    if (isBrake) {
      drivebase.applyRequest(() -> brake);
    }
  }

  public void teleopInit() {
    arm.setVoid(0.);
    shooter.setDefaultCommand(shooter.set(ShooterConstants.IDLE_DUTYCYCLE));
    indexer.setDefaultCommand(indexer.stop());
    intake.setDefaultCommand(intake.stop());
    lastHeading = drivebase.getState().Pose.getRotation();
    // Swerve
    drivebase.setDefaultCommand(drivebase.applyRequest(() -> {
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

    public boolean intakeIsStalling() {
      return intake.isStalling();
    }

    public boolean shooterAtGoal() {
      return shooter.atGoal();
    }
    public boolean shooterUsingNonDefaultCommand() {
      if (shooter.getCurrentCommand() != shooter.getDefaultCommand()) {
        return true;
      } else {return false;}
    }
}
