// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AimTowardsGoal;
import frc.robot.math.Vector3;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final SwerveSubsystem drivebase = TunerConstants.createDrivetrain();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final ClimbSubsystem climb = new ClimbSubsystem();

  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(DrivebaseConstants.MAX_SPEED * 0.1) // Add a 10% deadband
      .withMaxAbsRotationalRate(DrivebaseConstants.MAX_SPIN_SPEED_RADIANS_PER_SECOND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withHeadingPID(DrivebaseConstants.HEADING_CONTROLLER.getP(), DrivebaseConstants.HEADING_CONTROLLER.getI(), DrivebaseConstants.HEADING_CONTROLLER.getD());
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private AimTowardsGoal aimingCommand = null;

  private boolean driverAllowIndexing = false;
  private Rotation2d lastHeading = Rotation2d.kZero;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final PowerDistribution pdh = new PowerDistribution();

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    shooter.setDefaultCommand(shooter.set(0.));
    indexer.setDefaultCommand(indexer.stop());
    intake.setDefaultCommand(intake.stop());
    arm.setDefaultCommand(arm.setAngle(ArmConstants.TOP_ANGLE));

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    pdh.setSwitchableChannel(true);
    pdh.clearStickyFaults();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Swerve
    drivebase.setDefaultCommand(drivebase.applyRequest(() -> {
        if (Math.hypot(driverController.getRightX(), driverController.getRightY()) > DrivebaseConstants.HEADING_DEADBAND) {
          lastHeading = new Rotation2d(-driverController.getRightY(), -driverController.getRightX());
        }
        return driveFacingAngle.withVelocityX(-driverController.getLeftY() * DrivebaseConstants.MAX_SPEED) // Drive forward with negative Y (forward)
             .withVelocityY(-driverController.getLeftX() * DrivebaseConstants.MAX_SPEED) // Drive left with negative X (left)
             .withTargetDirection(lastHeading); // Drive counterclockwise with negative X (left)
    }));

    // Driver

    // Stuff
    driverController.start().onTrue((Commands.runOnce(drivebase::seedFieldCentric)));

    // Sim stuff
    // driverController.leftTrigger().onTrue(Commands.runOnce(() -> {if(aimingCommand != null) {aimingCommand.getMath().logSim();}}));
    // driverController.rightTrigger().onTrue(Commands.runOnce(() -> {if(aimingCommand != null) {aimingCommand.getMath().resetSim();}}));

    driverController.x().onTrue(Commands.runOnce(() -> {
        if (aimingCommand != null  &&  aimingCommand.isScheduled()) {
          aimingCommand.cancel();
        }
        aimingCommand = new AimTowardsGoal(() -> {
          Vector3 goalVector = Vector3.normalize(Vector3.subtract(ShooterConstants.getGoal(DriverStation.getAlliance().get()),
                                                              new Vector3(drivebase.getState().Pose.getX(), drivebase.getState().Pose.getY(),0)).get2D());
          double goalAngle = Vector3.getCounterclockwiseAngle(goalVector);
          Vector3 input = new Vector3(driverController.getLeftY() * DrivebaseConstants.SHOOT_WHILE_MOVING_SPEED, driverController.getLeftX() * DrivebaseConstants.SHOOT_WHILE_MOVING_SPEED, 0);
          if (DriverStation.getAlliance().get() == Alliance.Red) {
            input = Vector3.scale(input, -1.);
          }
          input = Vector3.rotate(input, Vector3.getOrigin(), goalAngle);
          return input;
        },
                                           shooter, 
                                           drivebase,
                                           ShooterConstants.getGoal(DriverStation.getAlliance().get()));
        CommandScheduler.getInstance().schedule(aimingCommand);}))
                        .onFalse(Commands.runOnce(() -> aimingCommand.cancel()));
    driverController.y().whileTrue(arm.setAngle(ArmConstants.BOTTOM_ANGLE)
                                   .alongWith(drivebase.applyRequest(() -> driveFacingAngle.withVelocityX(-driverController.getLeftY() * DrivebaseConstants.MAX_SPEED) // Drive forward with negative Y (forward)
                                                                                           .withVelocityY(-driverController.getLeftX() * DrivebaseConstants.MAX_SPEED)
                                                                                           .withTargetDirection(Rotation2d.kZero))));
    driverController.a().whileTrue(arm.setAngle(ArmConstants.BOTTOM_ANGLE)
                                   .alongWith(intake.set(IntakeConstants.DUTY_CYCLE))
                                   .alongWith(drivebase.applyRequest(() -> driveFacingAngle.withVelocityX(-driverController.getLeftY() * DrivebaseConstants.MAX_SPEED) // Drive forward with negative Y (forward)
                                                                                           .withVelocityY(-driverController.getLeftX() * DrivebaseConstants.MAX_SPEED)
                                                                                           .withTargetDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX())))));
    driverController.leftBumper().onTrue(Commands.runOnce(() -> {
        if (aimingCommand != null  &&  aimingCommand.isScheduled()) {
          aimingCommand.cancel();
        }
        aimingCommand = new AimTowardsGoal(() -> new Vector3(-driverController.getLeftY() * DrivebaseConstants.SHOOT_WHILE_MOVING_SPEED, 
                                                             -driverController.getLeftX() * DrivebaseConstants.SHOOT_WHILE_MOVING_SPEED, 0),
                                           shooter, 
                                           drivebase,
                                           ShooterConstants.getGoal(DriverStation.getAlliance().get()));
        CommandScheduler.getInstance().schedule(aimingCommand);}))
                        .onFalse(Commands.runOnce(() -> aimingCommand.cancel()));

    // Indexer
    driverController.rightBumper().whileTrue(Commands.run(() -> {driverAllowIndexing = true;}))
                                  .onFalse(Commands.run(() -> {driverAllowIndexing = false;}));

    // Operator
    
    operatorController.rightTrigger().whileTrue(arm.setAngle(ArmConstants.BOTTOM_ANGLE)
                                               .alongWith(intake.set(1.)))
                                    .onFalse(intake.set(0.));
    operatorController.leftTrigger().whileTrue(intake.set(-1).alongWith(arm.setAngle(ArmConstants.ZERO_ANGLE))).onFalse(intake.set(0.));
    
    operatorController.rightBumper().whileTrue(Commands.run(() -> {
          if (driverAllowIndexing) {
            CommandScheduler.getInstance().schedule(indexer.set(1.));
          }
          else {
            CommandScheduler.getInstance().schedule(indexer.getDefaultCommand());
          }
        }))
                                     .onFalse(indexer.getDefaultCommand());
    operatorController.leftBumper().whileTrue(indexer.set(1.));

    operatorController.x().whileTrue(shooter.set(0.67));

    operatorController.y().onTrue(climb.setAngularPosition(ClimbConstants.UP_ANGLE));
    operatorController.a().onTrue(climb.setAngularPosition(Radians.of(0.)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brakee) {
    if(brakee) {drivebase.applyRequest(() -> brake);}
  }
}
