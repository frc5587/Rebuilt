// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.commands.AimTowardsGoal;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
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

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(DrivebaseConstants.MAX_SPEED * 0.1).withRotationalDeadband(DrivebaseConstants.MAX_SPIN_SPEED_RADIANS_PER_SECOND * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(DrivebaseConstants.MAX_SPEED * 0.1) // Add a 10% deadband
      .withMaxAbsRotationalRate(DrivebaseConstants.MAX_SPIN_SPEED_RADIANS_PER_SECOND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withHeadingPID(DrivebaseConstants.HEADING_CONTROLLER.getP(), DrivebaseConstants.HEADING_CONTROLLER.getI(), DrivebaseConstants.HEADING_CONTROLLER.getD());
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private AimTowardsGoal aimingCommand;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    shooter.setDefaultCommand(shooter.stop());
    arm.setDefaultCommand(arm.setAngle(Degrees.of(0)));
    intake.setDefaultCommand(intake.stop());
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
    // Driver

    // Swerve
    drivebase.setDefaultCommand(drivebase.applyRequest(() ->
              drive.withVelocityX(-driverController.getLeftY() * DrivebaseConstants.MAX_SPEED) // Drive forward with negative Y (forward)
                   .withVelocityY(-driverController.getLeftX() * DrivebaseConstants.MAX_SPEED) // Drive left with negative X (left)
                   .withRotationalRate(-driverController.getRightX() * DrivebaseConstants.MAX_SPIN_SPEED_RADIANS_PER_SECOND) // Drive counterclockwise with negative X (left)
    ));

    // Stuff
    driverController.start().onTrue((Commands.runOnce(drivebase::seedFieldCentric)));

    // Shoot while moving
    driverController.x().onTrue(Commands.runOnce(() -> {
        aimingCommand = new AimTowardsGoal(() -> -driverController.getLeftY() * DrivebaseConstants.SHOOT_WHILE_MOVING_SPEED, 
                                           () -> -driverController.getLeftX() * DrivebaseConstants.SHOOT_WHILE_MOVING_SPEED, 
                                           shooter, 
                                           drivebase);
        CommandScheduler.getInstance().schedule(aimingCommand);}))
                        .onFalse(Commands.runOnce(() -> aimingCommand.cancel()));
    driverController.leftTrigger().onTrue(Commands.runOnce(() -> {if(aimingCommand != null) {aimingCommand.getMath().logSim();}}));
    driverController.rightTrigger().onTrue(Commands.runOnce(() -> {if(aimingCommand != null) {aimingCommand.getMath().resetSim();}}));

    // Operator
    
    operatorController.rightBumper().whileTrue(arm.setAngle(ArmConstants.DOWN_ANGLE)).onFalse(arm.setAngle(ArmConstants.UP_ANGLE));
    operatorController.rightBumper().whileTrue(intake.set(1.)).onFalse(intake.stop());
    
    operatorController.leftTrigger().whileTrue(indexer.set(1.)).onFalse(indexer.stop());
    operatorController.rightTrigger().whileTrue(shooter.setHigh()).onFalse(shooter.setZero());

    operatorController.leftBumper().whileTrue(intake.set(-1)).onFalse(intake.stop());
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
