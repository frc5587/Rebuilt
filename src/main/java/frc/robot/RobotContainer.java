// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.math.AimingMath;
import frc.robot.math.Vector3;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  // The robot's subsystems and commands are defined here...
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveFieldOriented = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverController.getLeftY() * -1,
      () -> driverController.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverController.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true)
      .robotRelative(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverController.getLeftY() * -1,
                                                                () -> driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);
  
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  Supplier<Vector3> velocity = () -> {
    ChassisSpeeds velocity = drivebase.getFieldVelocity();
    return new Vector3(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, 0);
  };
  DoubleSupplier angularVelocity = () -> drivebase.getFieldVelocity().omegaRadiansPerSecond;
  Supplier<Vector3> position = () -> {
    Pose2d position = drivebase.getPose();
    return new Vector3(position.getX(), position.getY(), 0);
  };
  DoubleSupplier heading = () -> drivebase.getHeading().getRadians();
  AimingMath aimingMath = new AimingMath(velocity, angularVelocity, position, heading, Vector3.origin());


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();//TODO fix robot error pls

    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveFieldOriented);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } 
    else if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
    }
    else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // this is the main drive command
    }

    // Dpad to rotate robot
    driverController.povUp().onTrue(Commands.runOnce(() -> {drivebase.overrideHeading(0.);}));
    driverController.povRight().onTrue(Commands.runOnce(() -> {drivebase.overrideHeading(Math.PI/-2.);}));
    driverController.povDown().onTrue(Commands.runOnce(() -> {drivebase.overrideHeading(-1*Math.PI);}));
    driverController.povLeft().onTrue(Commands.runOnce(() -> {drivebase.overrideHeading(Math.PI/2.);}));

    // Holds the current heading when maintainHeading is toggled on
    driverController.leftBumper().onTrue(Commands.runOnce(() -> {drivebase.overrideHeading(drivebase.getIdealHeadingRadians());}));
    driverController.rightBumper().onTrue(Commands.runOnce(() -> {drivebase.deactivateOverrideHeading();}));

    // Stuff
    driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverController.back().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    operatorController.leftBumper().whileTrue(shooter.setLowVelocity()).onFalse(shooter.setZeroVelocity());
    operatorController.rightBumper().whileTrue(shooter.setHighVelocity()).onFalse(shooter.setZeroVelocity());
    operatorController.leftTrigger().whileTrue(shooter.setLow()).onFalse(shooter.setZero());
    operatorController.rightTrigger().whileTrue(shooter.setHigh()).onFalse(shooter.setZero());

    operatorController.start().onTrue(Commands.runOnce(aimingMath::logSim));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
