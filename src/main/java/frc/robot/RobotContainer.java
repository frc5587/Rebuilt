// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.math.AimingMath;
import frc.robot.math.Vector3;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final SwerveSubsystem drivebase = TunerConstants.createDrivetrain();
  // private final ArmSubsystem arm = new ArmSubsystem();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(DrivebaseConstants.MAX_SPEED * 0.1).withRotationalDeadband(DrivebaseConstants.MAX_SPIN_SPEED_RADIANS_PER_SECOND * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(DrivebaseConstants.MAX_SPEED * 0.1) // Add a 10% deadband
      .withMaxAbsRotationalRate(DrivebaseConstants.MAX_SPIN_SPEED_RADIANS_PER_SECOND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withHeadingPID(DrivebaseConstants.SHOOT_WHILE_MOVE_HEADING_CONTROLLER.getP(), DrivebaseConstants.SHOOT_WHILE_MOVE_HEADING_CONTROLLER.getI(), DrivebaseConstants.SHOOT_WHILE_MOVE_HEADING_CONTROLLER.getD());
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private Vector3 shootWhileMovingVelocity = Vector3.origin();
  private Vector3 shootWhileMovingCalculationVelocity = Vector3.origin();

  // Aiming math
  Supplier<Vector3> position = () -> {
    Pose2d position = drivebase.getState().Pose;
    return new Vector3(position.getX(), position.getY(), 0);
  };
  DoubleSupplier heading = () -> drivebase.getState().RawHeading.getRadians();
  Supplier<Vector3> velocity = () -> {
    ChassisSpeeds velocity = drivebase.getState().Speeds;
    velocity = ChassisSpeeds.fromRobotRelativeSpeeds(velocity, drivebase.getState().Pose.getRotation());
    return new Vector3(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, 0);
  };
  DoubleSupplier angularVelocity = () -> drivebase.getState().Speeds.omegaRadiansPerSecond;
  Supplier<Vector3> inputVelocity = () -> {
    return new Vector3(-driverController.getLeftY()*DrivebaseConstants.SHOOT_WHILE_MOVING_SPEED, -driverController.getLeftX()*DrivebaseConstants.SHOOT_WHILE_MOVING_SPEED, 0);
  };
  DoubleSupplier inputAngularVelocity = () -> 0.;
  AimingMath aimingMath = new AimingMath(position, heading, velocity, angularVelocity, inputVelocity, inputAngularVelocity, () -> shooter.getVelocity().in(RPM), new Vector3(4.625626,4.0346315,1.8288));

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
    // arm.setDefaultCommand(arm.setAngle(Degrees.of(0)));
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
    drivebase.setDefaultCommand(drivebase.applyRequest(() ->
              drive.withVelocityX(-driverController.getLeftY() * DrivebaseConstants.MAX_SPEED) // Drive forward with negative Y (forward)
                   .withVelocityY(-driverController.getLeftX() * DrivebaseConstants.MAX_SPEED) // Drive left with negative X (left)
                   .withRotationalRate(-driverController.getRightX() * DrivebaseConstants.MAX_SPIN_SPEED_RADIANS_PER_SECOND) // Drive counterclockwise with negative X (left)
    ));

    // Stuff
    driverController.start().onTrue((Commands.runOnce(drivebase::seedFieldCentric)));
    //driverController.back().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    // Shoot while move sim
    driverController.leftTrigger().onTrue(Commands.runOnce(aimingMath::logSim));
    driverController.rightTrigger().onTrue(Commands.runOnce(aimingMath::resetSim));

    // Shoot while move
    driverController.x().onTrue(Commands.runOnce(() -> {
                            shootWhileMovingVelocity = Vector3.origin();
                        }))
                        .whileTrue(drivebase.applyRequest(() -> driveFacingAngle.withVelocityX(shootWhileMovingVelocity.x)
                                                                                .withVelocityY(shootWhileMovingVelocity.y)
                                                                                .withTargetDirection(Rotation2d.fromRadians(aimingMath.getIdealHeading(aimingMath.getIdealShotSpeed(DrivebaseConstants.LOOKAHEAD),DrivebaseConstants.LOOKAHEAD))))
                                   .alongWith(shooter.setVelocity(() -> RPM.of(aimingMath.getIdealShotSpeed(ShooterConstants.LOOKAHEAD)*ShooterConstants.SHOT_SPEED_CONVERSION_FACTOR)))
                                   .alongWith(Commands.run(() -> {
                                       aimingMath.isShooting = false;
                                       Vector3 difference = Vector3.subtract(new Vector3(-driverController.getLeftY()*DrivebaseConstants.SHOOT_WHILE_MOVING_SPEED, 
                                                                                         -driverController.getLeftX()*DrivebaseConstants.SHOOT_WHILE_MOVING_SPEED, 0),
                                                                             shootWhileMovingVelocity);
                                       shootWhileMovingVelocity = Vector3.add(shootWhileMovingVelocity, Vector3.scale(Vector3.normalize(difference), 
                                                                                                                      0.02*DrivebaseConstants.SHOOT_WHILE_MOVE_ACCEL_LIMIT));
                                       
                                   })))
                        .onFalse(Commands.runOnce(() -> {aimingMath.isShooting = false;})
                                 .alongWith(shooter.stop()));
    driverController.b().whileTrue(shooter.setVelocity(() -> RPM.of(Timer.getFPGATimestamp()*10)));

    // Shooter sim stuff
    // operatorController.leftBumper().whileTrue(shooter.setLowVelocity()).onFalse(shooter.setZeroVelocity());
    // operatorController.rightBumper().whileTrue(shooter.setHighVelocity()).onFalse(shooter.setZeroVelocity());
    // operatorController.leftTrigger().whileTrue(shooter.setLow()).onFalse(shooter.setZero());
    // operatorController.rightTrigger().whileTrue(shooter.setHigh()).onFalse(shooter.setZero());
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
