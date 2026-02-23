package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.math.AimingMath;
import frc.robot.math.Vector3;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AimTowardsGoal extends Command {
  private Vector3 shootWhileMovingVelocity = Vector3.origin();
  private Vector3 shootWhileMovingCalculationVelocity = Vector3.origin();
  private Supplier<Vector3> inputTargetVelocity;
  private double lastLogTimestamp = 0.;
  private double lastShotTimestamp = 0.;
  private ShooterSubsystem shooter;
  private SwerveSubsystem swerve;
  private AimingMath aimingMath;
  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(DrivebaseConstants.MAX_SPEED * 0.1) // Add a 10% deadband
      .withMaxAbsRotationalRate(DrivebaseConstants.MAX_SPIN_SPEED_RADIANS_PER_SECOND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withHeadingPID(DrivebaseConstants.SHOOT_WHILE_MOVE_HEADING_CONTROLLER.getP(), DrivebaseConstants.SHOOT_WHILE_MOVE_HEADING_CONTROLLER.getI(), DrivebaseConstants.SHOOT_WHILE_MOVE_HEADING_CONTROLLER.getD());

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
  Supplier<Vector3> inputVelocity = () -> {
    Vector3 input = shootWhileMovingCalculationVelocity;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      input = Vector3.scale(input, -1.);
    }
    return input;
  };
  DoubleSupplier inputAngularVelocity = () -> 0.;
  Vector3 goal;

  public AimTowardsGoal(Supplier<Vector3> _inputTargetVelocity, ShooterSubsystem _shooter, SwerveSubsystem _swerve, Vector3 _goalPosition) {
    inputTargetVelocity = _inputTargetVelocity;
    shooter = _shooter;
    swerve = _swerve;
    aimingMath = new AimingMath(position, heading, velocity, angularVelocity, inputVelocity, inputAngularVelocity, () -> shooter.getVelocity().in(RPM), _goalPosition);
  }

  @Override
  public void initialize() {
    shootWhileMovingVelocity = Vector3.origin();
    shooter.set(0.);
  }

  @Override
  public void execute() {
    Vector3 difference = Vector3.subtract(inputTargetVelocity.get(),
                                          shootWhileMovingVelocity);
    // if (difference.length() < 50.) {
    //   shootWhileMovingCalculationVelocity = Vector3.add(shootWhileMovingVelocity, difference);
    // }
    // else {
    //   shootWhileMovingCalculationVelocity = Vector3.add(shootWhileMovingVelocity, Vector3.scale(Vector3.normalize(difference), 50.));
    // }
    shootWhileMovingCalculationVelocity = Vector3.add(shootWhileMovingVelocity, difference);
    shootWhileMovingVelocity = Vector3.add(shootWhileMovingVelocity, Vector3.scale(Vector3.normalize(difference), 0.02*DrivebaseConstants.SHOOT_WHILE_MOVE_ACCEL_LIMIT));
    CommandScheduler.getInstance().schedule(swerve.applyRequest(() -> driveFacingAngle.withVelocityX(shootWhileMovingVelocity.x)
                                                                                      .withVelocityY(shootWhileMovingVelocity.y)
                                                                                      .withTargetDirection(Rotation2d.fromRadians(aimingMath.getIdealHeading(aimingMath.getIdealShotSpeed(DrivebaseConstants.LOOKAHEAD),DrivebaseConstants.LOOKAHEAD) + (DriverStation.getAlliance().get() == Alliance.Blue ? 0. : Math.PI)))));
    CommandScheduler.getInstance().schedule(shooter.setAngularVelocity(() -> RPM.of(aimingMath.getIdealShotSpeed(ShooterConstants.LOOKAHEAD)*ShooterConstants.SHOT_SPEED_CONVERSION_FACTOR)));

    if (lastLogTimestamp < Timer.getFPGATimestamp() - ShooterConstants.TIME_BETWEEN_LOG_TIMESTAMPS) {
      lastLogTimestamp = Timer.getFPGATimestamp();
      aimingMath.addSimSnapshot();
    }
    if (lastShotTimestamp < Timer.getFPGATimestamp() - 1/ShooterConstants.SHOTS_PER_SECOND) {
      lastShotTimestamp = Timer.getFPGATimestamp();
      aimingMath.shoot(aimingMath.getIdealShotSpeed());
    }
    aimingMath.logAdvantagescopeStuff();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.getCurrentCommand().cancel();
    shooter.getCurrentCommand().cancel();
  }

  public AimingMath getMath() {
    return aimingMath;
  }
}
