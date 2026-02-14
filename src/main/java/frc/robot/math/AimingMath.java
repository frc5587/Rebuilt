package frc.robot.math;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.math.BigDecimal;
import java.util.ArrayList;

public class AimingMath extends SubsystemBase {
  private final Supplier<Vector3> robotPosition;
  private final DoubleSupplier headingRadians;
  private final Supplier<Vector3> robotVelocity;
  private final DoubleSupplier angularVelocityRadians;
  
  private Vector3 goalPosition;

  private String simLog = "";
  private ArrayList<Double> times = new ArrayList<Double>();
  private ArrayList<Double> shotSpeeds = new ArrayList<Double>();
  private ArrayList<Double> angles = new ArrayList<Double>();
  private ArrayList<Vector3> positions = new ArrayList<Vector3>();
  private ArrayList<Vector3> velocities = new ArrayList<Vector3>();
  private ArrayList<Double> headings = new ArrayList<Double>();
  private ArrayList<Double> angularVelocities = new ArrayList<Double>();

  public boolean isShooting = false;
  private ArrayList<Double> shotTimes = new ArrayList<Double>();
  private StructArrayPublisher<Pose3d> fuelPosePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyPoseArray", Pose3d.struct)
      .publish();

  private Field2d field = new Field2d();
  
  public AimingMath(Supplier<Vector3> _robotVelocity, 
                    DoubleSupplier _angularVelocityRadians,
                    Supplier<Vector3> _robotPosition,
                    DoubleSupplier _heading,
                    Vector3 _goalPosition) {
                  
    robotVelocity = _robotVelocity;
    angularVelocityRadians = _angularVelocityRadians;
    robotPosition = _robotPosition;
    headingRadians = _heading;
    goalPosition = _goalPosition;

    times.add(0.);
    shotTimes.add(0.);

    SmartDashboard.putString("SimResults", simLog);
  }
  
  public void setGoal(Vector3 _goalPosition) {
    goalPosition = _goalPosition;
  }

  public double getIdealShotSpeed() {
    return getIdealShotSpeed(0.);
  }

  public double getIdealShotSpeed(double lookahead) {
    Vector3 position = Vector3.add(robotPosition.get(),
                                   Vector3.rotate(Constants.ShooterConstants.SHOOTER_POSITION,
                                                  Vector3.origin(),
                                                  headingRadians.getAsDouble()));
    double turretDistance = Constants.ShooterConstants.SHOOTER_POSITION.get2D().length();
    Vector3 tangent = Vector3.scale(Vector3.rotate(Constants.ShooterConstants.SHOOTER_POSITION.get2D(),
                                                  Vector3.origin(),
                                                  Math.PI/2 + headingRadians.getAsDouble()),
                                    1.0/turretDistance);
    Vector3 velocity = Vector3.add(robotVelocity.get(),
                                  Vector3.scale(tangent,angularVelocityRadians.getAsDouble()*turretDistance));
    position = Vector3.add(position, Vector3.scale(velocity, lookahead));

    double speed = 0;
    Vector3 adjustedGoalPosition = new Vector3(goalPosition.x, goalPosition.y, goalPosition.z);
    for (int i = 0; i < Constants.ShooterConstants.SEARCH_DEPTH; i++) {
      Vector3 idealVector = Vector3.subtract(adjustedGoalPosition, position).get2D();
      double distance = idealVector.get2D().length();
      speed = ((distance*Math.sqrt(Constants.ShooterConstants.GRAVITY))/Math.cos(Constants.ShooterConstants.PITCH)) /
              (Math.sqrt(2.)*Math.sqrt(Math.abs(adjustedGoalPosition.z-position.z-(distance*Math.tan(Constants.ShooterConstants.PITCH)))));
      double time = distance/(speed*Math.cos(Constants.ShooterConstants.PITCH));

      adjustedGoalPosition = Vector3.subtract(goalPosition,Vector3.scale(velocity, time));
    }
    return speed;
  }

  public double getIdealHeading() {
    return getIdealHeading(getIdealShotSpeed(0.),0.);
  }

  public double getIdealHeading(double speed, double lookahead) {
    Vector3 position = Vector3.add(robotPosition.get(),
                                   Vector3.rotate(ShooterConstants.SHOOTER_POSITION,
                                                  Vector3.origin(),
                                                  headingRadians.getAsDouble()));
    double turretDistance = ShooterConstants.SHOOTER_POSITION.get2D().length();
    Vector3 tangent = Vector3.scale(Vector3.rotate(ShooterConstants.SHOOTER_POSITION.get2D(),
                                                   Vector3.origin(),
                                                   Math.PI/2 + headingRadians.getAsDouble()),
                                    1.0/turretDistance);
    Vector3 velocity = Vector3.add(robotVelocity.get(),
                                   Vector3.scale(tangent,angularVelocityRadians.getAsDouble()*turretDistance));
    position = Vector3.add(position, Vector3.scale(velocity, lookahead));
    
    double a = speed * Math.cos(ShooterConstants.PITCH);
    double c = velocity.length();

    Vector3 idealVector = Vector3.subtract(goalPosition, position).get2D();
    double targetAngle = idealVector.getCounterclockwiseAngle();
    double driveAngle = velocity.getCounterclockwiseAngle();
    double A = (targetAngle - driveAngle) % (2 * Math.PI);
    double C = Math.asin(Math.max(-1.,  Math.min((c*Math.sin(A)) / a, 1.0)));

    if (c > 0.0001) {
      return targetAngle + C;
    }
    else {
      return targetAngle;
    }
  }

  public void shoot(double shotSpeed) {
    SimulatedArena.getInstance()
                  .addGamePieceProjectile(new RebuiltFuelOnFly(
                      new Translation2d(robotPosition.get().x,robotPosition.get().y),
                      new Translation2d(), // shooter offet from center
                      new ChassisSpeeds(robotVelocity.get().x,robotVelocity.get().y,angularVelocityRadians.getAsDouble()),
                      new Rotation2d(headingRadians.getAsDouble()),
                      Meters.of(ShooterConstants.SHOOTER_POSITION.z), // initial height of the ball, in meters
                      MetersPerSecond.of(shotSpeed), // initial velocity, in m/s
                      Angle.ofRelativeUnits(ShooterConstants.PITCH,Units.Radian)));
  }

  @Override
  public void periodic() {
    if (times.get(times.size()-1) < Timer.getFPGATimestamp() - 0.07) {
      double idealShotSpeed = getIdealShotSpeed(0.);
      // Replace with actual or simulated values
      double shotSpeed = idealShotSpeed * 1.0;
      double angle = 0;
      
      times.add(Timer.getFPGATimestamp());
      shotSpeeds.add(shotSpeed);
      angles.add(angle);
      positions.add(robotPosition.get());
      velocities.add(robotVelocity.get());
      headings.add(headingRadians.getAsDouble());
      // headings.add(getIdealHeading());
      angularVelocities.add(angularVelocityRadians.getAsDouble());
    }

    if (isShooting  &&  shotTimes.get(shotTimes.size()-1) < Timer.getFPGATimestamp() - 1/ShooterConstants.SHOTS_PER_SECOND) {
      double idealShotSpeed = getIdealShotSpeed();
      shoot(idealShotSpeed);

      shotTimes.add(Timer.getFPGATimestamp());
    }

    SmartDashboard.putString("SimResults", simLog);
    field.setRobotPose(new Pose2d(new Translation2d(robotPosition.get().x, robotPosition.get().y), Rotation2d.fromRadians(getIdealHeading())));
    SmartDashboard.putData(field);
    SmartDashboard.putNumber("ideal heading", getIdealHeading());

    // Get the positions of the fuel (both on the field and in the air)
      Pose3d[] fuelPoses = SimulatedArena.getInstance()
            .getGamePiecesArrayByType("Fuel");
      fuelPosePublisher.accept(fuelPoses);
  }

  public void resetSim() {
    times.clear();
    shotSpeeds.clear();
    angles.clear();
    positions.clear();
    velocities.clear();
    headings.clear();
    angularVelocities.clear();

    times.add(0.);
    shotSpeeds.add(0.);
    angles.add(0.);
    positions.add(Vector3.origin());
    velocities.add(Vector3.origin());
    headings.add(0.);
    angularVelocities.add(0.);
  }
  
  public void logSim() {
    simLog = "";
    simLog += parseList("t_list",times,(Double item) -> BigDecimal.valueOf(item).toPlainString());
    simLog += parseList("s_list",shotSpeeds,(Double item) -> BigDecimal.valueOf(item).toPlainString());
    simLog += parseList("a_ngleList",angles,(Double item) -> BigDecimal.valueOf(item).toPlainString());
    simLog += parseList("p_ositionList",positions,(Vector3 item) -> item.toString());
    simLog += parseList("v_elocityList",velocities,(Vector3 item) -> item.toString());
    simLog += parseList("h_eadingList",headings,(Double item) -> BigDecimal.valueOf(item).toPlainString());
    simLog += parseList("a_ngularVelocityList",angularVelocities,(Double item) -> BigDecimal.valueOf(item).toPlainString());
  }
  
  private <T> String parseList(String name, ArrayList<T> list, Function<T,String> function) {
    String result = "";
    result += name.charAt(0)+"_{"+name.substring(2)+"}=\\left[";
    for (int i = 0; i < list.size()-1; i++) {
      result += function.apply(list.get(i));
      result += ",";
    }
    result += function.apply(list.get(list.size()-1));
    result += "\\right]\n";
    return result;
  }
}