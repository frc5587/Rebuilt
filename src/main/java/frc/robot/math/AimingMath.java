package frc.robot.math;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ShooterConstants;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.math.BigDecimal;
import java.util.ArrayList;

public class AimingMath {
  private final Supplier<Vector3> robotPosition;
  private final DoubleSupplier headingRadians;
  private final Supplier<Vector3> robotVelocity;
  private final DoubleSupplier angularVelocityRadians;
  private final Supplier<Vector3> inputRobotVelocity;
  private final DoubleSupplier inputAngularVelocityRadians;
  private final DoubleSupplier flywheelRPM;
  
  private Vector3 goalPosition;

  private String simLog = "";
  private ArrayList<Double> times = new ArrayList<Double>();
  private ArrayList<Double> shotSpeeds = new ArrayList<Double>();
  private ArrayList<Double> angles = new ArrayList<Double>();
  private ArrayList<Vector3> positions = new ArrayList<Vector3>();
  private ArrayList<Double> headings = new ArrayList<Double>();
  private ArrayList<Vector3> velocities = new ArrayList<Vector3>();
  private ArrayList<Double> angularVelocities = new ArrayList<Double>();

  Function<String,StructPublisher<Pose2d>> publisher = (String name) -> NetworkTableInstance.getDefault()
                                                                                            .getStructTopic("aiming debug/"+name, Pose2d.struct).publish();
  StructPublisher<Pose2d> idealPosePublisher = publisher.apply("ideal pose");
  StructPublisher<Pose2d> futurePosePublisher = publisher.apply("future pose");
  
  public AimingMath(Supplier<Vector3> _robotPosition,
                    DoubleSupplier _heading,
                    Supplier<Vector3> _robotVelocity, 
                    DoubleSupplier _angularVelocityRadians,
                    Supplier<Vector3> _inputRobotVelocity,
                    DoubleSupplier _inputAngularVelocityRadians,
                    DoubleSupplier _flywheelRPM,
                    Vector3 _goalPosition) {
                  
    robotPosition = _robotPosition;
    headingRadians = _heading;
    robotVelocity = _robotVelocity;
    angularVelocityRadians = _angularVelocityRadians;
    inputRobotVelocity = _inputRobotVelocity;
    inputAngularVelocityRadians = _inputAngularVelocityRadians;
    goalPosition = _goalPosition;
    flywheelRPM = _flywheelRPM;

    times.add(0.);

    SmartDashboard.putString("SimResults", simLog);
  }
  
  public void setGoal(Vector3 _goalPosition) {
    goalPosition = _goalPosition;
  }

  public double getIdealShotSpeed() {
    return getIdealShotSpeed(0.);
  }

  public double getIdealShotSpeed(double lookahead) {
    return getIdealShotSpeed(lookahead, robotPosition.get(), headingRadians.getAsDouble(), inputRobotVelocity.get(), inputAngularVelocityRadians.getAsDouble());
  }

  public double getIdealShotSpeed(Vector3 robotPosition, double heading, Vector3 robotVelocity, double angularVelocity) {
    return getIdealShotSpeed(0.,robotPosition, heading, robotVelocity, angularVelocity);
  }

  public double getIdealShotSpeed(double lookahead, Vector3 robotPosition, double heading, Vector3 robotVelocity, double angularVelocity) {
    Vector3 position = Vector3.add(robotPosition,
                                   Vector3.rotate(ShooterConstants.SHOOTER_POSITION,
                                                  Vector3.origin(),
                                                  heading));
    double turretDistance = ShooterConstants.SHOOTER_POSITION.get2D().length();
    Vector3 tangent = Vector3.scale(Vector3.rotate(ShooterConstants.SHOOTER_POSITION.get2D(),
                                                   Vector3.origin(),
                                                   Math.PI/2 + heading),
                                    1.0/turretDistance);
    Vector3 velocity = Vector3.add(robotVelocity,
                                   Vector3.scale(tangent,angularVelocity*turretDistance));
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
    return getIdealHeading(speed, lookahead, robotPosition.get(), headingRadians.getAsDouble(), inputRobotVelocity.get(), inputAngularVelocityRadians.getAsDouble());
  }

  public double getIdealHeading(Vector3 robotPosition, double heading, Vector3 robotVelocity, double angularVelocity) {
    return getIdealHeading(getIdealShotSpeed(0., robotPosition, heading, robotVelocity, angularVelocity), 0., robotPosition, heading, robotVelocity, angularVelocity);
  }

  public double getIdealHeading(double speed, double lookahead, Vector3 robotPosition, double heading, Vector3 robotVelocity, double angularVelocity) {
    Vector3 position = Vector3.add(robotPosition,
                                   Vector3.rotate(ShooterConstants.SHOOTER_POSITION,
                                                  Vector3.origin(),
                                                  heading));
    double turretDistance = ShooterConstants.SHOOTER_POSITION.get2D().length();
    Vector3 tangent = Vector3.scale(Vector3.rotate(ShooterConstants.SHOOTER_POSITION.get2D(),
                                                   Vector3.origin(),
                                                   Math.PI/2 + heading),
                                    1.0/turretDistance);
    Vector3 velocity = Vector3.add(robotVelocity,
                                   Vector3.scale(tangent,angularVelocity*turretDistance));
    position = Vector3.add(position, Vector3.scale(velocity, lookahead));
    
    double ballRelativeHorizontalSpeed = speed * Math.cos(ShooterConstants.PITCH);
    double turretSpeed = velocity.length();

    Vector3 idealVector = Vector3.subtract(goalPosition, position).get2D();
    double targetAngle = Vector3.getCounterclockwiseAngle(idealVector);
    double driveAngle = Vector3.getCounterclockwiseAngle(velocity);
    double correctionAngle = Math.asin(Math.max(-1.,  Math.min((turretSpeed*Math.sin(targetAngle - driveAngle)) / ballRelativeHorizontalSpeed, 1.0)));

    if (turretSpeed > 0.0001) {
      return targetAngle + correctionAngle;
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

  public void logAdvantagescopeStuff() {
    idealPosePublisher.accept(new Pose2d(new Translation2d(robotPosition.get().x, robotPosition.get().y), Rotation2d.fromRadians(getIdealHeading())));
    futurePosePublisher.accept(new Pose2d(new Translation2d(robotPosition.get().x + DrivebaseConstants.LOOKAHEAD*robotVelocity.get().x, 
                                                                             robotPosition.get().y + DrivebaseConstants.LOOKAHEAD*robotVelocity.get().y),
                                                                             Rotation2d.fromRadians(getIdealHeading(getIdealShotSpeed(DrivebaseConstants.LOOKAHEAD),DrivebaseConstants.LOOKAHEAD,robotPosition.get(),headingRadians.getAsDouble(),robotVelocity.get(),angularVelocityRadians.getAsDouble()))));
    SmartDashboard.putNumber("ideal heading", getIdealHeading());
    SmartDashboard.putString("SimResults", simLog);
    SmartDashboard.putNumber("ideal RPM", getIdealShotSpeed() * ShooterConstants.SHOT_SPEED_CONVERSION_FACTOR);
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
  
  public void addSimSnapshot() {
    double shotSpeed = flywheelRPM.getAsDouble() / ShooterConstants.SHOT_SPEED_CONVERSION_FACTOR;
    double angle = 0;
      
    times.add(Timer.getFPGATimestamp());
    shotSpeeds.add(shotSpeed);
    angles.add(angle);
    positions.add(robotPosition.get());
    headings.add(headingRadians.getAsDouble());
    velocities.add(robotVelocity.get());
    angularVelocities.add(angularVelocityRadians.getAsDouble());
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