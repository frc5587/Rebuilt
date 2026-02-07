package frc.robot.math;

import java.util.function.Supplier;

import org.dyn4j.collision.FixtureModificationHandler;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
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
import java.util.Arrays;

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

  public boolean IsShooting = false;
  private ArrayList<Double> shotTimes = new ArrayList<Double>();
  private StructArrayPublisher<Pose3d> fuelPosePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyPoseArray", Pose3d.struct)
      .publish();
  
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
    double c = velocity.length();
    Vector3 idealVector = Vector3.subtract(goalPosition, position).get2D();
    double targetAngle = idealVector.getCounterclockwiseAngle();
    double driveAngle = velocity.getCounterclockwiseAngle();
    double A = (targetAngle - driveAngle) % (2 * Math.PI);
    double distance = idealVector.get2D().length();

    double min = Constants.ShooterConstants.MIN_SHOOT_SPEED;
    double max = Constants.ShooterConstants.MAX_SHOT_SPEED;
    for (int i = 0; i < Constants.ShooterConstants.SEARCH_DEPTH; i++) {
      double temp = (min+max)/2;
      
      double a = temp * Math.cos(Constants.ShooterConstants.PITCH);
      double C = Math.asin((c*Math.sin(A)) / a);
      double B = Math.PI - A - C;
      double b = 0;
      if (A == 0) {
        b = a + c;
      }
      else if (A == Math.PI) {
        b = a - c;
      }
      else {
        b = (a*Math.sin(B))/Math.sin(A);
      }
      double time = distance / b;
      double current = temp*time*Math.sin(Constants.ShooterConstants.PITCH) - (Constants.ShooterConstants.GRAVITY/2)*time*time - goalPosition.z + Constants.ShooterConstants.SHOOTER_POSITION.z;
      
      if (current < 0) {
        min = temp;
      }
      else {
        max = temp;
      }
    }
    return max;
  }

  public double getIdealHeading(double speed) {
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
    
    double a = speed * Math.cos(Constants.ShooterConstants.PITCH);
    double c = velocity.length();

    Vector3 idealVector = Vector3.subtract(goalPosition, position).get2D();
    double targetAngle = idealVector.getCounterclockwiseAngle();
    double driveAngle = velocity.getCounterclockwiseAngle();
    double A = (targetAngle - driveAngle) % (2 * Math.PI);
    double C = Math.asin((c*Math.sin(A)) / a);

    return targetAngle + C;
  }

  public double getIdealHeading() {
    return getIdealHeading(getIdealShotSpeed());
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
    // if (times.get(times.size()-1) < Timer.getFPGATimestamp() - 0.2) {
    //   double idealShotSpeed = getIdealShotSpeed();
    //   // Replace with actual or simulated values
    //   double shotSpeed = idealShotSpeed * 1.0;
    //   double angle = 0;
      
    //   times.add(Timer.getFPGATimestamp());
    //   shotSpeeds.add(shotSpeed);
    //   angles.add(angle);
    //   positions.add(robotPosition.get());
    //   velocities.add(robotVelocity.get());
    //   headings.add(headingRadians.getAsDouble());
    //   angularVelocities.add(angularVelocityRadians.getAsDouble());
    // }

    if (IsShooting  &&  shotTimes.get(shotTimes.size()-1) < Timer.getFPGATimestamp() - 1/ShooterConstants.SHOTS_PER_SECOND) {
      double idealShotSpeed = getIdealShotSpeed();
      shoot(idealShotSpeed);

      shotTimes.add(Timer.getFPGATimestamp());
    }

    SmartDashboard.putString("SimResults", simLog);

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