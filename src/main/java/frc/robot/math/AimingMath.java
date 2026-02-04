package frc.robot.math;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.ArrayList;

public class AimingMath extends SubsystemBase {
  private final Supplier<Vector3> robotVelocity;
  private final DoubleSupplier angularVelocityRadians;
  private final Supplier<Vector3> robotPosition;
  private final DoubleSupplier heading;
  
  private Vector3 goalPosition;
  
  private ArrayList<Double> times = new ArrayList<Double>();
  private ArrayList<Double> shotSpeeds = new ArrayList<Double>();
  private ArrayList<Double> angles = new ArrayList<Double>();
  private ArrayList<Vector3> positions = new ArrayList<Vector3>();
  private ArrayList<Vector3> velocities = new ArrayList<Vector3>();
  private ArrayList<Double> headings = new ArrayList<Double>();
  private ArrayList<Double> angularVelocities = new ArrayList<Double>();
  
  public AimingMath(Supplier<Vector3> _robotVelocity, 
                    DoubleSupplier _angularVelocityRadians,
                    Supplier<Vector3> _robotPosition,
                    DoubleSupplier _heading,
                    Vector3 _goalPosition) {
                  
    robotVelocity = _robotVelocity;
    angularVelocityRadians = _angularVelocityRadians;
    robotPosition = _robotPosition;
    heading = _heading;
    goalPosition = _goalPosition;
  }
  
  public void setGoal(Vector3 _goalPosition) {
    goalPosition = _goalPosition;
  }
  
  private double evalShotSpeed(double speed) {
    Vector3 position = Vector3.add(robotPosition.get(),
                                   Vector3.rotate(Constants.ShooterConstants.turretPlacement,
                                                  Vector3.origin(),
                                                  heading.getAsDouble()));
    double turretDistance = Constants.ShooterConstants.turretPlacement.get2D().length();
    Vector3 tangent = Vector3.scale(Vector3.rotate(Constants.ShooterConstants.turretPlacement.get2D(),
                                                   Vector3.origin(),
                                                   Math.PI/2 + heading.getAsDouble()),
                                    1.0/turretDistance);
    Vector3 velocity = Vector3.add(robotVelocity.get(),
                                   Vector3.scale(tangent,angularVelocityRadians.getAsDouble()*turretDistance));
    
    double a = speed * Math.cos(Constants.ShooterConstants.pitch);
    double c = velocity.length();

    Vector3 idealVector = Vector3.subtract(goalPosition, position).get2D();
    double targetAngle = idealVector.getCounterclockwiseAngle();
    double driveAngle = velocity.getCounterclockwiseAngle();
    double A = (targetAngle - driveAngle) % (2 * Math.PI);
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

    double distance = idealVector.get2D().length();
    double time = distance / b;
    return speed*time*Math.sin(Constants.ShooterConstants.pitch) - (Constants.ShooterConstants.gravity/2)*time*time - goalPosition.z + Constants.ShooterConstants.turretPlacement.z;
  }

  public double getIdealShotSpeed() {
    double min = Constants.ShooterConstants.minShotSpeed;
    double max = Constants.ShooterConstants.maxShotSpeed;
    for (int i = 0; i < Constants.ShooterConstants.searchDepth; i++) {
      double temp = (min+max)/2;
      double current = evalShotSpeed(temp);
      if (current < 0) {
        min = temp;
      }
      else {
        max = temp;
      }
    }
    return max;
  }

  public double getIdealAngle(double speed) {
    Vector3 position = Vector3.add(robotPosition.get(),
                                   Vector3.rotate(Constants.ShooterConstants.turretPlacement,
                                                  Vector3.origin(),
                                                  heading.getAsDouble()));
    double turretDistance = Constants.ShooterConstants.turretPlacement.get2D().length();
    Vector3 tangent = Vector3.scale(Vector3.rotate(Constants.ShooterConstants.turretPlacement.get2D(),
                                                   Vector3.origin(),
                                                   Math.PI/2 + heading.getAsDouble()),
                                    1.0/turretDistance);
    Vector3 velocity = Vector3.add(robotVelocity.get(),
                                   Vector3.scale(tangent,angularVelocityRadians.getAsDouble()*turretDistance));
    
    double a = speed * Math.cos(Constants.ShooterConstants.pitch);
    double c = velocity.length();

    Vector3 idealVector = Vector3.subtract(goalPosition, position).get2D();
    double targetAngle = idealVector.getCounterclockwiseAngle();
    double driveAngle = velocity.getCounterclockwiseAngle();
    double A = (targetAngle - driveAngle) % (2 * Math.PI);
    double C = Math.asin((c*Math.sin(A)) / a);

    return targetAngle + C + heading.getAsDouble();
  }

  @Override
  public void periodic() {
    double idealShotSpeed = getIdealShotSpeed();
    double idealAngle = getIdealAngle(idealShotSpeed);
    
    // Replace with actual or simulated values
    double shotSpeed = idealShotSpeed * 1.0;
    double angle = idealAngle * 1.0;
    
    times.add(Timer.getFPGATimestamp());
    shotSpeeds.add(shotSpeed);
    angles.add(angle);
    positions.add(robotPosition.get());
    velocities.add(robotVelocity.get());
    headings.add(heading.getAsDouble());
    angularVelocities.add(angularVelocityRadians.getAsDouble());
  }
  
  public Command logSim() {
    return run( () -> {
      String sim = "";
      sim += parseList("t_list",times,(Double item) -> ""+item);
      sim += parseList("s_list",shotSpeeds,(Double item) -> ""+item);
      sim += parseList("a_ngleList",angles,(Double item) -> ""+item);
      sim += parseList("p_ositionList",positions,(Vector3 item) -> item.toString());
      sim += parseList("v_elocityList",velocities,(Vector3 item) -> item.toString());
      sim += parseList("h_eadingList",headings,(Double item) -> ""+item);
      sim += parseList("a_ngularVelocityList",angularVelocities,(Double item) -> ""+item);
      SmartDashboard.putString("Sim results", sim);
    });
  }
  
  private <T> String parseList(String name, ArrayList<T> list, Function<T,String> function) {
    String result = "";
    result += name.charAt(0)+"_{"+name.substring(2)+"}=\\left[";
    for (int i = 0; i < list.size()-1; i++) {
      result += function.apply(list.get(i));
      result += ",";
    }
    result += list.get(list.size()-1);
    result += "\\right]\n";
    return result;
  }
}