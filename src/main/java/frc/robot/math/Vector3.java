package frc.robot.math;

import java.math.BigDecimal;

public class Vector3 {
  public double x;
  public double y;
  public double z;

  public Vector3(double inputX, double inputY, double inputZ) {
    x = inputX;
    y = inputY;
    z = inputZ;
  }
  
  public static Vector3 origin() {
    return new Vector3(0,0,0);
  }

  public static Vector3 add(Vector3 vector1, Vector3 vector2) {
    return new Vector3(vector1.x+vector2.x, vector1.y+vector2.y, vector1.z+vector2.z);
  }

  public static Vector3 subtract(Vector3 vector1, Vector3 vector2) {
    return new Vector3(vector1.x-vector2.x, vector1.y-vector2.y, vector1.z-vector2.z);
  }

  public static Vector3 scale(Vector3 vector, double scalar) {
    return new Vector3(vector.x*scalar, vector.y*scalar, vector.z*scalar);
  }

  public static Vector3 normalize(Vector3 vector) {
    return scale(vector, 1/vector.length());
  }
  
  public static Vector3 rotate(Vector3 point, Vector3 pivot, double angle) {
    double x = pivot.x + (pivot.x-point.x)*Math.cos(angle) - (pivot.y-point.y)*Math.sin(angle);
    double y = pivot.y + (pivot.x-point.x)*Math.sin(angle) + (pivot.y-point.y)*Math.cos(angle);
    return new Vector3(x,y,point.z);
  }
  
  public static double dotProduct(Vector3 vector1, Vector3 vector2) {
    return vector1.x*vector2.x + vector1.y*vector2.y + vector1.z*vector2.z;
  }
  
  public static double getAngle(Vector3 vector1, Vector3 vector2) {
    return Math.acos(dotProduct(vector1, vector2) / (vector1.length() * vector2.length()));
  }

  public static double getAngle(Vector3 vector) {
    return getAngle(new Vector3(1,0,0),vector);
  }
  
  public static double getCounterclockwiseAngle(Vector3 vector) {
    if (vector.y < 0) {
      return 2*Math.PI-getAngle(vector);
    }
    else {
      return getAngle(vector);
    }
  }
  
  public double length() {
    return Math.sqrt(x*x + y*y + z*z);
  }
  
  public Vector3 get2D() {
    return new Vector3(x, y, 0);
  }
  
  public String toString() {
    return "("+BigDecimal.valueOf(x).toPlainString()+","+BigDecimal.valueOf(y).toPlainString()+","+BigDecimal.valueOf(z).toPlainString()+")";
  }
}