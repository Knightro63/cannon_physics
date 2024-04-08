import 'dart:math' as math;
import 'package:vector_math/vector_math.dart';

final vec3TangentsN = Vector3.zero();
final vec3TangentsRandVec = Vector3.zero();
final antipNeg = Vector3.zero();

/// 3-dimensional vector
/// @example
///     final v = new Vec3(1, 2, 3)
///     console.log('x=' + v.x) // x=1
extension Vec3 on Vector3 {
  static Vector3 unitX = Vector3(1, 0, 0);
  static Vector3 unitY = Vector3(0, 1, 0);
  static Vector3 unitZ = Vector3(0, 0, 1);

  /// Vector addition
  Vector3 add2(Vector3 vector, Vector3 target){
    target.x = vector.x + x;
    target.y = vector.y + y;
    target.z = vector.z + z;
    return target;
  }
  /// Vector subtraction
  ///  @param target Optional target to save in.
  Vector3 sub2(Vector3 vector, Vector3 target){
    target.x =  x-vector.x;
    target.y =  y-vector.y;
    target.z =  z-vector.z;
    return target;
  }
  /// Vector cross product
  /// @param target Optional target to save in.
  Vector3 inverse(){
    x *=-1;
    y *= -1;
    z *= -1;
    return this;
  }
  /// Do a linear interpolation between two vectors
  /// @param t A number between 0 and 1. 0 will make this function return u, and 1 will make it return v. Numbers in between will generate a vector in between them.
  void lerp(Vector3 vector,double t, Vector3 target) {
    target.x = x + (vector.x - x) * t;
    target.y = y + (vector.y - y) * t;
    target.z = z + (vector.z - z) * t;
  }
  /// Multiply all the components of the vector with a scalar.
  /// @param target The vector to save the result in.
  Vector3 scale2(double scalar, [Vector3? target]){
    target ??= Vector3.zero();
    target.x = scalar * x;
    target.y = scalar * y;
    target.z = scalar * z;
    return target;
  }
  /// Vector cross product
  /// @param target Optional target to save in.
  Vector3 cross2(Vector3 vector, [Vector3? target]){
    target ??= Vector3.zero();
    final vx = vector.x;
    final vy = vector.y;
    final vz = vector.z;

    target.x = y * vz - z * vy;
    target.y = z * vx - x * vz;
    target.z = x * vy - y * vx;

    return target;
  }
  /// Multiply the vector with an other vector, component-wise.
  /// @param target The vector to save the result in.
  Vector3 multiply2(Vector3 vector, [Vector3? target]){
    target ??= Vector3.zero();
    target.x = vector.x * x;
    target.y = vector.y * y;
    target.z = vector.z * z;
    return target;
  }
  /// Get distance from this point to another point
  double distanceTo(Vector3 p) {
    final px = p.x;
    final py = p.y;
    final pz = p.z;
    return math.sqrt((px - x) * (px - x) + (py - y) * (py - y) + (pz - z) * (pz - z));
  }

  /// Get squared distance from this point to another point
  double distanceSquared(Vector3 p) {
    final px = p.x;
    final py = p.y;
    final pz = p.z;
    return (px - x) * (px - x) + (py - y) * (py - y) + (pz - z) * (pz - z);
  }
  /// Compute two artificial tangents to the vector
  /// @param t1 Vector object to save the first tangent in
  /// @param t2 Vector object to save the second tangent in
  void tangents(Vector3 t1, Vector3 t2){
    final norm = length;
    if (norm > 0.0) {
      final n = vec3TangentsN;
      final inorm = 1 / norm;
      n.setValues(x * inorm, y * inorm, z * inorm);
      final randVec = vec3TangentsRandVec;
      if (n.x.abs() < 0.9) {
        randVec.setValues(1, 0, 0);
        n.cross2(randVec, t1);
      } else {
        randVec.setValues(0, 1, 0);
        n.cross2(randVec, t1);
      }
      n.cross2(t1, t2);
    } else {
      // The normal length is zero, make something up
      t1.setValues(1, 0, 0);
      t2.setValues(0, 1, 0);
    }
  }
  /// Get the version of this vector that is of length 1.
  /// @param target Optional target to save in
  /// @return Returns the unit vector
  Vector3 unit([Vector3? target]) {
    target ??= Vector3.zero();
    double ninv = math.sqrt(x * x + y * y + z * z);
    if (ninv > 0.0) {
      ninv = 1.0 / ninv;
      target.x = x * ninv;
      target.y = y * ninv;
      target.z = z * ninv;
    } else {
      target.x = 1;
      target.y = 0;
      target.z = 0;
    }
    return target;
  }
  /// Scale a vector and add it to this vector. Save the result in "target". (target = this + vector * scalar)
  /// @param target The vector to save the result in.
  Vector3 addScaledVector(double scalar, Vector3 vector, [Vector3? target]) {
    target ??= Vector3.zero();
    target.x = x + scalar * vector.x;
    target.y = y + scalar * vector.y;
    target.z = z + scalar * vector.z;
    return target;
  }
  /// Check if a vector equals is almost equal to another one.
  bool almostEquals(Vector3 vector,[double? precision]) {
    precision ??= 1e-6;
    if (
      (x - vector.x).abs() > precision ||
      (y - vector.y).abs() > precision ||
      (z - vector.z).abs() > precision
    ) {
      return false;
    }
    return true;
  }

  /// Check if a vector is almost zero
  bool almostZero([double precision = 1e-6]) {
    if (x.abs() > precision || y.abs() > precision || z.abs() > precision) {
      return false;
    }
    return true;
  }
  bool isZero() {
    return x == 0 && y == 0 && z == 0;
  }
}