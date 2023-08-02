import 'package:cannon/math/quaternion.dart';

import  'mat3.dart';
import 'dart:math' as math;

final Vec3_tangents_n = new Vec3();
final Vec3_tangents_randVec = new Vec3();
final antip_neg = new Vec3();

/// 3-dimensional vector
/// @example
///     final v = new Vec3(1, 2, 3)
///     console.log('x=' + v.x) // x=1
class Vec3 {
  double x;
  double y;
  double z;

  static Vec3 zero = Vec3(0, 0, 0);
  static Vec3 unitX = Vec3(0, 0, 0);
  static Vec3 unitY = Vec3(0, 0, 0);
  static Vec3 unitZ = Vec3(0, 0, 0);

  Vec3([this.x = 0.0, this.y = 0.0, this.z = 0.0]);

  void operator []=(int addr, double value){
    if(addr == 0){
      x = value;
    }
    else if(addr == 1){
      y = value;
    }
    else if(addr == 2){
      z = value;
    }
    else{
      throw('Value can not be bigger than 2 nd less than 0');
    }
  }

  double operator [](int addr){
    if(addr == 0){
      return x;
    }
    else if(addr == 1){
      return y;
    }
    else if(addr == 2){
      return z;
    }
    else{
      throw('Value can not be bigger than 2 nd less than 0');
    }
  }

  /// Vector cross product
  /// @param target Optional target to save in.
  Vec3 cross(Vec3 vector, [Vec3? target]){
    target ??= Vec3();
    final vx = vector.x;
    final vy = vector.y;
    final vz = vector.z;
    final x = this.x;
    final y = this.y;
    final z = this.z;

    target.x = y * vz - z * vy;
    target.y = z * vx - x * vz;
    target.z = x * vy - y * vx;

    return target;
  }

  /// Set the vectors' 3 elements
  Vec3 set(double x,double y,double z){
    this.x = x;
    this.y = y;
    this.z = z;
    return this;
  }

  /// Set all components of the vector to zero.
  void setZero() {
    x = y = z = 0;
  }

  /// Vector addition
  Vec3? vadd(Vec3 vector, [Vec3? target]){
    if (target != null) {
      target.x = vector.x + x;
      target.y = vector.y + y;
      target.z = vector.z + z;
      return null;
    } else {
      return Vec3(x + vector.x, y + vector.y, z + vector.z);
    }
  }

  /// Vector subtraction
  ///  @param target Optional target to save in.
  Vec3? vsub(Vec3 vector, [Vec3? target]){
    if (target != null) {
      target.x = x - vector.x;
      target.y = y - vector.y;
      target.z = z - vector.z;
      return null;
    } else {
      return Vec3(x - vector.x, y - vector.y, z - vector.z);
    }
  }

  /// Get the cross product matrix a_cross from a vector, such that a x b = a_cross * b = c
  ///
  /// See {@link https://www8.cs.umu.se/kurser/TDBD24/VT06/lectures/Lecture6.pdf Umeå University Lecture}
  Mat3 crossmat(){
    return Mat3([0, -z, y, z, 0, -x, -y, x, 0]);
  }

  /// Normalize the vector. Note that this changes the values in the vector.
  /// @return Returns the norm of the vector
  double normalize() {
    final x = this.x;
    final y = this.y;
    final z = this.z;
    final n = math.sqrt(x * x + y * y + z * z);
    if (n > 0.0) {
      final invN = 1 / n;
      this.x *= invN;
      this.y *= invN;
      this.z *= invN;
    } else {
      // Make something up
      this.x = 0;
      this.y = 0;
      this.z = 0;
    }
    return n;
  }

  /// Get the version of this vector that is of length 1.
  /// @param target Optional target to save in
  /// @return Returns the unit vector
  Vec3 unit([Vec3? target]) {
    target ??= Vec3();
    final x = this.x;
    final y = this.y;
    final z = this.z;
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

  /// Get the length of the vector
  double length() {
    final x = this.x;
    final y = this.y;
    final z = this.z;
    return math.sqrt(x * x + y * y + z * z);
  }

  /// Get the squared length of the vector.
  double lengthSquared() {
    return dot(this);
  }

  /// Get distance from this point to another point
  double distanceTo(Vec3 p) {
    final x = this.x;
    final y = this.y;
    final z = this.z;
    final px = p.x;
    final py = p.y;
    final pz = p.z;
    return math.sqrt((px - x) * (px - x) + (py - y) * (py - y) + (pz - z) * (pz - z));
  }

  /// Get squared distance from this point to another point
  double distanceSquared(Vec3 p) {
    final x = this.x;
    final y = this.y;
    final z = this.z;
    final px = p.x;
    final py = p.y;
    final pz = p.z;
    return (px - x) * (px - x) + (py - y) * (py - y) + (pz - z) * (pz - z);
  }

  /// Multiply all the components of the vector with a scalar.
  /// @param target The vector to save the result in.
  Vec3 scale(double scalar, [Vec3? target]){
    target ??= Vec3();
    final x = this.x;
    final y = this.y;
    final z = this.z;
    target.x = scalar * x;
    target.y = scalar * y;
    target.z = scalar * z;
    return target;
  }

  /// Multiply the vector with an other vector, component-wise.
  /// @param target The vector to save the result in.
  Vec3 vmul(Vec3 vector, [Vec3? target]){
    target ??= Vec3();
    target.x = vector.x * x;
    target.y = vector.y * y;
    target.z = vector.z * z;
    return target;
  }

  /// Scale a vector and add it to this vector. Save the result in "target". (target = this + vector * scalar)
  /// @param target The vector to save the result in.
  Vec3 addScaledVector(double scalar, Vec3 vector, [Vec3? target]) {
    target ??= Vec3();
    target.x = x + scalar * vector.x;
    target.y = y + scalar * vector.y;
    target.z = z + scalar * vector.z;
    return target;
  }

  /// Calculate dot product
  /// @param vector
  double dot(Vec3 vector) {
    return x * vector.x + y * vector.y + z * vector.z;
  }

  bool isZero() {
    return x == 0 && y == 0 && z == 0;
  }

  /// Make the vector point in the opposite direction.
  /// @param target Optional target to save in
  Vec3 negate([Vec3? target]) {
    target ??= Vec3();
    target.x = -x;
    target.y = -y;
    target.z = -z;
    return target;
  }

  /// Compute two artificial tangents to the vector
  /// @param t1 Vector object to save the first tangent in
  /// @param t2 Vector object to save the second tangent in
  void tangents(Vec3 t1, Vec3 t2){
    final norm = length();
    if (norm > 0.0) {
      final n = Vec3_tangents_n;
      final inorm = 1 / norm;
      n.set(x * inorm, y * inorm, z * inorm);
      final randVec = Vec3_tangents_randVec;
      if (n.x.abs() < 0.9) {
        randVec.set(1, 0, 0);
        n.cross(randVec, t1);
      } else {
        randVec.set(0, 1, 0);
        n.cross(randVec, t1);
      }
      n.cross(t1, t2);
    } else {
      // The normal length is zero, make something up
      t1.set(1, 0, 0);
      t2.set(0, 1, 0);
    }
  }

  /// Converts to a more readable format
  @override
  String toString(){
    return '$x,$y,$z';
  }

  /// Converts to an array
  List<double> toArray(){
    return [x, y, z];
  }

  /// Copies value of source to this vector.
  Vec3 copy(Vec3 vector) {
    x = vector.x;
    y = vector.y;
    z = vector.z;
    return this;
  }

  /// Do a linear interpolation between two vectors
  /// @param t A number between 0 and 1. 0 will make this function return u, and 1 will make it return v. Numbers in between will generate a vector in between them.
  void lerp(Vec3 vector,double t, Vec3 target) {
    final x = this.x;
    final y = this.y;
    final z = this.z;
    target.x = x + (vector.x - x) * t;
    target.y = y + (vector.y - y) * t;
    target.z = z + (vector.z - z) * t;
  }

  /// Check if a vector equals is almost equal to another one.
  bool almostEquals(Vec3 vector,[double? precision]) {
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
  
  /// Check if the vector is anti-parallel to another vector.
  /// @param precision Set to zero for exact comparisons
  bool isAntiparallelTo(Vec3 vector, [double? precision]) {
    negate(antip_neg);
    return antip_neg.almostEquals(vector, precision);
  }

  /// Clone the vector
  Vec3 clone() {
    return Vec3(x, y, z);
  }

  Quaternion toQuant(){
    return Quaternion(x,y,z);
  }
}
