import 'quaternion.dart';
import  'mat3.dart';
import 'dart:math' as math;

final vec3TangentsN = Vec3();
final vec3TangentsRandVec = Vec3();
final antipNeg = Vec3();

/// 3-dimensional vector
/// @example
///     final v = new Vec3(1, 2, 3)
///     console.log('x=' + v.x) // x=1
class Vec3 {
  double x;
  double y;
  double z;



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
  Vec3 crossVectors(Vec3 a, Vec3 b) {
    final ax = a.x, ay = a.y, az = a.z;
    final bx = b.x, by = b.y, bz = b.z;

    x = ay * bz - az * by;
    y = az * bx - ax * bz;
    z = ax * by - ay * bx;

    return this;
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
  Vec3 vadd(Vec3 vector, [Vec3? target]){
    if (target != null) {
      target.x = vector.x + x;
      target.y = vector.y + y;
      target.z = vector.z + z;
      return target;
    } else {
      return Vec3(x + vector.x, y + vector.y, z + vector.z);
    }
  }

  /// Vector subtraction
  ///  @param target Optional target to save in.
  Vec3 vsub(Vec3 vector, [Vec3? target]){
    if (target != null) {
      target.x =  x-vector.x;
      target.y =  y-vector.y;
      target.z =  z-vector.z;
      return target;
    } else {
      return Vec3(x - vector.x, y - vector.y, z - vector.z);
    }
  }
  Vec3 subVectors(Vec3 a, Vec3 b) {
    x = a.x - b.x;
    y = a.y - b.y;
    z = a.z - b.z;

    return this;
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



  /// Get the length of the vector
  double length() {
    return math.sqrt(x * x + y * y + z * z);
  }

  /// Get the squared length of the vector.
  double lengthSquared() {
    return dot(this);
  }



  /// Multiply all the components of the vector with a scalar.
  /// @param target The vector to save the result in.
  Vec3 scale(double scalar, [Vec3? target]){
    target ??= Vec3();
    target.x = scalar * x;
    target.y = scalar * y;
    target.z = scalar * z;
    return target;
  }
  int sum(){
    return (x+y+z).toInt();
  }




  /// Calculate dot product
  /// @param vector
  double dot(Vec3 vector) {
    return x * vector.x + y * vector.y + z * vector.z;
  }



  /// Make the vector point in the opposite direction.
  /// @param target Optional target to save in
  Vec3 negate(Vec3 target) {
    target.x = -x;
    target.y = -y;
    target.z = -z;
    return target;
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




  
  /// Check if the vector is anti-parallel to another vector.
  /// @param precision Set to zero for exact comparisons
  bool isAntiparallelTo(Vec3 vector, [double? precision]) {
    negate(antipNeg);
    return antipNeg.almostEquals(vector, precision);
  }

  /// Clone the vector
  Vec3 clone() {
    return Vec3(x, y, z);
  }

  Quaternion toQuant(){
    return Quaternion(x,y,z);
  }
}
