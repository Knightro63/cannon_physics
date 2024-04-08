import 'vec3.dart';
import 'dart:math' as math;

enum Order{xyz,yxz,zxy,zyx,yzx,xzy}

class AxisAngle{
  AxisAngle(this.axis,this.angle);

  num angle;
  Vec3 axis;
}

/// A Quaternion describes a rotation in 3D space. The Quaternion is mathematically defined as Q = x*i + y*j + z*k + w, where (i,j,k) are imaginary basis vectors. (x,y,z) can be seen as a vector related to the axis of rotation, while the real multiplier, w, is related to the amount of rotation.
/// @param x Multiplier of the imaginary basis vector i.
/// @param y Multiplier of the imaginary basis vector j.
/// @param z Multiplier of the imaginary basis vector k.
/// @param w Multiplier of the real part.
/// @see http://en.wikipedia.org/wiki/Quaternion
class Quaternion {
  double x;
  double y;
  double z;
  double w;

  Quaternion([this.x = 0, this.y = 0, this.z = 0, this.w = 1]);

  final _sfvT1 = Vec3();
  final _sfvT2 = Vec3();

  /// Set the value of the quaternion.
  Quaternion set(double x, double y, double z, double w){
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
    return this;
  }

  /// Convert to a readable format
  /// @return "x,y,z,w"
  @override
  String toString() {
    return '$x,$y,$z,$w';
  }

  /// Convert to an Array
  /// @return [x, y, z, w]
  List<double> toArray(){
    return [x,y,z,w];
  }



  ///Converts the quaternion to [ axis, angle ] representation.
  /// @param targetAxis A vector object to reuse for storing the axis.
  /// @return An array, first element is the axis and the second is the angle in radians.
  AxisAngle toAxisAngle(Vec3? targetAxis){
    targetAxis ??= Vec3();
    normalize(); // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
    final angle = 2 * math.acos(w);
    final s = math.sqrt(1 - w * w); // assuming quaternion normalised then w is less than 1, so term always positive.
    if (s < 0.001) {
      // test to avoid divide by zero, s is always positive due to sqrt
      // if s close to zero then direction of axis not important
      targetAxis.x = x; // if it is important that axis is normalised then replace with x=1; y=z=0;
      targetAxis.y = y;
      targetAxis.z = z;
    } else {
      targetAxis.x = x / s; // normalise axis
      targetAxis.y = y / s;
      targetAxis.z = z / s;
    }
    return AxisAngle(targetAxis, angle);
  }

  /// Set the quaternion value given two vectors. The resulting rotation will be the needed rotation to rotate u to v.
  Quaternion setFromVectors(Vec3 u, Vec3 v){
    if (u.isAntiparallelTo(v)) {
      final t1 = _sfvT1;
      final t2 = _sfvT2;

      u.tangents(t1, t2);
      setFromAxisAngle(t1, math.pi);
    } else {
      final a = u.cross(v);
      x = a.x;
      y = a.y;
      z = a.z;
      w = math.sqrt(math.pow(u.length(),2) * math.pow(v.length(),2)) + u.dot(v);
      normalize();
    }
    return this;
  }



  /// Get the inverse quaternion rotation.
  Quaternion inverse([Quaternion? target]){
    target ??= Quaternion();
    conjugate(target);
    final inorm2 = 1 / (x * x + y * y + z * z + w * w);
    target.x *= inorm2;
    target.y *= inorm2;
    target.z *= inorm2;
    target.w *= inorm2;

    return target;
  }

  /// Get the quaternion conjugate
  Quaternion conjugate([Quaternion? target]){
    target ??= Quaternion();
    target.x = -x;
    target.y = -y;
    target.z = -z;
    target.w = w;

    return target;
  }

  /// Normalize the quaternion. Note that this changes the values of the quaternion.
  Quaternion normalize(){
    double l = math.sqrt(x * x + y *y + z *z + w * w);
    if (l == 0) {
      x = 0;
      y = 0;
      z = 0;
      w = 0;
    } else {
      l = 1 / l;
      x *= l;
      y *= l;
      z *= l;
      w *= l;
    }
    return this;
  }





  /// Copies value of source to this quaternion.
  /// @return this
  Quaternion copy(Quaternion quat){
    x = quat.x;
    y = quat.y;
    z = quat.z;
    w = quat.w;
    return this;
  }

  /// Convert the quaternion to euler angle representation. Order: YZX, as this page describes: https://www.euclideanspace.com/maths/standards/index.htm
  /// @param order Three-character string, defaults to "YZX"
  void toEuler(Vec3 target, [Order order = Order.xyz]) {
    double? heading;
    late double attitude;
    late double bank;

    switch (order) {
      case Order.xyz:
        final test = x * y + z * w;
        if (test > 0.499) {
          // singularity at north pole
          heading = 2 * math.atan2(x, w);
          attitude = math.pi / 2;
          bank = 0;
        }
        if (test < -0.499) {
          // singularity at south pole
          heading = -2 * math.atan2(x, w);
          attitude = -math.pi / 2;
          bank = 0;
        }
        if (heading == null) {
          final sqx = x * x;
          final sqy = y * y;
          final sqz = z * z;
          heading = math.atan2(2 * y * w - 2 * x * z, 1 - 2 * sqy - 2 * sqz); // Heading
          attitude = math.asin(2 * test); // attitude
          bank = math.atan2(2 * x * w - 2 * y * z, 1 - 2 * sqx - 2 * sqz); // bank
        }
        break;
      default:
        throw('Euler order $order not supported yet.');
    }

    target.y = heading;
    target.z = attitude;
    target.x = bank;
  }

  Quaternion clone(){
    return Quaternion(x, y, z, w);
  }
}
