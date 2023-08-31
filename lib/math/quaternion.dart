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

  /// Set the quaternion components given an axis and an angle in radians.
  Quaternion setFromAxisAngle(Vec3 vector, double angle){
    final s = math.sin(angle * 0.5);
    x = vector.x * s;
    y = vector.y * s;
    z = vector.z * s;
    w = math.cos(angle * 0.5);
    return this;
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

  /// Multiply the quaternion with an other quaternion.
  Quaternion mult(Quaternion quat, [Quaternion? target]){
    target ??= Quaternion();
    final ax = x;
    final ay = y;
    final az = z;
    final aw = w;
    final bx = quat.x;
    final by = quat.y;
    final bz = quat.z;
    final bw = quat.w;

    target.x = ax * bw + aw * bx + ay * bz - az * by;
    target.y = ay * bw + aw * by + az * bx - ax * bz;
    target.z = az * bw + aw * bz + ax * by - ay * bx;
    target.w = aw * bw - ax * bx - ay * by - az * bz;

    return target;
  }

  /// Get the inverse quaternion rotation.
  Quaternion inverse([Quaternion? target]){
    target ??= Quaternion();
    final x = this.x;
    final y = this.y;
    final z = this.z;
    final w = this.w;

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

  /// Approximation of quaternion normalization. Works best when quat is already almost-normalized.
  /// @author unphased, https://github.com/unphased
  Quaternion normalizeFast(){
    final f = (3.0 - (x * x + y * y + z * z + w * w)) / 2.0;
    if (f == 0) {
      x = 0;
      y = 0;
      z = 0;
      w = 0;
    } else {
      x *= f;
      y *= f;
      z *= f;
      w *= f;
    }
    return this;
  }

  /// Multiply the quaternion by a vector
  Vec3 vmult(Vec3 v, [Vec3? target]){
    target ??= Vec3();
    final x = v.x;
    final y = v.y;
    final z = v.z;
    final qx = this.x;
    final qy = this.y;
    final qz = this.z;
    final qw = w;

    // q*v
    final ix = qw * x + qy * z - qz * y;

    final iy = qw * y + qz * x - qx * z;
    final iz = qw * z + qx * y - qy * x;
    final iw = -qx * x - qy * y - qz * z;

    target.x = ix * qw + iw * -qx + iy * -qz - iz * -qy;
    target.y = iy * qw + iw * -qy + iz * -qx - ix * -qz;
    target.z = iz * qw + iw * -qz + ix * -qy - iy * -qx;

    return target;
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

    final x = this.x;
    final y = this.y;
    final z = this.z;
    final w = this.w;

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

  /// Set the quaternion components given Euler angle representation.
  ///
  /// @param order The order to apply angles: 'XYZ' or 'YXZ' or any other combination.
  ///
  /// See {@link https://www.mathworks.com/matlabcentral/fileexchange/20696-function-to-convert-between-dcm-euler-angles-quaternions-and-euler-vectors mathWorks} reference
  Quaternion setFromEuler(double x,double y,double z, [Order order = Order.xyz]){
    final c1 = math.cos(x / 2);
    final c2 = math.cos(y / 2);
    final c3 = math.cos(z / 2);
    final s1 = math.sin(x / 2);
    final s2 = math.sin(y / 2);
    final s3 = math.sin(z / 2);

    if (order == Order.xyz) {
      this.x = s1 * c2 * c3 + c1 * s2 * s3;
      this.y = c1 * s2 * c3 - s1 * c2 * s3;
      this.z = c1 * c2 * s3 + s1 * s2 * c3;
      w = c1 * c2 * c3 - s1 * s2 * s3;
    } else if (order == Order.yxz) {
      this.x = s1 * c2 * c3 + c1 * s2 * s3;
      this.y = c1 * s2 * c3 - s1 * c2 * s3;
      this.z = c1 * c2 * s3 - s1 * s2 * c3;
      w = c1 * c2 * c3 + s1 * s2 * s3;
    } else if (order == Order.zxy) {
      this.x = s1 * c2 * c3 - c1 * s2 * s3;
      this.y = c1 * s2 * c3 + s1 * c2 * s3;
      this.z = c1 * c2 * s3 + s1 * s2 * c3;
      w = c1 * c2 * c3 - s1 * s2 * s3;
    } else if (order == Order.zyx) {
      this.x = s1 * c2 * c3 - c1 * s2 * s3;
      this.y = c1 * s2 * c3 + s1 * c2 * s3;
      this.z = c1 * c2 * s3 - s1 * s2 * c3;
      w = c1 * c2 * c3 + s1 * s2 * s3;
    } else if (order == Order.yzx) {
      this.x = s1 * c2 * c3 + c1 * s2 * s3;
      this.y = c1 * s2 * c3 + s1 * c2 * s3;
      this.z = c1 * c2 * s3 - s1 * s2 * c3;
      w = c1 * c2 * c3 - s1 * s2 * s3;
    } else if (order == Order.xzy) {
      this.x = s1 * c2 * c3 - c1 * s2 * s3;
      this.y = c1 * s2 * c3 - s1 * c2 * s3;
      this.z = c1 * c2 * s3 + s1 * s2 * c3;
      w = c1 * c2 * c3 + s1 * s2 * s3;
    }

    return this;
  }

  Quaternion clone(){
    return Quaternion(x, y, z, w);
  }

  /// Performs a spherical linear interpolation between two quat
  ///
  /// @param toQuat second operand
  /// @param t interpolation amount between the self quaternion and toQuat
  /// @param target A quaternion to store the result in. If not provided, a new one will be created.
  /// @returns {Quaternion} The "target" object
  Quaternion slerp(Quaternion toQuat,double t, [Quaternion? target]){
    target ??= Quaternion();
    final ax = x;
    final ay = y;
    final az = z;
    final aw = w;

    double bx = toQuat.x;
    double by = toQuat.y;
    double bz = toQuat.z;
    double bw = toQuat.w;
    double omega;
    double cosom;
    double sinom;
    double scale0;
    double scale1;

    // calc cosine
    cosom = ax * bx + ay * by + az * bz + aw * bw;

    // adjust signs (if necessary)
    if (cosom < 0.0) {
      cosom = -cosom;
      bx = -bx;
      by = -by;
      bz = -bz;
      bw = -bw;
    }

    // calculate coefficients
    if (1.0 - cosom > 0.000001) {
      // standard case (slerp)
      omega = math.acos(cosom);
      sinom = math.sin(omega);
      scale0 = math.sin((1.0 - t) * omega) / sinom;
      scale1 = math.sin(t * omega) / sinom;
    } else {
      // "from" and "to" quaternions are very close
      //  ... so we can do a linear interpolation
      scale0 = 1.0 - t;
      scale1 = t;
    }

    // calculate final values
    target.x = scale0 * ax + scale1 * bx;
    target.y = scale0 * ay + scale1 * by;
    target.z = scale0 * az + scale1 * bz;
    target.w = scale0 * aw + scale1 * bw;

    return target;
  }

  /// Rotate an absolute orientation quaternion given an angular velocity and a time step.
  Quaternion integrate(Vec3 angularVelocity,double dt,Vec3 angularFactor, [Quaternion? target]){
    target ??= Quaternion();
    final ax = angularVelocity.x * angularFactor.x,
      ay = angularVelocity.y * angularFactor.y,
      az = angularVelocity.z * angularFactor.z,
      bx = x,
      by = y,
      bz = z,
      bw = w;

    final halfDt = dt * 0.5;

    target.x += halfDt * (ax * bw + ay * bz - az * by);
    target.y += halfDt * (ay * bw + az * bx - ax * bz);
    target.z += halfDt * (az * bw + ax * by - ay * bx);
    target.w += halfDt * (-ax * bx - ay * by - az * bz);

    return target;
  }
}
