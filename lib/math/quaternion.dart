import 'dart:math' as math;
import 'package:vector_math/vector_math.dart';

enum Order{xyz,yxz,zxy,zyx,yzx,xzy}

class AxisAngle{
  AxisAngle(this.axis,this.angle);

  num angle;
  Vector3 axis;
}

/// A Quaternion describes a rotation in 3D space. The Quaternion is mathematically defined as Q = x*i + y*j + z*k + w, where (i,j,k) are imaginary basis vectors. (x,y,z) can be seen as a vector related to the axis of rotation, while the real multiplier, w, is related to the amount of rotation.
/// @param x Multiplier of the imaginary basis vector i.
/// @param y Multiplier of the imaginary basis vector j.
/// @param z Multiplier of the imaginary basis vector k.
/// @param w Multiplier of the real part.
/// @see http://en.wikipedia.org/wiki/Quaternion
extension Quat on Quaternion {
    /// Multiply the quaternion by a vector
  Vector3 vmult(Vector3 v, [Vector3? target]){
    target ??= Vector3.zero();
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
  /// Multiply the quaternion with an other quaternion.
  Quaternion multiply(Quaternion quat){
    Quaternion target = Quaternion(0,0,0,1);
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
  /// Multiply the quaternion with an other quaternion.
  Quaternion multiply2(Quaternion quat, Quaternion target){
    //target ??= Quaternion(0,0,0,1);
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
  /// Set the quaternion components given an axis and an angle in radians.
  Quaternion setFromAxisAngle(Vector3 vector, double angle){
    final s = math.sin(angle * 0.5);
    x = vector.x * s;
    y = vector.y * s;
    z = vector.z * s;
    w = math.cos(angle * 0.5);
    return this;
  }
  /// Rotate an absolute orientation quaternion given an angular velocity and a time step.
  Quaternion integrate(Vector3 angularVelocity,double dt,Vector3 angularFactor, [Quaternion? target]){
    target ??= Quaternion(0,0,0,1);
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
  /// Performs a spherical linear interpolation between two quat
  ///
  /// @param toQuat second operand
  /// @param t interpolation amount between the self quaternion and toQuat
  /// @param target A quaternion to store the result in. If not provided, a new one will be created.
  /// @returns {Quaternion} The "target" object
  Quaternion slerp(Quaternion toQuat,double t, [Quaternion? target]){
    target ??= Quaternion(0,0,0,1);
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
}