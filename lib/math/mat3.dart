import 'package:vector_math/vector_math.dart';

/// A 3x3 matrix.
/// Authored by {@link http://github.com/schteppe/ schteppe}
extension Mat3 on Matrix3{
  /// Matrix-Vector multiplication
  /// @param v The vector to multiply with
  /// @param target Optional, target to save the result in.
  Vector3 vmult(Vector3 v, Vector3? target){
    target ??= Vector3.zero();
    final e = storage;
    final x = v.x;
    final y = v.y;
    final z = v.z;
    target.x = e[0] * x + e[1] * y + e[2] * z;
    target.y = e[3] * x + e[4] * y + e[5] * z;
    target.z = e[6] * x + e[7] * y + e[8] * z;

    return target;
  }
  /// Set the matrix from a quaterion
  Matrix3 setRotationFromQuaternion(Quaternion q){
    final x = q.x;
    final y = q.y;
    final z = q.z;
    final w = q.w;
    final x2 = x + x;
    final y2 = y + y;
    final z2 = z + z;
    final xx = x * x2;
    final xy = x * y2;
    final xz = x * z2;
    final yy = y * y2;
    final yz = y * z2;
    final zz = z * z2;
    final wx = w * x2;
    final wy = w * y2;
    final wz = w * z2;
    List<double> e = storage;

    e[3 * 0 + 0] = 1 - (yy + zz);
    e[3 * 0 + 1] = xy - wz;
    e[3 * 0 + 2] = xz + wy;

    e[3 * 1 + 0] = xy + wz;
    e[3 * 1 + 1] = 1 - (xx + zz);
    e[3 * 1 + 2] = yz - wx;

    e[3 * 2 + 0] = xz - wy;
    e[3 * 2 + 1] = yz + wx;
    e[3 * 2 + 2] = 1 - (xx + yy);

    return this;
  }
  /// Matrix multiplication
  /// @param matrix Matrix to multiply with from left side.
  Matrix3 multiply2(Matrix3 matrix, [Matrix3? target]) {
    target ??= Matrix3.zero();
    final A = storage;
    final B = matrix.storage;
    final T = target.storage;

    final a11 = A[0],
      a12 = A[1],
      a13 = A[2],
      a21 = A[3],
      a22 = A[4],
      a23 = A[5],
      a31 = A[6],
      a32 = A[7],
      a33 = A[8];

    final b11 = B[0],
      b12 = B[1],
      b13 = B[2],
      b21 = B[3],
      b22 = B[4],
      b23 = B[5],
      b31 = B[6],
      b32 = B[7],
      b33 = B[8];

    T[0] = a11 * b11 + a12 * b21 + a13 * b31;
    T[1] = a11 * b12 + a12 * b22 + a13 * b32;
    T[2] = a11 * b13 + a12 * b23 + a13 * b33;

    T[3] = a21 * b11 + a22 * b21 + a23 * b31;
    T[4] = a21 * b12 + a22 * b22 + a23 * b32;
    T[5] = a21 * b13 + a22 * b23 + a23 * b33;

    T[6] = a31 * b11 + a32 * b21 + a33 * b31;
    T[7] = a31 * b12 + a32 * b22 + a33 * b32;
    T[8] = a31 * b13 + a32 * b23 + a33 * b33;

    return target;
  }

  /// Scale each column of the matrix
  Matrix3 vscale(Vector3 vector,[Matrix3? target]) {
    target ??= Matrix3.zero();
    final e = storage;
    final t = target.storage;
    for (int i = 0; i != 3; i++) {
      t[3 * i + 0] = vector.x * e[3 * i + 0];
      t[3 * i + 1] = vector.y * e[3 * i + 1];
      t[3 * i + 2] = vector.z * e[3 * i + 2];
    }
    return target;
  }
}