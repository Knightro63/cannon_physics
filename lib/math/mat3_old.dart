import 'vec3.dart';
import 'quaternion.dart';

/// A 3x3 matrix.
/// Authored by {@link http://github.com/schteppe/ schteppe}
class Mat3 {
  Mat3([List<double>? elements]){
    this.elements = elements ?? [0, 0, 0, 0, 0, 0, 0, 0, 0];
  }

  final _reverseEqns = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
  /// A vector of length 9, containing all matrix elements.
  late List<double> elements;

  /// Sets the matrix to identity
  /// @todo Should perhaps be renamed to `setIdentity()` to be more clear.
  /// @todo Create another function that immediately creates an identity matrix eg. `eye()`
  void identity(){
    final e = elements;
    e[0] = 1;
    e[1] = 0;
    e[2] = 0;

    e[3] = 0;
    e[4] = 1;
    e[5] = 0;

    e[6] = 0;
    e[7] = 0;
    e[8] = 1;
  }

  /// et all elements to zero
  void setZero(){
    final e = elements;
    e[0] = 0;
    e[1] = 0;
    e[2] = 0;
    e[3] = 0;
    e[4] = 0;
    e[5] = 0;
    e[6] = 0;
    e[7] = 0;
    e[8] = 0;
  }

  /// Sets the matrix diagonal elements from a Vec3
  void setTrace(Vec3 vector){
    final e = elements;
    e[0] = vector.x;
    e[4] = vector.y;
    e[8] = vector.z;
  }

  /// Gets the matrix diagonal elements
  Vec3 getTrace([Vec3? target]) {
    target ??= Vec3();
    final e = elements;
    target.x = e[0];
    target.y = e[4];
    target.z = e[8];
    return target;
  }

  /// Matrix-scalar multiplication
  void smult(double s) {
    for (int i = 0; i < elements.length; i++) {
      elements[i] *= s;
    }
  }



  /// Solve Ax=b
  /// @param b The right hand side
  /// @param target Optional. Target vector to save in.
  /// @return The solution x
  /// @todo should reuse arrays
  Vec3 solve(Vec3 b,[ Vec3? target]) {
    target ??= Vec3();
    // finalruct equations
    const nr = 3; // num rows
    const nc = 4; // num cols
    List<double> eqns = [];
    int i;
    int j;
    for (i = 0; i < nr * nc; i++) {
      eqns.add(0);
    }
    for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
        eqns[i + nc * j] = elements[i + 3 * j];
      }
    }
    eqns[3 + 4 * 0] = b.x;
    eqns[3 + 4 * 1] = b.y;
    eqns[3 + 4 * 2] = b.z;

    // Compute right upper triangular version of the matrix - Gauss elimination
    int n = 3;

    final k = n;
    int np;
    const kp = 4; // num rows
    int p;
    for(;n>0;n--) {
      i = k - n;
      if (eqns[i + nc * i] == 0) {
        // the pivot is null, swap lines
        for (j = i + 1; j < k; j++) {
          if (eqns[i + nc * j] != 0) {
            for(np = kp;np > 0 ;np--){
              // do ligne( i ) = ligne( i ) + ligne( k )
              p = kp - np;
              eqns[p + nc * i] += eqns[p + nc * j];
            }
            //break;
          }
        }
      }
      if (eqns[i + nc * i] != 0) {
        for (j = i + 1; j < k; j++) {
          final num multiplier = eqns[i + nc * j] / eqns[i + nc * i];
          for (np = kp; np > 0; --np) {
            // do ligne( k ) = ligne( k ) - multiplier * ligne( i )
            p = kp - np;
            eqns[p + nc * j] = p <= i ? 0 : eqns[p + nc * j] - eqns[p + nc * i] * multiplier;
          } 
        }
      }
    } 

    // Get the solution
    target.z = eqns[2 * nc + 3] / eqns[2 * nc + 2];
    target.y = (eqns[1 * nc + 3] - eqns[1 * nc + 2] * target.z) / eqns[1 * nc + 1];
    target.x = (eqns[0 * nc + 3] - eqns[0 * nc + 2] * target.z - eqns[0 * nc + 1] * target.y) / eqns[0 * nc + 0];

    if (
      target.x.isNaN ||
      target.y.isNaN ||
      target.z.isNaN ||
      target.x == double.infinity ||
      target.y == double.infinity ||
      target.z == double.infinity
    ) {
      throw('Could not solve equation! Got x=[${target.toString()}], b=[${b.toString()}], A=[${toString()}]');
    }

    return target;
  }

  /// Get an element in the matrix by index. Index starts at 0, not 1!!!
  /// @param value If provided, the matrix element will be set to this value.
  double? e(int row, int column, [double? value]){
    if (value == null) {
      return elements[column + 3 * row];
    } else {
      // Set value
      elements[column + 3 * row] = value;
      return null;
    }
  }

  /// Copy another matrix into this matrix object.
  Mat3 copy(Mat3 matrix) {
    for (int i = 0; i < matrix.elements.length; i++) {
      elements[i] = matrix.elements[i];
    }
    return this;
  }

  /// Returns a string representation of the matrix.
  @override
  String toString() {
    String r = '';
    const sep = ',';
    for (int i = 0; i < 9; i++) {
      r += '${elements[i]}$sep';
    }
    return r;
  }

  /// reverse the matrix
  /// @param target Target matrix to save in.
  /// @return The solution x
  Mat3 reverse([Mat3? target]) {
    target ??= Mat3();
    // finalruct equations
    const nr = 3; // num rows
    const nc = 6; // num cols
    List<num> eqns = _reverseEqns;
    int i;
    int j;
    for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
        eqns[i + nc * j] = elements[i + 3 * j];
      }
    }
    eqns[3 + 6 * 0] = 1;
    eqns[3 + 6 * 1] = 0;
    eqns[3 + 6 * 2] = 0;
    eqns[4 + 6 * 0] = 0;
    eqns[4 + 6 * 1] = 1;
    eqns[4 + 6 * 2] = 0;
    eqns[5 + 6 * 0] = 0;
    eqns[5 + 6 * 1] = 0;
    eqns[5 + 6 * 2] = 1;

    // Compute right upper triangular version of the matrix - Gauss elimination
    int n = 3;

    final k = n;
    int np;
    const kp = nc; // num rows
    num p;
    for (; n > 0; n--) {
      i = k - n;
      if (eqns[i + nc * i] == 0) {
        // the pivot is null, swap lines
        for (j = i + 1; j < k; j++) {
          if (eqns[i + nc * j] != 0) {
            for (np = kp;np > 0;--np) {
              // do line( i ) = line( i ) + line( k )
              p = kp - np;
              eqns[p.toInt() + nc * i] += eqns[p.toInt() + nc * j];
            } 
            break;
          }
        }
      }
      if (eqns[i + nc * i] != 0) {
        for (j = i + 1; j < k; j++) {
          final multiplier = eqns[i + nc * j] / eqns[i + nc * i];
          for (np = kp;np > 0;np--) {
            // do line( k ) = line( k ) - multiplier * line( i )
            p = kp - np;
            eqns[p.toInt() + nc * j] = p <= i ? 0 : eqns[p.toInt() + nc * j] - eqns[p.toInt() + nc * i] * multiplier;
          } 
        }
      }
    } 

    // eliminate the upper left triangle of the matrix
    for (i = 2; i > 0;i--) {
      for (j = i-1;j > 0;j--) {
        final multiplier = eqns[i + nc * j] / eqns[i + nc * i];
        for(np = nc;np > 0;np--){
          p = nc - np;
          eqns[p.toInt() + nc * j] = eqns[p.toInt() + nc * j] - eqns[p.toInt() + nc * i] * multiplier;
        } 
      } 
    } 

    // operations on the diagonal
    for (i=2;i >0;i--) {
      final multiplier = 1 / eqns[i + nc * i];
      for (np = nc;np>0;np--) {
        p = nc - np;
        eqns[p.toInt() + nc * i] = eqns[p.toInt() + nc * i] * multiplier;
      } 
    } 
    
    for (i = 2;i >0;i--) {
      for (j= 2; j > 0;j--) {
        p = eqns[nr + j + nc * i];
        if (p.isNaN || p == double.infinity) {
          throw ('Could not reverse! A=[${toString()}]');
        }
        target.e(i, j, p.toDouble());
      } 
    } 

    return target;
  }



  /// Transpose the matrix
  /// @param target Optional. Where to store the result.
  /// @return The target Mat3, or a new Mat3 if target was omitted.
  Mat3 transpose([Mat3? target]) {
    target ??= Mat3();
    final M = elements;
    final T = target.elements;
    double tmp;

    //Set diagonals
    T[0] = M[0];
    T[4] = M[4];
    T[8] = M[8];

    tmp = M[1];
    T[1] = M[3];
    T[3] = tmp;

    tmp = M[2];
    T[2] = M[6];
    T[6] = tmp;

    tmp = M[5];
    T[5] = M[7];
    T[7] = tmp;

    return target;
  }
}
