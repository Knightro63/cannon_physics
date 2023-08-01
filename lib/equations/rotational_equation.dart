import 'dart:math' as math;
import '../equations/equation.dart';
import '../math/vec3.dart';
import '../objects/body.dart';

/**
 * Rotational constraint. Works to keep the local vectors orthogonal to each other in world space.
 */
class RotationalEquation extends Equation {
  /**
   * World oriented rotational axis.
   */
  late Vec3 axisA;
  /**
   * World oriented rotational axis.
   */
  late Vec3 axisB;
  /**
   * maxAngle
   */
  double maxAngle;

  RotationalEquation(
    Body bodyA,
    Body bodyB,
    {
      Vec3? axisA,
      Vec3? axisB,
      this.maxAngle = math.pi/2,
      double maxForce = 1e6
    }
  ):super(bodyA, bodyB, -maxForce, maxForce) {
    this.axisA = axisA ?? Vec3(1, 0, 0);
    this.axisB = axisB ?? Vec3(0, 1, 0);
  }

  @override
  double computeB(double h) {
    final a = this.a;
    final b = this.b;
    final ni = axisA;
    final nj = axisB;
    final nixnj = tmpVec1;
    final njxni = tmpVec2;
    final GA = jacobianElementA;
    final GB = jacobianElementB;

    // Caluclate cross products
    ni.cross(nj, nixnj);
    nj.cross(ni, njxni);

    // g = ni * nj
    // gdot = (nj x ni) * wi + (ni x nj) * wj
    // G = [0 njxni 0 nixnj]
    // W = [vi wi vj wj]
    GA.rotational.copy(njxni);
    GB.rotational.copy(nixnj);

    final g = math.cos(maxAngle) - ni.dot(nj);
    final GW = computeGW();
    final GiMf = computeGiMf();

    final B = -g * a - GW * b - h * GiMf;

    return B;
  }
}

final tmpVec1 = Vec3();
final tmpVec2 = Vec3();
