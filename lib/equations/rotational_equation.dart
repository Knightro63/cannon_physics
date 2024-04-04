import 'dart:math' as math;
import 'equation_class.dart';
import '../math/vec3.dart';
import '../objects/rigid_body.dart';

/// Rotational constraint. Works to keep the local vectors orthogonal to each other in world space.
class RotationalEquation extends Equation {
  /// World oriented rotational axis.
  late Vec3 axisA;
  /// World oriented rotational axis.
  late Vec3 axisB;
  /// maxAngle
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
    this.axisA = axisA?.clone() ?? Vec3(1, 0, 0);
    this.axisB = axisB?.clone() ?? Vec3(0, 1, 0);
  }

  final _tmpVec1 = Vec3();
  final _tmpVec2 = Vec3();

  @override
  double computeB(double h) {
    final a = this.a;
    final b = this.b;
    final ni = axisA;
    final nj = axisB;
    final nixnj = _tmpVec1;
    final njxni = _tmpVec2;
    final ga = jacobianElementA;
    final gb = jacobianElementB;

    // Caluclate cross products
    ni.cross(nj, nixnj);
    nj.cross(ni, njxni);

    ga.rotational.copy(njxni);
    gb.rotational.copy(nixnj);

    final g = math.cos(maxAngle) - ni.dot(nj);
    final gW = computeGW();
    final giMf = computeGiMf();

    final B = -g * a - gW * b - h * giMf;

    return B;
  }
}
