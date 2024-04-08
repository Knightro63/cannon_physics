import 'dart:math' as math;
import 'equation_class.dart';
import '../math/vec3.dart';
import '../objects/rigid_body.dart';
import 'package:vector_math/vector_math.dart';

/// Rotational constraint. Works to keep the local vectors orthogonal to each other in world space.
class RotationalEquation extends Equation {
  /// World oriented rotational axis.
  late Vector3 axisA;
  /// World oriented rotational axis.
  late Vector3 axisB;
  /// maxAngle
  double maxAngle;

  RotationalEquation(
    Body bodyA,
    Body bodyB,
    {
      Vector3? axisA,
      Vector3? axisB,
      this.maxAngle = math.pi/2,
      double maxForce = 1e6
    }
  ):super(bodyA, bodyB, -maxForce, maxForce) {
    this.axisA = axisA?.clone() ?? Vector3(1, 0, 0);
    this.axisB = axisB?.clone() ?? Vector3(0, 1, 0);
  }

  final _tmpVec1 = Vector3.zero();
  final _tmpVec2 = Vector3.zero();

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
    ni.cross2(nj, nixnj);
    nj.cross2(ni, njxni);

    ga.rotational.setFrom(njxni);
    gb.rotational.setFrom(nixnj);

    final g = math.cos(maxAngle) - ni.dot(nj);
    final gW = computeGW();
    final giMf = computeGiMf();

    final B = -g * a - gW * b - h * giMf;

    return B;
  }
}
