import 'dart:math' as math;
import '../math/vec3.dart';
import 'equation_class.dart';
import '../objects/rigid_body.dart';
import 'package:vector_math/vector_math.dart';

/// Cone equation. Works to keep the given body world vectors aligned, or tilted within a given angle from each other.
class ConeEquation extends Equation {
  /// Local axis in A
  late Vector3 axisA;
  /// Local axis in B
  late Vector3 axisB;
  /// The "cone angle" to keep
  double angle;

  ConeEquation(
    Body bodyA,
    Body bodyB,
    {
      Vector3? axisA,
      Vector3? axisB,
      this.angle = 0,
      double maxForce = 1e6,
    }
  ):super(bodyA,bodyB,-maxForce,maxForce) {
    this.axisA = axisA?.clone() ?? Vector3(1, 0, 0);
    this.axisB = axisB?.clone() ?? Vector3(0, 1, 0);
  }

  final _tmpVec1 = Vector3.zero();
  final _tmpVec2 = Vector3.zero();

  @override
  double computeB(double h){
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

    final g = math.cos(angle) - ni.dot(nj);
    final gw = computeGW();
    final giMf = computeGiMf();
    final B = -g * a - gw * b - h * giMf;
    return B;
  }
}
