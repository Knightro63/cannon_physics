import 'dart:math' as math;
import '../math/vec3.dart';
import '../equations/equation.dart';
import '../objects/body.dart';

/// Cone equation. Works to keep the given body world vectors aligned, or tilted within a given angle from each other.
class ConeEquation extends Equation {
  /// Local axis in A
  late Vec3 axisA;
  /// Local axis in B
  late Vec3 axisB;
  /// The "cone angle" to keep
  double angle;

  ConeEquation(
    Body bodyA,
    Body bodyB,
    {
      double maxForce = 1e6,
      Vec3? axisA,
      Vec3? axisB,
      this.angle = 0
    }
  ):super(bodyA,bodyB,-maxForce,maxForce) {
    this.axisA = axisA != null? axisA.clone() : Vec3(1, 0, 0);
    this.axisB = axisB != null? axisB.clone() : Vec3(0, 1, 0);
  }

  final _tmpVec1 = Vec3();
  final _tmpVec2 = Vec3();

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
    ni.cross(nj, nixnj);
    nj.cross(ni, njxni);

    ga.rotational.copy(njxni);
    gb.rotational.copy(nixnj);

    final g = math.cos(angle) - ni.dot(nj);
    final gw = computeGW();
    final giMf = computeGiMf();
    final B = -g * a - gw * b - h * giMf;
    return B;
  }
}
