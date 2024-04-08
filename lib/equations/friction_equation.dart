import 'equation_class.dart';
import '../math/vec3.dart';
import '../objects/rigid_body.dart';
import 'package:vector_math/vector_math.dart';

/// Constrains the slipping in a contact along a tangent
class FrictionEquation extends Equation {
  Vector3 ri = Vector3.zero();
  Vector3 rj = Vector3.zero(); 
  Vector3 t = Vector3.zero(); // Tangent

  /// @param slipForce should be +-F_friction = +-mu * F_normal = +-mu * m * g
  FrictionEquation (Body bodyA, Body bodyB, double slipForce):super(bodyA, bodyB, -slipForce, slipForce);

  final _frictionEquationComputeBTemp1 = Vector3.zero();
  final _frictionEquationComputeBTemp2 = Vector3.zero();

  @override
  double computeB(double h){
    final b = this.b;
    final ri = this.ri;
    final rj = this.rj;
    final rixt = _frictionEquationComputeBTemp1;
    final rjxt = _frictionEquationComputeBTemp2;
    final t = this.t;

    // Caluclate cross products
    ri.cross2(t, rixt);
    rj.cross2(t, rjxt);

    // G = [-t -rixt t rjxt]
    // And remember, this is a pure velocity constraint, g is always zero!
    final ga = jacobianElementA;

    final gb = jacobianElementB;
    ga.spatial..setFrom(t)..negate();
    ga.rotational..setFrom(rixt)..negate();
    gb.spatial.setFrom(t);
    gb.rotational.setFrom(rjxt);

    final gw = computeGW();
    final giMf = computeGiMf();

    final B = -gw * b - h * giMf;

    return B;
  }
}
