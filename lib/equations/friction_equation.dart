import 'equation_class.dart';
import '../math/vec3.dart';
import '../objects/body.dart';

/// Constrains the slipping in a contact along a tangent
class FrictionEquation extends Equation {
  Vec3 ri = Vec3();
  Vec3 rj = Vec3(); 
  Vec3 t = Vec3(); // Tangent

  /// @param slipForce should be +-F_friction = +-mu * F_normal = +-mu * m * g
  FrictionEquation (Body bodyA, Body bodyB, double slipForce):super(bodyA, bodyB, -slipForce, slipForce);

  final _frictionEquationComputeBTemp1 = Vec3();
  final _frictionEquationComputeBTemp2 = Vec3();

  @override
  double computeB(double h){
    final b = this.b;
    final ri = this.ri;
    final rj = this.rj;
    final rixt = _frictionEquationComputeBTemp1;
    final rjxt = _frictionEquationComputeBTemp2;
    final t = this.t;

    // Caluclate cross products
    ri.cross(t, rixt);
    rj.cross(t, rjxt);

    // G = [-t -rixt t rjxt]
    // And remember, this is a pure velocity constraint, g is always zero!
    final ga = jacobianElementA;

    final gb = jacobianElementB;
    t.negate(ga.spatial);
    rixt.negate(ga.rotational);
    gb.spatial.copy(t);
    gb.rotational.copy(rjxt);

    final gw = computeGW();
    final giMf = computeGiMf();

    final B = -gw * b - h * giMf;

    return B;
  }
}
