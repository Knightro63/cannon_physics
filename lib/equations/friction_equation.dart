import '../equations/equation.dart';
import '../math/vec3.dart';
import '../objects/body.dart';

/**
 * Constrains the slipping in a contact along a tangent
 */
class FrictionEquation extends Equation {
  Vec3 ri = Vec3();
  Vec3 rj = Vec3(); 
  Vec3 t = Vec3(); // Tangent

  /**
   * @param slipForce should be +-F_friction = +-mu * F_normal = +-mu * m * g
   */
  FrictionEquation (Body bodyA, Body bodyB, double slipForce):super(bodyA, bodyB, -slipForce, slipForce);
  @override
  double computeB(double h){
    final b = this.b;
    final ri = this.ri;
    final rj = this.rj;
    final rixt = FrictionEquation_computeB_temp1;
    final rjxt = FrictionEquation_computeB_temp2;
    final t = this.t;

    // Caluclate cross products
    ri.cross(t, rixt);
    rj.cross(t, rjxt);

    // G = [-t -rixt t rjxt]
    // And remember, this is a pure velocity constraint, g is always zero!
    final GA = jacobianElementA;

    final GB = jacobianElementB;
    t.negate(GA.spatial);
    rixt.negate(GA.rotational);
    GB.spatial.copy(t);
    GB.rotational.copy(rjxt);

    final GW = computeGW();
    final GiMf = computeGiMf();

    final B = -GW * b - h * GiMf;

    return B;
  }
}

final FrictionEquation_computeB_temp1 = Vec3();
final FrictionEquation_computeB_temp2 = Vec3();
