import '../equations/equation.dart';
import '../math/vec3.dart';
import '../objects/body.dart';

/**
 * Contact/non-penetration constraint equation
 */
class ContactEquation extends Equation {
  ContactEquation(
    Body bodyA, 
    Body bodyB, 
    [double maxForce = 1e6]
  ):super(bodyA, bodyB, 0, maxForce);

  /**
   * "bounciness": u1 = -e*u0
   */
  double restitution = 0.0;
  /**
   * World-oriented vector that goes from the center of bi to the contact point.
   */
  Vec3 ri = Vec3();
  /**
   * World-oriented vector that starts in body j position and goes to the contact point.
   */
  Vec3 rj = Vec3();
  /**
   * Contact normal, pointing out of body i.
   */
  Vec3 ni = Vec3();

  final ContactEquation_computeB_temp1 = Vec3(); // Temp vectors
  final ContactEquation_computeB_temp2 = Vec3();
  final ContactEquation_computeB_temp3 = Vec3();

  final ContactEquation_getImpactVelocityAlongNormal_vi = Vec3();
  final ContactEquation_getImpactVelocityAlongNormal_vj = Vec3();
  final ContactEquation_getImpactVelocityAlongNormal_xi = Vec3();
  final ContactEquation_getImpactVelocityAlongNormal_xj = Vec3();
  final ContactEquation_getImpactVelocityAlongNormal_relVel = Vec3();

  @override
  double computeB(double h) {
    final a = this.a;
    final b = this.b;
    final bi = this.bi;
    final bj = this.bj;
    final ri = this.ri;
    final rj = this.rj;
    final rixn = ContactEquation_computeB_temp1;
    final rjxn = ContactEquation_computeB_temp2;
    final vi = bi.velocity;
    final wi = bi.angularVelocity;
    final vj = bj.velocity;
    final wj = bj.angularVelocity;
    final penetrationVec = ContactEquation_computeB_temp3;
    final GA = jacobianElementA;
    final GB = jacobianElementB;
    final n = ni;

    // Caluclate cross products
    ri.cross(n, rixn);
    rj.cross(n, rjxn);

    // g = xj+rj -(xi+ri)
    // G = [ -ni  -rixn  ni  rjxn ]
    n.negate(GA.spatial);
    rixn.negate(GA.rotational);
    GB.spatial.copy(n);
    GB.rotational.copy(rjxn);

    // Calculate the penetration vector
    penetrationVec.copy(bj.position);
    penetrationVec.vadd(rj, penetrationVec);
    penetrationVec.vsub(bi.position, penetrationVec);
    penetrationVec.vsub(ri, penetrationVec);

    final g = n.dot(penetrationVec);

    // Compute iteration
    final ePlusOne = restitution + 1;
    final GW = ePlusOne * vj.dot(n) - ePlusOne * vi.dot(n) + wj.dot(rjxn) - wi.dot(rixn);
    final GiMf = computeGiMf();

    return (-g * a - GW * b - h * GiMf);

  }

  /**
   * Get the current relative velocity in the contact point.
   */
  double getImpactVelocityAlongNormal(){
    final vi = ContactEquation_getImpactVelocityAlongNormal_vi;
    final vj = ContactEquation_getImpactVelocityAlongNormal_vj;
    final xi = ContactEquation_getImpactVelocityAlongNormal_xi;
    final xj = ContactEquation_getImpactVelocityAlongNormal_xj;
    final relVel = ContactEquation_getImpactVelocityAlongNormal_relVel;

    bi.position.vadd(ri, xi);
    bj.position.vadd(rj, xj);

    bi.getVelocityAtWorldPoint(xi, vi);
    bj.getVelocityAtWorldPoint(xj, vj);

    vi.vsub(vj, relVel);

    return ni.dot(relVel);
  }
}
