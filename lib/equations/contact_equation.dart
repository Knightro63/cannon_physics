import '../equations/equation.dart';
import '../math/vec3.dart';
import '../objects/body.dart';

/**
 * Contact/non-penetration constraint equation
 */
class ContactEquation extends Equation {
  ContactEquation(Body bodyA, Body bodyB, [double maxForce = 1e6]):super(bodyA, bodyB, 0, maxForce);

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

  double computeB(num h) {
    const a = this.a;
    const b = this.b;
    const bi = this.bi;
    const bj = this.bj;
    const ri = this.ri;
    const rj = this.rj;
    const rixn = ContactEquation_computeB_temp1;
    const rjxn = ContactEquation_computeB_temp2;
    const vi = bi.velocity;
    const wi = bi.angularVelocity;
    const fi = bi.force;
    const taui = bi.torque;
    const vj = bj.velocity;
    const wj = bj.angularVelocity;
    const fj = bj.force;
    const tauj = bj.torque;
    const penetrationVec = ContactEquation_computeB_temp3;
    const GA = this.jacobianElementA;
    const GB = this.jacobianElementB;
    const n = this.ni;

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

    const g = n.dot(penetrationVec);

    // Compute iteration
    const ePlusOne = this.restitution + 1;
    const GW = ePlusOne * vj.dot(n) - ePlusOne * vi.dot(n) + wj.dot(rjxn) - wi.dot(rixn);
    const GiMf = this.computeGiMf();

    return (-g * a - GW * b - h * GiMf);

  }

  /**
   * Get the current relative velocity in the contact point.
   */
  double getImpactVelocityAlongNormal(){
    const vi = ContactEquation_getImpactVelocityAlongNormal_vi;
    const vj = ContactEquation_getImpactVelocityAlongNormal_vj;
    const xi = ContactEquation_getImpactVelocityAlongNormal_xi;
    const xj = ContactEquation_getImpactVelocityAlongNormal_xj;
    const relVel = ContactEquation_getImpactVelocityAlongNormal_relVel;

    bi.position.vadd(ri, xi);
    bj.position.vadd(rj, xj);

    bi.getVelocityAtWorldPoint(xi, vi);
    bj.getVelocityAtWorldPoint(xj, vj);

    vi.vsub(vj, relVel);

    return ni.dot(relVel);
  }
}
