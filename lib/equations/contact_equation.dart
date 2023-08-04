import '../equations/equation.dart';
import '../math/vec3.dart';
import '../objects/body.dart';

/// Contact/non-penetration constraint equation
class ContactEquation extends Equation {
  ContactEquation(
    Body bodyA, 
    Body bodyB, 
    [double maxForce = 1e6]
  ):super(bodyA, bodyB, 0, maxForce);

  /// "bounciness": u1 = -e*u0
  double restitution = 0.0;
  /// World-oriented vector that goes from the center of bi to the contact point.
  Vec3 ri = Vec3();
  /// World-oriented vector that starts in body j position and goes to the contact point.
  Vec3 rj = Vec3();
  /// Contact normal, pointing out of body i.
  Vec3 ni = Vec3();

  final _contactEquationComputeBTemp1 = Vec3(); // Temp vectors
  final _contactEquationComputeBTemp2 = Vec3();
  final _contactEquationComputeBTemp3 = Vec3();

  final _contactEquationGetImpactVelocityAlongNormalVi = Vec3();
  final _contactEquationGetImpactVelocityAlongNormalVj = Vec3();
  final _contactEquationGetImpactVelocityAlongNormalXi = Vec3();
  final _contactEquationGetImpactVelocityAlongNormalXj = Vec3();
  final _contactEquationGetImpactVelocityAlongNormalRelVel = Vec3();

  @override
  double computeB(double h) {
    final a = this.a;
    final b = this.b;
    final bi = this.bi;
    final bj = this.bj;
    final ri = this.ri;
    final rj = this.rj;
    final rixn = _contactEquationComputeBTemp1;
    final rjxn = _contactEquationComputeBTemp2;
    final vi = bi.velocity;
    final wi = bi.angularVelocity;
    final vj = bj.velocity;
    final wj = bj.angularVelocity;
    final penetrationVec = _contactEquationComputeBTemp3;
    final ga = jacobianElementA;
    final gb = jacobianElementB;
    final n = ni;

    // Caluclate cross products
    ri.cross(n, rixn);
    rj.cross(n, rjxn);

    // g = xj+rj -(xi+ri)
    // G = [ -ni  -rixn  ni  rjxn ]
    n.negate(ga.spatial);
    rixn.negate(ga.rotational);
    gb.spatial.copy(n);
    gb.rotational.copy(rjxn);
    // Calculate the penetration vector
    penetrationVec.copy(bj.position);
    penetrationVec.vadd(rj, penetrationVec);
    penetrationVec.vsub(bi.position, penetrationVec);
    penetrationVec.vsub(ri, penetrationVec);

    final g = n.dot(penetrationVec);

    // Compute iteration
    final ePlusOne = restitution + 1;
    final gw = ePlusOne * vj.dot(n) - ePlusOne * vi.dot(n) + wj.dot(rjxn) - wi.dot(rixn);
    final giMf = computeGiMf();
    return (-g * a - gw * b - h * giMf);
  }

  /// Get the current relative velocity in the contact point.
  double getImpactVelocityAlongNormal(){
    final vi = _contactEquationGetImpactVelocityAlongNormalVi;
    final vj = _contactEquationGetImpactVelocityAlongNormalVj;
    final xi = _contactEquationGetImpactVelocityAlongNormalXi;
    final xj = _contactEquationGetImpactVelocityAlongNormalXj;
    final relVel = _contactEquationGetImpactVelocityAlongNormalRelVel;

    bi.position.vadd(ri, xi);
    bj.position.vadd(rj, xj);

    bi.getVelocityAtWorldPoint(xi, vi);
    bj.getVelocityAtWorldPoint(xj, vj);

    vi.vsub(vj, relVel);

    return ni.dot(relVel);
  }
}
