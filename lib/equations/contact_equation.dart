import 'equation_class.dart';
import '../math/vec3.dart';
import '../objects/rigid_body.dart';
import 'package:vector_math/vector_math.dart';

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
  Vector3 ri = Vector3.zero();
  /// World-oriented vector that starts in body j position and goes to the contact point.
  Vector3 rj = Vector3.zero();
  /// Contact normal, pointing out of body i.
  Vector3 ni = Vector3.zero();

  final _contactEquationComputeBTemp1 = Vector3.zero(); // Temp vectors
  final _contactEquationComputeBTemp2 = Vector3.zero();
  final _contactEquationComputeBTemp3 = Vector3.zero();

  final _contactEquationGetImpactVelocityAlongNormalVi = Vector3.zero();
  final _contactEquationGetImpactVelocityAlongNormalVj = Vector3.zero();
  final _contactEquationGetImpactVelocityAlongNormalXi = Vector3.zero();
  final _contactEquationGetImpactVelocityAlongNormalXj = Vector3.zero();
  final _contactEquationGetImpactVelocityAlongNormalRelVel = Vector3.zero();

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
    ri.cross2(n, rixn);
    rj.cross2(n, rjxn);

    ga.spatial..setFrom(n)..negate();
    ga.rotational..setFrom(rixn)..negate();
    gb.spatial.setFrom(n);
    gb.rotational.setFrom(rjxn);

    // Calculate the penetration vector
    penetrationVec.setFrom(bj.position);
    penetrationVec.add2(rj, penetrationVec);
    penetrationVec.sub2(bi.position, penetrationVec);
    penetrationVec.sub2(ri, penetrationVec);

    final g = n.dot(penetrationVec);

    // Compute iteration
    final ePlusOne = restitution + 1;
    final gw = ePlusOne * vj.dot(n) - ePlusOne * vi.dot(n) + wj.dot(rjxn) - wi.dot(rixn);
    final giMf = computeGiMf();

    final B = -g * a - gw * b - h * giMf;

    return B;
  }

  /// Get the current relative velocity in the contact point.
  double getImpactVelocityAlongNormal(){
    final vi = _contactEquationGetImpactVelocityAlongNormalVi;
    final vj = _contactEquationGetImpactVelocityAlongNormalVj;
    final xi = _contactEquationGetImpactVelocityAlongNormalXi;
    final xj = _contactEquationGetImpactVelocityAlongNormalXj;
    final relVel = _contactEquationGetImpactVelocityAlongNormalRelVel;

    bi.position.add2(ri, xi);
    bj.position.add2(rj, xj);

    bi.getVelocityAtWorldPoint(xi, vi);
    bj.getVelocityAtWorldPoint(xj, vj);

    vi.sub2(vj, relVel);

    return ni.dot(relVel);
  }
}
