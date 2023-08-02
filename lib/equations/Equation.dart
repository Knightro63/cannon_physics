import '../math/jacobian_element.dart';
import '../math/vec3.dart';
import '../objects/body.dart';
import '../shapes/shape.dart';

/// Equation base class.
///
/// `a`, `b` and `eps` are {@link https://www8.cs.umu.se/kurser/5DV058/VT15/lectures/SPOOKlabnotes.pdf SPOOK} parameters that default to `0.0`. See {@link https://github.com/schteppe/cannon.js/issues/238#issuecomment-147172327 this exchange} for more details on Cannon's physics implementation.
class Equation {
  late int id;
  /// Minimum (read: negative max) force to be applied by the constraint.
  double minForce;
  ///Maximum (read: positive max) force to be applied by the constraint.
  double maxForce;
  Body bi;
  Body bj;

  late Shape si;
  late Shape sj;
  /// SPOOK parameter
  double a = 0.0;
  /// SPOOK parameter
  double b = 0.0;
  /// SPOOK parameter
  double eps = 0.0;
  JacobianElement jacobianElementA = JacobianElement();
  JacobianElement jacobianElementB = JacobianElement();
  bool enabled = true;
  /// A number, proportional to the force added to the bodies.
  double multiplier = 0;

  static int idCounter = 0;

  Equation(this.bi, this.bj, [this.minForce = -1e6, this.maxForce = 1e6]) {
    id = Equation.idCounter++;
    setSpookParams(1e7, 4, 1 / 60); // Set typical spook params
  }

  final _iMfi = Vec3();
  final _iMfj = Vec3();
  final _invIiVmultTaui = Vec3();
  final _invIjVmultTauj = Vec3();

  final _tmp = Vec3();
  final _addToWlambdaTemp = Vec3();

  /// Recalculates a, b, and eps.
  ///
  /// The Equation constructor sets typical SPOOK parameters as such:
  /// * `stiffness` = 1e7
  /// * `relaxation` = 4
  /// * `timeStep`= 1 / 60, _note the hardcoded refresh rate._
  void setSpookParams(double stiffness, double relaxation, double timeStep) {
    final double d = relaxation;
    final double k = stiffness;
    final double h = timeStep;
    a = 4.0 / (h * (1 + 4 * d));
    b = (4.0 * d) / (1 + 4 * d);
    eps = 4.0 / (h * h * k * (1 + 4 * d));
  }

  /// Computes the right hand side of the SPOOK equation
  double computeB(double h) {
    final double gw = computeGW();
    final double gq = computeGq();
    final double gimf = computeGiMf();
    return -gq * a - gw * b - gimf * h;
  }

  /// Computes G*q, where q are the generalized body coordinates
  double computeGq() {
    final ga = jacobianElementA;
    final gb = jacobianElementB;
    final bi = this.bi;
    final bj = this.bj;
    final xi = bi.position;
    final xj = bj.position;
    return ga.spatial.dot(xi) + gb.spatial.dot(xj);
  }

  /// Computes G*W, where W are the body velocities
  double computeGW() {
    final ga = jacobianElementA;
    final gb = jacobianElementB;
    final bi = this.bi;
    final bj = this.bj;
    final vi = bi.velocity;
    final vj = bj.velocity;
    final wi = bi.angularVelocity;
    final wj = bj.angularVelocity;
    return ga.multiplyVectors(vi, wi) + gb.multiplyVectors(vj, wj);
  }

  /// Computes G*Wlambda, where W are the body velocities
  double computeGWlambda() {
    final ga = jacobianElementA;
    final gb = jacobianElementB;
    final bi = this.bi;
    final bj = this.bj;
    final vi = bi.vlambda;
    final vj = bj.vlambda;
    final wi = bi.wlambda;
    final wj = bj.wlambda;
    return ga.multiplyVectors(vi, wi) + gb.multiplyVectors(vj, wj);
  }

  /// Computes G*inv(M)*f, where M is the mass matrix with diagonal blocks for each body, and f are the forces on the bodies.
  double computeGiMf() {
    final ga = jacobianElementA;
    final gb = jacobianElementB;
    final bi = this.bi;
    final bj = this.bj;
    final fi = bi.force;
    final ti = bi.torque;
    final fj = bj.force;
    final tj = bj.torque;
    final invMassi = bi.invMassSolve;
    final invMassj = bj.invMassSolve;

    fi.scale(invMassi, _iMfi);
    fj.scale(invMassj, _iMfj);

    bi.invInertiaWorldSolve.vmult(ti, _invIiVmultTaui);
    bj.invInertiaWorldSolve.vmult(tj, _invIjVmultTauj);

    return ga.multiplyVectors(_iMfi, _invIiVmultTaui) + gb.multiplyVectors(_iMfj, _invIjVmultTauj);
  }

  /// Computes G*inv(M)*G'
  double computeGiMGt() {
    final ga = jacobianElementA;
    final gb = jacobianElementB;
    final bi = this.bi;
    final bj = this.bj;
    final invMassi = bi.invMassSolve;
    final invMassj = bj.invMassSolve;
    final invIi = bi.invInertiaWorldSolve;
    final invIj = bj.invInertiaWorldSolve;
    double result = invMassi + invMassj;

    invIi.vmult(ga.rotational, _tmp);
    result += _tmp.dot(ga.rotational);

    invIj.vmult(gb.rotational, _tmp);
    result += _tmp.dot(gb.rotational);

    return result;
  }

  /// Add constraint velocity to the bodies.
  void addToWlambda(double deltalambda) {
    final ga = jacobianElementA;
    final gb = jacobianElementB;
    final bi = this.bi;
    final bj = this.bj;
    final temp = _addToWlambdaTemp;

    // Add to linear velocity
    // v_lambda += inv(M) * delta_lamba * G
    bi.vlambda.addScaledVector(bi.invMassSolve * deltalambda, ga.spatial, bi.vlambda);
    bj.vlambda.addScaledVector(bj.invMassSolve * deltalambda, gb.spatial, bj.vlambda);

    // Add to angular velocity
    bi.invInertiaWorldSolve.vmult(ga.rotational, temp);
    bi.wlambda.addScaledVector(deltalambda, temp, bi.wlambda);

    bj.invInertiaWorldSolve.vmult(gb.rotational, temp);
    bj.wlambda.addScaledVector(deltalambda, temp, bj.wlambda);
  }

  /// Compute the denominator part of the SPOOK equation: C = G*inv(M)*G' + eps
  double computeC() {
    return computeGiMGt() + eps;
  }
}
