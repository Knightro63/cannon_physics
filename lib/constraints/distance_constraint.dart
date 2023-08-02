import '../constraints/constraint_class.dart';
import '../equations/contact_equation.dart';
import '../objects/body.dart';

/// Constrains two bodies to be at a constant distance from each others center of mass.
class DistanceConstraint extends Constraint {
  /// The distance to keep. If undefined, it will be set to the current distance between bodyA and bodyB
  late double distance;
  late ContactEquation distanceEquation;

  /// [distance] The distance to keep. If undefined, it will be set to the current distance between bodyA and bodyB.
  /// [maxForce] The maximum force that should be applied to constrain the bodies.
  DistanceConstraint(Body bodyA, Body bodyB, [double? distance, double maxForce = 1e6]):super(bodyA, bodyB) {
    this.distance = distance ?? bodyA.position.distanceTo(bodyB.position);
    distanceEquation = ContactEquation(bodyA, bodyB);
    final eq = distanceEquation;
    equations.add(eq);

    // Make it bidirectional
    eq.minForce = -maxForce;
    eq.maxForce = maxForce;
  }

  /// update
  @override
  void update() {
    final bodyA = this.bodyA;
    final bodyB = this.bodyB;
    final eq = distanceEquation;
    final halfDist = distance * 0.5;
    final normal = eq.ni;

    bodyB.position.vsub(bodyA.position, normal);
    normal.normalize();
    normal.scale(halfDist, eq.ri);
    normal.scale(-halfDist, eq.rj);
  }
}
