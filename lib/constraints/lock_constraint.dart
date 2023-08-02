import '../constraints/point_to_point_constraint.dart';
import '../equations/rotational_equation.dart';
import '../math/vec3.dart';
import '../objects/body.dart';
import '../equations/rotational_motor_equation.dart';

/// Lock constraint. Will remove all degrees of freedom between the bodies.
class LockConstraint extends PointToPointConstraint {
  late Vec3 xA;
  late Vec3 xB;
  late Vec3 yA;
  late Vec3 yB;
  late Vec3 zA;
  late Vec3 zB;

  late RotationalEquation rotationalEquation1;
  late RotationalEquation rotationalEquation2;
  late RotationalEquation rotationalEquation3;
  late RotationalMotorEquation? motorEquation;

  LockConstraint(
    Body bodyA,
    Body bodyB,
    {
      double maxForce = 1e6
    }
  ):super(bodyA, bodyB, Vec3(),Vec3(), maxForce) {

    final halfWay = Vec3();
    bodyA.position.vadd(bodyB.position, halfWay);
    halfWay.scale(0.5, halfWay);
    bodyB.pointToLocalFrame(halfWay, pivotB);
    bodyA.pointToLocalFrame(halfWay, pivotA);

    // The point-to-point constraint will keep a point shared between the bodies
    

    // Store initial rotation of the bodies as unit vectors in the local body spaces
    xA = bodyA.vectorToLocalFrame(Vec3.unitX);
    xB = bodyB.vectorToLocalFrame(Vec3.unitX);
    yA = bodyA.vectorToLocalFrame(Vec3.unitY);
    yB = bodyB.vectorToLocalFrame(Vec3.unitY);
    zA = bodyA.vectorToLocalFrame(Vec3.unitZ);
    zB = bodyB.vectorToLocalFrame(Vec3.unitZ);

    // ...and the following rotational equations will keep all rotational DOF's in place
    rotationalEquation1 = RotationalEquation(bodyA, bodyB, maxForce: maxForce);
    final r1 = rotationalEquation1;
    rotationalEquation2 = RotationalEquation(bodyA, bodyB, maxForce: maxForce);
    final r2 = rotationalEquation2;
    rotationalEquation3 = RotationalEquation(bodyA, bodyB, maxForce: maxForce);
    final r3 = rotationalEquation3;

    equations.add(r1);
    equations.add(r2);
    equations.add(r3);
  }


  //final _lockConstraintUpdateTmpVec1 = Vec3();
  //final _lockConstraintUpdateTmpVec2 = Vec3();

  /// update
  @override
  void update(){
    final bodyA = this.bodyA;
    final bodyB = this.bodyB;
    final r1 = rotationalEquation1;
    final r2 = rotationalEquation2;
    final r3 = rotationalEquation3;

    super.update();

    // These vector pairs must be orthogonal
    bodyA.vectorToWorldFrame(xA, r1.axisA);
    bodyB.vectorToWorldFrame(yB, r1.axisB);

    bodyA.vectorToWorldFrame(yA, r2.axisA);
    bodyB.vectorToWorldFrame(zB, r2.axisB);

    bodyA.vectorToWorldFrame(zA, r3.axisA);
    bodyB.vectorToWorldFrame(xB, r3.axisB);
  }
}
