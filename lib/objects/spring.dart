import '../math/vec3.dart';
import 'rigid_body.dart';
import 'package:vector_math/vector_math.dart';

/// A spring, connecting two bodies.
/// @example
///     const spring = Spring(boxBody, sphereBody, {
///       restLength: 0,
///       stiffness: 50,
///       damping: 1,
///     })
///
///     // Compute the force after each step
///     world.addEventListener('postStep', (event) => {
///       spring.applyForce()
///     })
class Spring {
  /// Rest length of the spring. A number > 0.
  double restLength;

  /// Stiffness of the spring. A number >= 0.
  double stiffness;

  /// Damping of the spring. A number >= 0.
  double damping;

  /// First connected body.
  Body bodyA;

  /// Second connected body.
  Body bodyB;

  /// Anchor for bodyA in local bodyA coordinates.
  /// Where to hook the spring to body A, in local body coordinates.
  late Vector3 localAnchorA;

  /// Anchor for bodyB in local bodyB coordinates.
  /// Where to hook the spring to body B, in local body coordinates.
  late Vector3 localAnchorB;

  Spring(
    this.bodyA,
    this.bodyB,
    {
      this.restLength = 1,
      this.stiffness = 100,
      this.damping = 1,
      Vector3? localAnchorA,
      Vector3? localAnchorB,
      Vector3? worldAnchorA,
      Vector3? worldAnchorB,
  }) {
    this.localAnchorA = Vector3.zero();
    this.localAnchorB = Vector3.zero();

    if (localAnchorA != null) {
      this.localAnchorA.setFrom(localAnchorA);
    }
    if (localAnchorB != null) {
      this.localAnchorB.setFrom(localAnchorB);
    }
    if (worldAnchorA != null) {
      setWorldAnchorA(worldAnchorA);
    }
    if (worldAnchorB != null) {
      setWorldAnchorB(worldAnchorB);
    }
  }

  final _applyForceR = Vector3.zero();
  final _applyForceRUnit = Vector3.zero();
  final _applyForceU = Vector3.zero();
  final _applyForceF = Vector3.zero();
  final _applyForceWorldAnchorA = Vector3.zero();
  final _applyForceWorldAnchorB = Vector3.zero();
  final _applyForceRi = Vector3.zero();
  final _applyForceRj = Vector3.zero();
  final _applyForceRixf = Vector3.zero();
  final _applyForceRjxf = Vector3.zero();
  final _applyForceTmp = Vector3.zero();

  /// et the anchor point on body A, using world coordinates.
  void setWorldAnchorA(Vector3 worldAnchorA) {
    bodyA.pointToLocalFrame(worldAnchorA, localAnchorA);
  }

  /// Set the anchor point on body B, using world coordinates.
  void setWorldAnchorB(Vector3 worldAnchorB) {
    bodyB.pointToLocalFrame(worldAnchorB, localAnchorB);
  }

  /// Get the anchor point on body A, in world coordinates.
  /// @param result The vector to store the result in.
  void getWorldAnchorA(Vector3 result) {
    bodyA.pointToWorldFrame(localAnchorA, result);
  }

  /// Get the anchor point on body B, in world coordinates.
  /// @param result The vector to store the result in.
  void getWorldAnchorB(Vector3 result) {
    bodyB.pointToWorldFrame(localAnchorB, result);
  }

  /// Apply the spring force to the connected bodies.
  void applyForce() {
    final k = stiffness;
    final d = damping;
    final l = restLength;
    final bodyA = this.bodyA;
    final bodyB = this.bodyB;
    final r = _applyForceR;
    final rUnit = _applyForceRUnit;
    final u = _applyForceU;
    final f = _applyForceF;
    final tmp = _applyForceTmp;
    final worldAnchorA = _applyForceWorldAnchorA;
    final worldAnchorB = _applyForceWorldAnchorB;
    final ri = _applyForceRi;
    final rj = _applyForceRj;
    final rixf = _applyForceRixf;
    final rjxf = _applyForceRjxf;

    // Get world anchors
    getWorldAnchorA(worldAnchorA);
    getWorldAnchorB(worldAnchorB);

    // Get offset points
    worldAnchorA.sub2(bodyA.position, ri);
    worldAnchorB.sub2(bodyB.position, rj);

    // Compute distance vector between world anchor points
    worldAnchorB.sub2(worldAnchorA, r);
    final rlen = r.length;
    rUnit.setFrom(r);
    rUnit.normalize();

    // Compute relative velocity of the anchor points, u
    bodyB.velocity.sub2(bodyA.velocity, u);
    // Add rotational velocity

    bodyB.angularVelocity.cross2(rj, tmp);
    u.add2(tmp, u);
    bodyA.angularVelocity.cross2(ri, tmp);
    u.sub2(tmp, u);

    // F = - k * ( x - L ) - D * ( u )
    rUnit.scale2(-k * (rlen - l) - d * u.dot(rUnit), f);

    // Add forces to bodies
    bodyA.force.sub2(f, bodyA.force);
    bodyB.force.add2(f, bodyB.force);

    // Angular force
    ri.cross2(f, rixf);
    rj.cross2(f, rjxf);
    bodyA.torque.sub2(rixf, bodyA.torque);
    bodyB.torque.add2(rjxf, bodyB.torque);
  }
}
