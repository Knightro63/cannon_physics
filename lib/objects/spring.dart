import '../math/vec3.dart';
import '../objects/body.dart';

/**
 * A spring, connecting two bodies.
 * @example
 *     const spring = Spring(boxBody, sphereBody, {
 *       restLength: 0,
 *       stiffness: 50,
 *       damping: 1,
 *     })
 *
 *     // Compute the force after each step
 *     world.addEventListener('postStep', (event) => {
 *       spring.applyForce()
 *     })
 */
class Spring {
  /**
   * Rest length of the spring. A number > 0.
   * @default 1
   */
  double restLength;

  /**
   * Stiffness of the spring. A number >= 0.
   * @default 100
   */
  double stiffness;

  /**
   * Damping of the spring. A number >= 0.
   * @default 1
   */
  double damping;

  /**
   * First connected body.
   */
  Body bodyA;

  /**
   * Second connected body.
   */
  Body bodyB;

  /**
   * Anchor for bodyA in local bodyA coordinates.
   * Where to hook the spring to body A, in local body coordinates.
   * @default Vec3()
   */
  late Vec3 localAnchorA;

  /**
   * Anchor for bodyB in local bodyB coordinates.
   * Where to hook the spring to body B, in local body coordinates.
   * @default Vec3()
   */
  late Vec3 localAnchorB;

  Spring(
    this.bodyA,
    this.bodyB,
    {
      this.restLength = 1,
      this.stiffness = 100,
      this.damping = 1,
      Vec3? localAnchorA,
      Vec3? localAnchorB,
      Vec3? worldAnchorA,
      Vec3? worldAnchorB,
  }) {
    this.localAnchorA = Vec3();
    this.localAnchorB = Vec3();

    if (localAnchorA != null) {
      this.localAnchorA.copy(localAnchorA);
    }
    if (localAnchorB != null) {
      this.localAnchorB.copy(localAnchorB);
    }
    if (worldAnchorA != null) {
      setWorldAnchorA(worldAnchorA);
    }
    if (worldAnchorB != null) {
      setWorldAnchorB(worldAnchorB);
    }
  }

  /**
   * Set the anchor point on body A, using world coordinates.
   */
  void setWorldAnchorA(Vec3 worldAnchorA) {
    bodyA.pointToLocalFrame(worldAnchorA, localAnchorA);
  }

  /**
   * Set the anchor point on body B, using world coordinates.
   */
  void setWorldAnchorB(Vec3 worldAnchorB) {
    bodyB.pointToLocalFrame(worldAnchorB, localAnchorB);
  }

  /**
   * Get the anchor point on body A, in world coordinates.
   * @param result The vector to store the result in.
   */
  void getWorldAnchorA(Vec3 result) {
    bodyA.pointToWorldFrame(localAnchorA, result);
  }

  /**
   * Get the anchor point on body B, in world coordinates.
   * @param result The vector to store the result in.
   */
  void getWorldAnchorB(Vec3 result) {
    bodyB.pointToWorldFrame(localAnchorB, result);
  }

  /**
   * Apply the spring force to the connected bodies.
   */
  void applyForce() {
    final k = stiffness;
    final d = damping;
    final l = restLength;
    final bodyA = this.bodyA;
    final bodyB = this.bodyB;
    final r = applyForce_r;
    final r_unit = applyForce_r_unit;
    final u = applyForce_u;
    final f = applyForce_f;
    final tmp = applyForce_tmp;
    final worldAnchorA = applyForce_worldAnchorA;
    final worldAnchorB = applyForce_worldAnchorB;
    final ri = applyForce_ri;
    final rj = applyForce_rj;
    final ri_x_f = applyForce_ri_x_f;
    final rj_x_f = applyForce_rj_x_f;

    // Get world anchors
    getWorldAnchorA(worldAnchorA);
    getWorldAnchorB(worldAnchorB);

    // Get offset points
    worldAnchorA.vsub(bodyA.position, ri);
    worldAnchorB.vsub(bodyB.position, rj);

    // Compute distance vector between world anchor points
    worldAnchorB.vsub(worldAnchorA, r);
    final rlen = r.length();
    r_unit.copy(r);
    r_unit.normalize();

    // Compute relative velocity of the anchor points, u
    bodyB.velocity.vsub(bodyA.velocity, u);
    // Add rotational velocity

    bodyB.angularVelocity.cross(rj, tmp);
    u.vadd(tmp, u);
    bodyA.angularVelocity.cross(ri, tmp);
    u.vsub(tmp, u);

    // F = - k * ( x - L ) - D * ( u )
    r_unit.scale(-k * (rlen - l) - d * u.dot(r_unit), f);

    // Add forces to bodies
    bodyA.force.vsub(f, bodyA.force);
    bodyB.force.vadd(f, bodyB.force);

    // Angular force
    ri.cross(f, ri_x_f);
    rj.cross(f, rj_x_f);
    bodyA.torque.vsub(ri_x_f, bodyA.torque);
    bodyB.torque.vadd(rj_x_f, bodyB.torque);
  }
}

final applyForce_r = Vec3();
final applyForce_r_unit = Vec3();
final applyForce_u = Vec3();
final applyForce_f = Vec3();
final applyForce_worldAnchorA = Vec3();
final applyForce_worldAnchorB = Vec3();
final applyForce_ri = Vec3();
final applyForce_rj = Vec3();
final applyForce_ri_x_f = Vec3();
final applyForce_rj_x_f = Vec3();
final applyForce_tmp = Vec3();
