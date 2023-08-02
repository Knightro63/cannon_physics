import 'dart:math' as math;
import '../shapes/shape.dart';
import '../math/vec3.dart';
import '../math/transform.dart';
import '../math/quaternion.dart';
import '../objects/body.dart';
import '../collision/aabb.dart';
import '../collision/ray.dart';
import '../utils/vec3_pool.dart';
import '../equations/contact_equation.dart';
import '../equations/friction_equation.dart';
import '../shapes/box.dart';
import '../shapes/sphere.dart';
import '../shapes/convex_polyhedron.dart';
import '../shapes/particle.dart';
import '../shapes/plane.dart';
import '../shapes/trimesh.dart';
import '../shapes/heightfield.dart';
import '../shapes/cylinder.dart';
import '../material/contact_material.dart';
import '../world/world_class.dart';

// Naming rule: based of the order in SHAPE_TYPES,
// the first part of the method is formed by the
// shape type that comes before, in the second part
// there is the shape type that comes after in the SHAPE_TYPES list
enum CollisionType{
  sphereSphere,
  spherePlane,
  boxBox,
  sphereBox,
  planeBox,
  convexConvex,
  sphereConvex,
  planeConvex,
  boxConvex,
  sphereHeightfield,
  boxHeightfield,
  convexHeightfield,
  sphereParticle,
  planeParticle,
  boxParticle,
  convexParticle,
  cylinderCylinder,
  sphereCylinder,
  planeCylinder,
  boxCylinder,
  convexCylinder,
  heightfieldCylinder,
  particleCylinder,
  sphereTrimesh,
  planeTrimesh,
}

/// Helper class for the World. Generates ContactEquations.
/// @todo Sphere-ConvexPolyhedron contacts
/// @todo Contact reduction
/// @todo should move methods to prototype
class Narrowphase {
  /// Internal storage of pooled contact points
  List<ContactEquation> contactPointPool = [];
  List<FrictionEquation> frictionEquationPool = [];
  List<ContactEquation> result = [];
  List<FrictionEquation> frictionResult = [];
  Vec3Pool v3pool = Vec3Pool(); 
  World world; 
  late ContactMaterial currentContactMaterial;
  bool enableFrictionReduction = false;

  operator [](CollisionType type){
    if(type == CollisionType.sphereSphere) {
      return sphereSphere;
    }
    else if(type == CollisionType.spherePlane) {
      return spherePlane;
    }
    else if(type == CollisionType.boxBox) {
      return boxBox;
    }
    else if(type == CollisionType.sphereBox) {
      return sphereBox;
    }
    else if(type == CollisionType.planeBox) {
      return planeBox;
    }
    else if(type == CollisionType.convexConvex) {
      return convexConvex;
    }
    else if(type == CollisionType.sphereConvex) {
      return sphereConvex;
    }
    else if(type == CollisionType.planeConvex) {
      return planeConvex;
    }
    else if(type == CollisionType.boxConvex) {
      return boxConvex;
    }
    else if(type == CollisionType.sphereHeightfield) {
      return sphereHeightfield;
    }
    else if(type == CollisionType.boxHeightfield) {
      return boxHeightfield;
    }
    else if(type == CollisionType.convexHeightfield) {
      return convexHeightfield;
    }
    else if(type == CollisionType.sphereParticle) {
      return sphereParticle;
    }
    else if(type == CollisionType.planeParticle) {
      return planeParticle;
    }
    else if(type == CollisionType.boxParticle) {
      return boxParticle;
    }
    else if(type == CollisionType.convexParticle) {
      return convexParticle;
    }
    else if(type == CollisionType.cylinderCylinder) {
      return convexConvex;
    }
    else if(type == CollisionType.sphereCylinder) {
      return sphereConvex;
    }
    else if(type == CollisionType.planeCylinder) {
      return planeConvex;
    }
    else if(type == CollisionType.boxCylinder) {
      return boxConvex;
    }
    else if(type == CollisionType.convexCylinder) {
      return convexConvex;
    }
    else if(type == CollisionType.heightfieldCylinder) {
      return heightfieldCylinder;
    }
    else if(type == CollisionType.particleCylinder) {
      return particleCylinder;
    }
    else if(type == CollisionType.sphereTrimesh) {
      return sphereTrimesh;
    }
    else if(type == CollisionType.planeTrimesh) {
      return planeTrimesh;
    }
    // else if(type == CollisionType.convexTrimesh) {
    //   return this.convexTrimesh
    // }
  }

  Narrowphase(this.world) {
    currentContactMaterial = world.defaultContactMaterial;
  }

  /**
   * Make a contact object, by using the internal pool or creating a one.
   */
  ContactEquation createContactEquation(
    Body bi,
    Body bj,
    Shape si,
    Shape sj,
    [
      Shape? overrideShapeA,
      Shape? overrideShapeB
  ]){
    ContactEquation c;
    if (contactPointPool.isNotEmpty) {
      c = contactPointPool.removeLast();
      c.bi = bi;
      c.bj = bj;
    } else {
      c = ContactEquation(bi, bj);
    }

    c.enabled = bi.collisionResponse && bj.collisionResponse && si.collisionResponse && sj.collisionResponse;

    final cm = currentContactMaterial;

    c.restitution = cm.restitution;

    c.setSpookParams(cm.contactEquationStiffness, cm.contactEquationRelaxation, world.dt);

    final matA = si.material ?? bi.material;
    final matB = sj.material ?? bj.material;
    if (matA != null && matB != null && matA.restitution >= 0 && matB.restitution >= 0) {
      c.restitution = matA.restitution * matB.restitution;
    }

    c.si = overrideShapeA ?? si;
    c.sj = overrideShapeB ?? sj;

    return c;
  }

  bool createFrictionEquationsFromContact(ContactEquation contactEquation, List<FrictionEquation>outArray) {
    final bodyA = contactEquation.bi;
    final bodyB = contactEquation.bj;
    final shapeA = contactEquation.si;
    final shapeB = contactEquation.sj;

    final world = this.world;
    final cm = currentContactMaterial;

    // If friction or restitution were specified in the material, use them
    double friction = cm.friction;
    final matA = shapeA.material ?? bodyA.material;
    final matB = shapeB.material ?? bodyB.material;
    if (matA != null && matB != null && matA.friction >= 0 && matB.friction >= 0) {
      friction = matA.friction * matB.friction;
    }

    if (friction > 0) {
      // Create 2 tangent equations
      // Users may provide a force different from global gravity to use when computing contact friction.
      final mug = friction * (world.frictionGravity ?? world.gravity).length();
      double reducedMass = bodyA.invMass + bodyB.invMass;
      if (reducedMass > 0) {
        reducedMass = 1 / reducedMass;
      }
      final pool = frictionEquationPool;
      final c1 = pool.length ? pool.removeLast() : FrictionEquation(bodyA, bodyB, mug * reducedMass);
      final c2 = pool.length ? pool.removeLast() : FrictionEquation(bodyA, bodyB, mug * reducedMass);

      c1.bi = c2.bi = bodyA;
      c1.bj = c2.bj = bodyB;
      c1.minForce = c2.minForce = -mug * reducedMass;
      c1.maxForce = c2.maxForce = mug * reducedMass;

      // Copy over the relative vectors
      c1.ri.copy(contactEquation.ri);
      c1.rj.copy(contactEquation.rj);
      c2.ri.copy(contactEquation.ri);
      c2.rj.copy(contactEquation.rj);

      // Construct tangents
      contactEquation.ni.tangents(c1.t, c2.t);

      // Set spook params
      c1.setSpookParams(cm.frictionEquationStiffness, cm.frictionEquationRelaxation, world.dt);
      c2.setSpookParams(cm.frictionEquationStiffness, cm.frictionEquationRelaxation, world.dt);

      c1.enabled = c2.enabled = contactEquation.enabled;

      outArray.addAll([c1, c2]);

      return true;
    }

    return false;
  }

  /**
   * Take the average N latest contact point on the plane.
   */
  void createFrictionFromAverage(int numContacts) {
    // The last contactEquation
    ContactEquation c = result[result.length - 1];

    // Create the result: two "average" friction equations
    if (!createFrictionEquationsFromContact(c,frictionResult) || numContacts == 1) {
      return;
    }

    final f1 = frictionResult[frictionResult.length - 2];
    final f2 = frictionResult[frictionResult.length - 1];

    averageNormal.setZero();
    averageContactPointA.setZero();
    averageContactPointB.setZero();

    final bodyA = c.bi;
    //final bodyB = c.bj;
    for (int i = 0; i != numContacts; i++) {
      c = result[result.length - 1 - i];
      if (c.bi != bodyA) {
        averageNormal.vadd(c.ni, averageNormal);
        averageContactPointA.vadd(c.ri, averageContactPointA);
        averageContactPointB.vadd(c.rj, averageContactPointB);
      } else {
        averageNormal.vsub(c.ni, averageNormal);
        averageContactPointA.vadd(c.rj, averageContactPointA);
        averageContactPointB.vadd(c.ri, averageContactPointB);
      }
    }

    final invNumContacts = 1 / numContacts;
    averageContactPointA.scale(invNumContacts, f1.ri);
    averageContactPointB.scale(invNumContacts, f1.rj);
    f2.ri.copy(f1.ri); // Should be the same
    f2.rj.copy(f1.rj);
    averageNormal.normalize();
    averageNormal.tangents(f1.t, f2.t);
    // return eq;
  }

  /**
   * Generate all contacts between a list of body pairs
   * @param p1 Array of body indices
   * @param p2 Array of body indices
   * @param result Array to store generated contacts
   * @param oldcontacts Optional. Array of reusable contact objects
   */
  void getContacts(
    List<Body> p1,
    List<Body> p2,
    World world,
    List<ContactEquation> result,
    List<ContactEquation> oldcontacts,
    List<FrictionEquation> frictionResult,
    List<FrictionEquation> frictionPool
   ){
    // Save old contact objects
    contactPointPool = oldcontacts;
    frictionEquationPool = frictionPool;
    this.result = result;
    this.frictionResult = frictionResult;

    final qi = tmpQuat1;
    final qj = tmpQuat2;
    final xi = tmpVec1;
    final xj = tmpVec2;

    for (int k = 0, N = p1.length; k != N; k++) {
      // Get current collision bodies
      final bi = p1[k];
      final bj = p2[k];

      // Get contact material
      ContactMaterial? bodyContactMaterial;
      if (bi.material != null && bj.material != null) {
        bodyContactMaterial = world.getContactMaterial(bi.material!, bj.material!);
      }

      final justTest =
        (bi.type == BodyTypes.kinematic && bj.type == BodyTypes.static) ||
        (bi.type == BodyTypes.static && bj.type == BodyTypes.kinematic) ||
        (bi.type == BodyTypes.kinematic && bj.type == BodyTypes.kinematic);

      for (int i = 0; i < bi.shapes.length; i++) {
        bi.quaternion.mult(bi.shapeOrientations[i], qi);
        bi.quaternion.vmult(bi.shapeOffsets[i], xi);
        xi.vadd(bi.position, xi);
        final si = bi.shapes[i];

        for (int j = 0; j < bj.shapes.length; j++) {
          // Compute world transform of shapes
          bj.quaternion.mult(bj.shapeOrientations[j], qj);
          bj.quaternion.vmult(bj.shapeOffsets[j], xj);
          xj.vadd(bj.position, xj);
          final sj = bj.shapes[j];

          if (!(si.collisionFilterMask & sj.collisionFilterGroup && sj.collisionFilterMask & si.collisionFilterGroup)) {
            continue;
          }

          if (xi.distanceTo(xj) > si.boundingSphereRadius + sj.boundingSphereRadius) {
            continue;
          }

          // Get collision material
          ContactMaterial? shapeContactMaterial;
          if (si.material != null && sj.material != null) {
            shapeContactMaterial = world.getContactMaterial(si.material!, sj.material!);
          }

          currentContactMaterial = shapeContactMaterial ?? bodyContactMaterial ?? world.defaultContactMaterial;

          // Get contacts
          final resolverIndex = (si.type | sj.type) as CollisionType;
          final resolver = this[resolverIndex];
          if (resolver) {
            bool retval = false;

            // TO DO: investigate why sphereParticle and convexParticle
            // resolvers expect si and sj shapes to be in reverse order
            // (i.e. larger integer value type first instead of smaller first)
            if (si.type.index < sj.type.index) {
              retval = (resolver as any).call(this, si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest);
            } else {
              retval = (resolver as any).call(this, sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
            }

            if (retval && justTest) {
              // Register overlap
              world.shapeOverlapKeeper.set(si.id, sj.id);
              world.bodyOverlapKeeper.set(bi.id, bj.id);
            }
          }
        }
      }
    }
  }

  bool? sphereSphere(
    Sphere si,
    Sphere sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    if (justTest) {
      return xi.distanceSquared(xj) < math.pow((si.radius + sj.radius),2);
    }

    // We will have only one contact in this case
    final contactEq = createContactEquation(bi, bj, si, sj, rsi, rsj);

    // Contact normal
    xj.vsub(xi, contactEq.ni);
    contactEq.ni.normalize();

    // Contact point locations
    contactEq.ri.copy(contactEq.ni);
    contactEq.rj.copy(contactEq.ni);
    contactEq.ri.scale(si.radius, contactEq.ri);
    contactEq.rj.scale(-sj.radius, contactEq.rj);

    contactEq.ri.vadd(xi, contactEq.ri);
    contactEq.ri.vsub(bi.position, contactEq.ri);

    contactEq.rj.vadd(xj, contactEq.rj);
    contactEq.rj.vsub(bj.position, contactEq.rj);

    result.add(contactEq);

    createFrictionEquationsFromContact(contactEq, this.frictionResult);
  }

  bool? spherePlane(
    Sphere si,
    Plane sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    // We will have one contact in this case
    final r = createContactEquation(bi, bj, si, sj, rsi, rsj);

    // Contact normal
    r.ni.set(0, 0, 1);
    qj.vmult(r.ni, r.ni);
    r.ni.negate(r.ni); // body i is the sphere, flip normal
    r.ni.normalize(); // Needed?

    // Vector from sphere center to contact point
    r.ni.scale(si.radius, r.ri);

    // Project down sphere on plane
    xi.vsub(xj, point_on_plane_to_sphere);
    r.ni.scale(r.ni.dot(point_on_plane_to_sphere), plane_to_sphere_ortho);
    point_on_plane_to_sphere.vsub(plane_to_sphere_ortho, r.rj); // The sphere position projected to plane

    if (-point_on_plane_to_sphere.dot(r.ni) <= si.radius) {
      if (justTest) {
        return true;
      }

      // Make it relative to the body
      final ri = r.ri;
      final rj = r.rj;
      ri.vadd(xi, ri);
      ri.vsub(bi.position, ri);
      rj.vadd(xj, rj);
      rj.vsub(bj.position, rj);

      result.add(r);
      createFrictionEquationsFromContact(r, this.frictionResult);
    }
  }

  bool? boxBox(
    Box si,
    Box sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    si.convexPolyhedronRepresentation?.material = si.material;
    sj.convexPolyhedronRepresentation?.material = sj.material;
    si.convexPolyhedronRepresentation?.collisionResponse = si.collisionResponse;
    sj.convexPolyhedronRepresentation?.collisionResponse = sj.collisionResponse;
    return convexConvex(
      si.convexPolyhedronRepresentation!,
      sj.convexPolyhedronRepresentation!,
      xi,
      xj,
      qi,
      qj,
      bi,
      bj,
      si,
      sj,
      justTest
    );
  }

  bool? sphereBox(
    Sphere si,
    Box sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    final v3pool = this.v3pool;

    // we refer to the box as body j
    final sides = sphereBox_sides;
    xi.vsub(xj, box_to_sphere);
    sj.getSideNormals(sides, qj);
    final R = si.radius;
    //final penetrating_sides = [];

    // Check side (plane) intersections
    bool found = false;

    // Store the resulting side penetration info
    final side_ns = sphereBox_side_ns;
    final side_ns1 = sphereBox_side_ns1;
    final side_ns2 = sphereBox_side_ns2;

    double? side_h;
    double side_penetrations = 0;
    double side_dot1 = 0;
    double side_dot2 = 0;
    double? side_distance;
    for (int idx = 0, nsides = sides.length; idx != nsides && found == false; idx++) {
      // Get the plane side normal (ns)
      final ns = sphereBox_ns;
      ns.copy(sides[idx]);

      final h = ns.length();
      ns.normalize();

      // The normal/distance dot product tells which side of the plane we are
      final dot = box_to_sphere.dot(ns);

      if (dot < h + R && dot > 0) {
        // Intersects plane. Now check the other two dimensions
        final ns1 = sphereBox_ns1;
        final ns2 = sphereBox_ns2;
        ns1.copy(sides[(idx + 1) % 3]);
        ns2.copy(sides[(idx + 2) % 3]);
        final h1 = ns1.length();
        final h2 = ns2.length();
        ns1.normalize();
        ns2.normalize();
        final dot1 = box_to_sphere.dot(ns1);
        final dot2 = box_to_sphere.dot(ns2);
        if (dot1 < h1 && dot1 > -h1 && dot2 < h2 && dot2 > -h2) {
          final dist = (dot - h - R).abs();
          if (side_distance == null || dist < side_distance) {
            side_distance = dist;
            side_dot1 = dot1;
            side_dot2 = dot2;
            side_h = h;
            side_ns.copy(ns);
            side_ns1.copy(ns1);
            side_ns2.copy(ns2);
            side_penetrations++;

            if (justTest) {
              return true;
            }
          }
        }
      }
    }
    if (side_penetrations != 0) {
      found = true;
      final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
      side_ns.scale(-R, r.ri); // Sphere r
      r.ni.copy(side_ns);
      r.ni.negate(r.ni); // Normal should be out of sphere
      side_ns.scale(side_h!, side_ns);
      side_ns1.scale(side_dot1, side_ns1);
      side_ns.vadd(side_ns1, side_ns);
      side_ns2.scale(side_dot2, side_ns2);
      side_ns.vadd(side_ns2, r.rj);

      // Make relative to bodies
      r.ri.vadd(xi, r.ri);
      r.ri.vsub(bi.position, r.ri);
      r.rj.vadd(xj, r.rj);
      r.rj.vsub(bj.position, r.rj);

      result.add(r);
      createFrictionEquationsFromContact(r, frictionResult);
    }

    // Check corners
    let rj = v3pool.get();
    final sphere_to_corner = sphereBox_sphere_to_corner;
    for (int j = 0; j != 2 && !found; j++) {
      for (int k = 0; k != 2 && !found; k++) {
        for (int l = 0; l != 2 && !found; l++) {
          rj.set(0, 0, 0);
          if (j != 0) {
            rj.vadd(sides[0], rj);
          } else {
            rj.vsub(sides[0], rj);
          }
          if (k != 0) {
            rj.vadd(sides[1], rj);
          } else {
            rj.vsub(sides[1], rj);
          }
          if (l != 0) {
            rj.vadd(sides[2], rj);
          } else {
            rj.vsub(sides[2], rj);
          }

          // World position of corner
          xj.vadd(rj, sphere_to_corner);
          sphere_to_corner.vsub(xi, sphere_to_corner);

          if (sphere_to_corner.lengthSquared() < R * R) {
            if (justTest) {
              return true;
            }
            found = true;
            final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
            r.ri.copy(sphere_to_corner);
            r.ri.normalize();
            r.ni.copy(r.ri);
            r.ri.scale(R, r.ri);
            r.rj.copy(rj);

            // Make relative to bodies
            r.ri.vadd(xi, r.ri);
            r.ri.vsub(bi.position, r.ri);
            r.rj.vadd(xj, r.rj);
            r.rj.vsub(bj.position, r.rj);

            result.add(r);
            createFrictionEquationsFromContact(r, frictionResult);
          }
        }
      }
    }
    v3pool.release(rj);
    rj = null;

    // Check edges
    final edgeTangent = v3pool.get();
    final edgeCenter = v3pool.get();
    final r = v3pool.get(); // r = edge center to sphere center
    final orthogonal = v3pool.get();
    final dist = v3pool.get();
    final Nsides = sides.length;
    for (int j = 0; j != Nsides && !found; j++) {
      for (int k = 0; k != Nsides && !found; k++) {
        if (j % 3 != k % 3) {
          // Get edge tangent
          sides[k].cross(sides[j], edgeTangent);
          edgeTangent.normalize();
          sides[j].vadd(sides[k], edgeCenter);
          r.copy(xi);
          r.vsub(edgeCenter, r);
          r.vsub(xj, r);
          final orthonorm = r.dot(edgeTangent); // distance from edge center to sphere center in the tangent direction
          edgeTangent.scale(orthonorm, orthogonal); // Vector from edge center to sphere center in the tangent direction

          // Find the third side orthogonal to this one
          int l = 0;
          while (l == j % 3 || l == k % 3) {
            l++;
          }

          // vec from edge center to sphere projected to the plane orthogonal to the edge tangent
          dist.copy(xi);
          dist.vsub(orthogonal, dist);
          dist.vsub(edgeCenter, dist);
          dist.vsub(xj, dist);

          // Distances in tangent direction and distance in the plane orthogonal to it
          final tdist = orthonorm.abs();
          final ndist = dist.length();

          if (tdist < sides[l].length() && ndist < R) {
            if (justTest) {
              return true;
            }
            found = true;
            final res = createContactEquation(bi, bj, si, sj, rsi, rsj);
            edgeCenter.vadd(orthogonal, res.rj); // box rj
            res.rj.copy(res.rj);
            dist.negate(res.ni);
            res.ni.normalize();

            res.ri.copy(res.rj);
            res.ri.vadd(xj, res.ri);
            res.ri.vsub(xi, res.ri);
            res.ri.normalize();
            res.ri.scale(R, res.ri);

            // Make relative to bodies
            res.ri.vadd(xi, res.ri);
            res.ri.vsub(bi.position, res.ri);
            res.rj.vadd(xj, res.rj);
            res.rj.vsub(bj.position, res.rj);

            result.add(res);
            createFrictionEquationsFromContact(res, frictionResult);
          }
        }
      }
    }
    v3pool.release(edgeTangent, edgeCenter, r, orthogonal, dist);
  }

  bool? planeBox(
    Plane si,
    Box sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool? justTest
  ]){
    sj.convexPolyhedronRepresentation?.material = sj.material;
    sj.convexPolyhedronRepresentation?.collisionResponse = sj.collisionResponse;
    sj.convexPolyhedronRepresentation?.id = sj.id;
    return planeConvex(si, sj.convexPolyhedronRepresentation!, xi, xj, qi, qj, bi, bj, si, sj, justTest);
  }

  bool? convexConvex(
    ConvexPolyhedron si,
    ConvexPolyhedron sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    final sepAxis = convexConvex_sepAxis;

    if (xi.distanceTo(xj) > si.boundingSphereRadius + sj.boundingSphereRadius) {
      return null;
    }

    if (si.findSeparatingAxis(sj, xi, qi, xj, qj, sepAxis, faceListA, faceListB)) {
      final List<ConvexPolyhedronContactPoint> res = [];
      final q = convexConvex_q;
      si.clipAgainstHull(xi, qi, sj, xj, qj, sepAxis, -100, 100, res);
      int numContacts = 0;
      for (int j = 0; j != res.length; j++) {
        if (justTest) {
          return true;
        }
        final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
        final ri = r.ri;
        final rj = r.rj;
        sepAxis.negate(r.ni);
        res[j].normal.negate(q);
        q.scale(res[j].depth, q);
        res[j].point.vadd(q, ri);
        rj.copy(res[j].point);

        // Contact points are in world coordinates. Transform back to relative
        ri.vsub(xi, ri);
        rj.vsub(xj, rj);

        // Make relative to bodies
        ri.vadd(xi, ri);
        ri.vsub(bi.position, ri);
        rj.vadd(xj, rj);
        rj.vsub(bj.position, rj);

        result.add(r);
        numContacts++;
        if (!enableFrictionReduction) {
          createFrictionEquationsFromContact(r, frictionResult);
        }
      }
      if (enableFrictionReduction && numContacts != 0) {
        createFrictionFromAverage(numContacts);
      }
    }
  }

  bool? sphereConvex(
    Sphere si,
    ConvexPolyhedron sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    final v3pool = this.v3pool;
    xi.vsub(xj, convex_to_sphere);
    final normals = sj.faceNormals;
    final faces = sj.faces;
    final verts = sj.vertices;
    final R = si.radius;
    final penetrating_sides = [];

    // if(convex_to_sphere.lengthSquared() > si.boundingSphereRadius + sj.boundingSphereRadius){
    //     return;
    // }
    bool found = false;

    // Check corners
    for (int i = 0; i != verts.length; i++) {
      final v = verts[i];

      // World position of corner
      final worldCorner = sphereConvex_worldCorner;
      qj.vmult(v, worldCorner);
      xj.vadd(worldCorner, worldCorner);
      final sphere_to_corner = sphereConvex_sphereToCorner;
      worldCorner.vsub(xi, sphere_to_corner);
      if (sphere_to_corner.lengthSquared() < R * R) {
        if (justTest) {
          return true;
        }
        found = true;
        final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
        r.ri.copy(sphere_to_corner);
        r.ri.normalize();
        r.ni.copy(r.ri);
        r.ri.scale(R, r.ri);
        worldCorner.vsub(xj, r.rj);

        // Should be relative to the body.
        r.ri.vadd(xi, r.ri);
        r.ri.vsub(bi.position, r.ri);

        // Should be relative to the body.
        r.rj.vadd(xj, r.rj);
        r.rj.vsub(bj.position, r.rj);

        result.add(r);
        createFrictionEquationsFromContact(r, frictionResult);
        return null;
      }
    }

    // Check side (plane) intersections
    for (int i = 0, nfaces = faces.length; i != nfaces && found == false; i++) {
      final normal = normals[i];
      final face = faces[i];

      // Get world-transformed normal of the face
      final worldNormal = sphereConvex_worldNormal;
      qj.vmult(normal, worldNormal);

      // Get a world vertex from the face
      final worldPoint = sphereConvex_worldPoint;
      qj.vmult(verts[face[0]], worldPoint);
      worldPoint.vadd(xj, worldPoint);

      // Get a point on the sphere, closest to the face normal
      final worldSpherePointClosestToPlane = sphereConvex_worldSpherePointClosestToPlane;
      worldNormal.scale(-R, worldSpherePointClosestToPlane);
      xi.vadd(worldSpherePointClosestToPlane, worldSpherePointClosestToPlane);

      // Vector from a face point to the closest point on the sphere
      final penetrationVec = sphereConvex_penetrationVec;
      worldSpherePointClosestToPlane.vsub(worldPoint, penetrationVec);

      // The penetration. Negative value means overlap.
      final penetration = penetrationVec.dot(worldNormal);

      final worldPointToSphere = sphereConvex_sphereToWorldPoint;
      xi.vsub(worldPoint, worldPointToSphere);

      if (penetration < 0 && worldPointToSphere.dot(worldNormal) > 0) {
        // Intersects plane. Now check if the sphere is inside the face polygon
        final faceVerts = []; // Face vertices, in world coords
        for (int j = 0, Nverts = face.length; j != Nverts; j++) {
          final worldVertex = v3pool.get();
          qj.vmult(verts[face[j]], worldVertex);
          xj.vadd(worldVertex, worldVertex);
          faceVerts.add(worldVertex);
        }

        if (pointInPolygon(faceVerts, worldNormal, xi)) {
          // Is the sphere center in the face polygon?
          if (justTest) {
            return true;
          }
          found = true;
          final r = createContactEquation(bi, bj, si, sj, rsi, rsj);

          worldNormal.scale(-R, r.ri); // Contact offset, from sphere center to contact
          worldNormal.negate(r.ni); // Normal pointing out of sphere

          final penetrationVec2 = v3pool.get();
          worldNormal.scale(-penetration, penetrationVec2);
          final penetrationSpherePoint = v3pool.get();
          worldNormal.scale(-R, penetrationSpherePoint);

          //xi.vsub(xj).vadd(penetrationSpherePoint).vadd(penetrationVec2 , r.rj);
          xi.vsub(xj, r.rj);
          r.rj.vadd(penetrationSpherePoint, r.rj);
          r.rj.vadd(penetrationVec2, r.rj);

          // Should be relative to the body.
          r.rj.vadd(xj, r.rj);
          r.rj.vsub(bj.position, r.rj);

          // Should be relative to the body.
          r.ri.vadd(xi, r.ri);
          r.ri.vsub(bi.position, r.ri);

          v3pool.release(penetrationVec2);
          v3pool.release(penetrationSpherePoint);

          result.add(r);
          createFrictionEquationsFromContact(r, frictionResult);

          // Release world vertices
          for (int j = 0, Nfaceverts = faceVerts.length; j != Nfaceverts; j++) {
            v3pool.release(faceVerts[j]);
          }

          return null; // We only expect *one* face contact
        } else {
          // Edge?
          for (int j = 0; j != face.length; j++) {
            // Get two world transformed vertices
            final v1 = v3pool.get();
            final v2 = v3pool.get();
            qj.vmult(verts[face[(j + 1) % face.length]], v1);
            qj.vmult(verts[face[(j + 2) % face.length]], v2);
            xj.vadd(v1, v1);
            xj.vadd(v2, v2);

            // Construct edge vector
            final edge = sphereConvex_edge;
            v2.vsub(v1, edge);

            // Construct the same vector, but normalized
            final edgeUnit = sphereConvex_edgeUnit;
            edge.unit(edgeUnit);

            // p is xi projected onto the edge
            final p = v3pool.get();
            final v1_to_xi = v3pool.get();
            xi.vsub(v1, v1_to_xi);
            final dot = v1_to_xi.dot(edgeUnit);
            edgeUnit.scale(dot, p);
            p.vadd(v1, p);

            // Compute a vector from p to the center of the sphere
            final xi_to_p = v3pool.get();
            p.vsub(xi, xi_to_p);

            // Collision if the edge-sphere distance is less than the radius
            // AND if p is in between v1 and v2
            if (dot > 0 && dot * dot < edge.lengthSquared() && xi_to_p.lengthSquared() < R * R) {
              // Collision if the edge-sphere distance is less than the radius
              // Edge contact!
              if (justTest) {
                return true;
              }
              final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
              p.vsub(xj, r.rj);

              p.vsub(xi, r.ni);
              r.ni.normalize();

              r.ni.scale(R, r.ri);

              // Should be relative to the body.
              r.rj.vadd(xj, r.rj);
              r.rj.vsub(bj.position, r.rj);

              // Should be relative to the body.
              r.ri.vadd(xi, r.ri);
              r.ri.vsub(bi.position, r.ri);

              result.add(r);
              createFrictionEquationsFromContact(r, frictionResult);

              // Release world vertices
              for (int j = 0, Nfaceverts = faceVerts.length; j != Nfaceverts; j++) {
                v3pool.release(faceVerts[j]);
              }

              v3pool.release(v1);
              v3pool.release(v2);
              v3pool.release(p);
              v3pool.release(xi_to_p);
              v3pool.release(v1_to_xi);

              return null;
            }

            v3pool.release(v1);
            v3pool.release(v2);
            v3pool.release(p);
            v3pool.release(xi_to_p);
            v3pool.release(v1_to_xi);
          }
        }

        // Release world vertices
        for (int j = 0, Nfaceverts = faceVerts.length; j != Nfaceverts; j++) {
          v3pool.release(faceVerts[j]);
        }
      }
    }
  }

  bool? planeConvex(
    Plane si,
    ConvexPolyhedron sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    // Simply return the points behind the plane.
    final worldVertex = planeConvex_v;

    final worldNormal = planeConvex_normal;
    worldNormal.set(0, 0, 1);
    planeQuat.vmult(worldNormal, worldNormal) ;// Turn normal according to plane orientation

    int numContacts = 0;
    final relpos = planeConvex_relpos;
    for (int i = 0; i != convexShape.vertices.length; i++) {
      // Get world convex vertex
      worldVertex.copy(convexShape.vertices[i]);
      convexQuat.vmult(worldVertex, worldVertex);
      convexPosition.vadd(worldVertex, worldVertex);
      worldVertex.vsub(planePosition, relpos);

      final dot = worldNormal.dot(relpos);
      if (dot <= 0.0) {
        if (justTest) {
          return true;
        }

        final r = createContactEquation(planeBody, convexBody, planeShape, convexShape, si, sj);

        // Get vertex position projected on plane
        final projected = planeConvex_projected;
        worldNormal.scale(worldNormal.dot(relpos), projected);
        worldVertex.vsub(projected, projected);
        projected.vsub(planePosition, r.ri); // From plane to vertex projected on plane

        r.ni.copy(worldNormal) ;// Contact normal is the plane normal out from plane

        // rj is now just the vector from the convex center to the vertex
        worldVertex.vsub(convexPosition, r.rj);

        // Make it relative to the body
        r.ri.vadd(planePosition, r.ri);
        r.ri.vsub(planeBody.position, r.ri);
        r.rj.vadd(convexPosition, r.rj);
        r.rj.vsub(convexBody.position, r.rj);

        result.add(r);
        numContacts++;
        if (!enableFrictionReduction) {
          createFrictionEquationsFromContact(r, frictionResult);
        }
      }
    }

    if (enableFrictionReduction && numContacts != 0) {
      createFrictionFromAverage(numContacts);
    }
  }

  bool?  boxConvex(
    Box si,
    ConvexPolyhedron sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    si.convexPolyhedronRepresentation?.material = si.material;
    si.convexPolyhedronRepresentation?.collisionResponse = si.collisionResponse;
    return convexConvex(si.convexPolyhedronRepresentation!, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest);
  }

  bool? sphereHeightfield(
    Sphere si,
    Heightfield sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    final data = hfShape.data;
    final radius = sphereShape.radius;
    final w = hfShape.elementSize;
    final worldPillarOffset = sphereHeightfield_tmp2;

    // Get sphere position to heightfield local!
    final localSpherePos = sphereHeightfield_tmp1;
    Transform.pointToLocalFrame(hfPos, hfQuat, spherePos, localSpherePos);

    // Get the index of the data points to test against
    int iMinX = ((localSpherePos.x - radius) / w).floor() - 1;

    int iMaxX = ((localSpherePos.x + radius) / w).ceil() + 1;
    int iMinY = ((localSpherePos.y - radius) / w).floor() - 1;
    int iMaxY = ((localSpherePos.y + radius) / w).ceil() + 1;

    // Bail out if we are out of the terrain
    if (iMaxX < 0 || iMaxY < 0 || iMinX > data.length || iMinY > data[0].length) {
      return null;
    }

    // Clamp index to edges
    if (iMinX < 0) {
      iMinX = 0;
    }
    if (iMaxX < 0) {
      iMaxX = 0;
    }
    if (iMinY < 0) {
      iMinY = 0;
    }
    if (iMaxY < 0) {
      iMaxY = 0;
    }
    if (iMinX >= data.length) {
      iMinX = data.length - 1;
    }
    if (iMaxX >= data.length) {
      iMaxX = data.length - 1;
    }
    if (iMaxY >= data[0].length) {
      iMaxY = data[0].length - 1;
    }
    if (iMinY >= data[0].length) {
      iMinY = data[0].length - 1;
    }

    final minMax: number[] = [];
    hfShape.getRectMinMax(iMinX, iMinY, iMaxX, iMaxY, minMax);
    final min = minMax[0];
    final max = minMax[1];

    // Bail out if we can't touch the bounding height box
    if (localSpherePos.z - radius > max || localSpherePos.z + radius < min) {
      return null;
    }

    final result = this.result;
    for (int i = iMinX; i < iMaxX; i++) {
      for (int j = iMinY; j < iMaxY; j++) {
        final numContactsBefore = result.length;

        bool? intersecting = false;

        // Lower triangle
        hfShape.getConvexTrianglePillar(i, j, false);
        Transform.pointToWorldFrame(hfPos, hfQuat, hfShape.pillarOffset, worldPillarOffset);
        if (
          spherePos.distanceTo(worldPillarOffset) <
          hfShape.pillarConvex.boundingSphereRadius + sphereShape.boundingSphereRadius
        ) {
          intersecting = sphereConvex(
            sphereShape,
            hfShape.pillarConvex,
            spherePos,
            worldPillarOffset,
            sphereQuat,
            hfQuat,
            sphereBody,
            hfBody,
            sphereShape,
            hfShape,
            justTest
          );
        }

        if (justTest && intersecting) {
          return true;
        }

        // Upper triangle
        hfShape.getConvexTrianglePillar(i, j, true);
        Transform.pointToWorldFrame(hfPos, hfQuat, hfShape.pillarOffset, worldPillarOffset)
        if (
          spherePos.distanceTo(worldPillarOffset) <
          hfShape.pillarConvex.boundingSphereRadius + sphereShape.boundingSphereRadius
        ) {
          intersecting = sphereConvex(
            sphereShape,
            hfShape.pillarConvex,
            spherePos,
            worldPillarOffset,
            sphereQuat,
            hfQuat,
            sphereBody,
            hfBody,
            sphereShape,
            hfShape,
            justTest
          );
        }

        if (justTest && intersecting) {
          return true;
        }

        final numContacts = result.length - numContactsBefore;

        if (numContacts > 2) {
          return null;
        }
        /*
          // Skip all but 1
          for (let k = 0; k < numContacts - 1; k++) {
              result.pop();
          }
        */
      }
    }
  }

  bool? boxHeightfield(
    Box si,
    Heightfield sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool? justTest
  ]){
    si.convexPolyhedronRepresentation?.material = si.material;
    si.convexPolyhedronRepresentation?.collisionResponse = si.collisionResponse;
    return convexHeightfield(si.convexPolyhedronRepresentation!, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest);
  }

  bool? convexHeightfield(
    ConvexPolyhedron si,
    Heightfield sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool? justTest
  ]){
    final data = hfShape.data;
    final w = hfShape.elementSize;
    final radius = convexShape.boundingSphereRadius;
    final worldPillarOffset = convexHeightfield_tmp2;
    final faceList = convexHeightfield_faceList;

    // Get sphere position to heightfield local!
    final localConvexPos = convexHeightfield_tmp1;
    Transform.pointToLocalFrame(hfPos, hfQuat, convexPos, localConvexPos);

    // Get the index of the data points to test against
    int iMinX = ((localConvexPos.x - radius) / w).floor() - 1;

    int iMaxX = ((localConvexPos.x + radius) / w).ceil() + 1;
    int iMinY = ((localConvexPos.y - radius) / w).floor() - 1;
    int iMaxY = ((localConvexPos.y + radius) / w).ceil() + 1;

    // Bail out if we are out of the terrain
    if (iMaxX < 0 || iMaxY < 0 || iMinX > data.length || iMinY > data[0].length) {
      return;
    }

    // Clamp index to edges
    if (iMinX < 0) {
      iMinX = 0;
    }
    if (iMaxX < 0) {
      iMaxX = 0;
    }
    if (iMinY < 0) {
      iMinY = 0;
    }
    if (iMaxY < 0) {
      iMaxY = 0;
    }
    if (iMinX >= data.length) {
      iMinX = data.length - 1;
    }
    if (iMaxX >= data.length) {
      iMaxX = data.length - 1;
    }
    if (iMaxY >= data[0].length) {
      iMaxY = data[0].length - 1;
    }
    if (iMinY >= data[0].length) {
      iMinY = data[0].length - 1;
    }

    final minMax: number[] = [];
    hfShape.getRectMinMax(iMinX, iMinY, iMaxX, iMaxY, minMax);
    final min = minMax[0];
    final max = minMax[1];

    // Bail out if we're cant touch the bounding height box
    if (localConvexPos.z - radius > max || localConvexPos.z + radius < min) {
      return;
    }

    for (int i = iMinX; i < iMaxX; i++) {
      for (int j = iMinY; j < iMaxY; j++) {
        bool? intersecting = false;

        // Lower triangle
        hfShape.getConvexTrianglePillar(i, j, false)
        Transform.pointToWorldFrame(hfPos, hfQuat, hfShape.pillarOffset, worldPillarOffset)
        if (
          convexPos.distanceTo(worldPillarOffset) <
          hfShape.pillarConvex.boundingSphereRadius + convexShape.boundingSphereRadius
        ) {
          intersecting = this.convexConvex(
            convexShape,
            hfShape.pillarConvex,
            convexPos,
            worldPillarOffset,
            convexQuat,
            hfQuat,
            convexBody,
            hfBody,
            null,
            null,
            justTest,
            faceList,
            null
          );
        }

        if (justTest && intersecting) {
          return true;
        }

        // Upper triangle
        hfShape.getConvexTrianglePillar(i, j, true);
        Transform.pointToWorldFrame(hfPos, hfQuat, hfShape.pillarOffset, worldPillarOffset);
        if (
          convexPos.distanceTo(worldPillarOffset) <
          hfShape.pillarConvex.boundingSphereRadius + convexShape.boundingSphereRadius
        ) {
          intersecting = this.convexConvex(
            convexShape,
            hfShape.pillarConvex,
            convexPos,
            worldPillarOffset,
            convexQuat,
            hfQuat,
            convexBody,
            hfBody,
            null,
            null,
            justTest,
            faceList,
            null
          );
        }

        if (justTest && intersecting) {
          return true;
        }
      }
    }
  }

  bool? sphereParticle(
    Sphere sj,
    Particle si,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool? justTest
  ]){
    // The normal is the unit vector from sphere center to particle center
    final normal = particleSphere_normal;
    normal.set(0, 0, 1);
    xi.vsub(xj, normal);
    final lengthSquared = normal.lengthSquared();

    if (lengthSquared <= sj.radius * sj.radius) {
      if (justTest) {
        return true;
      }
      final r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
      normal.normalize();
      r.rj.copy(normal);
      r.rj.scale(sj.radius, r.rj);
      r.ni.copy(normal); // Contact normal
      r.ni.negate(r.ni);
      r.ri.set(0, 0, 0); // Center of particle
      this.result.add(r);
      this.createFrictionEquationsFromContact(r, this.frictionResult);
    }
  }

  bool? planeParticle(
    Plane sj,
    Particle si,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool? justTest
  ]){
    final normal = particlePlane_normal;
    normal.set(0, 0, 1);
    bj.quaternion.vmult(normal, normal); // Turn normal according to plane orientation
    final relpos = particlePlane_relpos;
    xi.vsub(bj.position, relpos);
    final dot = normal.dot(relpos);
    if (dot <= 0.0) {
      if (justTest) {
        return true;
      }

      final r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
      r.ni.copy(normal); // Contact normal is the plane normal
      r.ni.negate(r.ni);
      r.ri.set(0, 0, 0); // Center of particle

      // Get particle position projected on plane
      final projected = particlePlane_projected;
      normal.scale(normal.dot(xi), projected);
      xi.vsub(projected, projected);
      //projected.vadd(bj.position,projected);

      // rj is now the projected world position minus plane position
      r.rj.copy(projected);
      this.result.add(r);
      this.createFrictionEquationsFromContact(r, this.frictionResult);
    }
  }

  bool? boxParticle(
    Box si,
    Particle sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool? justTest
  ]){
    si.convexPolyhedronRepresentation.material = si.material;
    si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
    return this.convexParticle(si.convexPolyhedronRepresentation, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest);
  }

  bool? convexParticle(
    ConvexPolyhedron sj,
    Particle si,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool? justTest
  ]){
    int penetratedFaceIndex = -1;
    final penetratedFaceNormal = convexParticle_penetratedFaceNormal;
    final worldPenetrationVec = convexParticle_worldPenetrationVec;
    let minPenetration = null;
    int numDetectedFaces = 0;

    // Convert particle position xi to local coords in the convex
    final local = convexParticle_local;
    local.copy(xi);
    local.vsub(xj, local); // Convert position to relative the convex origin
    qj.conjugate(cqj);
    cqj.vmult(local, local);

    if (sj.pointIsInside(local)) {
      if (sj.worldVerticesNeedsUpdate) {
        sj.computeWorldVertices(xj, qj);
      }
      if (sj.worldFaceNormalsNeedsUpdate) {
        sj.computeWorldFaceNormals(qj);
      }

      // For each world polygon in the polyhedra
      for (int i = 0, nfaces = sj.faces.length; i != nfaces; i++) {
        // Construct world face vertices
        final verts = [sj.worldVertices[sj.faces[i][0]]];
        final normal = sj.worldFaceNormals[i];

        // Check how much the particle penetrates the polygon plane.
        xi.vsub(verts[0], convexParticle_vertexToParticle);
        final penetration = -normal.dot(convexParticle_vertexToParticle);
        if (minPenetration == null || penetration.abs() < minPenetration.abs()) {
          if (justTest) {
            return true;
          }

          minPenetration = penetration;
          penetratedFaceIndex = i;
          penetratedFaceNormal.copy(normal);
          numDetectedFaces++;
        }
      }

      if (penetratedFaceIndex != -1) {
        // Setup contact
        final r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
        penetratedFaceNormal.scale(minPenetration!, worldPenetrationVec);

        // rj is the particle position projected to the face
        worldPenetrationVec.vadd(xi, worldPenetrationVec);
        worldPenetrationVec.vsub(xj, worldPenetrationVec);
        r.rj.copy(worldPenetrationVec);
        //final projectedToFace = xi.vsub(xj).vadd(worldPenetrationVec);
        //projectedToFace.copy(r.rj);

        //qj.vmult(r.rj,r.rj);
        penetratedFaceNormal.negate(r.ni); // Contact normal
        r.ri.set(0, 0, 0); // Center of particle

        final ri = r.ri;
        final rj = r.rj;

        // Make relative to bodies
        ri.vadd(xi, ri);
        ri.vsub(bi.position, ri);
        rj.vadd(xj, rj);
        rj.vsub(bj.position, rj);

        this.result.add(r);
        this.createFrictionEquationsFromContact(r, this.frictionResult);
      } else {
        print('Point found inside convex, but did not find penetrating face!');
      }
    }
  }

  bool? heightfieldCylinder(
    Heightfield si,
    Cylinder sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool? justTest
  ]){
    return this.convexHeightfield(
      si as ConvexPolyhedron,
      sj,
      xi,
      xj,
      qi,
      qj,
      bi,
      bj,
      rsi,
      rsj,
      justTest
    );
  }

  bool?  particleCylinder(
    Particle si,
    Cylinder sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool? justTest
  ]){
    return this.convexParticle(sj as ConvexPolyhedron, si, xj, xi, qj, qi, bj, bi, rsi, rsj, justTest);
  }

  bool?  sphereTrimesh(
    Sphere si,
    Trimesh sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool? justTest
  ]){
    final edgeVertexA = sphereTrimesh_edgeVertexA;
    final edgeVertexB = sphereTrimesh_edgeVertexB;
    final edgeVector = sphereTrimesh_edgeVector;
    final edgeVectorUnit = sphereTrimesh_edgeVectorUnit;
    final localSpherePos = sphereTrimesh_localSpherePos;
    final tmp = sphereTrimesh_tmp;
    final localSphereAABB = sphereTrimesh_localSphereAABB;
    final v2 = sphereTrimesh_v2;
    final relpos = sphereTrimesh_relpos;
    final triangles = sphereTrimesh_triangles;

    // Convert sphere position to local in the trimesh
    Transform.pointToLocalFrame(trimeshPos, trimeshQuat, spherePos, localSpherePos);

    // Get the aabb of the sphere locally in the trimesh
    final sphereRadius = sphereShape.radius;
    localSphereAABB.lowerBound.set(
      localSpherePos.x - sphereRadius,
      localSpherePos.y - sphereRadius,
      localSpherePos.z - sphereRadius
    );
    localSphereAABB.upperBound.set(
      localSpherePos.x + sphereRadius,
      localSpherePos.y + sphereRadius,
      localSpherePos.z + sphereRadius
    );

    trimeshShape.getTrianglesInAABB(localSphereAABB, triangles);
    //for (let i = 0; i < trimeshShape.indices.length / 3; i++) triangles.push(i); // All

    // Vertices
    final v = sphereTrimesh_v;
    final radiusSquared = sphereShape.radius * sphereShape.radius;
    for (int i = 0; i < triangles.length; i++) {
      for (int j = 0; j < 3; j++) {
        trimeshShape.getVertex(trimeshShape.indices[triangles[i] * 3 + j], v);

        // Check vertex overlap in sphere
        v.vsub(localSpherePos, relpos);

        if (relpos.lengthSquared() <= radiusSquared) {
          // Safe up
          v2.copy(v);
          Transform.pointToWorldFrame(trimeshPos, trimeshQuat, v2, v);

          v.vsub(spherePos, relpos);

          if (justTest) {
            return true;
          }

          ContactEquation r = this.createContactEquation(sphereBody, trimeshBody, sphereShape, trimeshShape, rsi, rsj);
          r.ni.copy(relpos);
          r.ni.normalize();

          // ri is the vector from sphere center to the sphere surface
          r.ri.copy(r.ni);
          r.ri.scale(sphereShape.radius, r.ri);
          r.ri.vadd(spherePos, r.ri);
          r.ri.vsub(sphereBody.position, r.ri);

          r.rj.copy(v);
          r.rj.vsub(trimeshBody.position, r.rj);

          // Store result
          this.result.add(r);
          this.createFrictionEquationsFromContact(r, this.frictionResult);
        }
      }
    }

    // Check all edges
    for (int i = 0; i < triangles.length; i++) {
      for (int j = 0; j < 3; j++) {
        trimeshShape.getVertex(trimeshShape.indices[triangles[i] * 3 + j], edgeVertexA);
        trimeshShape.getVertex(trimeshShape.indices[triangles[i] * 3 + ((j + 1) % 3)], edgeVertexB);
        edgeVertexB.vsub(edgeVertexA, edgeVector);

        // Project sphere position to the edge
        localSpherePos.vsub(edgeVertexB, tmp);
        final positionAlongEdgeB = tmp.dot(edgeVector);

        localSpherePos.vsub(edgeVertexA, tmp);
        double positionAlongEdgeA = tmp.dot(edgeVector);

        if (positionAlongEdgeA > 0 && positionAlongEdgeB < 0) {
          // Now check the orthogonal distance from edge to sphere center
          localSpherePos.vsub(edgeVertexA, tmp);

          edgeVectorUnit.copy(edgeVector);
          edgeVectorUnit.normalize();
          positionAlongEdgeA = tmp.dot(edgeVectorUnit);

          edgeVectorUnit.scale(positionAlongEdgeA, tmp);
          tmp.vadd(edgeVertexA, tmp);

          // tmp is now the sphere center position projected to the edge, defined locally in the trimesh frame
          final dist = tmp.distanceTo(localSpherePos);
          if (dist < sphereShape.radius) {
            if (justTest) {
              return true;
            }

            final r = this.createContactEquation(sphereBody, trimeshBody, sphereShape, trimeshShape, rsi, rsj);

            tmp.vsub(localSpherePos, r.ni);
            r.ni.normalize();
            r.ni.scale(sphereShape.radius, r.ri);
            r.ri.vadd(spherePos, r.ri);
            r.ri.vsub(sphereBody.position, r.ri);

            Transform.pointToWorldFrame(trimeshPos, trimeshQuat, tmp, tmp);
            tmp.vsub(trimeshBody.position, r.rj);

            Transform.vectorToWorldFrame(trimeshQuat, r.ni, r.ni);
            Transform.vectorToWorldFrame(trimeshQuat, r.ri, r.ri);

            this.result.add(r);
            this.createFrictionEquationsFromContact(r, this.frictionResult);
          }
        }
      }
    }

    // Triangle faces
    final va = sphereTrimesh_va;
    final vb = sphereTrimesh_vb;
    final vc = sphereTrimesh_vc;
    final normal = sphereTrimesh_normal;
    for (int i = 0, N = triangles.length; i != N; i++) {
      trimeshShape.getTriangleVertices(triangles[i], va, vb, vc);
      trimeshShape.getNormal(triangles[i], normal);
      localSpherePos.vsub(va, tmp);
      double dist = tmp.dot(normal);
      normal.scale(dist, tmp);
      localSpherePos.vsub(tmp, tmp);

      // tmp is now the sphere position projected to the triangle plane
      dist = tmp.distanceTo(localSpherePos);
      if (Ray.pointInTriangle(tmp, va, vb, vc) && dist < sphereShape.radius) {
        if (justTest) {
          return true;
        }
        ContactEquation r = this.createContactEquation(sphereBody, trimeshBody, sphereShape, trimeshShape, rsi, rsj);

        tmp.vsub(localSpherePos, r.ni);
        r.ni.normalize();
        r.ni.scale(sphereShape.radius, r.ri);
        r.ri.vadd(spherePos, r.ri);
        r.ri.vsub(sphereBody.position, r.ri);

        Transform.pointToWorldFrame(trimeshPos, trimeshQuat, tmp, tmp);
        tmp.vsub(trimeshBody.position, r.rj);

        Transform.vectorToWorldFrame(trimeshQuat, r.ni, r.ni);
        Transform.vectorToWorldFrame(trimeshQuat, r.ri, r.ri);

        this.result.add(r);
        this.createFrictionEquationsFromContact(r, this.frictionResult);
      }
    }

    triangles.length = 0;
  }

  bool? planeTrimesh(
    Plane si,
    Trimesh sj,
    Vec3 xi,
    Vec3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool? justTest
  ]){
    // Make contacts!
    final v = Vec3();

    final normal = planeTrimesh_normal;
    normal.set(0, 0, 1);
    planeQuat.vmult(normal, normal) ;// Turn normal according to plane

    for (int i = 0; i < trimeshShape.vertices.length / 3; i++) {
      // Get world vertex from trimesh
      trimeshShape.getVertex(i, v);

      // Safe up
      final v2 = Vec3();
      v2.copy(v);
      Transform.pointToWorldFrame(trimeshPos, trimeshQuat, v2, v);

      // Check plane side
      final relpos = planeTrimesh_relpos;
      v.vsub(planePos, relpos);
      final dot = normal.dot(relpos);

      if (dot <= 0.0) {
        if (justTest) {
          return true;
        }

        final r = this.createContactEquation(planeBody, trimeshBody, planeShape, trimeshShape, rsi, rsj);

        r.ni.copy(normal); // Contact normal is the plane normal

        // Get vertex position projected on plane
        final projected = planeTrimesh_projected;
        normal.scale(relpos.dot(normal), projected);
        v.vsub(projected, projected);

        // ri is the projected world position minus plane position
        r.ri.copy(projected);
        r.ri.vsub(planeBody.position, r.ri);

        r.rj.copy(v);
        r.rj.vsub(trimeshBody.position, r.rj);

        // Store result
        this.result.add(r);
        this.createFrictionEquationsFromContact(r, this.frictionResult);
      }
    }
  }

  // convexTrimesh(
  //   si: ConvexPolyhedron, sj: Trimesh, xi: Vec3, xj: Vec3, qi: Quaternion, qj: Quaternion,
  //   bi: Body, bj: Body, rsi?: Shape | null, rsj?: Shape | null,
  //   faceListA?: number[] | null, faceListB?: number[] | null,
  // ) {
  //   final sepAxis = convexConvex_sepAxis;

  //   if(xi.distanceTo(xj) > si.boundingSphereRadius + sj.boundingSphereRadius){
  //       return;
  //   }

  //   // Construct a temp hull for each triangle
  //   final hullB = ConvexPolyhedron();

  //   hullB.faces = [[0,1,2]];
  //   final va = Vec3();
  //   final vb = Vec3();
  //   final vc = Vec3();
  //   hullB.vertices = [
  //       va,
  //       vb,
  //       vc
  //   ];

  //   for (let i = 0; i < sj.indices.length / 3; i++) {

  //       final triangleNormal = Vec3();
  //       sj.getNormal(i, triangleNormal);
  //       hullB.faceNormals = [triangleNormal];

  //       sj.getTriangleVertices(i, va, vb, vc);

  //       let d = si.testSepAxis(triangleNormal, hullB, xi, qi, xj, qj);
  //       if(!d){
  //           triangleNormal.scale(-1, triangleNormal);
  //           d = si.testSepAxis(triangleNormal, hullB, xi, qi, xj, qj);

  //           if(!d){
  //               continue;
  //           }
  //       }

  //       final res: ConvexPolyhedronContactPoint[] = [];
  //       final q = convexConvex_q;
  //       si.clipAgainstHull(xi,qi,hullB,xj,qj,triangleNormal,-100,100,res);
  //       for(let j = 0; j !== res.length; j++){
  //           final r = this.createContactEquation(bi,bj,si,sj,rsi,rsj),
  //               ri = r.ri,
  //               rj = r.rj;
  //           r.ni.copy(triangleNormal);
  //           r.ni.negate(r.ni);
  //           res[j].normal.negate(q);
  //           q.mult(res[j].depth, q);
  //           res[j].point.vadd(q, ri);
  //           rj.copy(res[j].point);

  //           // Contact points are in world coordinates. Transform back to relative
  //           ri.vsub(xi,ri);
  //           rj.vsub(xj,rj);

  //           // Make relative to bodies
  //           ri.vadd(xi, ri);
  //           ri.vsub(bi.position, ri);
  //           rj.vadd(xj, rj);
  //           rj.vsub(bj.position, rj);

  //           result.push(r);
  //       }
  //   }
  // }
}

final averageNormal = Vec3();
final averageContactPointA = Vec3();
final averageContactPointB = Vec3();

final tmpVec1 = Vec3();
final tmpVec2 = Vec3();
final tmpQuat1 = Quaternion();
final tmpQuat2 = Quaternion();

int numWarnings = 0;
final maxWarnings = 10;

function warn(msg: string): void {
  if (numWarnings > maxWarnings) {
    return;
  }
  numWarnings++;
  console.warn(msg);
}

final planeTrimesh_normal = Vec3();
final planeTrimesh_relpos = Vec3();
final planeTrimesh_projected = Vec3();

final sphereTrimesh_normal = Vec3();
final sphereTrimesh_relpos = Vec3();
final sphereTrimesh_projected = Vec3();
final sphereTrimesh_v = Vec3();
final sphereTrimesh_v2 = Vec3();
final sphereTrimesh_edgeVertexA = Vec3();
final sphereTrimesh_edgeVertexB = Vec3();
final sphereTrimesh_edgeVector = Vec3();
final sphereTrimesh_edgeVectorUnit = Vec3();
final sphereTrimesh_localSpherePos = Vec3();
final sphereTrimesh_tmp = Vec3();
final sphereTrimesh_va = Vec3();
final sphereTrimesh_vb = Vec3();
final sphereTrimesh_vc = Vec3();
final sphereTrimesh_localSphereAABB = AABB();
final sphereTrimesh_triangles: number[] = [];

final point_on_plane_to_sphere = Vec3();
final plane_to_sphere_ortho = Vec3();

// See http://bulletphysics.com/Bullet/BulletFull/SphereTriangleDetector_8cpp_source.html
final pointInPolygon_edge = Vec3();
final pointInPolygon_edge_x_normal = Vec3();
final pointInPolygon_vtp = Vec3();
function pointInPolygon(verts: Vec3[], normal: Vec3, p: Vec3): boolean {
  let positiveResult = null;
  final N = verts.length;
  for (int i = 0; i != N; i++) {
    final v = verts[i];

    // Get edge to the next vertex
    final edge = pointInPolygon_edge;
    verts[(i + 1) % N].vsub(v, edge);

    // Get cross product between polygon normal and the edge
    final edge_x_normal = pointInPolygon_edge_x_normal;
    //final edge_x_normal = Vec3();
    edge.cross(normal, edge_x_normal);

    // Get vector between point and current vertex
    final vertex_to_p = pointInPolygon_vtp;
    p.vsub(v, vertex_to_p);

    // This dot product determines which side of the edge the point is
    final r = edge_x_normal.dot(vertex_to_p);

    // If all such dot products have same sign, we are inside the polygon.
    if (positiveResult == null || (r > 0 && positiveResult == true) || (r <= 0 && positiveResult == false)) {
      if (positiveResult == null) {
        positiveResult = r > 0;
      }
      continue;
    } else {
      return false; // Encountered some other sign. Exit.
    }
  }

  // If we got here, all dot products were of the same sign.
  return true;
}

final box_to_sphere = Vec3();
final sphereBox_ns = Vec3();
final sphereBox_ns1 = Vec3();
final sphereBox_ns2 = Vec3();
final sphereBox_sides = [Vec3(), Vec3(), Vec3(), Vec3(), Vec3(), Vec3()];
final sphereBox_sphere_to_corner = Vec3();
final sphereBox_side_ns = Vec3();
final sphereBox_side_ns1 = Vec3();
final sphereBox_side_ns2 = Vec3();

final convex_to_sphere = Vec3();
final sphereConvex_edge = Vec3();
final sphereConvex_edgeUnit = Vec3();
final sphereConvex_sphereToCorner = Vec3();
final sphereConvex_worldCorner = Vec3();
final sphereConvex_worldNormal = Vec3();
final sphereConvex_worldPoint = Vec3();
final sphereConvex_worldSpherePointClosestToPlane = Vec3();
final sphereConvex_penetrationVec = Vec3();
final sphereConvex_sphereToWorldPoint = Vec3();

final planeBox_normal = Vec3();
final plane_to_corner = Vec3();

final planeConvex_v = Vec3();
final planeConvex_normal = Vec3();
final planeConvex_relpos = Vec3();
final planeConvex_projected = Vec3();

final convexConvex_sepAxis = Vec3();
final convexConvex_q = Vec3();

final particlePlane_normal = Vec3();
final particlePlane_relpos = Vec3();
final particlePlane_projected = Vec3();

final particleSphere_normal = Vec3();

// WIP
final cqj = Quaternion();
final convexParticle_local = Vec3();
final convexParticle_normal = Vec3();
final convexParticle_penetratedFaceNormal = Vec3();
final convexParticle_vertexToParticle = Vec3();
final convexParticle_worldPenetrationVec = Vec3();

final convexHeightfield_tmp1 = Vec3();
final convexHeightfield_tmp2 = Vec3();
final convexHeightfield_faceList = [0];

final sphereHeightfield_tmp1 = Vec3();
final sphereHeightfield_tmp2 = Vec3();
