import 'dart:math' as math;
import '../shapes/shape.dart';
import '../math/vec3.dart';
import '../math/transform.dart';
import '../math/quaternion.dart';
import '../objects/body.dart';
import '../collision/aabb.dart';
import '../collision/ray_class.dart';
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
  particleTrimesh,
  sphereTrimesh,
  planeTrimesh,
  boxTrimesh,
  cylinderTrimesh,
  convexTrimesh
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
    else if(type == CollisionType.particleTrimesh) {
      return particleTrimesh;
    }
    else if(type == CollisionType.sphereTrimesh) {
      return sphereTrimesh;
    }
    else if(type == CollisionType.planeTrimesh) {
      return planeTrimesh;
    }
    else if(type == CollisionType.boxTrimesh) {
      return boxTrimesh;
    }
    // else if(type == CollisionType.cylinderTrimesh) {
    //   return cylinderTrimesh;
    // }
    else if(type == CollisionType.convexTrimesh) {
      return convexTrimesh;
    }
  }

  Narrowphase(this.world) {
    currentContactMaterial = world.defaultContactMaterial;
  }

  final _averageNormal = Vec3();
  final _averageContactPointA = Vec3();
  final _averageContactPointB = Vec3();

  final _tmpVec1 = Vec3();
  final _tmpVec2 = Vec3();
  final _tmpQuat1 = Quaternion();
  final _tmpQuat2 = Quaternion();

  final _planeTrimeshNormal = Vec3();
  final _planeTrimeshRelpos = Vec3();
  final _planeTrimeshProjected = Vec3();

  final _sphereTrimeshNormal = Vec3();
  final _sphereTrimeshRelpos = Vec3();
  //final _sphereTrimeshProjected = Vec3();
  final _sphereTrimeshV = Vec3();
  final _sphereTrimeshV2 = Vec3();
  final _sphereTrimeshEdgeVertexA = Vec3();
  final _sphereTrimeshEdgeVertexB = Vec3();
  final _sphereTrimeshEdgeVector = Vec3();
  final _sphereTrimeshEdgeVectorUnit = Vec3();
  final _sphereTrimeshLocalSpherePos = Vec3();
  final _sphereTrimeshTmp = Vec3();
  final _sphereTrimeshVa = Vec3();
  final _sphereTrimeshVb = Vec3();
  final _sphereTrimeshVc = Vec3();
  final _sphereTrimeshLocalSphereAABB = AABB();
  final List<int> _sphereTrimeshTriangles = [];

  final _pointOnPlaneToSphere = Vec3();
  final _planeToSphereOrtho = Vec3();

  // See http://bulletphysics.com/Bullet/BulletFull/SphereTriangleDetector_8cpp_source.html
  final _pointInPolygonEdge = Vec3();
  final _pointInPolygonEdgeXNormal = Vec3();
  final _pointInPolygonVtp = Vec3();


  final _boxToSphere = Vec3();
  final _sphereBoxNs = Vec3();
  final _sphereBoxNs1 = Vec3();
  final _sphereBoxNs2 = Vec3();
  final List<Vec3> sphereBoxSides = [Vec3(), Vec3(), Vec3(), Vec3(), Vec3(), Vec3()];
  final _sphereBoxSphereToCorner = Vec3();
  final _sphereBoxSideNs = Vec3();
  final _sphereBoxSideNs1 = Vec3();
  final _sphereBoxSideNs2 = Vec3();

  final _convexToSphere = Vec3();
  final _sphereConvexEdge = Vec3();
  final _sphereConvexEdgeUnit = Vec3();
  final _sphereConvexSphereToCorner = Vec3();
  final _sphereConvexWorldCorner = Vec3();
  final _sphereConvexWorldNormal = Vec3();
  final _sphereConvexWorldPoint = Vec3();
  final _sphereConvexWorldSpherePointClosestToPlane = Vec3();
  final _sphereConvexPenetrationVec = Vec3();
  final _sphereConvexSphereToWorldPoint = Vec3();

  //final _planeBoxNormal = Vec3();
  //final _planeToCorner = Vec3();

  final _planeConvexV = Vec3();
  final _planeConvexNormal = Vec3();
  final _planeConvexRelpos = Vec3();
  final _planeConvexProjected = Vec3();

  final _convexConvexSepAxis = Vec3();
  final _convexConvexQ = Vec3();

  final _particlePlaneNormal = Vec3();
  final _particlePlaneRelpos = Vec3();
  final _particlePlaneProjected = Vec3();

  final _particleSphereNormal = Vec3();

  // WIP
  
  final _convexParticleLocal = Vec3();
  //final _convexParticleNormal = Vec3();
  final _convexParticlePenetratedFaceNormal = Vec3();
  
  final _convexParticleWorldPenetrationVec = Vec3();

  final _convexHeightfieldTmp1 = Vec3();
  final _convexHeightfieldTmp2 = Vec3();
  List<int> convexHeightfieldFaceList = [0];

  final _sphereHeightfieldTmp1 = Vec3();
  final _sphereHeightfieldTmp2 = Vec3();

  CollisionType? getCollisionType(ShapeType a, ShapeType b){
    String n1 = a.name+b.name;
    String n2 = b.name+a.name;
    for(int i = 0; i < CollisionType.values.length; i++){
      if(n1 == CollisionType.values[i].name.toLowerCase()){
        return CollisionType.values[i];
      }
      else if(n2 == CollisionType.values[i].name.toLowerCase()){
        return CollisionType.values[i];
      }
    }

    return null;
  }

  /// Make a contact object, by using the internal pool or creating a one.
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
      final c1 = pool.isNotEmpty ? pool.removeLast() : FrictionEquation(bodyA, bodyB, mug * reducedMass);
      final c2 = pool.isNotEmpty ? pool.removeLast() : FrictionEquation(bodyA, bodyB, mug * reducedMass);

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

      c1.enabled = contactEquation.enabled;
      c2.enabled = contactEquation.enabled;

      outArray.addAll([c1, c2]);

      return true;
    }

    return false;
  }

  /// Take the average N latest contact point on the plane.
  void createFrictionFromAverage(int numContacts) {
    // The last contactEquation
    ContactEquation c = result[result.length - 1];

    // Create the result: two "average" friction equations
    if (!createFrictionEquationsFromContact(c,frictionResult) || numContacts == 1) {
      return;
    }

    final f1 = frictionResult[frictionResult.length - 2];
    final f2 = frictionResult[frictionResult.length - 1];

    _averageNormal.setZero();
    _averageContactPointA.setZero();
    _averageContactPointB.setZero();

    final bodyA = c.bi;
    //final bodyB = c.bj;
    for (int i = 0; i != numContacts; i++) {
      c = result[result.length - 1 - i];
      if (c.bi != bodyA) {
        _averageNormal.vadd(c.ni, _averageNormal);
        _averageContactPointA.vadd(c.ri, _averageContactPointA);
        _averageContactPointB.vadd(c.rj, _averageContactPointB);
      } else {
        _averageNormal.vsub(c.ni, _averageNormal);
        _averageContactPointA.vadd(c.rj, _averageContactPointA);
        _averageContactPointB.vadd(c.ri, _averageContactPointB);
      }
    }

    final invNumContacts = 1 / numContacts;
    _averageContactPointA.scale(invNumContacts, f1.ri);
    _averageContactPointB.scale(invNumContacts, f1.rj);
    f2.ri.copy(f1.ri); // Should be the same
    f2.rj.copy(f1.rj);
    _averageNormal.normalize();
    _averageNormal.tangents(f1.t, f2.t);
  }

  /// Generate all contacts between a list of body pairs
  /// @param p1 Array of body indices
  /// @param p2 Array of body indices
  /// @param result Array to store generated contacts
  /// @param oldcontacts Optional. Array of reusable contact objects
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

    final qi = _tmpQuat1;
    final qj = _tmpQuat2;
    final xi = _tmpVec1;
    final xj = _tmpVec2;

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
   
          if (!(si.collisionFilterMask & sj.collisionFilterGroup != 0 && sj.collisionFilterMask & si.collisionFilterGroup != 0)) {
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
          final resolverIndex = getCollisionType(si.type, sj.type);//(si.type | sj.type) as CollisionType;
          final resolver = resolverIndex == null? null:this[resolverIndex];
          if (resolver != null) {
            bool retval = false;

            // TO DO: investigate why sphereParticle and convexParticle
            // resolvers expect si and sj shapes to be in reverse order
            // (i.e. larger integer value type first instead of smaller first)
            if (si.type.index < sj.type.index) {
              retval = resolver.call(si, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest);
            } else {
              retval = resolver.call(sj, si, xj, xi, qj, qi, bj, bi, si, sj, justTest);
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

  bool sphereSphere(
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

    createFrictionEquationsFromContact(contactEq, frictionResult);

    return false;
  }

  bool spherePlane(
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
    xi.vsub(xj, _pointOnPlaneToSphere);
    r.ni.scale(r.ni.dot(_pointOnPlaneToSphere), _planeToSphereOrtho);
    _pointOnPlaneToSphere.vsub(_planeToSphereOrtho, r.rj); // The sphere position projected to plane

    if (-_pointOnPlaneToSphere.dot(r.ni) <= si.radius) {
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
      createFrictionEquationsFromContact(r, frictionResult);
    }

    return false;
  }

  bool boxBox(
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
    si.convexPolyhedronRepresentation.material = si.material;
    sj.convexPolyhedronRepresentation.material = sj.material;
    si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
    sj.convexPolyhedronRepresentation.collisionResponse = sj.collisionResponse;
    return convexConvex(si.convexPolyhedronRepresentation,sj.convexPolyhedronRepresentation,xi,xj,qi,qj,bi,bj,si,sj,justTest);
  }
  
  bool boxConvex(
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
    si.convexPolyhedronRepresentation.material = si.material;
    si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
    return convexConvex(si.convexPolyhedronRepresentation, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest);
  }

  bool boxParticle(
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
      bool justTest = false
  ]){
    si.convexPolyhedronRepresentation.material = si.material;
    si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
    return convexParticle(si.convexPolyhedronRepresentation, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest);
  }

  bool sphereBox(
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
    final sides = sphereBoxSides;
    xi.vsub(xj, _boxToSphere);
    sj.getSideNormals(sides, qj);
    final R = si.radius;
    //final penetrating_sides = [];

    // Check side (plane) intersections
    bool found = false;

    // Store the resulting side penetration info
    final sideNs = _sphereBoxSideNs;
    final sideNs1 = _sphereBoxSideNs1;
    final sideNs2 = _sphereBoxSideNs2;

    double? sideH;
    double sidePenetrations = 0;
    double sideDot1 = 0;
    double sideDot2 = 0;
    double? sideDistance;
    for (int idx = 0, nsides = sides.length; idx != nsides && found == false; idx++) {
      // Get the plane side normal (ns)
      final ns = _sphereBoxNs;
      ns.copy(sides[idx]);

      final h = ns.length();
      ns.normalize();

      // The normal/distance dot product tells which side of the plane we are
      final dot = _boxToSphere.dot(ns);

      if (dot < h + R && dot > 0) {
        // Intersects plane. Now check the other two dimensions
        final ns1 = _sphereBoxNs1;
        final ns2 = _sphereBoxNs2;
        ns1.copy(sides[(idx + 1) % 3]);
        ns2.copy(sides[(idx + 2) % 3]);
        final h1 = ns1.length();
        final h2 = ns2.length();
        ns1.normalize();
        ns2.normalize();
        final dot1 = _boxToSphere.dot(ns1);
        final dot2 = _boxToSphere.dot(ns2);
        if (dot1 < h1 && dot1 > -h1 && dot2 < h2 && dot2 > -h2) {
          final dist = (dot - h - R).abs();
          if (sideDistance == null || dist < sideDistance) {
            sideDistance = dist;
            sideDot1 = dot1;
            sideDot2 = dot2;
            sideH = h;
            sideNs.copy(ns);
            sideNs1.copy(ns1);
            sideNs2.copy(ns2);
            sidePenetrations++;

            if (justTest) {
              return true;
            }
          }
        }
      }
    }
    if (sidePenetrations != 0) {
      found = true;
      final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
      sideNs.scale(-R, r.ri); // Sphere r
      r.ni.copy(sideNs);
      r.ni.negate(r.ni); // Normal should be out of sphere
      sideNs.scale(sideH!, sideNs);
      sideNs1.scale(sideDot1, sideNs1);
      sideNs.vadd(sideNs1, sideNs);
      sideNs2.scale(sideDot2, sideNs2);
      sideNs.vadd(sideNs2, r.rj);

      // Make relative to bodies
      r.ri.vadd(xi, r.ri);
      r.ri.vsub(bi.position, r.ri);
      r.rj.vadd(xj, r.rj);
      r.rj.vsub(bj.position, r.rj);

      result.add(r);
      createFrictionEquationsFromContact(r, frictionResult);
    }

    // Check corners
    dynamic rj = v3pool.get();
    final sphereToCorner = _sphereBoxSphereToCorner;
    for (int j = 0; j != 2 && !found; j++) {
      for (int k = 0; k != 2 && !found; k++) {
        for (int l = 0; l != 2 && !found; l++) {
          rj.set(0.0, 0.0, 0.0);
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
          xj.vadd(rj, sphereToCorner);
          sphereToCorner.vsub(xi, sphereToCorner);

          if (sphereToCorner.lengthSquared() < R * R) {
            if (justTest) {
              return true;
            }
            found = true;
            final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
            r.ri.copy(sphereToCorner);
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
    v3pool.release([rj]);
    rj = null;

    // Check edges
    final edgeTangent = v3pool.get();
    final edgeCenter = v3pool.get();
    final r = v3pool.get(); // r = edge center to sphere center
    final orthogonal = v3pool.get();
    final dist = v3pool.get();
    final nSides = sides.length;
    for (int j = 0; j != nSides && !found; j++) {
      for (int k = 0; k != nSides && !found; k++) {
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
    v3pool.release([edgeTangent, edgeCenter, r, orthogonal, dist]);

    return false;
  }

  bool planeBox(
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
      bool justTest = false
  ]){
    sj.convexPolyhedronRepresentation.material = sj.material;
    sj.convexPolyhedronRepresentation.collisionResponse = sj.collisionResponse;
    sj.convexPolyhedronRepresentation.id = sj.id;
    return planeConvex(si, sj.convexPolyhedronRepresentation, xi, xj, qi, qj, bi, bj, si, sj, justTest);
  }

  bool convexConvex(
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
      bool justTest = false,
      List<int>? faceListA,
      List<int>? faceListB
  ]){
    final sepAxis = _convexConvexSepAxis;

    if (xi.distanceTo(xj) > si.boundingSphereRadius + sj.boundingSphereRadius) {
      return false;
    }

    if (si.findSeparatingAxis(sj, xi, qi, xj, qj, sepAxis, faceListA, faceListB)) {
      final List<ConvexPolyhedronContactPoint> res = [];
      final q = _convexConvexQ;
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

    return false;
  }

  bool sphereConvex(
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
    xi.vsub(xj, _convexToSphere);
    final normals = sj.faceNormals;
    final faces = sj.faces;
    final verts = sj.vertices;
    final R = si.radius;
    bool found = false;

    // Check corners
    for (int i = 0; i != verts.length; i++) {
      final v = verts[i];

      // World position of corner
      final worldCorner = _sphereConvexWorldCorner;
      qj.vmult(v, worldCorner);
      xj.vadd(worldCorner, worldCorner);
      final sphereToCorner = _sphereConvexSphereToCorner;
      worldCorner.vsub(xi, sphereToCorner);
      if (sphereToCorner.lengthSquared() < R * R) {
        if (justTest) {
          return true;
        }
        found = true;
        final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
        r.ri.copy(sphereToCorner);
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
        return false;
      }
    }

    // Check side (plane) intersections
    for (int i = 0, nfaces = faces.length; i != nfaces && found == false; i++) {
      final normal = normals[i]!;
      final face = faces[i];

      // Get world-transformed normal of the face
      final worldNormal = _sphereConvexWorldNormal;
      qj.vmult(normal, worldNormal);

      // Get a world vertex from the face
      final worldPoint = _sphereConvexWorldPoint;
      qj.vmult(verts[face[0]], worldPoint);
      worldPoint.vadd(xj, worldPoint);

      // Get a point on the sphere, closest to the face normal
      final worldSpherePointClosestToPlane = _sphereConvexWorldSpherePointClosestToPlane;
      worldNormal.scale(-R, worldSpherePointClosestToPlane);
      xi.vadd(worldSpherePointClosestToPlane, worldSpherePointClosestToPlane);

      // Vector from a face point to the closest point on the sphere
      final penetrationVec = _sphereConvexPenetrationVec;
      worldSpherePointClosestToPlane.vsub(worldPoint, penetrationVec);

      // The penetration. Negative value means overlap.
      final penetration = penetrationVec.dot(worldNormal);

      final worldPointToSphere = _sphereConvexSphereToWorldPoint;
      xi.vsub(worldPoint, worldPointToSphere);

      if (penetration < 0 && worldPointToSphere.dot(worldNormal) > 0) {
        // Intersects plane. Now check if the sphere is inside the face polygon
        final List<Vec3> faceVerts = []; // Face vertices, in world coords
        for (int j = 0, nVerts = face.length; j != nVerts; j++) {
          final worldVertex = v3pool.get();
          qj.vmult(verts[face[j]], worldVertex);
          xj.vadd(worldVertex, worldVertex);
          faceVerts.add(worldVertex);
        }

        if (_pointInPolygon(faceVerts, worldNormal, xi)) {
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

          v3pool.release([penetrationVec2,penetrationSpherePoint]);

          result.add(r);
          createFrictionEquationsFromContact(r, frictionResult);

          // Release world vertices
          for (int j = 0, nFaceverts = faceVerts.length; j != nFaceverts; j++) {
            v3pool.release([faceVerts[j]]);
          }

          return false; // We only expect *one* face contact
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
            final edge = _sphereConvexEdge;
            v2.vsub(v1, edge);

            // Construct the same vector, but normalized
            final edgeUnit = _sphereConvexEdgeUnit;
            edge.unit(edgeUnit);

            // p is xi projected onto the edge
            final p = v3pool.get() as Vec3;
            final v1ToXi = v3pool.get() as Vec3;
            xi.vsub(v1, v1ToXi);
            final dot = v1ToXi.dot(edgeUnit);
            edgeUnit.scale(dot, p);
            p.vadd(v1, p);

            // Compute a vector from p to the center of the sphere
            final xiToP = v3pool.get();
            p.vsub(xi, xiToP);

            // Collision if the edge-sphere distance is less than the radius
            // AND if p is in between v1 and v2
            if (dot > 0 && dot * dot < edge.lengthSquared() && xiToP.lengthSquared() < R * R) {
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
              for (int j = 0, nFaceverts = faceVerts.length; j != nFaceverts; j++) {
                v3pool.release([faceVerts[j]]);
              }

              v3pool.release([v1,v2,p,xiToP,v1ToXi]);
              return false;
            }

            v3pool.release([v1,v2,p,xiToP,v1ToXi]);
          }
        }

        // Release world vertices
        List<Vec3> toRelease = [];
        for (int j = 0, nFaceverts = faceVerts.length; j != nFaceverts; j++) {
          toRelease.add(faceVerts[j]);
        }
        v3pool.release(toRelease);
      }
    }

    return false;
  }

  bool planeConvex(
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
    final worldVertex = _planeConvexV;

    final worldNormal = _planeConvexNormal;
    worldNormal.set(0, 0, 1);
    qi.vmult(worldNormal, worldNormal) ;// Turn normal according to plane orientation

    int numContacts = 0;
    final relpos = _planeConvexRelpos;
    for (int i = 0; i != sj.vertices.length; i++) {
      // Get world convex vertex
      worldVertex.copy(sj.vertices[i]);
      qj.vmult(worldVertex, worldVertex);
      xj.vadd(worldVertex, worldVertex);
      worldVertex.vsub(xi, relpos);

      final dot = worldNormal.dot(relpos);
      if (dot <= 0.0) {
        if (justTest) {
          return true;
        }

        final r = createContactEquation(bi, bj, si, sj, rsi, rsj);

        // Get vertex position projected on plane
        final projected = _planeConvexProjected;
        worldNormal.scale(worldNormal.dot(relpos), projected);
        worldVertex.vsub(projected, projected);
        projected.vsub(xi, r.ri); // From plane to vertex projected on plane

        r.ni.copy(worldNormal) ;// Contact normal is the plane normal out from plane

        // rj is now just the vector from the convex center to the vertex
        worldVertex.vsub(xj, r.rj);

        // Make it relative to the body
        r.ri.vadd(xi, r.ri);
        r.ri.vsub(bi.position, r.ri);
        r.rj.vadd(xj, r.rj);
        r.rj.vsub(bj.position, r.rj);

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

    return false;
  }

  bool sphereHeightfield(
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
    final data = sj.data;
    final radius = si.radius;
    final w = sj.elementSize;
    final worldPillarOffset = _sphereHeightfieldTmp2;

    // Get sphere position to heightfield local!
    final localSpherePos = _sphereHeightfieldTmp1;
    Transform.pointToLocalFrame(xj, qj, xi, localSpherePos);

    // Get the index of the data points to test against
    int iMinX = ((localSpherePos.x - radius) / w).floor() - 1;
    int iMaxX = ((localSpherePos.x + radius) / w).ceil() + 1;
    int iMinY = ((localSpherePos.y - radius) / w).floor() - 1;
    int iMaxY = ((localSpherePos.y + radius) / w).ceil() + 1;
    // Bail out if we are out of the terrain
    if (iMaxX < 0 || iMaxY < 0 || iMinX > data.length || iMinY > data[0].length) {
      return false;
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

    List<double> minMax = [];
    sj.getRectMinMax(iMinX, iMinY, iMaxX, iMaxY, minMax);
    final min = minMax[0];
    final max = minMax[1];

    // Bail out if we can't touch the bounding height box
    if (localSpherePos.z - radius > max || localSpherePos.z + radius < min) {
      return false;
    }

    final result = this.result;
    for (int i = iMinX; i < iMaxX; i++) {
      for (int j = iMinY; j < iMaxY; j++) {
        final numContactsBefore = result.length;

        bool intersecting = false;

        // Lower triangle
        sj.getConvexTrianglePillar(i, j, false);
        Transform.pointToWorldFrame(xj, qj, sj.pillarOffset, worldPillarOffset);
        if (
          xi.distanceTo(worldPillarOffset) <
          sj.pillarConvex.boundingSphereRadius + si.boundingSphereRadius
        ) {
          intersecting = sphereConvex(
            si,
            sj.pillarConvex,
            xi,
            worldPillarOffset,
            qi,
            qj,
            bi,
            bj,
            si,
            sj,
            justTest
          );
        }

        if (justTest && intersecting) {
          return true;
        }

        // Upper triangle
        sj.getConvexTrianglePillar(i, j, true);
        Transform.pointToWorldFrame(xj, qj, sj.pillarOffset, worldPillarOffset);
        if (
          xi.distanceTo(worldPillarOffset) <
          sj.pillarConvex.boundingSphereRadius + si.boundingSphereRadius
        ) {
          intersecting = sphereConvex(
            si,
            sj.pillarConvex,
            xi,
            worldPillarOffset,
            qi,
            qj,
            bi,
            bj,
            si,
            sj,
            justTest
          );
        }

        if (justTest && intersecting) {
          return true;
        }

        final numContacts = result.length - numContactsBefore;

        if (numContacts > 2) {
          return false;
        }
        /*
          // Skip all but 1
          for (let k = 0; k < numContacts - 1; k++) {
              result.pop();
          }
        */
      }
    }
    return false;
  }

  bool boxHeightfield(
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
      bool justTest = false
  ]){
    si.convexPolyhedronRepresentation.material = si.material;
    si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
    return convexHeightfield(si.convexPolyhedronRepresentation, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest);
  }

  bool convexHeightfield(
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
      bool justTest = false
  ]){
    final data = sj.data;
    final w = sj.elementSize;
    final radius = si.boundingSphereRadius;
    final worldPillarOffset = _convexHeightfieldTmp2;
    final faceList = convexHeightfieldFaceList;

    // Get sphere position to heightfield local!
    final localConvexPos = _convexHeightfieldTmp1;
    Transform.pointToLocalFrame(xj, qj, xi, localConvexPos);

    // Get the index of the data points to test against
    int iMinX = ((localConvexPos.x - radius) / w).floor() - 1;

    int iMaxX = ((localConvexPos.x + radius) / w).ceil() + 1;
    int iMinY = ((localConvexPos.y - radius) / w).floor() - 1;
    int iMaxY = ((localConvexPos.y + radius) / w).ceil() + 1;

    // Bail out if we are out of the terrain
    if (iMaxX < 0 || iMaxY < 0 || iMinX > data.length || iMinY > data[0].length) {
      return false;
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

    final List<double> minMax = [0,0];
    sj.getRectMinMax(iMinX, iMinY, iMaxX, iMaxY, minMax);
    final min = minMax[0];
    final max = minMax[1];

    // Bail out if we're cant touch the bounding height box
    if (localConvexPos.z - radius > max || localConvexPos.z + radius < min) {
      return false;
    }

    for (int i = iMinX; i < iMaxX; i++) {
      for (int j = iMinY; j < iMaxY; j++) {
        bool? intersecting = false;

        // Lower triangle
        sj.getConvexTrianglePillar(i, j, false);
        Transform.pointToWorldFrame(xj, qj, sj.pillarOffset, worldPillarOffset);
        if (
          xi.distanceTo(worldPillarOffset) <
          sj.pillarConvex.boundingSphereRadius + si.boundingSphereRadius
        ) {
          intersecting = convexConvex(
            si,
            sj.pillarConvex,
            xi,
            worldPillarOffset,
            qi,
            qj,
            bi,
            bj,
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
        sj.getConvexTrianglePillar(i, j, true);
        Transform.pointToWorldFrame(xj, qj, sj.pillarOffset, worldPillarOffset);
        if (
          xi.distanceTo(worldPillarOffset) <
          sj.pillarConvex.boundingSphereRadius + si.boundingSphereRadius
        ) {
          intersecting = convexConvex(
            si,
            sj.pillarConvex,
            xi,
            worldPillarOffset,
            qi,
            qj,
            bi,
            bj,
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

    return false;
  }

  bool sphereParticle(
    Sphere sj,
    Particle si,
    Vec3 xj,
    Vec3 xi,
    Quaternion qj,
    Quaternion qi,
    Body bj,
    Body bi,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    // The normal is the unit vector from sphere center to particle center
    final normal = _particleSphereNormal;
    normal.set(0, 0, 1);
    xi.vsub(xj, normal);
    final lengthSquared = normal.lengthSquared();

    if (lengthSquared <= sj.radius * sj.radius) {
      if (justTest) {
        return true;
      }
      final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
      normal.normalize();
      r.rj.copy(normal);
      r.rj.scale(sj.radius, r.rj);
      r.ni.copy(normal); // Contact normal
      r.ni.negate(r.ni);
      r.ri.set(0, 0, 0); // Center of particle
      result.add(r);
      createFrictionEquationsFromContact(r, frictionResult);
    }

    return false;
  }

  bool planeParticle(
    Plane sj,
    Particle si,
    Vec3 xj,
    Vec3 xi,
    Quaternion qj,
    Quaternion qi,
    Body bj,
    Body bi,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    final normal = _particlePlaneNormal;
    normal.set(0, 0, 1);
    bj.quaternion.vmult(normal, normal); // Turn normal according to plane orientation
    final relpos = _particlePlaneRelpos;
    xi.vsub(bj.position, relpos);
    final dot = normal.dot(relpos);
    if (dot <= 0.0) {
      if (justTest) {
        return true;
      }

      final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
      r.ni.copy(normal); // Contact normal is the plane normal
      r.ni.negate(r.ni);
      r.ri.set(0, 0, 0); // Center of particle

      // Get particle position projected on plane
      final projected = _particlePlaneProjected;
      normal.scale(normal.dot(xi), projected);
      xi.vsub(projected, projected);
      //projected.vadd(bj.position,projected);

      // rj is now the projected world position minus plane position
      r.rj.copy(projected);
      result.add(r);
      createFrictionEquationsFromContact(r, frictionResult);
    }
    return false;
  }

  bool convexParticle(
    ConvexPolyhedron sj,
    Particle si,
    Vec3 xj,
    Vec3 xi,
    Quaternion qj,
    Quaternion qi,
    Body bj,
    Body bi,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    int penetratedFaceIndex = -1;
    final penetratedFaceNormal = _convexParticlePenetratedFaceNormal;
    final worldPenetrationVec = _convexParticleWorldPenetrationVec;
    double? minPenetration;
    //int numDetectedFaces = 0;

    // Convert particle position xi to local coords in the convex
    final local = _convexParticleLocal;
    final cqj = Quaternion();
    local.copy(xi);
    local.vsub(xj, local); // Convert position to relative the convex origin
    qj.conjugate(cqj);
    cqj.vmult(local, local);

    if (sj.pointIsInside(local)) {
      //if (true|| sj.worldVerticesNeedsUpdate) {
        sj.computeWorldVertices(xj, qj);
      //}
      if (sj.worldFaceNormalsNeedsUpdate) {
        sj.computeWorldFaceNormals(qj);
      }

      // For each world polygon in the polyhedra
      for (int i = 0, nfaces = sj.faces.length; i != nfaces; i++) {
        // Construct world face vertices
        final verts = sj.worldVertices[sj.faces[i][0]];
        final normal = sj.worldFaceNormals[i];
        final convexParticleVertexToParticle = Vec3();
        // Check how much the particle penetrates the polygon plane.
        xi.vsub(verts, convexParticleVertexToParticle);
        final penetration = -normal.dot(convexParticleVertexToParticle);
        if (minPenetration == null || penetration.abs() < minPenetration.abs()) {
          if (justTest) {
            return true;
          }

          minPenetration = penetration;
          penetratedFaceIndex = i;
          penetratedFaceNormal.copy(normal);
          //numDetectedFaces++;
        }
      }

      if (penetratedFaceIndex != -1) {
        // Setup contact
        var r = createContactEquation(bi, bj, si, sj, rsi, rsj);
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

        result.add(r);
        createFrictionEquationsFromContact(r, frictionResult);
      } else {
        print('Point found inside convex, but did not find penetrating face!');
      }
    }
    return false;
  }

  bool heightfieldCylinder(
    Heightfield sj,
    Cylinder si,
    Vec3 xj,
    Vec3 xi,
    Quaternion qj,
    Quaternion qi,
    Body bj,
    Body bi,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    return convexHeightfield(
      si,
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

  bool particleCylinder(
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
      bool justTest = false
  ]){
    return convexParticle(sj, si, xj, xi, qj, qi, bj, bi, rsi, rsj, justTest);
  }

  bool sphereTrimesh(
    Sphere sphereShape,
    Trimesh trimeshShape,
    Vec3 spherePos,
    Vec3 trimeshPos,
    Quaternion sphereQuat,
    Quaternion trimeshQuat,
    Body sphereBody,
    Body trimeshBody,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
    ]
  ){
    final edgeVertexA = _sphereTrimeshEdgeVertexA;
    final edgeVertexB = _sphereTrimeshEdgeVertexB;
    final edgeVector = _sphereTrimeshEdgeVector;
    final edgeVectorUnit = _sphereTrimeshEdgeVectorUnit;
    final localSpherePos = _sphereTrimeshLocalSpherePos;
    final localSphereAABB = _sphereTrimeshLocalSphereAABB;
    final tmp = _sphereTrimeshTmp;
    final v2 = _sphereTrimeshV2;
    final relpos = _sphereTrimeshRelpos;
    final triangles = _sphereTrimeshTriangles;

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

    trimeshShape.getTrianglesInAABB(localSphereAABB, triangles); //TODO fix retreived triangles
    
    final v = _sphereTrimeshV;
    final radiusSquared = sphereShape.radius * sphereShape.radius;
    for (int i = 0; i < trimeshShape.indices.length/3; i++){//triangles.length
      for (int j = 0; j < 3; j++) {
        
        trimeshShape.getVertex(trimeshShape.indices[i*3 + j], v);//i->triangles[i]
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

          final r = createContactEquation(sphereBody, trimeshBody, sphereShape, trimeshShape, rsi, rsj);
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
          result.add(r);
          createFrictionEquationsFromContact(r, frictionResult);
        }
      }
    }
    
    // Check all edges
    for (int i = 0; i < trimeshShape.indices.length/3; i++) {
      for (int j = 0; j < 3; j++) {
        trimeshShape.getVertex(trimeshShape.indices[i*3 + j], edgeVertexA);
        trimeshShape.getVertex(trimeshShape.indices[i*3 + ((j + 1) % 3)], edgeVertexB);
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
          tmp.vadd(edgeVertexA, tmp); // tmp is now the sphere center position projected to the edge, defined locally in the trimesh frame
          
          final dist = tmp.distanceTo(localSpherePos);
          if (dist < sphereShape.radius) {
            if (justTest) {
              return true;
            }
            final r = createContactEquation(sphereBody, trimeshBody, sphereShape, trimeshShape, rsi, rsj);

            tmp.vsub(localSpherePos, r.ni);
            r.ni.normalize();
            r.ni.scale(sphereShape.radius, r.ri);
            r.ri.vadd(spherePos, r.ri);
            r.ri.vsub(sphereBody.position, r.ri);

            Transform.pointToWorldFrame(trimeshPos, trimeshQuat, tmp, tmp);
            tmp.vsub(trimeshBody.position, r.rj);

            Transform.vectorToWorldFrame(trimeshQuat, r.ni, r.ni);
            Transform.vectorToWorldFrame(trimeshQuat, r.ri, r.ri);

            result.add(r);
            createFrictionEquationsFromContact(r, frictionResult);
          }
        }
      }
    }

    // Triangle faces
    final va = _sphereTrimeshVa;
    final vb = _sphereTrimeshVb;
    final vc = _sphereTrimeshVc;
    final normal = _sphereTrimeshNormal;
    for (int i = 0; i < triangles.length; i++) {//N = triangles.length; i != N
      trimeshShape.getTriangleVertices(triangles[i], va, vb, vc);
      trimeshShape.getFaceNormal(triangles[i], normal);
      
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
        final r = createContactEquation(sphereBody, trimeshBody, sphereShape, trimeshShape, rsi, rsj);

        tmp.vsub(localSpherePos, r.ni);
        r.ni.normalize();
        r.ni.scale(sphereShape.radius, r.ri);
        r.ri.vadd(spherePos, r.ri);
        r.ri.vsub(sphereBody.position, r.ri);

        Transform.pointToWorldFrame(trimeshPos, trimeshQuat, tmp, tmp);
        tmp.vsub(trimeshBody.position, r.rj);

        Transform.vectorToWorldFrame(trimeshQuat, r.ni, r.ni);
        Transform.vectorToWorldFrame(trimeshQuat, r.ri, r.ri);

        result.add(r);
        createFrictionEquationsFromContact(r, frictionResult);
      }
    }

    triangles.clear();
    return false;
  }

  bool planeTrimesh(
    Plane planeShape,
    Trimesh trimeshShape,
    Vec3 planePos,
    Vec3 trimeshPos,
    Quaternion planeQuat,
    Quaternion trimeshQuat,
    Body planeBody,
    Body trimeshBody,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    // Make contacts!
    final v = Vec3();

    final normal = _planeTrimeshNormal;
    normal.set(0, 0, 1);
    planeQuat.vmult(normal, normal);// Turn normal according to plane

    for (int i = 0; i < trimeshShape.vertices.length / 3; i++) {
      // Get world vertex from trimesh
      trimeshShape.getVertex(i, v);

      // Safe up
      final v2 = Vec3();
      v2.copy(v);
      Transform.pointToWorldFrame(trimeshPos, trimeshQuat, v2, v);

      // Check plane side
      final relpos = _planeTrimeshRelpos;
      v.vsub(planePos, relpos);
      final dot = normal.dot(relpos);

      if (dot <= 0.0) {
        if (justTest) {
          return true;
        }

        final r = createContactEquation(planeBody,trimeshBody,planeShape,trimeshShape, rsi, rsj);

        r.ni.copy(normal); // Contact normal is the plane normal

        // Get vertex position projected on plane
        final projected = _planeTrimeshProjected;
        normal.scale(relpos.dot(normal), projected);
        v.vsub(projected, projected);

        // ri is the projected world position minus plane position
        r.ri.copy(projected);
        r.ri.vsub(planeBody.position, r.ri);

        r.rj.copy(v);
        r.rj.vsub(trimeshBody.position, r.rj);

        // Store result
        result.add(r);
        createFrictionEquationsFromContact(r, frictionResult);
      }
    }

    return false;
  }
  bool particleTrimesh(
    Particle si,
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
      bool justTest = false
  ]){
    final vector = Vec3();
    final local = Vec3();

    // Transform ray to local space!
    Transform.pointToLocalFrame(xj, qj, xi, local);
    //local.normalize();

    final va = Vec3();
    final vb = Vec3();
    final vc = Vec3();
    final normal = Vec3();
    final v = Vec3();
    final v2 = Vec3();
    final relpos = Vec3();

    //trimeshShape.getTrianglesInAABB(localSphereAABB, triangles); //TODO fix retreived triangles
    for (int i = 0; i < sj.indices.length/3; i++) {//N = triangles.length; i != N
      sj.getTriangleVertices(sj.indices[i], va, vb, vc);
      if(Ray.pointInTriangle(vector, va, vb, vc)){
        sj.getFaceNormal(sj.indices[i], normal);

        xi.vsub(bj.position, relpos);
        final dot = normal.dot(relpos);
        if (dot <= 0.0) {
          if (justTest) {
            return true;
          }

          final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
          r.ni.copy(normal); // Contact normal is the plane normal
          r.ni.negate(r.ni);
          r.ri.set(0, 0, 0); // Center of particle

          // Get particle position projected on plane
          final projected = _particlePlaneProjected;
          normal.scale(normal.dot(xi), projected);
          xi.vsub(projected, projected);
          //projected.vadd(bj.position,projected);

          // rj is now the projected world position minus plane position
          r.rj.copy(projected);
          result.add(r);
          createFrictionEquationsFromContact(r, frictionResult);
        }
      }

      // sj.getTriangleVertices(sj.indices[i], va, vb, vc);
      // sj.getFaceNormal(sj.indices[i], normal);

      // va.vsub(local, vector);
      // double dist = local.dot(normal);
      // normal.scale(dist, vector);
      // local.vsub(vector, vector);

      // final scalar = normal.dot(vector) / dist;
      // for (int j = 0; j < 3; j++) {
      //   sj.getVertex(sj.indices[i*3 + j], v);
      //   // Check vertex overlap in sphere
      //   v.vsub(local, relpos);
        
      //   if (Ray.pointInTriangle(vector, va, vb, vc) || relpos.lengthSquared() <= 1) {
      //       v2.copy(v);
      //       Transform.pointToWorldFrame(xj, qj, v2, v);

      //       v.vsub(xi, relpos);

      //       if (justTest) {
      //         return true;
      //       }

      //       final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
      //       r.ni.copy(relpos);
      //       r.ni.normalize();

      //       // ri is the vector from sphere center to the sphere surface
      //       r.ri.copy(r.ni);
      //       r.ri.scale(1, r.ri);
      //       r.ri.vadd(xi, r.ri);
      //       r.ri.vsub(bi.position, r.ri);

      //       r.rj.copy(v);
      //       r.rj.vsub(bj.position, r.rj);

      //       // Store result
      //       result.add(r);
      //       createFrictionEquationsFromContact(r, frictionResult);
      //   }
      // }
      
      // if (Ray.pointInTriangle(vector, va, vb, vc) && scalar >0) {
      //   if (justTest) {
      //     return true;
      //   }
      //   final r = createContactEquation(bi, bj, si, sj, rsi, rsj);

      //   vector.vsub(local, r.ni);
      //   r.ni.normalize();
      //   r.ni.scale(1, r.ri);
      //   r.ri.vadd(xi, r.ri);
      //   r.ri.vsub(bi.position, r.ri);

      //   Transform.pointToWorldFrame(xj, qj, vector, vector);
      //   vector.vsub(bj.position, r.rj);

      //   Transform.vectorToWorldFrame(qj, r.ni, r.ni);
      //   Transform.vectorToWorldFrame(qj, r.ri, r.ri);

      //   result.add(r);
      //   createFrictionEquationsFromContact(r, frictionResult);
      //   //return false;
      // }
    }

    return false;
  }
  bool boxTrimesh(
    Box si,
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
      bool justTest = false
  ]){
    si.convexPolyhedronRepresentation.material = si.material;
    si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
    return convexTrimesh(si.convexPolyhedronRepresentation, sj, xi, xj, qi, qj, bi, bj, rsi, rsj,justTest);
  }
  bool convexTrimesh(
    ConvexPolyhedron si,
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
      bool justTest = false,
      List<int>? faceListA,
      List<int>? faceListB
    ]
  ){
    final sepAxis = _convexConvexSepAxis;
    if(xi.distanceTo(xj) > si.boundingSphereRadius + sj.boundingSphereRadius){
      return false;
    }

    // Construct a temp hull for each triangle
    final hullB = ConvexPolyhedron();

    hullB.faces = [[0,1,2]];
    final va = Vec3();
    final vb = Vec3();
    final vc = Vec3();
    
    hullB.vertices = [va,vb,vc];

    for (int i = 0; i < sj.indices.length / 3; i++) {
      sj.getTriangleVertices(i, va, vb, vc);

      final triangleNormal = Vec3();
      sj.getFaceNormal(i, triangleNormal);
      hullB.faceNormals = [triangleNormal];

      double? d = si.testSepAxis(triangleNormal, hullB, xi, qi, xj, qj);
      if(si.findSeparatingAxis(si, xj, qj, xi, qi, sepAxis, faceListA, faceListB)){
        triangleNormal.scale(1, triangleNormal);
        d = si.testSepAxis(triangleNormal, hullB, xi, qi, xj, qj);

        if(d == null){
          continue;
        }
      }

      final List<ConvexPolyhedronContactPoint> res = [];
      final q = _convexConvexQ;
      si.clipAgainstHull(xi,qi,hullB,xj,qj,triangleNormal,-100,100,res);
      for(int j = 0; j != res.length; j++){
        final r = createContactEquation(bi,bj,si,sj,rsi,rsj);
        final ri = r.ri;
        final rj = r.rj;
        r.ni.copy(triangleNormal);
        r.ni.negate(r.ni);
        res[j].normal.negate(q);
        q.scale(res[j].depth, q);
        res[j].point.vadd(q, ri);
        rj.copy(res[j].point);

        // Contact points are in world coordinates. Transform back to relative
        ri.vsub(xi,ri);
        rj.vsub(xj,rj);

        // Make relative to bodies
        ri.vadd(xi, ri);
        ri.vsub(bi.position, ri);
        rj.vadd(xj, rj);
        rj.vsub(bj.position, rj);

        result.add(r);
      }
    }
    return false;
  }

  bool _pointInPolygon(List<Vec3> verts, Vec3 normal, Vec3 p) {
    bool? positiveResult;
    final N = verts.length;
    for (int i = 0; i != N; i++) {
      final v = verts[i];

      // Get edge to the next vertex
      final edge = _pointInPolygonEdge;
      verts[(i + 1) % N].vsub(v, edge);

      // Get cross product between polygon normal and the edge
      final edgeXNormal = _pointInPolygonEdgeXNormal;
      //final edge_x_normal = Vec3();
      edge.cross(normal, edgeXNormal);

      // Get vector between point and current vertex
      final vertexToP = _pointInPolygonVtp;
      p.vsub(v, vertexToP);

      // This dot product determines which side of the edge the point is
      final r = edgeXNormal.dot(vertexToP);

      // If all such dot products have same sign, we are inside the polygon.
      if (positiveResult == null || (r > 0 && positiveResult == true) || (r <= 0 && positiveResult == false)) {
        positiveResult ??= r > 0;
        continue;
      } else {
        return false; // Encountered some other sign. Exit.
      }
    }

    // If we got here, all dot products were of the same sign.
    return true;
  }
}