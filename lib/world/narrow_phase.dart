import 'dart:math' as math;
import '../rigid_body_shapes/shape.dart';
import '../math/vec3.dart';
import '../math/transform.dart';
import '../math/quaternion.dart';
import '../objects/rigid_body.dart';
import '../collision/aabb.dart';
import '../collision/ray_class.dart';
import '../utils/vec3_pool.dart';
import '../equations/contact_equation.dart';
import '../equations/friction_equation.dart';
import '../rigid_body_shapes/box.dart';
import '../rigid_body_shapes/sphere.dart';
import '../rigid_body_shapes/convex_polyhedron.dart';
import '../rigid_body_shapes/particle.dart';
import '../rigid_body_shapes/plane.dart';
import '../rigid_body_shapes/trimesh.dart';
import '../rigid_body_shapes/heightfield.dart';
import '../material/contact_material.dart';
import '../world/world_class.dart';
import 'package:vector_math/vector_math.dart' hide Plane, Sphere, Ray;

// Naming rule: based of the order in SHAPE_TYPES,
// the first part of the method is formed by the
// shape type that comes before, in the second part
// there is the shape type that comes after in the SHAPE_TYPES list
enum CollisionType{
  sphereSphere,
  spherePlane,
  sphereBox,
  sphereConvex,
  sphereHeightfield,
  sphereParticle,
  sphereCylinder,
  sphereTrimesh,
  sphereCone,
  sphereCapsule,
  sphereSizedPlane,

  boxBox,
  boxConvex,
  boxHeightfield,
  boxParticle,
  boxCylinder,
  boxTrimesh,
  boxCapsule,
  boxCone,
  boxSizedPlane,

  planeBox,
  planeConvex,
  planeParticle,
  planeCylinder,
  planeTrimesh,
  planeCone,
  planeCapsule,
  planeSizedPlane,

  particleHeightfield,
  particleCylinder,
  particleTrimesh,
  particleCone,
  particleCapsule,
  particleConvex,
  particleSizedPlane,

  trimeshTrimesh,
  trimeshCone,
  trimeshCylinder,
  trimeshCapsule,
  trimeshConvex,
  trimeshSizedPlane,

  heightfieldCapsule,
  heightfieldCone,
  heightfieldCylinder,
  heightfieldConvex,
  heightfieldSizedPlane,

  convexConvex,
  convexCylinder,
  convexCone,
  convexCapsule,
  convexSizedPlane,
  
  cylinderCylinder,
  cylinderCapsule,
  cylinderCone,
  cylinderSizedPlane,

  capsuleCapsule,
  capsuleCone,
  capsuleSizedPlane,

  coneCone,
  coneSizedPlane,

  sizedPlaneSizedPlane
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
    else if(
      type == CollisionType.convexConvex ||
      type == CollisionType.cylinderCapsule ||
      type == CollisionType.capsuleCapsule ||
      type == CollisionType.coneCone ||
      type == CollisionType.capsuleCone || 
      type == CollisionType.cylinderCone || 
      type == CollisionType.cylinderCylinder ||
      type == CollisionType.convexCapsule || 
      type == CollisionType.convexCone || 
      type == CollisionType.convexCylinder ||
      type == CollisionType.sizedPlaneSizedPlane ||
      type == CollisionType.convexSizedPlane ||
      type == CollisionType.cylinderSizedPlane ||
      type == CollisionType.capsuleSizedPlane || 
      type == CollisionType.coneSizedPlane
    ) {
      return convexConvex;
    }
    else if(
      type == CollisionType.sphereConvex || 
      type == CollisionType.sphereCylinder ||
      type == CollisionType.sphereCone ||
      type == CollisionType.sphereCapsule ||
      type == CollisionType.sphereSizedPlane
    ) {
      return sphereConvex;
    }
    else if(
      type == CollisionType.planeConvex ||
      type == CollisionType.planeCylinder ||
      type == CollisionType.planeCapsule ||
      type == CollisionType.planeCone ||
      type == CollisionType.planeSizedPlane
    ) {
      return planeConvex;
    }
    else if(
      type == CollisionType.boxConvex || 
      type == CollisionType.boxCylinder ||
      type == CollisionType.boxCapsule ||
      type == CollisionType.boxCone ||
      type == CollisionType.boxSizedPlane
    ) {
      return boxConvex;
    }
    else if(type == CollisionType.sphereHeightfield) {
      return sphereHeightfield;
    }
    else if(type == CollisionType.boxHeightfield) {
      return boxHeightfield;
    }
    else if(
      type == CollisionType.heightfieldConvex || 
      type == CollisionType.heightfieldCylinder ||
      type == CollisionType.heightfieldCapsule || 
      type == CollisionType.heightfieldCone ||
      type == CollisionType.heightfieldSizedPlane
    ) {
      return heightfieldConvex;
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
    else if(
      type == CollisionType.particleConvex ||
      type == CollisionType.particleCylinder ||
      type == CollisionType.particleCapsule || 
      type == CollisionType.particleCone ||
      type == CollisionType.particleSizedPlane
    ) {
      return particleConvex;
    }
    else if(type == CollisionType.particleHeightfield) {
      return heightfieldParticle;
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
    else if(
      type == CollisionType.trimeshConvex ||
      type == CollisionType.trimeshCylinder ||
      type == CollisionType.trimeshCone ||
      type == CollisionType.trimeshCapsule ||
      type == CollisionType.trimeshSizedPlane
    ) {
      return trimeshConvex;
    }
    else if(type == CollisionType.trimeshTrimesh) {
      return trimeshTrimesh;
    }
  }

  Narrowphase(this.world) {
    currentContactMaterial = world.defaultContactMaterial;
  }

  final _averageNormal = Vector3.zero();
  final _averageContactPointA = Vector3.zero();
  final _averageContactPointB = Vector3.zero();

  final _tmpVec1 = Vector3.zero();
  final _tmpVec2 = Vector3.zero();
  final _tmpQuat1 = Quaternion(0,0,0,1);
  final _tmpQuat2 = Quaternion(0,0,0,1);

  final _planeTrimeshNormal = Vector3.zero();
  final _planeTrimeshRelpos = Vector3.zero();
  final _planeTrimeshProjected = Vector3.zero();

  final _sphereTrimeshNormal = Vector3.zero();
  final _sphereTrimeshRelpos = Vector3.zero();
  //final _sphereTrimeshProjected = Vector3.zero();
  final _sphereTrimeshV = Vector3.zero();
  final _sphereTrimeshV2 = Vector3.zero();
  final _sphereTrimeshEdgeVertexA = Vector3.zero();
  final _sphereTrimeshEdgeVertexB = Vector3.zero();
  final _sphereTrimeshEdgeVector = Vector3.zero();
  final _sphereTrimeshEdgeVectorUnit = Vector3.zero();
  final _sphereTrimeshlocal = Vector3.zero();
  final _sphereTrimeshTmp = Vector3.zero();
  final _sphereTrimeshVa = Vector3.zero();
  final _sphereTrimeshVb = Vector3.zero();
  final _sphereTrimeshVc = Vector3.zero();
  final _sphereTrimeshLocalSphereAABB = AABB();
  final List<int> _sphereTrimeshTriangles = [];

  final _pointOnPlaneToSphere = Vector3.zero();
  final _planeToSphereOrtho = Vector3.zero();

  // See http://bulletphysics.com/Bullet/BulletFull/SphereTriangleDetector_8cpp_source.html
  final _pointInPolygonEdge = Vector3.zero();
  final _pointInPolygonEdgeXNormal = Vector3.zero();
  final _pointInPolygonVtp = Vector3.zero();


  final _boxToSphere = Vector3.zero();
  final _sphereBoxNs = Vector3.zero();
  final _sphereBoxNs1 = Vector3.zero();
  final _sphereBoxNs2 = Vector3.zero();
  final List<Vector3> sphereBoxSides = [Vector3.zero(), Vector3.zero(), Vector3.zero(), Vector3.zero(), Vector3.zero(), Vector3.zero()];
  final _sphereBoxSphereToCorner = Vector3.zero();
  final _sphereBoxSideNs = Vector3.zero();
  final _sphereBoxSideNs1 = Vector3.zero();
  final _sphereBoxSideNs2 = Vector3.zero();

  final _convexToSphere = Vector3.zero();
  final _sphereConvexEdge = Vector3.zero();
  final _sphereConvexEdgeUnit = Vector3.zero();
  final _sphereConvexSphereToCorner = Vector3.zero();
  final _sphereConvexWorldCorner = Vector3.zero();
  final _sphereConvexWorldNormal = Vector3.zero();
  final _sphereConvexWorldPoint = Vector3.zero();
  final _sphereConvexWorldSpherePointClosestToPlane = Vector3.zero();
  final _sphereConvexPenetrationVec = Vector3.zero();
  final _sphereConvexSphereToWorldPoint = Vector3.zero();

  //final _planeBoxNormal = Vector3.zero();
  //final _planeToCorner = Vector3.zero();

  final _planeConvexV = Vector3.zero();
  final _planeConvexNormal = Vector3.zero();
  final _planeConvexRelpos = Vector3.zero();
  final _planeConvexProjected = Vector3.zero();

  final _convexConvexSepAxis = Vector3.zero();
  final _convexConvexQ = Vector3.zero();

  final _particlePlaneNormal = Vector3.zero();
  final _particlePlaneRelpos = Vector3.zero();
  final _particlePlaneProjected = Vector3.zero();

  final _particleSphereNormal = Vector3.zero();

  // WIP
  
  final _convexParticleLocal = Vector3.zero();
  //final _convexParticleNormal = Vector3.zero();
  final _convexParticlePenetratedFaceNormal = Vector3.zero();
  
  final _convexParticleWorldPenetrationVec = Vector3.zero();

  final _convexHeightfieldTmp1 = Vector3.zero();
  final _convexHeightfieldTmp2 = Vector3.zero();
  List<int> convexHeightfieldFaceList = [0];

  final _sphereHeightfieldTmp1 = Vector3.zero();
  final _sphereHeightfieldTmp2 = Vector3.zero();

  CollisionType? _getCollisionType(String type){
    switch (type) {
      case "spheresphere":
        return CollisionType.sphereSphere;
      case "sphereplane":
        return CollisionType.spherePlane;
      case "sphereparticle":
        return CollisionType.sphereParticle;
      case "sphereheightfield":
        return CollisionType.sphereHeightfield;
      case "spherebox":
        return CollisionType.sphereBox;
      case "sphereconvex":
        return CollisionType.sphereConvex;
      case "spherecylinder":
        return CollisionType.sphereCylinder;
      case "spherecapsule":
        return CollisionType.sphereCapsule;
      case "spherecone":
        return CollisionType.sphereCone;
      case "spheretrimesh":
        return CollisionType.sphereTrimesh;
      case "spheresizedplane":
        return CollisionType.sphereSizedPlane;

      case "boxbox":
        return CollisionType.boxBox;
      case "boxconvex":
        return CollisionType.boxConvex;
      case "boxheightfield":
        return CollisionType.boxHeightfield;
      case "boxparticle":
        return CollisionType.boxParticle;
      case "boxcylinder":
        return CollisionType.boxCylinder;
      case "boxcapsule":
        return CollisionType.boxCapsule;
      case "boxcone":
        return CollisionType.boxCone;
      case "boxtrimesh":
        return CollisionType.boxTrimesh;
      case "boxsizedplane":
        return CollisionType.boxSizedPlane;

      case "planeconvex":
        return CollisionType.planeConvex;
      case "planecone":
        return CollisionType.planeCone;
      case "planecapsule":
        return CollisionType.planeCapsule;
      case "planebox":
        return CollisionType.planeBox;
      case "planeparticle":
        return CollisionType.planeParticle;
      case "planecylinder":
        return CollisionType.planeCylinder;
      case "planetrimesh":
        return CollisionType.planeTrimesh;
      case "planesizedplane":
        return CollisionType.planeSizedPlane;

      case "particleconvex":
        return CollisionType.particleConvex;
      case "particlecone":
        return CollisionType.particleCone;
      case "particlecapsule":
        return CollisionType.particleCapsule;
      case "particlecylinder":
        return CollisionType.particleCylinder;
      case "particleheightfield":
        return CollisionType.particleHeightfield;
      case "particletrimesh":
        return CollisionType.particleTrimesh;
      case "particlesizedplane":
        return CollisionType.particleSizedPlane;

      case "trimeshcylinder":
        return CollisionType.trimeshCylinder;
      case "trimeshcone":
        return CollisionType.trimeshCone;
      case "trimeshcapsule":
        return CollisionType.trimeshCapsule;
      case "trimeshconvex":
        return CollisionType.trimeshConvex;
      case "trimeshtrimesh":
        return CollisionType.trimeshTrimesh;
      case "trimeshsizedplane":
        return CollisionType.trimeshSizedPlane;

      case "heightfieldcylinder":
        return CollisionType.heightfieldCylinder;
      case "heightfieldconvex":
        return CollisionType.heightfieldConvex;
      case "heightfieldcapsule":
        return CollisionType.heightfieldCapsule;
      case "heightfieldcone":
        return CollisionType.heightfieldCone;
      case "heightfieldsizedplane":
        return CollisionType.heightfieldSizedPlane;

      case "convexconvex":
        return CollisionType.convexConvex;
      case "convexcylinder":
        return CollisionType.convexCylinder;
      case "convexcone":
        return CollisionType.convexCone;
      case "convexcapsule":
        return CollisionType.convexCapsule;
      case "convexSizedPlane":
        return CollisionType.convexSizedPlane;

      case "cylindercylinder":
        return CollisionType.cylinderCylinder;
      case "cylindercone":
        return CollisionType.cylinderCone;
      case "cylindercapsule":
        return CollisionType.cylinderCapsule;
      case "cylindersizedplane":
        return CollisionType.cylinderSizedPlane;

      case "capsulecone":
        return CollisionType.capsuleCone;
      case "capsulecapsule":
        return CollisionType.capsuleCapsule;
      case "capsulesizedplane":
        return CollisionType.capsuleSizedPlane;

      case "conecone":
        return CollisionType.coneCone;
      case "conesizedplane":
        return CollisionType.coneSizedPlane;

      case "sizedplanesizedplane":
        return CollisionType.sizedPlaneSizedPlane;

      default:
        return null;
    }
  }

  CollisionType? getCollisionType(ShapeType a, ShapeType b){
    String n1 = a.name+b.name;
    String n2 = b.name+a.name;
    return _getCollisionType(n1.toLowerCase()) ?? _getCollisionType(n2.toLowerCase());
    // for(int i = 0; i < CollisionType.values.length; i++){
    //   if(n1 == CollisionType.values[i].name.toLowerCase()){
    //     return CollisionType.values[i];
    //   }
    //   else if(n2 == CollisionType.values[i].name.toLowerCase()){
    //     return CollisionType.values[i];
    //   }
    // }
    // return null;
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
      final mug = friction * (world.frictionGravity ?? world.gravity).length;
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
      c1.ri.setFrom(contactEquation.ri);
      c1.rj.setFrom(contactEquation.rj);
      c2.ri.setFrom(contactEquation.ri);
      c2.rj.setFrom(contactEquation.rj);

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
        _averageNormal.add2(c.ni, _averageNormal);
        _averageContactPointA.add2(c.ri, _averageContactPointA);
        _averageContactPointB.add2(c.rj, _averageContactPointB);
      } else {
        _averageNormal.sub2(c.ni, _averageNormal);
        _averageContactPointA.add2(c.rj, _averageContactPointA);
        _averageContactPointB.add2(c.ri, _averageContactPointB);
      }
    }

    final invNumContacts = 1 / numContacts;
    _averageContactPointA.scale2(invNumContacts, f1.ri);
    _averageContactPointB.scale2(invNumContacts, f1.rj);
    f2.ri.setFrom(f1.ri); // Should be the same
    f2.rj.setFrom(f1.rj);
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
        bi.quaternion.multiply2(bi.shapeOrientations[i], qi);
        bi.quaternion.vmult(bi.shapeOffsets[i], xi);
        xi.add2(bi.position, xi);
        final si = bi.shapes[i];

        for (int j = 0; j < bj.shapes.length; j++) {
          // Compute world transform of shapes
          bj.quaternion.multiply2(bj.shapeOrientations[j], qj);
          bj.quaternion.vmult(bj.shapeOffsets[j], xj);
          xj.add2(bj.position, xj);
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
    Vector3 xi,
    Vector3 xj,
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
    xj.sub2(xi, contactEq.ni);
    contactEq.ni.normalize();

    // Contact point locations
    contactEq.ri.setFrom(contactEq.ni);
    contactEq.rj.setFrom(contactEq.ni);
    contactEq.ri.scale2(si.radius, contactEq.ri);
    contactEq.rj.scale2(-sj.radius, contactEq.rj);

    contactEq.ri.add2(xi, contactEq.ri);
    contactEq.ri.sub2(bi.position, contactEq.ri);

    contactEq.rj.add2(xj, contactEq.rj);
    contactEq.rj.sub2(bj.position, contactEq.rj);

    result.add(contactEq);

    createFrictionEquationsFromContact(contactEq, frictionResult);

    return false;
  }
  bool spherePlane(
    Sphere si,
    Plane sj,
    Vector3 xi,
    Vector3 xj,
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
    r.ni.setValues(0, 0, 1);
    qj.vmult(r.ni, r.ni);
    r.ni.negate(); // body i is the sphere, flip normal
    r.ni.normalize(); // Needed?

    // Vector from sphere center to contact point
    r.ni.scale2(si.radius, r.ri);

    // Project down sphere on plane
    xi.sub2(xj, _pointOnPlaneToSphere);
    r.ni.scale2(r.ni.dot(_pointOnPlaneToSphere), _planeToSphereOrtho);
    _pointOnPlaneToSphere.sub2(_planeToSphereOrtho, r.rj); // The sphere position projected to plane

    if (-_pointOnPlaneToSphere.dot(r.ni) <= si.radius) {
      if (justTest) {
        return true;
      }

      // Make it relative to the body
      final ri = r.ri;
      final rj = r.rj;
      ri.add2(xi, ri);
      ri.sub2(bi.position, ri);
      rj.add2(xj, rj);
      rj.sub2(bj.position, rj);

      result.add(r);
      createFrictionEquationsFromContact(r, frictionResult);
    }

    return false;
  }
  bool sphereBox(
    Sphere si,
    Box sj,
    Vector3 xi,
    Vector3 xj,
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
    xi.sub2(xj, _boxToSphere);
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
      ns.setFrom(sides[idx]);

      final h = ns.length;
      ns.normalize();

      // The normal/distance dot product tells which side of the plane we are
      final dot = _boxToSphere.dot(ns);

      if (dot < h + R && dot > 0) {
        // Intersects plane. Now check the other two dimensions
        final ns1 = _sphereBoxNs1;
        final ns2 = _sphereBoxNs2;
        ns1.setFrom(sides[(idx + 1) % 3]);
        ns2.setFrom(sides[(idx + 2) % 3]);
        final h1 = ns1.length;
        final h2 = ns2.length;
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
            sideNs.setFrom(ns);
            sideNs1.setFrom(ns1);
            sideNs2.setFrom(ns2);
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
      sideNs.scale2(-R, r.ri); // Sphere r
      r.ni.setFrom(sideNs);
      r.ni.negate(); // Normal should be out of sphere
      sideNs.scale2(sideH!, sideNs);
      sideNs1.scale2(sideDot1, sideNs1);
      sideNs.add2(sideNs1, sideNs);
      sideNs2.scale2(sideDot2, sideNs2);
      sideNs.add2(sideNs2, r.rj);

      // Make relative to bodies
      r.ri.add2(xi, r.ri);
      r.ri.sub2(bi.position, r.ri);
      r.rj.add2(xj, r.rj);
      r.rj.sub2(bj.position, r.rj);

      result.add(r);
      createFrictionEquationsFromContact(r, frictionResult);
    }

    // Check corners
    Vector3? rj = v3pool.get();
    final sphereToCorner = _sphereBoxSphereToCorner;
    for (int j = 0; j != 2 && !found; j++) {
      for (int k = 0; k != 2 && !found; k++) {
        for (int l = 0; l != 2 && !found; l++) {
          rj.setValues(0.0, 0.0, 0.0);
          if (j != 0) {
            rj.add2(sides[0], rj);
          } else {
            rj.sub2(sides[0], rj);
          }
          if (k != 0) {
            rj.add2(sides[1], rj);
          } else {
            rj.sub2(sides[1], rj);
          }
          if (l != 0) {
            rj.add2(sides[2], rj);
          } else {
            rj.sub2(sides[2], rj);
          }

          // World position of corner
          xj.add2(rj, sphereToCorner);
          sphereToCorner.sub2(xi, sphereToCorner);

          if (sphereToCorner.length2 < R * R) {
            if (justTest) {
              return true;
            }
            found = true;
            final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
            r.ri.setFrom(sphereToCorner);
            r.ri.normalize();
            r.ni.setFrom(r.ri);
            r.ri.scale2(R, r.ri);
            r.rj.setFrom(rj);

            // Make relative to bodies
            r.ri.add2(xi, r.ri);
            r.ri.sub2(bi.position, r.ri);
            r.rj.add2(xj, r.rj);
            r.rj.sub2(bj.position, r.rj);

            result.add(r);
            createFrictionEquationsFromContact(r, frictionResult);
          }
        }
      }
    }
    v3pool.release([rj]);
    rj = null;

    // Check edges
    final Vector3 edgeTangent = v3pool.get();
    final Vector3 edgeCenter = v3pool.get();
    final Vector3 r = v3pool.get(); // r = edge center to sphere center
    final Vector3 orthogonal = v3pool.get();
    final Vector3 dist = v3pool.get();
    final int nSides = sides.length;
    for (int j = 0; j != nSides && !found; j++) {
      for (int k = 0; k != nSides && !found; k++) {
        if (j % 3 != k % 3) {
          // Get edge tangent
          sides[k].cross2(sides[j], edgeTangent);
          edgeTangent.normalize();
          sides[j].add2(sides[k], edgeCenter);
          r.setFrom(xi);
          r.sub2(edgeCenter, r);
          r.sub2(xj, r);
          final orthonorm = r.dot(edgeTangent); // distance from edge center to sphere center in the tangent direction
          edgeTangent.scale2(orthonorm, orthogonal); // Vector from edge center to sphere center in the tangent direction

          // Find the third side orthogonal to this one
          int l = 0;
          while (l == j % 3 || l == k % 3) {
            l++;
          }

          // vec from edge center to sphere projected to the plane orthogonal to the edge tangent
          dist.setFrom(xi);
          dist.sub2(orthogonal, dist);
          dist.sub2(edgeCenter, dist);
          dist.sub2(xj, dist);

          // Distances in tangent direction and distance in the plane orthogonal to it
          final tdist = orthonorm.abs();
          final ndist = dist.length;

          if (tdist < sides[l].length && ndist < R) {
            if (justTest) {
              return true;
            }
            found = true;
            final res = createContactEquation(bi, bj, si, sj, rsi, rsj);
            edgeCenter.add2(orthogonal, res.rj); // box rj
            res.rj.setFrom(res.rj);
            res.ni..setFrom(dist)..negate();
            res.ni.normalize();

            res.ri.setFrom(res.rj);
            res.ri.add2(xj, res.ri);
            res.ri.sub2(xi, res.ri);
            res.ri.normalize();
            res.ri.scale2(R, res.ri);

            // Make relative to bodies
            res.ri.add2(xi, res.ri);
            res.ri.sub2(bi.position, res.ri);
            res.rj.add2(xj, res.rj);
            res.rj.sub2(bj.position, res.rj);

            result.add(res);
            createFrictionEquationsFromContact(res, frictionResult);
          }
        }
      }
    }
    v3pool.release([edgeTangent, edgeCenter, r, orthogonal, dist]);

    return false;
  }
  bool sphereConvex(
    Sphere si,
    ConvexPolyhedron sj,
    Vector3 xi,
    Vector3 xj,
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
    xi.sub2(xj, _convexToSphere);
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
      xj.add2(worldCorner, worldCorner);
      final sphereToCorner = _sphereConvexSphereToCorner;
      worldCorner.sub2(xi, sphereToCorner);
      if (sphereToCorner.length2 < R * R) {
        if (justTest) {
          return true;
        }
        found = true;
        final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
        r.ri.setFrom(sphereToCorner);
        r.ri.normalize();
        r.ni.setFrom(r.ri);
        r.ri.scale2(R, r.ri);
        worldCorner.sub2(xj, r.rj);

        // Should be relative to the body.
        r.ri.add2(xi, r.ri);
        r.ri.sub2(bi.position, r.ri);

        // Should be relative to the body.
        r.rj.add2(xj, r.rj);
        r.rj.sub2(bj.position, r.rj);

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
      worldPoint.add2(xj, worldPoint);

      // Get a point on the sphere, closest to the face normal
      final worldSpherePointClosestToPlane = _sphereConvexWorldSpherePointClosestToPlane;
      worldNormal.scale2(-R, worldSpherePointClosestToPlane);
      xi.add2(worldSpherePointClosestToPlane, worldSpherePointClosestToPlane);

      // Vector from a face point to the closest point on the sphere
      final penetrationVec = _sphereConvexPenetrationVec;
      worldSpherePointClosestToPlane.sub2(worldPoint, penetrationVec);

      // The penetration. Negative value means overlap.
      final penetration = penetrationVec.dot(worldNormal);

      final worldPointToSphere = _sphereConvexSphereToWorldPoint;
      xi.sub2(worldPoint, worldPointToSphere);

      if (penetration < 0 && worldPointToSphere.dot(worldNormal) > 0) {
        // Intersects plane. Now check if the sphere is inside the face polygon
        final List<Vector3> faceVerts = []; // Face vertices, in world coords
        for (int j = 0, nVerts = face.length; j != nVerts; j++) {
          final worldVertex = v3pool.get();
          qj.vmult(verts[face[j]], worldVertex);
          xj.add2(worldVertex, worldVertex);
          faceVerts.add(worldVertex);
        }

        if (_pointInPolygon(faceVerts, worldNormal, xi)) {
          // Is the sphere center in the face polygon?
          if (justTest) {
            return true;
          }
          found = true;
          final r = createContactEquation(bi, bj, si, sj, rsi, rsj);

          worldNormal.scale2(-R, r.ri); // Contact offset, from sphere center to contact
          r.ni..setFrom(worldNormal)..negate(); // Normal pointing out of sphere

          final penetrationVec2 = v3pool.get();
          worldNormal.scale2(-penetration, penetrationVec2);
          final penetrationSpherePoint = v3pool.get();
          worldNormal.scale2(-R, penetrationSpherePoint);

          //xi.sub2(xj).add2(penetrationSpherePoint).add2(penetrationVec2 , r.rj);
          xi.sub2(xj, r.rj);
          r.rj.add2(penetrationSpherePoint, r.rj);
          r.rj.add2(penetrationVec2, r.rj);

          // Should be relative to the body.
          r.rj.add2(xj, r.rj);
          r.rj.sub2(bj.position, r.rj);

          // Should be relative to the body.
          r.ri.add2(xi, r.ri);
          r.ri.sub2(bi.position, r.ri);

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
            xj.add2(v1, v1);
            xj.add2(v2, v2);

            // Construct edge vector
            final edge = _sphereConvexEdge;
            v2.sub2(v1, edge);

            // Construct the same vector, but normalized
            final edgeUnit = _sphereConvexEdgeUnit;
            edge.unit(edgeUnit);

            // p is xi projected onto the edge
            final p = v3pool.get() as Vector3;
            final v1ToXi = v3pool.get() as Vector3;
            xi.sub2(v1, v1ToXi);
            final dot = v1ToXi.dot(edgeUnit);
            edgeUnit.scale2(dot, p);
            p.add2(v1, p);

            // Compute a vector from p to the center of the sphere
            final xiToP = v3pool.get();
            p.sub2(xi, xiToP);

            // Collision if the edge-sphere distance is less than the radius
            // AND if p is in between v1 and v2
            if (dot > 0 && dot * dot < edge.length2 && xiToP.length2 < R * R) {
              // Collision if the edge-sphere distance is less than the radius
              // Edge contact!
              if (justTest) {
                return true;
              }
              final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
              p.sub2(xj, r.rj);

              p.sub2(xi, r.ni);
              r.ni.normalize();

              r.ni.scale2(R, r.ri);

              // Should be relative to the body.
              r.rj.add2(xj, r.rj);
              r.rj.sub2(bj.position, r.rj);

              // Should be relative to the body.
              r.ri.add2(xi, r.ri);
              r.ri.sub2(bi.position, r.ri);

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
        List<Vector3> toRelease = [];
        for (int j = 0, nFaceverts = faceVerts.length; j != nFaceverts; j++) {
          toRelease.add(faceVerts[j]);
        }
        v3pool.release(toRelease);
      }
    }

    return false;
  }
  bool sphereParticle(
    Sphere sj,
    Particle si,
    Vector3 xj,
    Vector3 xi,
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
    normal.setValues(0, 0, 1);
    xi.sub2(xj, normal);
    final lengthSquared = normal.length2;

    if (lengthSquared <= sj.radius * sj.radius) {
      if (justTest) {
        return true;
      }
      final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
      normal.normalize();
      r.rj.setFrom(normal);
      r.rj.scale2(sj.radius, r.rj);
      r.ni.setFrom(normal); // Contact normal
      r.ni.negate();
      r.ri.setValues(0, 0, 0); // Center of particle
      result.add(r);
      createFrictionEquationsFromContact(r, frictionResult);
    }

    return false;
  }
  bool sphereHeightfield(
    Sphere si,
    Heightfield sj,
    Vector3 xi,
    Vector3 xj,
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
    final local = _sphereHeightfieldTmp1;
    Transform.pointToLocalFrame(xj, qj, xi, local);

    // Get the index of the data points to test against
    int iMinX = ((local.x - radius) / w).floor() - 1;
    int iMaxX = ((local.x + radius) / w).ceil() + 1;
    int iMinY = ((local.y - radius) / w).floor() - 1;
    int iMaxY = ((local.y + radius) / w).ceil() + 1;
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
    if (local.z - radius > max || local.z + radius < min) {
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
  bool sphereTrimesh(
    Sphere si,
    Trimesh sj,
    Vector3 xi,
    Vector3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
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
    final local = _sphereTrimeshlocal;
    final localSphereAABB = _sphereTrimeshLocalSphereAABB;
    final tmp = _sphereTrimeshTmp;
    final v2 = _sphereTrimeshV2;
    final relpos = _sphereTrimeshRelpos;
    final triangles = _sphereTrimeshTriangles;

    // Convert sphere position to local in the trimesh
    Transform.pointToLocalFrame(xj, qj, xi, local);

    // Get the aabb of the sphere locally in the trimesh
    final sphereRadius = si.radius;
    localSphereAABB.lowerBound.setValues(
      local.x - sphereRadius,
      local.y - sphereRadius,
      local.z - sphereRadius
    );
    localSphereAABB.upperBound.setValues(
      local.x + sphereRadius,
      local.y + sphereRadius,
      local.z + sphereRadius
    );

    sj.getTrianglesInAABB(localSphereAABB, triangles); //TODO fix retreived triangles

    // Triangle faces
    final va = _sphereTrimeshVa;
    final vb = _sphereTrimeshVb;
    final vc = _sphereTrimeshVc;
    final normal = _sphereTrimeshNormal;
    final v = _sphereTrimeshV;
    final radiusSquared = si.radius * si.radius;

    for (int i = 0; i < sj.indices.length/3; i++){//triangles.length
      for (int j = 0; j < 3; j++) {
        {
          sj.getVertex(sj.indices[i*3+j], v);//triangles[i]*3 + j
          v.sub2(local, relpos);
          if (relpos.length2 <= radiusSquared) {
            // Safe up
            v2.setFrom(v);
            Transform.pointToWorldFrame(xj, qj, v2, v);

            v.sub2(xi, relpos);

            if (justTest) {
              return true;
            }

            final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
            r.ni.setFrom(relpos);
            r.ni.normalize();

            // ri is the vector from sphere center to the sphere surface
            r.ri.setFrom(r.ni);
            r.ri.scale2(si.radius, r.ri);
            
            r.ri.add2(xi, r.ri);
            r.ri.sub2(bi.position, r.ri);
            r.rj.setFrom(v);
            r.rj.sub2(bj.position, r.rj);

            // Store result
            result.add(r);
            createFrictionEquationsFromContact(r, frictionResult);
          }
          {
            sj.getVertex(sj.indices[i*3 + j], edgeVertexA);//triangles[i]*3
            sj.getVertex(sj.indices[i*3 + ((j + 1) % 3)], edgeVertexB);//triangles[i]*3
            edgeVertexB.sub2(edgeVertexA, edgeVector);
            
            // Project sphere position to the edge
            local.sub2(edgeVertexB, tmp);
            final positionAlongEdgeB = tmp.dot(edgeVector);

            local.sub2(edgeVertexA, tmp);
            double positionAlongEdgeA = tmp.dot(edgeVector);

            if (positionAlongEdgeA > 0 && positionAlongEdgeB < 0) {
              // Now check the orthogonal distance from edge to sphere center
              local.sub2(edgeVertexA, tmp);
              edgeVectorUnit.setFrom(edgeVector);
              edgeVectorUnit.normalize();
              positionAlongEdgeA = tmp.dot(edgeVectorUnit);
              edgeVectorUnit.scale2(positionAlongEdgeA, tmp);
              tmp.add2(edgeVertexA, tmp); // tmp is now the sphere center position projected to the edge, defined locally in the trimesh frame
              
              final dist = tmp.distanceTo(local);
              if (dist < si.radius) {
                if (justTest) {
                  return true;
                }
                final r = createContactEquation(bi, bj, si, sj, rsi, rsj);

                tmp.sub2(local, r.ni);
                r.ni.normalize();
                r.ni.scale2(si.radius, r.ri);
                r.ri.add2(xi, r.ri);
                r.ri.sub2(bi.position, r.ri);

                Transform.pointToWorldFrame(xj, qj, tmp, tmp);
                tmp.sub2(bj.position, r.rj);

                Transform.vectorToWorldFrame(qj, r.ni, r.ni);
                Transform.vectorToWorldFrame(qj, r.ri, r.ri);

                result.add(r);
                createFrictionEquationsFromContact(r, frictionResult);
              }
            }
          }
        }
        {
          sj.getTriangleVertices(i, va, vb, vc);//triangles[i]
          sj.getIndicesNormal(i, normal);//triangles[i]

          local.sub2(va, tmp);
          double dist = tmp.dot(normal);
          normal.scale2(dist, tmp);
          local.sub2(tmp, tmp);

          // tmp is now the sphere position projected to the triangle plane
          dist = tmp.distanceTo(local);
          if (Ray.pointInTriangle(tmp, va, vb, vc) && dist < si.radius) {
            if (justTest) {
              return true;
            }
            final r = createContactEquation(bi, bj, si, sj, rsi, rsj);

            tmp.sub2(local, r.ni);
            r.ni.normalize();
            r.ni.scale2(si.radius, r.ri);
            r.ri.add2(xi, r.ri);
            r.ri.sub2(bi.position, r.ri);

            Transform.pointToWorldFrame(xj, qj, tmp, tmp);
            tmp.sub2(bj.position, r.rj);

            Transform.vectorToWorldFrame(qj, r.ni, r.ni);
            Transform.vectorToWorldFrame(qj, r.ri, r.ri);

            result.add(r);
            createFrictionEquationsFromContact(r, frictionResult);
          }
        }
      }
    }
    
    // // Check all edges
    // for (int i = 0; i < sj.indices.length/3; i++) {//triangles.length
    //   for (int j = 0; j < 3; j++) {
    //     sj.getVertex(sj.indices[i*3 + j], edgeVertexA);//triangles[i]*3
    //     sj.getVertex(sj.indices[i*3 + ((j + 1) % 3)], edgeVertexB);//triangles[i]*3
    //     edgeVertexB.sub2(edgeVertexA, edgeVector);
        
    //     // Project sphere position to the edge
    //     local.sub2(edgeVertexB, tmp);
    //     final positionAlongEdgeB = tmp.dot(edgeVector);

    //     local.sub2(edgeVertexA, tmp);
    //     double positionAlongEdgeA = tmp.dot(edgeVector);

    //     if (positionAlongEdgeA > 0 && positionAlongEdgeB < 0) {
    //       // Now check the orthogonal distance from edge to sphere center
    //       local.sub2(edgeVertexA, tmp);
    //       edgeVectorUnit.setFrom(edgeVector);
    //       edgeVectorUnit.normalize();
    //       positionAlongEdgeA = tmp.dot(edgeVectorUnit);
    //       edgeVectorUnit.scale(positionAlongEdgeA, tmp);
    //       tmp.add2(edgeVertexA, tmp); // tmp is now the sphere center position projected to the edge, defined locally in the trimesh frame
          
    //       final dist = tmp.distanceTo(local);
    //       if (dist < si.radius) {
    //         if (justTest) {
    //           return true;
    //         }
    //         final r = createContactEquation(bi, bj, si, sj, rsi, rsj);

    //         tmp.sub2(local, r.ni);
    //         r.ni.normalize();
    //         r.ni.scale(si.radius, r.ri);
    //         r.ri.add2(xi, r.ri);
    //         r.ri.sub2(bi.position, r.ri);

    //         Transform.pointToWorldFrame(xj, qj, tmp, tmp);
    //         tmp.sub2(bj.position, r.rj);

    //         Transform.vectorToWorldFrame(qj, r.ni, r.ni);
    //         Transform.vectorToWorldFrame(qj, r.ri, r.ri);

    //         result.add(r);
    //         createFrictionEquationsFromContact(r, frictionResult);
    //       }
    //     }
    //   }
    // }


    // for (int i = 0; i < sj.indices.length/3; i++) {//triangles.length
    //   sj.getTriangleVertices(i, va, vb, vc);//triangles[i]
    //   sj.getIndicesNormal(i, normal);//triangles[i]

    //   local.sub2(va, tmp);
    //   double dist = tmp.dot(normal);
    //   normal.scale(dist, tmp);
    //   local.sub2(tmp, tmp);

    //   // tmp is now the sphere position projected to the triangle plane
    //   dist = tmp.distanceTo(local);
    //   if (Ray.pointInTriangle(tmp, va, vb, vc) && dist < si.radius) {
    //     if (justTest) {
    //       return true;
    //     }
    //     final r = createContactEquation(bi, bj, si, sj, rsi, rsj);

    //     tmp.sub2(local, r.ni);
    //     r.ni.normalize();
    //     r.ni.scale(si.radius, r.ri);
    //     r.ri.add2(xi, r.ri);
    //     r.ri.sub2(bi.position, r.ri);

    //     Transform.pointToWorldFrame(xj, qj, tmp, tmp);
    //     tmp.sub2(bj.position, r.rj);

    //     Transform.vectorToWorldFrame(qj, r.ni, r.ni);
    //     Transform.vectorToWorldFrame(qj, r.ri, r.ri);

    //     result.add(r);
    //     createFrictionEquationsFromContact(r, frictionResult);
    //   }
    // }

    triangles.clear();
    return false;
  }

  bool boxBox(
    Box si,
    Box sj,
    Vector3 xi,
    Vector3 xj,
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
    Vector3 xi,
    Vector3 xj,
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
    Vector3 xi,
    Vector3 xj,
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
    return particleConvex(si.convexPolyhedronRepresentation, sj, xi, xj, qi, qj, bi, bj, si, sj, justTest);
  }
  bool boxHeightfield(
    Box si,
    Heightfield sj,
    Vector3 xi,
    Vector3 xj,
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
    return heightfieldConvex(si.convexPolyhedronRepresentation, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest);
  }
  bool boxTrimesh(
    Box si,
    Trimesh sj,
    Vector3 xi,
    Vector3 xj,
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
    return trimeshConvex(si.convexPolyhedronRepresentation, sj, xi, xj, qi, qj, bi, bj, rsi, rsj,justTest);
  }

  bool planeBox(
    Plane si,
    Box sj,
    Vector3 xi,
    Vector3 xj,
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
  bool planeParticle(
    Plane sj,
    Particle si,
    Vector3 xj,
    Vector3 xi,
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
    normal.setValues(0, 0, 1);
    bj.quaternion.vmult(normal, normal); // Turn normal according to plane orientation
    final relpos = _particlePlaneRelpos;
    xi.sub2(bj.position, relpos);
    final dot = normal.dot(relpos);
    if (dot <= 0.0) {
      if (justTest) {
        return true;
      }

      final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
      r.ni.setFrom(normal); // Contact normal is the plane normal
      r.ni.negate();
      r.ri.setValues(0, 0, 0); // Center of particle

      // Get particle position projected on plane
      final projected = _particlePlaneProjected;
      normal.scale2(normal.dot(xi), projected);
      xi.sub2(projected, projected);

      // rj is now the projected world position minus plane position
      r.rj.setFrom(projected);
      result.add(r);
      createFrictionEquationsFromContact(r, frictionResult);
    }
    return false;
  }
  bool planeConvex(
    Plane si,
    ConvexPolyhedron sj,
    Vector3 xi,
    Vector3 xj,
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
    worldNormal.setValues(0, 0, 1);
    qi.vmult(worldNormal, worldNormal) ;// Turn normal according to plane orientation

    int numContacts = 0;
    final relpos = _planeConvexRelpos;
    for (int i = 0; i != sj.vertices.length; i++) {
      // Get world convex vertex
      worldVertex.setFrom(sj.vertices[i]);
      qj.vmult(worldVertex, worldVertex);
      xj.add2(worldVertex, worldVertex);
      worldVertex.sub2(xi, relpos);

      final dot = worldNormal.dot(relpos);
      if (dot <= 0.0) {
        if (justTest) {
          return true;
        }

        final r = createContactEquation(bi, bj, si, sj, rsi, rsj);

        // Get vertex position projected on plane
        final projected = _planeConvexProjected;
        worldNormal.scale2(worldNormal.dot(relpos), projected);
        worldVertex.sub2(projected, projected);
        projected.sub2(xi, r.ri); // From plane to vertex projected on plane

        r.ni.setFrom(worldNormal) ;// Contact normal is the plane normal out from plane

        // rj is now just the vector from the convex center to the vertex
        worldVertex.sub2(xj, r.rj);

        // Make it relative to the body
        r.ri.add2(xi, r.ri);
        r.ri.sub2(bi.position, r.ri);
        r.rj.add2(xj, r.rj);
        r.rj.sub2(bj.position, r.rj);

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
  bool planeTrimesh(
    Plane planeShape,
    Trimesh sj,
    Vector3 planePos,
    Vector3 xj,
    Quaternion planeQuat,
    Quaternion qj,
    Body planeBody,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    // Make contacts!
    final v = Vector3.zero();

    final normal = _planeTrimeshNormal;
    normal.setValues(0, 0, 1);
    planeQuat.vmult(normal, normal);// Turn normal according to plane

    for (int i = 0; i < sj.vertices.length / 3; i++) {
      // Get world vertex from trimesh
      sj.getVertex(i, v);

      // Safe up
      final v2 = Vector3.zero();
      v2.setFrom(v);
      Transform.pointToWorldFrame(xj, qj, v2, v);

      // Check plane side
      final relpos = _planeTrimeshRelpos;
      v.sub2(planePos, relpos);
      final dot = normal.dot(relpos);

      if (dot <= 0.0) {
        if (justTest) {
          return true;
        }

        final r = createContactEquation(planeBody,bj,planeShape,sj, rsi, rsj);

        r.ni.setFrom(normal); // Contact normal is the plane normal

        // Get vertex position projected on plane
        final projected = _planeTrimeshProjected;
        normal.scale2(relpos.dot(normal), projected);
        v.sub2(projected, projected);

        // ri is the projected world position minus plane position
        r.ri.setFrom(projected);
        r.ri.sub2(planeBody.position, r.ri);

        r.rj.setFrom(v);
        r.rj.sub2(bj.position, r.rj);

        // Store result
        result.add(r);
        createFrictionEquationsFromContact(r, frictionResult);
      }
    }

    return false;
  }

  bool convexConvex(
    ConvexPolyhedron si,
    ConvexPolyhedron sj,
    Vector3 xi,
    Vector3 xj,
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
        r.ni..setFrom(sepAxis)..negate();
        q..setFrom(res[j].normal)..negate();
        q.scale2(res[j].depth, q);
        res[j].point.add2(q, ri);
        rj.setFrom(res[j].point);

        // Contact points are in world coordinates. Transform back to relative
        ri.sub2(xi, ri);
        rj.sub2(xj, rj);

        // Make relative to bodies
        ri.add2(xi, ri);
        ri.sub2(bi.position, ri);
        rj.add2(xj, rj);
        rj.sub2(bj.position, rj);

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
  bool heightfieldConvex(
    ConvexPolyhedron si,
    Heightfield sj,
    Vector3 xi,
    Vector3 xj,
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
  bool particleConvex(
    ConvexPolyhedron sj,
    Particle si,
    Vector3 xj,
    Vector3 xi,
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
    final cqj = Quaternion(0,0,0,1);
    local.setFrom(xi);
    local.sub2(xj, local); // Convert position to relative the convex origin
    cqj..setFrom(qj)..conjugate();
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
        final verts = sj.worldVertices[sj.faces[i][0]];
        final normal = sj.worldFaceNormals[i];
        final convexParticleVertexToParticle = Vector3.zero();
        // Check how much the particle penetrates the polygon plane.
        xi.sub2(verts, convexParticleVertexToParticle);
        final penetration = -normal.dot(convexParticleVertexToParticle);
        if (minPenetration == null || penetration.abs() < minPenetration.abs()) {
          if (justTest) {
            return true;
          }

          minPenetration = penetration;
          penetratedFaceIndex = i;
          penetratedFaceNormal.setFrom(normal);
          //numDetectedFaces++;
        }
      }

      if (penetratedFaceIndex != -1) {
        // Setup contact
        final r = createContactEquation(bi, bj, si, sj, rsi, rsj);
        penetratedFaceNormal.scale2(minPenetration!, worldPenetrationVec);
        // rj is the particle position projected to the face
        worldPenetrationVec.add2(xi, worldPenetrationVec);
        worldPenetrationVec.sub2(xj, worldPenetrationVec);
        r.rj.setFrom(worldPenetrationVec);
        //final projectedToFace = xi..sub(xj)..add(worldPenetrationVec);
        //projectedToFace.setFrom(r.rj);

        qj.vmult(r.rj,r.rj);
        r.ni..setFrom(penetratedFaceNormal)..negate(); // Contact normal
        r.ri.setValues(0, 0, 0); // Center of particle

        // Make relative to bodies
        r.ri.add2(xi, r.ri);
        r.ri.sub2(bi.position, r.ri);
        r.rj.add2(xj, r.rj);
        r.rj.sub2(bj.position, r.rj);

        result.add(r);
        createFrictionEquationsFromContact(r, frictionResult);
      } 
      else {
        //print('Point found inside convex, but did not find penetrating face!');
      }
    }
    return false;
  }
  bool trimeshConvex(
    ConvexPolyhedron si,
    Trimesh sj,
    Vector3 xi,
    Vector3 xj,
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
    final local = Vector3.zero();
    final localSphereAABB = _sphereTrimeshLocalSphereAABB;
    final triangles = _sphereTrimeshTriangles;
    final worldPillarOffset = _convexHeightfieldTmp2;

    Transform.pointToLocalFrame(xj, qj, xi, local);

    final sphereRadius = si.boundingSphereRadius;
    localSphereAABB.lowerBound.setValues(
      local.x - sphereRadius,
      local.y - sphereRadius,
      local.z - sphereRadius
    );
    localSphereAABB.upperBound.setValues(
      local.x + sphereRadius,
      local.y + sphereRadius,
      local.z + sphereRadius
    );

    sj.getTrianglesInAABB(localSphereAABB, triangles); //TODO fix 

    final va = Vector3.zero();
    final vb = Vector3.zero();
    final vc = Vector3.zero();
    final triangleNormal = Vector3.zero();
    final cp = ConvexPolyhedron(
      vertices:[va,vb,vc],
      faces: [[0, 1, 2]],
      normals: [triangleNormal]
    );
    // For each world polygon in the polyhedra
    for (int i = 0; i < sj.indices.length/3; i++) {
      bool intersecting = false;
      sj.getTriangleVertices(sj.indices[i], va, vb, vc);
      sj.getIndicesNormal(i, triangleNormal);

      Vector3 offsetResult = Vector3(
        (va.x+vb.x+vc.x)/3,
        (va.y+vb.y+vc.y)/3,
        (va.z+vb.z+vc.z)/3
      );
      cp.computeEdges();
      //cp.computeNormals();
      cp.updateBoundingSphereRadius();
      Transform.pointToWorldFrame(xj, qj, xi, worldPillarOffset);
      if (
        xi.distanceTo(worldPillarOffset) < sj.boundingSphereRadius
      ) {
        intersecting = convexConvex(cp, si, xj, worldPillarOffset, qj, qi, bj, bi, rsi, rsj, justTest);
      }

      if (justTest && intersecting) {
        return true;
      }
    }

    triangles.clear();
    
    return false;
  }

  bool heightfieldParticle(
    Heightfield si,
    Particle sj,
    Vector3 xi,
    Vector3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    final data = si.data;
    final radius = sj.boundingSphereRadius;
    final w = si.elementSize;
    final worldPillarOffset = _convexHeightfieldTmp2;

    // Get sphere position to heightfield local!
    final localConvexPos = _convexHeightfieldTmp1;
    Transform.pointToLocalFrame(xi, qi, xj, localConvexPos);

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

    List<double> minMax = [0,0];
    si.getRectMinMax(iMinX, iMinY, iMaxX, iMaxY, minMax);
    final min = minMax[0];
    final max = minMax[1];

    // Bail out if we can't touch the bounding height box
    if (localConvexPos.z - radius > max || localConvexPos.z + radius < min) {
      return false;
    }

    for (int i = iMinX; i < iMaxX; i++) {
      for (int j = iMinY; j < iMaxY; j++) {
        bool intersecting = false;
        // Lower triangle
        si.getConvexTrianglePillar(i, j, false);
        Transform.pointToWorldFrame(xi, qi, si.pillarOffset, worldPillarOffset);
        if (
          xj.distanceTo(worldPillarOffset) < si.pillarConvex.boundingSphereRadius
        ) {
          intersecting = particleConvex(si.pillarConvex, sj, worldPillarOffset, xj, qi, qj, bi, bj, rsi, rsj, justTest);
        }

        if (justTest && intersecting) {
          return true;
        }

        // Upper triangle
        si.getConvexTrianglePillar(i, j, true);
        Transform.pointToWorldFrame(xi, qi, si.pillarOffset, worldPillarOffset);
        if (
          xj.distanceTo(worldPillarOffset) < si.pillarConvex.boundingSphereRadius
        ) {
          intersecting = particleConvex(si.pillarConvex, sj, worldPillarOffset, xj, qi, qj, bi, bj, rsi, rsj, justTest);
        }

        if (justTest && intersecting) {
          return true;
        }
      }
    }
    return false;
  }
  bool particleTrimesh(
    Particle si,
    Trimesh sj,
    Vector3 xi,
    Vector3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false
  ]){
    final local = Vector3.zero();
    final localSphereAABB = _sphereTrimeshLocalSphereAABB;
    final triangles = _sphereTrimeshTriangles;
    final worldPillarOffset = _convexHeightfieldTmp2;

    Transform.pointToLocalFrame(xj, qj, xi, local);

    final sphereRadius = si.boundingSphereRadius;
    localSphereAABB.lowerBound.setValues(
      local.x - sphereRadius,
      local.y - sphereRadius,
      local.z - sphereRadius
    );
    localSphereAABB.upperBound.setValues(
      local.x + sphereRadius,
      local.y + sphereRadius,
      local.z + sphereRadius
    );

    sj.getTrianglesInAABB(localSphereAABB, triangles); //TODO fix 

    final va = Vector3.zero();
    final vb = Vector3.zero();
    final vc = Vector3.zero();
    final hullB = ConvexPolyhedron(
      vertices:[va,vb,vc],
      faces: [[0, 1, 2]],
    );
    // For each world polygon in the polyhedra
    for (int i = 0; i < triangles.length; i++) {
      bool intersecting = false;
      sj.getTriangleVertices(sj.indices[triangles[i]], va, vb, vc);
      Transform.pointToWorldFrame(xj, qj, xi, worldPillarOffset);
      if (
        xi.distanceTo(worldPillarOffset) < sj.boundingSphereRadius
      ) {
        intersecting = particleConvex(hullB, si, xj, xi, qj, qi, bj, bi, rsi, rsj, justTest);
      }

      if (justTest && intersecting) {
        return true;
      }
    }

    triangles.clear();
    
    return false;
  }

  bool trimeshTrimesh(
    Trimesh si,
    Trimesh sj,
    Vector3 xi,
    Vector3 xj,
    Quaternion qi,
    Quaternion qj,
    Body bi,
    Body bj,
    [
      Shape? rsi,
      Shape? rsj,
      bool justTest = false,
    ]
  ){
    final local = Vector3.zero();
    final localSphereAABB = _sphereTrimeshLocalSphereAABB;
    final triangles = _sphereTrimeshTriangles;
    final worldPillarOffset = _convexHeightfieldTmp2;

    Transform.pointToLocalFrame(xj, qj, xi, local);

    final sphereRadius = si.boundingSphereRadius;
    localSphereAABB.lowerBound.setValues(
      local.x - sphereRadius,
      local.y - sphereRadius,
      local.z - sphereRadius
    );
    localSphereAABB.upperBound.setValues(
      local.x + sphereRadius,
      local.y + sphereRadius,
      local.z + sphereRadius
    );

    sj.getTrianglesInAABB(localSphereAABB, triangles); //TODO fix 

    // Construct a temp hull for each triangle
    final va = Vector3.zero();
    final vb = Vector3.zero();
    final vc = Vector3.zero();
    final hullB = ConvexPolyhedron(
      vertices:[va,vb,vc],
      faces: [[0, 1, 2]],
    );
    final triangleNormal = Vector3.zero();

    for (int i = 0; i < si.indices.length / 3; i++) {
      bool intersecting = false;
      si.getTriangleVertices(i, va, vb, vc);
      si.getIndicesNormal(i, triangleNormal);

      //hullB.faceNormals = [triangleNormal];
      
      trimeshConvex(hullB, sj, xi, xj, qi, qj, bi, bj);

      
      sj.getTriangleVertices(sj.indices[triangles[i]], va, vb, vc);
      // Vector3 offsetResult = Vector3(
      //   (va.x+vb.x+vc.x)/3,
      //   (va.y+vb.y+vc.y)/3,
      //   (va.z+vb.z+vc.z)/3
      // );
      Transform.pointToWorldFrame(xj, qj, xi, worldPillarOffset);
      if (
        xi.distanceTo(worldPillarOffset) < sj.boundingSphereRadius
      ) {
        intersecting = trimeshConvex(hullB, sj, xj, xi, qj, qi, bj, bi, rsi, rsj, justTest);
      }

      if (justTest && intersecting) {
        return true;
      }
    }
    return false;
  }

  bool _pointInPolygon(List<Vector3> verts, Vector3 normal, Vector3 p) {
    bool? positiveResult;
    final N = verts.length;
    for (int i = 0; i != N; i++) {
      final v = verts[i];

      // Get edge to the next vertex
      final edge = _pointInPolygonEdge;
      verts[(i + 1) % N].sub2(v, edge);

      // Get cross product between polygon normal and the edge
      final edgeXNormal = _pointInPolygonEdgeXNormal;
      //final edge_x_normal = Vector3.zero();
      edge.cross2(normal, edgeXNormal);

      // Get vector between point and current vertex
      final vertexToP = _pointInPolygonVtp;
      p.sub2(v, vertexToP);

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