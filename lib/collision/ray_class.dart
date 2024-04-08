import 'dart:math' as math;
import '../math/vec3.dart';
import '../math/quaternion.dart';
import '../math/transform.dart';
import '../collision/raycast_result.dart';
import '../rigid_body_shapes/shape.dart';
import '../collision/aabb.dart';
import '../objects/rigid_body.dart';
import '../rigid_body_shapes/sphere.dart';
import '../rigid_body_shapes/box.dart';
import '../rigid_body_shapes/plane.dart';
import '../rigid_body_shapes/heightfield.dart';
import '../rigid_body_shapes/convex_polyhedron.dart';
import '../rigid_body_shapes/trimesh.dart';
import '../world/world_class.dart';
import 'package:vector_math/vector_math.dart' hide Plane, Sphere;

/// RAY_MODES
enum RayMode{closest,any,all}

typedef RaycastCallback = void Function(RaycastResult result);

/// RayOptions
class RayOptions{
  RayOptions({
    this.from,
    this.to,
    this.callback,
    this.mode,
    this.checkCollisionResponse,
    this.collisionFilterGroup,
    this.collisionFilterMask,
    this.result,
    this.skipBackfaces
  });
  Vector3? from;
  Vector3? to;
  RayMode? mode;
  RaycastResult? result;
  bool? skipBackfaces;
  int? collisionFilterMask;
  int? collisionFilterGroup;
  bool? checkCollisionResponse;
  RaycastCallback? callback;
}

class ConvexOptions{
  ConvexOptions({
    required this.faceList
});

  List<int> faceList;
}

final _v0 = Vector3.zero();
final _intersect = Vector3.zero();
final _v1 = Vector3.zero();
final _v2 = Vector3.zero();
final _intersectHeightfieldLocalRay = Ray();

/// A line in 3D space that intersects bodies and return points.
class Ray {
  late Vector3 from;
  late Vector3 to;
  Vector3 direction = Vector3.zero();
  
  /// The precision of the ray. Used when checking parallelity etc.
  /// [default] 0.0001
  double precision = 0.0001;

  /// Set to `false` if you don't want the Ray to take `collisionResponse` flags into account on bodies and shapes.
  /// @default true
  bool checkCollisionResponse = true;

  /// If set to `true`, the ray skips any hits with normal.dot(rayDirection) < 0.
  /// @default false
  bool skipBackfaces = false;

  /// collisionFilterMask
  /// @default -1
  int collisionFilterMask = -1;

  /// collisionFilterGroup
  /// @default -1
  int collisionFilterGroup = -1;
  
  /// The intersection mode. Should be any,all, closest.
  /// @default RayMode.any
  RayMode mode = RayMode.any;

  /// Current result object.
  RaycastResult result = RaycastResult();

  /// Will be set to `true` during intersectWorld() if the ray hit anything.
  bool hasHit = false;

  /// User-provided result callback. Will be used if mode is Ray.ALL.
  late RaycastCallback callback;

  //void operator []=(int addr, int value) => puts(addr,Uint8List.fromList([value]));
  operator [](ShapeType type){
    if(type == ShapeType.sphere) {
      return _intersectSphere;
    }
    if(type == ShapeType.plane) {
      return _intersectPlane;
    }
    if(type == ShapeType.box) {
      return _intersectBox;
    }
    if(type == ShapeType.cylinder) {
      return _intersectConvex;
    }
    if(type == ShapeType.convex) {
      return _intersectConvex;
    }
    if(type == ShapeType.heightfield) {
      return _intersectHeightfield;
    }
    if(type == ShapeType.trimesh) {
      return _intersectTrimesh;
    }
  }



  Ray([Vector3? from, Vector3? to]) {
    this.from = from ?? Vector3.zero();
    this.to = to ?? Vector3.zero();

    callback = (result){};
  }

  final _tmpAABB = AABB();
  final List<Body> _tmpArray = [];

  final _intersectBodyXi = Vector3.zero();
  final _intersectBodyQi = Quaternion(0,0,0,1);
  final _intersectPoint = Vector3.zero();

  final a = Vector3.zero();
  final b = Vector3.zero();
  final c = Vector3.zero();
  final d = Vector3.zero();

  //final _tmpRaycastResult = RaycastResult();
  final ConvexOptions _intersectConvexOptions = ConvexOptions(
    faceList: [0],
  );
  final worldPillarOffset = Vector3.zero();
  //final _intersectHeightfieldLocalRay = Ray();
  final List<int> _intersectHeightfieldIndex = [];
  //final _intersectHeightfieldMinMax = [];

  final _rayIntersectSphereIntersectionPoint = Vector3.zero();
  final _rayIntersectSphereNormal = Vector3.zero();

  final _intersectConvexNormal = Vector3.zero();
  //final _intersectConvexMinDistNormal = Vector3.zero();
  //final _intersectConvexMinDistIntersect = Vector3.zero();
  final _intersectConvexVector = Vector3.zero();

  final _intersectTrimeshNormal = Vector3.zero();
  final _intersectTrimeshLocalDirection = Vector3.zero();
  final _intersectTrimeshLocalFrom = Vector3.zero();
  final _intersectTrimeshLocalTo = Vector3.zero();
  final _intersectTrimeshWorldNormal = Vector3.zero();
  final _intersectTrimeshWorldIntersectPoint = Vector3.zero();
  //final _intersectTrimeshLocalAABB = AABB();
  final List<int> _intersectTrimeshTriangles = [];
  final _intersectTrimeshTreeTransform = Transform();

  /// Do itersection against all bodies in the given World.
  /// @return True if the ray hit anything, otherwise false.
  bool intersectWorld(World world, RayOptions options) {
    mode = options.mode ?? RayMode.any;
    result = options.result ?? RaycastResult();
    skipBackfaces = options.skipBackfaces ?? true;
    collisionFilterMask = options.collisionFilterMask ?? -1;
    collisionFilterGroup = options.collisionFilterGroup ?? -1;
    checkCollisionResponse = options.checkCollisionResponse ?? true;

    if (options.from != null) {
      from.setFrom(options.from!);
    }
    if (options.to != null) {
      to.setFrom(options.to!);
    }

    callback = options.callback ?? (RaycastResult result){};
    hasHit = false;

    result.reset();
    _updateDirection();

    getAABB(_tmpAABB);
    _tmpArray.clear();
    world.broadphase.aabbQuery(world, _tmpAABB, _tmpArray);
    intersectBodies(_tmpArray);

    return hasHit;
  }

  /// Shoot a ray at a body, get back information about the hit.
  /// @deprecated @param result set the result property of the Ray instead.
  void intersectBody(Body body, [RaycastResult? result]) {
    if (result != null) {
      this.result = result;
      _updateDirection();
    }
    final checkCollisionResponse = this.checkCollisionResponse;

    if (checkCollisionResponse && !body.collisionResponse) {
      return;
    }

    if (
      (collisionFilterGroup & body.collisionFilterMask) == 0 ||
      (body.collisionFilterGroup & collisionFilterMask) == 0
    ) {
      return;
    }

    final xi = _intersectBodyXi;
    final qi = _intersectBodyQi;

    for (int i = 0; i < body.shapes.length; i++) {
      final shape = body.shapes[i];

      if (checkCollisionResponse && !shape.collisionResponse) {
        continue; // Skip
      }

      body.quaternion.multiply2(body.shapeOrientations[i], qi);
      body.quaternion.vmult(body.shapeOffsets[i], xi);
      xi.add2(body.position, xi);

      _intersectShape(shape, qi, xi, body);

      if (this.result.shouldStop) {
        break;
      }
    }
  }

  /// Shoot a ray at an array bodies, get back information about the hit.
  /// @param bodies An array of Body objects.
  /// @deprecated @param result set the result property of the Ray instead.
  void intersectBodies(List<Body> bodies, [RaycastResult? result]) {
    if (result != null) {
      this.result = result;
      _updateDirection();
    }

    for (int i = 0, l = bodies.length; !this.result.shouldStop && i < l; i++) {
      intersectBody(bodies[i]);
    }
  }

  /// Updates the direction vector.
  void _updateDirection() {
    to.sub2(from, direction);
    direction.normalize();
  }

  void _intersectShape(Shape shape, Quaternion quat, Vector3 position, Body body) {
    final from = this.from;

    // Checking boundingSphere
    final distance = Ray.distanceFromIntersection(from, direction, position);
    if (distance > shape.boundingSphereRadius) {
      return;
    }

    final intersectMethod = this[shape.type];
    if (intersectMethod != null) {
      intersectMethod.call(shape, quat, position, body, shape);
    }
  }

  void _intersectBox(Box box, Quaternion quat, Vector3 position, Body body, Shape reportedShape) {
    return _intersectConvex(box.convexPolyhedronRepresentation, quat, position, body, reportedShape);
  }

  void _intersectPlane(Plane shape, Quaternion quat, Vector3 position, Body body, Shape reportedShape) {
    final from = this.from;
    final to = this.to;
    final direction = this.direction;

    // Get plane normal
    final worldNormal = Vector3(0, 0, 1);
    quat.vmult(worldNormal, worldNormal);

    final len = Vector3.zero();
    from.sub2(position, len);
    final planeToFrom = len.dot(worldNormal);
    to.sub2(position, len);
    final planeToTo = len.dot(worldNormal);

    if (planeToFrom * planeToTo > 0) {
      // "from" and "to" are on the same side of the plane... bail out
      return;
    }

    if (from.distanceTo(to) < planeToFrom) {
      return;
    }

    final nDotDir = worldNormal.dot(direction);

    if (nDotDir.abs() < precision) {
      // No intersection
      return;
    }

    final planePointToFrom = Vector3.zero();
    final dirScaledWithT = Vector3.zero();
    final hitPointWorld = Vector3.zero();

    from.sub2(position, planePointToFrom);
    final t = -worldNormal.dot(planePointToFrom) / nDotDir;
    direction.scale2(t, dirScaledWithT);
    from.add2(dirScaledWithT, hitPointWorld);

    _reportIntersection(worldNormal, hitPointWorld, reportedShape, body, -1);
  }

  /// Get the world AABB of the ray.
  void getAABB(AABB aabb) {
    final lowerBound = aabb.lowerBound;
    final upperBound = aabb.upperBound;
    final to = this.to;
    final from = this.from;
    lowerBound.x = math.min(to.x, from.x);
    lowerBound.y = math.min(to.y, from.y);
    lowerBound.z = math.min(to.z, from.z);
    upperBound.x = math.max(to.x, from.x);
    upperBound.y = math.max(to.y, from.y);
    upperBound.z = math.max(to.z, from.z);
  }

  void _intersectHeightfield(Heightfield shape, Quaternion quat, Vector3 position,Body body, Shape reportedShape) {
    //final data = shape.data;
    //final w = shape.elementSize;
    // Convert the ray to local heightfield coordinates
    final localRay = _intersectHeightfieldLocalRay; //Ray(from, to);
    localRay.from.setFrom(from);
    localRay.to.setFrom(to);
    Transform.pointToLocalFrame(position, quat, localRay.from, localRay.from);
    Transform.pointToLocalFrame(position, quat, localRay.to, localRay.to);
    localRay._updateDirection();

    // Get the index of the data points to test against
    final index = _intersectHeightfieldIndex;
    int iMinX;
    int iMinY;
    int iMaxX;
    int iMaxY;

    // Set to max
    iMinX = iMinY = 0;
    iMaxX = iMaxY = shape.data.length - 1;

    final aabb = AABB();
    localRay.getAABB(aabb);

    shape.getIndexOfPosition(aabb.lowerBound.x, aabb.lowerBound.y, index, true);
    iMinX = math.max(iMinX, index[0]);
    iMinY = math.max(iMinY, index[1]);
    shape.getIndexOfPosition(aabb.upperBound.x, aabb.upperBound.y, index, true);
    iMaxX = math.min(iMaxX, index[0] + 1);
    iMaxY = math.min(iMaxY, index[1] + 1);

    for (int i = iMinX; i < iMaxX; i++) {
      for (int j = iMinY; j < iMaxY; j++) {
        if (result.shouldStop) {
          return;
        }

        shape.getAabbAtIndex(i, j, aabb);
        if (!aabb.overlapsRay(localRay)) {
          continue;
        }

        // Lower triangle
        shape.getConvexTrianglePillar(i, j, false);
        Transform.pointToWorldFrame(position, quat, shape.pillarOffset, worldPillarOffset);
        _intersectConvex(shape.pillarConvex, quat, worldPillarOffset, body, reportedShape, _intersectConvexOptions);

        if (result.shouldStop) {
          return;
        }

        // Upper triangle
        shape.getConvexTrianglePillar(i, j, true);
        Transform.pointToWorldFrame(position, quat, shape.pillarOffset, worldPillarOffset);
        _intersectConvex(shape.pillarConvex, quat, worldPillarOffset, body, reportedShape, _intersectConvexOptions);
      }
    }
  }

  void _intersectSphere(Sphere sphere, Quaternion quat, Vector3 position, Body body, Shape reportedShape) {
    final from = this.from;
    final to = this.to;
    final r = sphere.radius;

    final a = math.pow((to.x - from.x), 2) + math.pow((to.y - from.y),2) + math.pow((to.z - from.z),2);
    final b =
      2 *
      ((to.x - from.x) * (from.x - position.x) +
        (to.y - from.y) * (from.y - position.y) +
        (to.z - from.z) * (from.z - position.z));
    final c = math.pow((from.x - position.x), 2) + math.pow((from.y - position.y), 2) + math.pow((from.z - position.z), 2) - math.pow(r, 2);

    final double delta = (math.pow(b, 2) - 4 * a * c).toDouble();

    final intersectionPoint = _rayIntersectSphereIntersectionPoint;
    final normal = _rayIntersectSphereNormal;

    if (delta < 0) {
      // No intersection
      return;
    } else if (delta == 0) {
      // single intersection point
      from.lerp(to, delta, intersectionPoint);

      intersectionPoint.sub2(position, normal);
      normal.normalize();

      _reportIntersection(normal, intersectionPoint, reportedShape, body, -1);
    } else {
      final d1 = (-b - math.sqrt(delta)) / (2 * a);
      final d2 = (-b + math.sqrt(delta)) / (2 * a);

      if (d1 >= 0 && d1 <= 1) {
        from.lerp(to, d1, intersectionPoint);
        intersectionPoint.sub2(position, normal);
        normal.normalize();
        _reportIntersection(normal, intersectionPoint, reportedShape, body, -1);
      }

      if (result.shouldStop) {
        return;
      }

      if (d2 >= 0 && d2 <= 1) {
        from.lerp(to, d2, intersectionPoint);
        intersectionPoint.sub2(position, normal);
        normal.normalize();
        _reportIntersection(normal, intersectionPoint, reportedShape, body, -1);
      }
    }
  }

  void _intersectConvex(
    ConvexPolyhedron shape,
    Quaternion quat,
    Vector3 position,
    Body body,
    Shape reportedShape,
    [ConvexOptions? options]
  ){
    //final minDistNormal = intersectConvex_minDistNormal;
    final normal = _intersectConvexNormal;
    final vector = _intersectConvexVector;
    //final minDistIntersect = intersectConvex_minDistIntersect;
    final faceList = options?.faceList;

    // Checking faces
    final faces = shape.faces;

    final vertices = shape.vertices;
    final normals = shape.faceNormals;
    final direction = this.direction;

    final from = this.from;
    final to = this.to;
    final fromToDistance = from.distanceTo(to);

    //const minDist = -1;
    final nFaces = faceList != null? faceList.length : faces.length;
    final result = this.result;

    for (int j = 0; !result.shouldStop && j < nFaces; j++) {
      final fi = faceList != null? faceList[j] : j;

      final face = faces[fi];
      final faceNormal = normals[fi]!;
      final q = quat;
      final x = position;

      // determine if ray intersects the plane of the face
      // note: this works regardless of the direction of the face normal

      // Get plane point in world coordinates...
      vector.setFrom(vertices[face[0]]);
      q.vmult(vector, vector);
      vector.add2(x, vector);

      // ...but make it relative to the ray from. We'll fix this later.
      vector.sub2(from, vector);

      // Get plane normal
      q.vmult(faceNormal, normal);

      // If this dot product is negative, we have something interesting
      final dot = direction.dot(normal);

      // Bail out if ray and plane are parallel
      // if (dot.abs() < precision) {
      //   continue;
      // }

      // calc distance to plane
      final scalar = normal.dot(vector) / dot;

      // if negative distance, then plane is behind ray
      if (scalar < 0) {
        continue;
      }

      // Intersection point is from + direction * scalar
      direction.scale2(scalar, _intersectPoint);
      _intersectPoint.add2(from, _intersectPoint);

      // a is the point we compare points b and c with.
      a.setFrom(vertices[face[0]]);
      q.vmult(a, a);
      x.add2(a, a);

      for (int i = 1; !result.shouldStop && i < face.length - 1; i++) {
        // Transform 3 vertices to world coords
        b.setFrom(vertices[face[i]]);
        c.setFrom(vertices[face[i + 1]]);
        q.vmult(b, b);
        q.vmult(c, c);
        x.add2(b, b);
        x.add2(c, c);

        final distance = _intersectPoint.distanceTo(from);

        if (!(Ray.pointInTriangle(_intersectPoint, a, b, c) || Ray.pointInTriangle(_intersectPoint, b, a, c)) || distance > fromToDistance) {
          continue;
        }

        _reportIntersection(normal, _intersectPoint, reportedShape, body, fi);
      }
    }
  }

  /// @todo Optimize by transforming the world to local space first.
  /// @todo Use Octree lookup
  void _intersectTrimesh(
    Trimesh mesh,
    Quaternion quat,
    Vector3 position,
    Body body,
    Shape reportedShape,
    [ConvexOptions? options]
  ){
    final normal = _intersectTrimeshNormal;
    final triangles = _intersectTrimeshTriangles;
    final treeTransform = _intersectTrimeshTreeTransform;
    final vector = _intersectConvexVector;
    final localDirection = _intersectTrimeshLocalDirection;
    final localFrom = _intersectTrimeshLocalFrom;
    final localTo = _intersectTrimeshLocalTo;
    final worldIntersectPoint = _intersectTrimeshWorldIntersectPoint;
    final worldNormal = _intersectTrimeshWorldNormal;

    // Checking faces
    final indices = mesh.indices;

    // final vertices = mesh.vertices;
    // final normals = mesh.faceNormals

    final from = this.from;
    final to = this.to;
    final direction = this.direction;

    treeTransform.position.setFrom(position);
    treeTransform.quaternion.setFrom(quat);

    // Transform ray to local space!
    Transform.vectorToLocalFrame(position, quat, direction, localDirection);
    Transform.pointToLocalFrame(position, quat, from, localFrom);
    Transform.pointToLocalFrame(position, quat, to, localTo);

    localTo.x *= mesh.scale.x;
    localTo.y *= mesh.scale.y;
    localTo.z *= mesh.scale.z;
    localFrom.x *= mesh.scale.x;
    localFrom.y *= mesh.scale.y;
    localFrom.z *= mesh.scale.z;

    localTo.sub2(localFrom, localDirection);
    localDirection.normalize();

    final fromToDistanceSquared = localFrom.distanceSquared(localTo);

    mesh.tree.rayQuery(this, treeTransform, triangles);
    
    for (int i = 0, N = triangles.length; !result.shouldStop && i != N; i++) {
      final trianglesIndex = triangles[i];

      mesh.getIndicesNormal(trianglesIndex, normal);

      // determine if ray intersects the plane of the face
      // note: this works regardless of the direction of the face normal

      // Get plane point in world coordinates...
      mesh.getVertex(indices[trianglesIndex * 3], a);

      // ...but make it relative to the ray from. We'll fix this later.
      a.sub2(localFrom, vector);

      // If this dot product is negative, we have something interesting
      final dot = localDirection.dot(normal);

      // Bail out if ray and plane are parallel
      // if (dot.abs() < precision){
      //   continue;
      // }

      // calc distance to plane
      final scalar = normal.dot(vector) / dot;

      // if negative distance, then plane is behind ray
      if (scalar < 0) {
        continue;
      }

      // Intersection point is from + direction * scalar
      localDirection.scale2(scalar, _intersectPoint);
      _intersectPoint.add2(localFrom, _intersectPoint);

      // Get triangle vertices
      mesh.getVertex(indices[trianglesIndex * 3 + 1], b);
      mesh.getVertex(indices[trianglesIndex * 3 + 2], c);

      final squaredDistance = _intersectPoint.distanceSquared(localFrom);

      if (!(Ray.pointInTriangle(_intersectPoint, b, a, c) || Ray.pointInTriangle(_intersectPoint, a, b, c)) || squaredDistance > fromToDistanceSquared) {
        continue;
      }

      // transform intersectpoint and normal to world
      Transform.vectorToWorldFrame(quat, normal, worldNormal);
      Transform.pointToWorldFrame(position, quat, _intersectPoint, worldIntersectPoint);
      _reportIntersection(worldNormal, worldIntersectPoint, reportedShape, body, trianglesIndex);
    }
    triangles.clear();
  }

  /// @return True if the intersections should continue
  void _reportIntersection(Vector3 normal, Vector3 hitPointWorld, Shape shape, Body body, int hitFaceIndex) {
    final from = this.from;
    final to = this.to;
    final distance = from.distanceTo(hitPointWorld);
    final result = this.result;

    // Skip back faces?
    if (skipBackfaces && normal.dot(direction) > 0) {
      return;
    }

    result.hitFaceIndex = hitFaceIndex;// ?? -1;

    switch (mode) {
      case RayMode.all:
        hasHit = true;
        result.set(from, to, normal, hitPointWorld, shape, body, distance);
        result.hasHit = true;
        callback(result);
        break;

      case RayMode.closest:
        // Store if closer than current closest
        if (distance < result.distance || !result.hasHit) {
          hasHit = true;
          result.hasHit = true;
          result.set(from, to, normal, hitPointWorld, shape, body, distance);
        }
        break;

      case RayMode.any:
        // Report and stop.
        hasHit = true;
        result.hasHit = true;
        result.set(from, to, normal, hitPointWorld, shape, body, distance);
        result.shouldStop = true;
        break;
    }
  }

  /// As per "Barycentric Technique" as named
  static bool pointInTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c,[Vector3? target]) {
    c.sub2(a, _v0);
    b.sub2(a, _v1);
    p.sub2(a, _v2);
    final dot00 = _v0.dot(_v0);
    final dot01 = _v0.dot(_v1);
    final dot02 = _v0.dot(_v2);
    final dot11 = _v1.dot(_v1);
    final dot12 = _v1.dot(_v2);
    final double u = dot11 * dot02 - dot01 * dot12;
    final double v = dot00 * dot12 - dot01 * dot02;
    return u  >= 0 && v >= 0 && (u + v) < (dot00 * dot11 - dot01 * dot01);
  }
  static double distanceFromIntersection(Vector3 from, Vector3 direction, Vector3 position) {
    // v0 is vector from from to position
    position.sub2(from, _v0);
    final dot = _v0.dot(direction);

    // intersect = direction*dot + from
    direction.scale2(dot, _intersect);
    _intersect.add2(from, _intersect);

    final distance = position.distanceTo(_intersect);

    return distance;
  }
}


