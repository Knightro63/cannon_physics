import 'dart:math' as math;
import '../math/vec3.dart';
import '../math/quaternion.dart';
import '../math/transform.dart';
import '../collision/raycast_result.dart';
import '../shapes/shape.dart';
import '../collision/aabb.dart';
import '../objects/body.dart';
import '../shapes/sphere.dart';
import '../shapes/box.dart';
import '../shapes/plane.dart';
import '../shapes/heightfield.dart';
import '../shapes/convex_polyhedron.dart';
import '../shapes/trimesh.dart';
import '../world/world_class.dart';

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
  Vec3? from;
  Vec3? to;
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

final _v0 = Vec3();
final _intersect = Vec3();
final _v1 = Vec3();
final _v2 = Vec3();
final _intersectHeightfieldLocalRay = Ray();

/// A line in 3D space that intersects bodies and return points.
class Ray {
  late Vec3 from;
  late Vec3 to;
  Vec3 direction = Vec3();
  
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



  Ray([Vec3? from, Vec3? to]) {
    this.from = from ?? Vec3();
    this.to = to ?? Vec3();

    callback = (result){};
  }

  final _tmpAABB = AABB();
  final List<Body> _tmpArray = [];

  final _intersectBodyXi = Vec3();
  final _intersectBodyQi = Quaternion();
  final _intersectPoint = Vec3();

  final a = Vec3();
  final b = Vec3();
  final c = Vec3();
  final d = Vec3();

  //final _tmpRaycastResult = RaycastResult();
  final ConvexOptions _intersectConvexOptions = ConvexOptions(
    faceList: [0],
  );
  final worldPillarOffset = Vec3();
  //final _intersectHeightfieldLocalRay = Ray();
  final List<int> _intersectHeightfieldIndex = [];
  //final _intersectHeightfieldMinMax = [];

  final _rayIntersectSphereIntersectionPoint = Vec3();
  final _rayIntersectSphereNormal = Vec3();

  final _intersectConvexNormal = Vec3();
  //final _intersectConvexMinDistNormal = Vec3();
  //final _intersectConvexMinDistIntersect = Vec3();
  final _intersectConvexVector = Vec3();

  final _intersectTrimeshNormal = Vec3();
  final _intersectTrimeshLocalDirection = Vec3();
  final _intersectTrimeshLocalFrom = Vec3();
  final _intersectTrimeshLocalTo = Vec3();
  final _intersectTrimeshWorldNormal = Vec3();
  final _intersectTrimeshWorldIntersectPoint = Vec3();
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
      from.copy(options.from!);
    }
    if (options.to != null) {
      to.copy(options.to!);
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

      body.quaternion.mult(body.shapeOrientations[i], qi);
      body.quaternion.vmult(body.shapeOffsets[i], xi);
      xi.vadd(body.position, xi);

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
    to.vsub(from, direction);
    direction.normalize();
  }

  void _intersectShape(Shape shape, Quaternion quat, Vec3 position, Body body) {
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

  void _intersectBox(Box box, Quaternion quat, Vec3 position, Body body, Shape reportedShape) {
    return _intersectConvex(box.convexPolyhedronRepresentation, quat, position, body, reportedShape);
  }

  void _intersectPlane(Plane shape, Quaternion quat, Vec3 position, Body body, Shape reportedShape) {
    final from = this.from;
    final to = this.to;
    final direction = this.direction;

    // Get plane normal
    final worldNormal = Vec3(0, 0, 1);
    quat.vmult(worldNormal, worldNormal);

    final len = Vec3();
    from.vsub(position, len);
    final planeToFrom = len.dot(worldNormal);
    to.vsub(position, len);
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

    final planePointToFrom = Vec3();
    final dirScaledWithT = Vec3();
    final hitPointWorld = Vec3();

    from.vsub(position, planePointToFrom);
    final t = -worldNormal.dot(planePointToFrom) / nDotDir;
    direction.scale(t, dirScaledWithT);
    from.vadd(dirScaledWithT, hitPointWorld);

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

  void _intersectHeightfield(Heightfield shape, Quaternion quat, Vec3 position,Body body, Shape reportedShape) {
    //final data = shape.data;
    //final w = shape.elementSize;
    // Convert the ray to local heightfield coordinates
    final localRay = _intersectHeightfieldLocalRay; //Ray(from, to);
    localRay.from.copy(from);
    localRay.to.copy(to);
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

  void _intersectSphere(Sphere sphere, Quaternion quat, Vec3 position, Body body, Shape reportedShape) {
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

      intersectionPoint.vsub(position, normal);
      normal.normalize();

      _reportIntersection(normal, intersectionPoint, reportedShape, body, -1);
    } else {
      final d1 = (-b - math.sqrt(delta)) / (2 * a);
      final d2 = (-b + math.sqrt(delta)) / (2 * a);

      if (d1 >= 0 && d1 <= 1) {
        from.lerp(to, d1, intersectionPoint);
        intersectionPoint.vsub(position, normal);
        normal.normalize();
        _reportIntersection(normal, intersectionPoint, reportedShape, body, -1);
      }

      if (result.shouldStop) {
        return;
      }

      if (d2 >= 0 && d2 <= 1) {
        from.lerp(to, d2, intersectionPoint);
        intersectionPoint.vsub(position, normal);
        normal.normalize();
        _reportIntersection(normal, intersectionPoint, reportedShape, body, -1);
      }
    }
  }

  void _intersectConvex(
    ConvexPolyhedron shape,
    Quaternion quat,
    Vec3 position,
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
      vector.copy(vertices[face[0]]);
      q.vmult(vector, vector);
      vector.vadd(x, vector);

      // ...but make it relative to the ray from. We'll fix this later.
      vector.vsub(from, vector);

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
      direction.scale(scalar, _intersectPoint);
      _intersectPoint.vadd(from, _intersectPoint);

      // a is the point we compare points b and c with.
      a.copy(vertices[face[0]]);
      q.vmult(a, a);
      x.vadd(a, a);

      for (int i = 1; !result.shouldStop && i < face.length - 1; i++) {
        // Transform 3 vertices to world coords
        b.copy(vertices[face[i]]);
        c.copy(vertices[face[i + 1]]);
        q.vmult(b, b);
        q.vmult(c, c);
        x.vadd(b, b);
        x.vadd(c, c);

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
    Vec3 position,
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

    treeTransform.position.copy(position);
    treeTransform.quaternion.copy(quat);

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

    localTo.vsub(localFrom, localDirection);
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
      a.vsub(localFrom, vector);

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
      localDirection.scale(scalar, _intersectPoint);
      _intersectPoint.vadd(localFrom, _intersectPoint);

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
  void _reportIntersection(Vec3 normal, Vec3 hitPointWorld, Shape shape, Body body, int hitFaceIndex) {
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
  /// {@link https://www.blackpawn.com/texts/pointinpoly/default.html here} but without the division
  static bool pointInTriangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c,[Vec3? target]) {
    c.vsub(a, _v0);
    b.vsub(a, _v1);
    p.vsub(a, _v2);
    final dot00 = _v0.dot(_v0);
    final dot01 = _v0.dot(_v1);
    final dot02 = _v0.dot(_v2);
    final dot11 = _v1.dot(_v1);
    final dot12 = _v1.dot(_v2);
    final double u = dot11 * dot02 - dot01 * dot12;
    final double v = dot00 * dot12 - dot01 * dot02;
    return u  >= 0 && v >= 0 && (u + v) < (dot00 * dot11 - dot01 * dot01);
  }
  static double distanceFromIntersection(Vec3 from, Vec3 direction, Vec3 position) {
    // v0 is vector from from to position
    position.vsub(from, _v0);
    final dot = _v0.dot(direction);

    // intersect = direction*dot + from
    direction.scale(dot, _intersect);
    _intersect.vadd(from, _intersect);

    final distance = position.distanceTo(_intersect);

    return distance;
  }
}


