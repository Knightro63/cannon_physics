import '../math/vec3.dart';
import 'ray_class.dart';
import 'dart:math' as math;
import '../math/transform.dart';
import '../math/quaternion.dart';

/// Axis aligned bounding box class.
class AABB {
  /// The lower bound of the bounding box
  Vec3 lowerBound = Vec3();
  /// The upper bound of the bounding box
  Vec3 upperBound = Vec3();

  /// Axis aligned bounding box class.
  /// 
  /// [upperBound] The max location of the boundry
  /// 
  /// [lowerBound] The min location of the boundry
  AABB({Vec3? upperBound,Vec3? lowerBound}) {
    if(lowerBound != null){
      this.lowerBound.copy(lowerBound);
    }
    if(upperBound != null){
      this.upperBound.copy(upperBound);
    }
  }

  final Vec3 _tmp = Vec3();

  final List<Vec3> _transformIntoFrameCorners = [
    Vec3(),
    Vec3(),
    Vec3(),
    Vec3(),
    Vec3(),
    Vec3(),
    Vec3(),
    Vec3(),
  ];

  /// Set the AABB bounds from a set of points.
  /// [points] An array of Vec3's.
  /// return The self object 
  AABB setFromPoints(List<Vec3> points, [Vec3? position, Quaternion? quaternion, num? skinSize]){
    final l = lowerBound;
    final u = upperBound;
    final q = quaternion;

    // Set to the first point
    l.copy(points[0]);
    if (q != null) {
      q.vmult(l, l);
    }
    u.copy(l);

    for (int i = 1; i < points.length; i++) {
      Vec3 p = points[i];
      if (q != null) {
        q.vmult(p, _tmp);
        p = _tmp;
      }

      if (p.x > u.x) {
        u.x = p.x;
      }
      if (p.x < l.x) {
        l.x = p.x;
      }
      if (p.y > u.y) {
        u.y = p.y;
      }
      if (p.y < l.y) {
        l.y = p.y;
      }
      if (p.z > u.z) {
        u.z = p.z;
      }
      if (p.z < l.z) {
        l.z = p.z;
      }
    }

    // Add offset
    if (position != null) {
      position.vadd(l, l);
      position.vadd(u, u);
    }

    if (skinSize != null) {
      l.x -= skinSize;
      l.y -= skinSize;
      l.z -= skinSize;
      u.x += skinSize;
      u.y += skinSize;
      u.z += skinSize;
    }

    return this;
  }

  /// Copy bounds from an AABB to this AABB
  /// [aabb] Source to copy from
  /// return this object, for chainability
  AABB copy(AABB aabb) {
    lowerBound.copy(aabb.lowerBound);
    upperBound.copy(aabb.upperBound);
    return this;
  }

  /// Clone an AABB
  AABB clone() {
    // return AABB(
    //   lowerBound: lowerBound.clone(),
    //   upperBound: upperBound.clone()
    // );
    return AABB().copy(this);
  }

  /// Extend this AABB so that it covers the given AABB too.
  void extend(AABB aabb) {
    lowerBound.x = math.min(lowerBound.x, aabb.lowerBound.x);
    upperBound.x = math.max(upperBound.x, aabb.upperBound.x);
    lowerBound.y = math.min(lowerBound.y, aabb.lowerBound.y);
    upperBound.y = math.max(upperBound.y, aabb.upperBound.y);
    lowerBound.z = math.min(lowerBound.z, aabb.lowerBound.z);
    upperBound.z = math.max(upperBound.z, aabb.upperBound.z);
  }

  /// Returns true if the given AABB overlaps this AABB.
  bool overlaps(AABB aabb) {
    final l1 = lowerBound;
    final u1 = upperBound;
    final l2 = aabb.lowerBound;
    final u2 = aabb.upperBound;

    //      l2        u2
    //      |---------|
    // |--------|
    // l1       u1

    final overlapsX = (l2.x <= u1.x && u1.x <= u2.x) || (l1.x <= u2.x && u2.x <= u1.x);
    final overlapsY = (l2.y <= u1.y && u1.y <= u2.y) || (l1.y <= u2.y && u2.y <= u1.y);
    final overlapsZ = (l2.z <= u1.z && u1.z <= u2.z) || (l1.z <= u2.z && u2.z <= u1.z);

    return overlapsX && overlapsY && overlapsZ;
  }

  // Mostly for debugging
  double volume() {
    final l = lowerBound;
    final u = upperBound;
    return (u.x - l.x) * (u.y - l.y) * (u.z - l.z);
  }

  /// Returns true if the given AABB is fully contained in this AABB.
  bool contains(AABB aabb) {
    final l1 = lowerBound;
    final u1 = upperBound;
    final l2 = aabb.lowerBound;
    final u2 = aabb.upperBound;

    //      l2        u2
    //      |---------|
    // |---------------|
    // l1              u1

    return (
      (l1.x <= l2.x && u1.x >= u2.x) && 
      (l1.y <= l2.y && u1.y >= u2.y) && 
      (l1.z <= l2.z && u1.z >= u2.z)
    );
  }

  void getCorners(Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3 e,Vec3 f,Vec3 g,Vec3 h) {
    final l = lowerBound;
    final u = upperBound;

    a.copy(l);
    b.set(u.x, l.y, l.z);
    c.set(u.x, u.y, l.z);
    d.set(l.x, u.y, u.z);
    e.set(u.x, l.y, u.z);
    f.set(l.x, u.y, l.z);
    g.set(l.x, l.y, u.z);
    h.copy(u);
  }

  /// Get the representation of an AABB in another frame.
  /// [return] The "target" AABB object.
  AABB toLocalFrame(Transform frame, AABB target){
    final corners = _transformIntoFrameCorners;
    final a = corners[0];
    final b = corners[1];
    final c = corners[2];
    final d = corners[3];
    final e = corners[4];
    final f = corners[5];
    final g = corners[6];
    final h = corners[7];

    // Get corners in current frame
    getCorners(a, b, c, d, e, f, g, h);

    // Transform them to local frame
    for (int i = 0; i != 8; i++) {
      final corner = corners[i];
      frame.pointToLocal(corner, corner);
    }

    return target.setFromPoints(corners);
  }

  /// Get the representation of an AABB in the global frame.
  /// [return] The "target" AABB object.
  AABB toWorldFrame(Transform frame,AABB target){
    final corners = _transformIntoFrameCorners;
    final a = corners[0];
    final b = corners[1];
    final c = corners[2];
    final d = corners[3];
    final e = corners[4];
    final f = corners[5];
    final g = corners[6];
    final h = corners[7];

    // Get corners in current frame
    getCorners(a, b, c, d, e, f, g, h);

    // Transform them to local frame
    for (int i = 0; i != 8; i++) {
      final corner = corners[i];
      frame.pointToWorld(corner, corner);
    }

    return target.setFromPoints(corners);
  }

  /// Check if the AABB is hit by a ray.
  bool overlapsRay(Ray ray) {
    final direction = ray.direction;
    final from = ray.from;
    // final t = 0

    // ray.direction is unit direction vector of ray
    final dirFracX = 1 / direction.x;
    final dirFracY = 1 / direction.y;
    final dirFracZ = 1 / direction.z;

    // this.lowerBound is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
    final t1 = (lowerBound.x - from.x) * dirFracX;
    final t2 = (upperBound.x - from.x) * dirFracX;
    final t3 = (lowerBound.y - from.y) * dirFracY;
    final t4 = (upperBound.y - from.y) * dirFracY;
    final t5 = (lowerBound.z - from.z) * dirFracZ;
    final t6 = (upperBound.z - from.z) * dirFracZ;

    // final tmin = math.max(math.max(math.min(t1, t2), math.min(t3, t4)));
    // final tmax = math.min(math.min(math.max(t1, t2), math.max(t3, t4)));
    final tmin = math.max(math.max(math.min(t1, t2), math.min(t3, t4)), math.min(t5, t6));
    final tmax = math.min(math.min(math.max(t1, t2), math.max(t3, t4)), math.max(t5, t6));

    // if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behing us
    if (tmax < 0) {
      //t = tmax;
      return false;
    }

    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax) {
      //t = tmax;
      return false;
    }

    return true;
  }
}
