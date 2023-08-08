import '../utils/utils.dart';

import '../collision/broadphase.dart';
import '../collision/aabb.dart';
import '../world/world_class.dart';
import '../objects/body.dart';

/// Sweep and prune broadphase along one axis.
class SAPBroadphase extends Broadphase {
  /// List of bodies currently in the broadphase.
  List<Body> axisList = [];

  /// The world to search in.
  //World? world;

  /// Axis to sort the bodies along.
  /// Set to 0 for x axis, and 1 for y axis.
  /// For best performance, pick the axis where bodies are most distributed.
  late AxisIndex axisIndex = AxisIndex.x;

  late void Function(Body body) _addBodyHandler;
  late void Function(Body body) _removeBodyHandler;

  SAPBroadphase(world):super() {
    final axisList = this.axisList;

    _addBodyHandler = (Body body){
      axisList.add(body);
    };

    _removeBodyHandler = (Body body){
      final idx = axisList.indexOf(body);
      if (idx != -1) {
        axisList.removeAt(idx);
      }
    };

    if (world) {
      setWorld(world);
    }
  }

  /// Check if the bounds of two bodies overlap, along the given SAP axis.
  static bool checkBounds(Body bi,Body bj, AxisIndex axisIndex) {
    late double biPos;
    late double bjPos;

    if (axisIndex == AxisIndex.x) {
      biPos = bi.position.x;
      bjPos = bj.position.x;
    } else if (axisIndex == AxisIndex.y) {
      biPos = bi.position.y;
      bjPos = bj.position.y;
    } else if (axisIndex == AxisIndex.z) {
      biPos = bi.position.z;
      bjPos = bj.position.z;
    }

    final ri = bi.boundingRadius,
      rj = bj.boundingRadius,
      boundA2 = biPos + ri,
      boundB1 = bjPos - rj;

    return boundB1 < boundA2;
  }

  // Note: these are identical, save for x/y/z lowerbound
  /// insertionSortX
  static List<Body> insertionSortX(List<Body> a){
    for (int i = 1, l = a.length; i < l; i++) {
      final v = a[i];
      int j;
      for (j = i - 1; j >= 0; j--) {
        if (a[j].aabb.lowerBound.x <= v.aabb.lowerBound.x) {
          break;
        }
        a[j + 1] = a[j];
      }
      a[j + 1] = v;
    }
    return a;
  }

  /// insertionSortY
  static List<Body> insertionSortY(List<Body> a){
    for (int i = 1, l = a.length; i < l; i++) {
      final v = a[i];
      int j;
      for (j = i - 1; j >= 0; j--) {
        if (a[j].aabb.lowerBound.y <= v.aabb.lowerBound.y) {
          break;
        }
        a[j + 1] = a[j];
      }
      a[j + 1] = v;
    }
    return a;
  }

  /// insertionSortZ
  static List<Body> insertionSortZ(List<Body> a){
    for (int i = 1, l = a.length; i < l; i++) {
      final v = a[i];
      int j;
      for (j = i - 1; j >= 0; j--) {
        if (a[j].aabb.lowerBound.z <= v.aabb.lowerBound.z) {
          break;
        }
        a[j + 1] = a[j];
      }
      a[j + 1] = v;
    }
    return a;
  }

  /// Change the world
  @override
  void setWorld(World world) {
    // Clear the old axis array
    axisList.clear();

    // Add all bodies from the new world
    for (int i = 0; i < world.bodies.length; i++) {
      axisList.add(world.bodies[i]);
    }

    // Remove old handlers, if any
    world.removeEventListener('addBody', _addBodyHandler);
    world.removeEventListener('removeBody', _removeBodyHandler);

    // Add handlers to update the list of bodies.
    world.addEventListener('addBody', _addBodyHandler);
    world.addEventListener('removeBody', _removeBodyHandler);

    this.world = world;
    dirty = true;
  }

  /// Collect all collision pairs
  @override
  void collisionPairs(World world, List<Body> p1, List<Body> p2) {
    final bodies = axisList;
    final N = bodies.length;
    final axisIndex = this.axisIndex;
    int i;
    int j;

    if (dirty) {
      sortList();
      dirty = false;
    }

    // Look through the list
    for (i = 0; i != N; i++) {
      final bi = bodies[i];

      for (j = i + 1; j < N; j++) {
        final bj = bodies[j];

        if (!needBroadphaseCollision(bi, bj)) {
          continue;
        }

        if (!SAPBroadphase.checkBounds(bi, bj, axisIndex)) {
          break;
        }

        intersectionTest(bi, bj, p1, p2);
      }
    }
  }

  void sortList() {
    final axisList = this.axisList;
    final axisIndex = this.axisIndex;
    final N = axisList.length;

    // Update AABBs
    for (int i = 0; i != N; i++) {
      final bi = axisList[i];
      if (bi.aabbNeedsUpdate) {
        bi.updateAABB();
      }
    }

    // Sort the list
    if (axisIndex == AxisIndex.x) {
      SAPBroadphase.insertionSortX(axisList);
    } else if (axisIndex == AxisIndex.y) {
      SAPBroadphase.insertionSortY(axisList);
    } else if (axisIndex == AxisIndex.z) {
      SAPBroadphase.insertionSortZ(axisList);
    }
  }

  /// Computes the variance of the body positions and estimates the best axis to use.
  /// Will automatically set property `axisIndex`.
  void autoDetectAxis() {
    double sumX = 0;
    double sumX2 = 0;
    double sumY = 0;
    double sumY2 = 0;
    double sumZ = 0;
    double sumZ2 = 0;
    final bodies = axisList;
    final N = bodies.length;
    final invN = 1 / N;

    for (int i = 0; i != N; i++) {
      final b = bodies[i];

      final centerX = b.position.x;
      sumX += centerX;
      sumX2 += centerX * centerX;

      final centerY = b.position.y;
      sumY += centerY;
      sumY2 += centerY * centerY;

      final centerZ = b.position.z;
      sumZ += centerZ;
      sumZ2 += centerZ * centerZ;
    }

    final varianceX = sumX2 - sumX * sumX * invN;
    final varianceY = sumY2 - sumY * sumY * invN;
    final varianceZ = sumZ2 - sumZ * sumZ * invN;

    if (varianceX > varianceY) {
      if (varianceX > varianceZ) {
        axisIndex = AxisIndex.x;
      } else {
        axisIndex = AxisIndex.z;
      }
    } else if (varianceY > varianceZ) {
      axisIndex = AxisIndex.y;
    } else {
      axisIndex = AxisIndex.z;
    }
  }

  /// Returns all the bodies within an AABB.
  /// @param result An array to store resulting bodies in.
  @override
  List<Body> aabbQuery(World world, AABB aabb, [List<Body>? result]){
    result ??= [];
    if (dirty) {
      sortList();
      dirty = false;
    }

    final axisList = this.axisList;
    for (int i = 0; i < axisList.length; i++) {
      final b = axisList[i];

      if (b.aabbNeedsUpdate) {
        b.updateAABB();
      }

      if (b.aabb.overlaps(aabb)) {
        result.add(b);
      }
    }

    return result;
  }
}
