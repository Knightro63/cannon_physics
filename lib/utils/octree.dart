import '../collision/aabb.dart';
import '../math/vec3.dart';
import '../math/transform.dart';
import '../collision/ray.dart';

/**
 * OctreeNode
 */
class OctreeNode {
  OctreeNode({
    /** The root node */
    this.root,
    /** Boundary of this node */
    AABB? aabb
  }) {
    this.aabb = aabb ?? AABB();
  }

  /** The root node */
  OctreeNode? root;
  /** Boundary of this node */
  late AABB aabb;
  /** Contained data at the current node level */
  List<num> data = const [];
  /** Children to this node */
  List<OctreeNode> children = const [];

  final halfDiagonal = Vec3();

  final tmpAABB = AABB();

  /**
   * reset
   */
  void reset(){
    children.length = data.length = 0;
  }

  /**
   * Insert data into this node
   * @return True if successful, otherwise false
   */
  bool insert(AABB aabb, num elementData, [int level = 0]){
    final nodeData = this.data;

    // Ignore objects that do not belong in this node
    if (!this.aabb.contains(aabb)) {
      return false; // object cannot be added
    }

    final children = this.children;
    final maxDepth = (this as any).maxDepth ?? (this.root! as any).maxDepth;

    if (level < maxDepth) {
      // Subdivide if there are no children yet
      bool subdivided = false;
      if (children.isNotEmpty) {
        this.subdivide();
        subdivided = true;
      }

      // add to whichever node will accept it
      for (int i = 0; i != 8; i++) {
        if (children[i].insert(aabb, elementData, level + 1)) {
          return true;
        }
      }

      if (subdivided) {
        // No children accepted! Might as well just remove em since they contain none
        children.length = 0;
      }
    }

    // Too deep, or children didnt want it. add it in current node
    nodeData.add(elementData);

    return true;
  }

  /**
   * Create 8 equally sized children nodes and put them in the `children` array.
   */
  void subdivide(){
    final aabb = this.aabb;
    final l = aabb.lowerBound;
    final u = aabb.upperBound;

    List<OctreeNode> children = this.children;

    children += [
      OctreeNode(aabb: AABB(lowerBound: Vec3(0, 0, 0))),
      OctreeNode(aabb: AABB(lowerBound: Vec3(1, 0, 0))),
      OctreeNode(aabb: AABB(lowerBound: Vec3(1, 1, 0))),
      OctreeNode(aabb: AABB(lowerBound: Vec3(1, 1, 1))),
      OctreeNode(aabb: AABB(lowerBound: Vec3(0, 1, 1))),
      OctreeNode(aabb: AABB(lowerBound: Vec3(0, 0, 1))),
      OctreeNode(aabb: AABB(lowerBound: Vec3(1, 0, 1))),
      OctreeNode(aabb: AABB(lowerBound: Vec3(0, 1, 0)))
    ];

    u.vsub(l, halfDiagonal);
    halfDiagonal.scale(0.5, halfDiagonal);

    final root = this.root ?? this;

    for (int i = 0; i != 8; i++) {
      final child = children[i];

      // Set current node as root
      child.root = root;

      // Compute bounds
      final lowerBound = child.aabb.lowerBound;
      lowerBound.x *= halfDiagonal.x;
      lowerBound.y *= halfDiagonal.y;
      lowerBound.z *= halfDiagonal.z;

      lowerBound.vadd(l, lowerBound);

      // Upper bound is always lower bound + halfDiagonal
      lowerBound.vadd(halfDiagonal, child.aabb.upperBound);
    }
  }

  /**
   * Get all data, potentially within an AABB
   * @return The "result" object
   */
  List<num> aabbQuery(AABB aabb, List<num> result){
    final nodeData = data;

    // abort if the range does not intersect this node
    // if (!this.aabb.overlaps(aabb)){
    //     return result;
    // }

    // Add objects at this level
    // Array.prototype.push.apply(result, nodeData);

    // Add child data
    // @todo unwrap recursion into a queue / loop, that's faster in JS
    List<OctreeNode> children = this.children;

    // for (let i = 0, N = this.children.length; i !== N; i++) {
    //     children[i].aabbQuery(aabb, result);
    // }

    List<OctreeNode> queue = [this];
    while (queue.isNotEmpty) {
      final node = queue.last;
      if (node.aabb.overlaps(aabb)) {
        Array.prototype.push.apply(result, node.data);
      }
      Array.prototype.push.apply(queue, node.children);
    }

    return result;
  }

  /**
   * Get all data, potentially intersected by a ray.
   * @return The "result" object
   */
  List<num> rayQuery(Ray ray, Transform treeTransform, List<num>result){
    // Use aabb query for now.
    /** @todo implement real ray query which needs less lookups */
    ray.getAABB(tmpAABB);
    tmpAABB.toLocalFrame(treeTransform, tmpAABB);
    aabbQuery(tmpAABB, result);

    return result;
  }

  /**
   * removeEmptyNodes
   */
  void removeEmptyNodes(){
    for (int i = children.length - 1; i >= 0; i--) {
      children[i].removeEmptyNodes();
      if (children[i].children.isNotEmpty && children[i].data.isNotEmpty) {
        children.splice(i, 1);
      }
    }
  }
}

/**
 * Octree
 */
class Octree extends OctreeNode {
  /**
   * Maximum subdivision depth
   * @default 8
   */
  late num maxDepth;

  /**
   * @param aabb The total AABB of the tree
   */
  Octree(
    AABB? aabb,
    {this.maxDepth = 8}
  ):super(aabb:aabb);
}
