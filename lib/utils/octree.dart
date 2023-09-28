import '../collision/aabb.dart';
import '../math/vec3.dart';
import '../math/transform.dart';
import '../collision/ray_class.dart';

class OctreeNode {
  OctreeNode({
    this.root,
    AABB? aabb
  }) {
    this.aabb = aabb?.clone() ?? AABB();
  }
  int? maxDepth;
  /// The root node 
  OctreeNode? root;
  /// Boundary of this node
  late AABB aabb;
  /// Contained data at the current node level
  List<int> data = [];
  /// Children to this node
  List<OctreeNode> children = [];

  Vec3 halfDiagonal = Vec3();

  AABB tmpAABB = AABB();

  /// reset
  void reset(){
    children.clear();
    data.clear();
  }

  /// Insert data into this node
  /// @return True if successful, otherwise false
  bool insert(AABB aabb, int elementData, [int level = 0]){
    final nodeData = data;

    // Ignore objects that do not belong in this node
    if (!this.aabb.contains(aabb)) {
      return false; // object cannot be added
    }

    final children = this.children;
    final maxDepth = this.maxDepth ?? root!.maxDepth;//(this as any).maxDepth ?? (root! as any).maxDepth;

    if (level < maxDepth!) {
      // Subdivide if there are no children yet
      bool subdivided = false;
      if (children.isEmpty) {
        subdivide();
        subdivided = true;
      }
      // add to whichever node will accept it
      for (int i = 0; i != 8; i++) {
        if (children[i].insert(aabb, elementData, level + 1)) {
          print(elementData);
          return true;
        }
      }

      if (subdivided) {
        // No children accepted! Might as well just remove em since they contain none
        children.clear();
      }
    }

    // Too deep, or children didnt want it. add it in current node
    nodeData.add(elementData);
    return true;
  }

  /// Create 8 equally sized children nodes and put them in the `children` array
  void subdivide(){
    final aabb = this.aabb;
    final l = aabb.lowerBound;
    final u = aabb.upperBound;

    final children = this.children;

    children.addAll([
      OctreeNode(aabb: AABB(lowerBound: Vec3(0, 0, 0))),
      OctreeNode(aabb: AABB(lowerBound: Vec3(1, 0, 0))),
      OctreeNode(aabb: AABB(lowerBound: Vec3(1, 1, 0))),
      OctreeNode(aabb: AABB(lowerBound: Vec3(1, 1, 1))),
      OctreeNode(aabb: AABB(lowerBound: Vec3(0, 1, 1))),
      OctreeNode(aabb: AABB(lowerBound: Vec3(0, 0, 1))),
      OctreeNode(aabb: AABB(lowerBound: Vec3(1, 0, 1))),
      OctreeNode(aabb: AABB(lowerBound: Vec3(0, 1, 0)))
    ]);

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

  /// Get all data, potentially within an AABB
  /// @return The "result" object
  List<int> aabbQuery(AABB aabb, List<int> result){
    var queue = [this];
    while (queue.isNotEmpty) {
      var node = queue.removeLast();
      if (node.aabb.overlaps(aabb)) {
        result.addAll(node.data);
      }
      queue.addAll(node.children);
    }

    return result;
  }

  /// Get all data, potentially intersected by a ray.
  /// @return The "result" object
  List<int> rayQuery(Ray ray, Transform treeTransform, List<int>result){
    //if(ray.direction.length() == 0) return;
    // Use aabb query for now.
    /** @todo implement real ray query which needs less lookups */
    ray.getAABB(tmpAABB);
    tmpAABB.toLocalFrame(treeTransform, tmpAABB);
    aabbQuery(tmpAABB, result);

    return result;
  }

  void removeEmptyNodes(){
    // for (int i = children.length - 1; i >= 0; i--) {
    //   children[i].removeEmptyNodes();
    //   if (children[i].children.isNotEmpty && children[i].data.isNotEmpty) {
    //     children.removeAt(i);
    //   }
    // }
    final queue = [this];
    while (queue.isNotEmpty) {
        final node = queue.removeLast();
        for (int i = node.children.length - 1; i >= 0; i--) {
            if(node.children[i].data.isNotEmpty){
                node.children.removeAt(i);
            }
        }
        queue.addAll(node.children);
    }
  }
}

class Octree extends OctreeNode {
  /// @param aabb The total AABB of the tree
  Octree({
    AABB? aabb,
    int maxDepth = 8
  }):super(aabb:aabb){
    this.maxDepth = maxDepth;
  }
}
