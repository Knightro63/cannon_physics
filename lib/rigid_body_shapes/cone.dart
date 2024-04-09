import 'dart:math' as math;
import 'shape.dart';
import 'package:vector_math/vector_math.dart';
import 'convex_polyhedron.dart';

/// Cone class.
/// @example
///     const radiusTop = 0.5
///     const radiusBottom = 0.5
///     const height = 2
///     const numSegments = 12
///     const ConeShape = new CANNON.Cone(radiusTop, radiusBottom, height, numSegments)
///     const ConeBody = new CANNON.Body({ mass: 1, shape: ConeShape })
///     world.addBody(ConeBody)
class Cone extends ConvexPolyhedron {
  /// The radius of the Cone.
  double radius;
  ///The height of the Cone.
  double height;
  /// The number of segments to build the cone out of.
  int numSegments;

  /// @param radius The radius of the Cone.
  /// @param height The height of the Cone.
  /// @param numSegments The number of segments to build the Cone out of.
  Cone({
    this.radius = 1,
    this.height = 1, 
    this.numSegments = 8,
  }):super(type: ShapeType.cone){
    if (radius < 0) {
      throw('The cylinder radiusBottom cannot be negative.');
    }

    final int N = numSegments;
    List<int> bottomface = [];
    List<Vector3> axes =[];

    vertices = [];
    faces = [];

    vertices.add(Vector3(0,height*0.5,0));
    vertices.add(Vector3(-radius * math.sin(0), -height*0.5, radius * math.cos(0)));
    bottomface.add(1);

    for(int i = 0; i < N; i++){
      final theta = ((2 * math.pi) / N)*(i+1);
      if (i < N - 1) {
        vertices.add(Vector3(-radius * math.sin(theta), -height*0.5, radius * math.cos(theta)));
        
        bottomface.add(i+2);
        // Face
        faces.add([0,i+2,i+1]);
      }
      else{
        faces.add([0,1,i+1]);
      }
    }
    axes.add(Vector3(0, 1, 0));
    faces.add(bottomface);

    init(vertices, faces, faceNormals, axes, boundingSphereRadius);
  }
}
