import 'package:vector_math/vector_math.dart';
import 'dart:math' as math;
import 'convex_polyhedron.dart';
import 'shape.dart';

class LatheShape extends ConvexPolyhedron {
  /// The radius of the top of the Cylinder.
  List<Vector2> points;
  /// The number of segments to build the cylinder out of.
  int numSegments;

	LatheShape({
    required this.points, // = [ Vector2( 0, - 0.5 ), Vector2( 0.5, 0 ), Vector2( 0, 0.5 ) ]
    this.numSegments = 8, 
    double phiStart = 0, 
    double phiLength = math.pi * 2,
    ShapeType type = ShapeType.convex
  }):super(type: type){

		// clamp phiLength so it's in range of [ 0, 2PI ]
		phiLength = phiLength.clamp(0, math.pi*2);

		// buffers
		final List<List<int>> indices = [];
		final List<Vector3> vertices = [];
    List<Vector3> axes =[];

		// helper variables
		double inverseSegments = 1.0 / numSegments;

		// generate vertices, uvs and normals
		for (int i = 0; i <= numSegments; i++) {
			final phi = phiStart + i * inverseSegments * phiLength;
			for(int j = points.length-1; j >= 0; j--){
        final vertex = Vector3.zero();
				vertex.x = points[j].x * math.sin(phi);
				vertex.y = points[j].y;
				vertex.z = points[j].x * math.cos(phi);
				vertices.add(vertex);

        //axes.add(Vector3(-math.sin(phi), 0, math.cos(phi)));
			}
		}

		// indices
		for(int i = 0; i < numSegments; i ++ ) {
			for(int j = points.length-2; j >= 0; j--) {
				final base = j + i * points.length;

				final a = base;
				final b = base + points.length;
				final c = base + points.length + 1;
				final d = base + 1;

				indices.add([ a, b, d ]);
				indices.add([ c, d, b ]);
			}
		}
    //axes.add(Vector3(0, 1, 0));
    numSegments = numSegments;

    // print("VERTS:");
    // for(int i = 0; i < vertices.length;i++){
    //   print(vertices[i].storage);
    // }

    init(vertices, indices);
	}
}