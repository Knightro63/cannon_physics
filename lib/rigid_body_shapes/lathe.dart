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
		final vertex = Vector3.zero();

		// generate vertices, uvs and normals
		for (int i = 0; i <= numSegments; i++) {
			final phi = phiStart + i * inverseSegments * phiLength;
			final sin = math.sin( phi );
			final cos = math.cos( phi );

			for(int j = 0; j <= points.length-1; j++){
				vertex.x = points[ j ].x * sin;
				vertex.y = points[ j ].y;
				vertex.z = points[ j ].x * cos;
				vertices.add(vertex);
			}
		}

		// indices
		for(int i = 0; i < numSegments; i ++ ) {
			for(int j = 0; j < points.length - 1; j++) {
				final base = j + i * points.length;

				final a = base;
				final b = base + points.length;
				final c = base + points.length + 1;
				final d = base + 1;

				indices.add([ a, b, d ]);
				indices.add([ c, d, b ]);
			}
		}
    axes.add(Vector3(0, 1, 0));
    numSegments = numSegments;

    init(vertices, indices, faceNormals, axes, boundingSphereRadius);
	}
}