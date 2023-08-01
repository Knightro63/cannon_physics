import 'dart:math' as math;
import '../shapes/convex_polyhedron.dart';
import '../math/vec3.dart';
import './shape.dart';

/**
 * Cylinder class.
 * @example
 *     const radiusTop = 0.5
 *     const radiusBottom = 0.5
 *     const height = 2
 *     const numSegments = 12
 *     const cylinderShape = new CANNON.Cylinder(radiusTop, radiusBottom, height, numSegments)
 *     const cylinderBody = new CANNON.Body({ mass: 1, shape: cylinderShape })
 *     world.addBody(cylinderBody)
 */
class Cylinder extends ConvexPolyhedron {
  /// The radius of the top of the Cylinder.
  double radiusTop;
  /// The radius of the bottom of the Cylinder.
  double radiusBottom;
  ///The height of the Cylinder.
  double height;
  /// The number of segments to build the cylinder out of.
  int numSegments;

  /**
   * @param radiusTop The radius of the top of the Cylinder.
   * @param radiusBottom The radius of the bottom of the Cylinder.
   * @param height The height of the Cylinder.
   * @param numSegments The number of segments to build the cylinder out of.
   */
  Cylinder({this.radiusTop = 1, this.radiusBottom = 1, this.height = 1, this.numSegments = 8}):super() {
    if (radiusTop < 0) {
      throw('The cylinder radiusTop cannot be negative.');
    }

    if (radiusBottom < 0) {
      throw('The cylinder radiusBottom cannot be negative.');
    }

    final int N = numSegments;
    final List<int> bottomface = [];
    final List<int> topface = [];
    final List<Vec3> axes =[];

    // First bottom point
    vertices.add(Vec3(-radiusBottom * math.sin(0), -height * 0.5, radiusBottom * math.cos(0)));
    bottomface.add(0);

    // First top point
    vertices.add(Vec3(-radiusTop * math.sin(0), height * 0.5, radiusTop * math.cos(0)));
    topface.add(1);

    for (int i = 0; i < N; i++) {
      final theta = ((2 * math.pi) / N) * (i + 1);
      final thetaN = ((2 * math.pi) / N) * (i + 0.5);
      if (i < N - 1) {
        // Bottom
        vertices.add(Vec3(-radiusBottom * math.sin(theta), -height * 0.5, radiusBottom * math.cos(theta)));
        bottomface.add(2 * i + 2);
        // Top
        vertices.add(Vec3(-radiusTop * math.sin(theta), height * 0.5, radiusTop * math.cos(theta)));
        topface.add(2 * i + 3);

        // Face
        faces.add([2 * i, 2 * i + 1, 2 * i + 3, 2 * i + 2]);
      } else {
        faces.add([2 * i, 2 * i + 1, 1, 0]); // Connect
      }

      // Axis: we can cut off half of them if we have even number of segments
      if (N % 2 == 1 || i < N / 2) {
        axes.add(Vec3(-math.sin(thetaN), 0, math.cos(thetaN)));
      }
    }
    faces.add(bottomface);
    axes.add(Vec3(0, 1, 0));

    // Reorder top face
    final List<int> temp = [];
    for (int i = 0; i < topface.length; i++) {
      temp.add(topface[topface.length - i - 1]);
    }
    faces.add(temp);

    type = ShapeType.cylinder;
    radiusTop = radiusTop;
    radiusBottom = radiusBottom;
    height = height;
    numSegments = numSegments;
  }
}
