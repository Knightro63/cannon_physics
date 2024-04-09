import 'dart:math' as math;
import 'lathe.dart';
import 'shape.dart';
import 'package:vector_math/vector_math.dart';

/// Cylinder class.
/// @example
///     const radiusTop = 0.5
///     const radiusBottom = 0.5
///     const height = 2
///     const numSegments = 12
///     const cylinderShape = new CANNON.Cylinder(radiusTop, radiusBottom, height, numSegments)
///     const cylinderBody = new CANNON.Body({ mass: 1, shape: cylinderShape })
///     world.addBody(cylinderBody)
class Capsule extends LatheShape {
  /// The radius of the top of the Cylinder.
  double radiusTop;
  /// The radius of the bottom of the Cylinder.
  double radiusBottom;
  ///The height of the Cylinder.
  double height;
  /// The number of segments to build the cylinder out of.
  int numSegments;

  List<Vector2> points;

  factory Capsule({
    double radiusTop = 1, 
    double radiusBottom = 1, 
    double height = 1,
    int numSegments = 8,
    int numHeightSegments = 4
  }){
    List<Vector2> ptsTop = [
      Vector2(0, height*0.5),
    ];
    List<Vector2> ptsBottom = [
      Vector2(0, -height*0.5),
    ];
    for(int i = 0; i < numHeightSegments; i++){
      final theta = (math.pi / numHeightSegments) * (i + 1);
      double xt = 0 + math.cos(theta);
      double yt = (height*0.5-radiusTop) + math.sin(theta);
      ptsTop.add(Vector2(xt, yt));
      double xb = 0 + math.cos(theta);
      double yb = (-height*0.5+radiusBottom) + math.sin(theta);
      ptsBottom.add(Vector2(xb, yb));
    }
    ptsTop.add(Vector2(0, radiusTop));
    ptsBottom.add(Vector2(0, radiusBottom));

    return Capsule.fromPoints(
      radiusBottom: radiusBottom,
      radiusTop: radiusTop,
      height: height,
      numSegments: numSegments,
      points: ptsTop..addAll(ptsBottom)
    );
  }

  /// @param radiusTop The radius of the top of the Cylinder.
  /// @param radiusBottom The radius of the bottom of the Cylinder.
  /// @param height The height of the Cylinder.
  /// @param numSegments The number of segments to build the cylinder out of.
  Capsule.fromPoints({
    required this.points,
    this.radiusTop = 1, 
    this.radiusBottom = 1, 
    this.height = 1,
    this.numSegments = 8,
  }):super(
    points: points,
    numSegments: numSegments,
    type: ShapeType.capsule
  ){
  //   if (radiusTop < 0) {
  //     throw('The cylinder radiusTop cannot be negative.');
  //   }

  //   if (radiusBottom < 0) {
  //     throw('The cylinder radiusBottom cannot be negative.');
  //   }

  //   final int N = numSegments;
  //   List<int> bottomface = [];
  //   List<int> topface = [];
  //   List<Vector3> axes =[];

  //   vertices = [];
  //   faces = [];
  //   // First bottom point
  //   vertices.add(Vector3(-radiusBottom * math.sin(0), -height * 0.5, radiusBottom * math.cos(0)));
  //   // First top point
  //   vertices.add(Vector3(-radiusTop * math.sin(0), height * 0.5, radiusTop * math.cos(0)));

  //   for (int i = 0; i < N; i++) {
  //     final theta = ((2 * math.pi) / N) * (i + 1);
  //     final thetaN = ((2 * math.pi) / N) * (i + 0.5);
  //     if (i < N - 1) {
  //       vertices.add(Vector3(-radiusBottom * math.sin(theta), -height * 0.5, radiusBottom * math.cos(theta)));
  //       vertices.add(Vector3(-radiusTop * math.sin(theta), height * 0.5, radiusTop * math.cos(theta)));
  //       // Face
  //       faces.add([
  //         2 * i, 
  //         2 * i + 1, 
  //         2 * i + 3, 
  //         2 * i + 2
  //       ]);
  //     } else {
  //       faces.add([
  //         2 * i, 
  //         2 * i + 1, 
  //         1, 
  //         0
  //       ]); // Connect
  //     }

  //     // Axis: we can cut off half of them if we have even number of segments
  //     if (N % 2 == 1 || i < N / 2) {
  //       axes.add(Vector3(-math.sin(thetaN), 0, math.cos(thetaN)));
  //     }
  //   }
  //   int widthSegments = numSegments;
  //   int heightSegments = numSegments;//4*4

  //   for (int iy = 0; iy <= heightSegments; iy++) {
  //     double v = iy / heightSegments;
  //     for (int ix = 0; ix <= widthSegments; ix++) {
  //       var u = ix / widthSegments;
  //       vertices.add(Vector3(
  //         -radiusTop * math.cos(u * math.pi * 2) * math.sin(v * math.pi),
  //         radiusTop * math.cos(v * math.pi)+height*0.5,
  //         radiusTop * math.sin( u * math.pi * 2) * math.sin(v * math.pi)
  //       ));
  //       topface.add([

  //       ]);
  //       vertices.add(Vector3(
  //         -radiusTop * math.cos(u * math.pi * 2) * math.sin(v * math.pi),
  //         -radiusTop * math.cos(v * math.pi)-height*0.5,
  //         radiusTop * math.sin( u * math.pi * 2) * math.sin(v * math.pi)
  //       ));
  //       bottomface.add([
          
  //       ]);
  //     }
  //   }

  //   faces.add(topface);
  //   faces.add(bottomface);
  //   axes.add(Vector3(0, 1, 0));

  //   type = ShapeType.capsule;
  //   radiusTop = radiusTop;
  //   radiusBottom = radiusBottom;
  //   height = height;
  //   numSegments = numSegments;

  //   init(vertices, faces, faceNormals, axes, boundingSphereRadius);
  }
}
