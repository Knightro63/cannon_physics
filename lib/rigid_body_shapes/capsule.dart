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

  factory Capsule({
    double radiusTop = 1, 
    double radiusBottom = 1, 
    double height = 1,
    int numSegments = 8,
    int numHeightSegments = 4
  }){
    List<Vector2> ptsTop = [
      Vector2(0, height*0.5+radiusTop),
    ];
    List<Vector2> ptsBottom = [];

    for(int i = 0; i < numHeightSegments-1; i++){
      final theta = ((math.pi/2) / numHeightSegments) * (i + 1)+(2*math.pi+math.pi/2);
      double xt = -math.cos(theta)*radiusTop;
      double yt = height*0.5 + math.sin(theta)*radiusTop;
      ptsTop.add(Vector2(xt, yt));

      double xb = -math.cos(theta)*radiusBottom;
      double yb = -height*0.5 - math.sin(theta)*radiusBottom;
      ptsBottom.insert(0,Vector2(xb, yb));
    }
    ptsTop.add(Vector2(radiusTop, (height*0.5)));
    ptsBottom.add(Vector2(0, -height*0.5-radiusBottom));
    ptsBottom.insert(0,Vector2(radiusBottom, -height*0.5));
    ptsTop.addAll(ptsBottom);

    print('NEW:');
    for(int i = 0; i < ptsTop.length; i++){
      print(ptsTop[i].storage);
    }
    
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
    required super.points,
    this.radiusTop = 1, 
    this.radiusBottom = 1, 
    this.height = 1,
    super.numSegments = 8,
  }):super(
    type: ShapeType.capsule
  );
}
