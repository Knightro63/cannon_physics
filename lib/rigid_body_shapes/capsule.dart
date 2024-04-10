import 'dart:math' as math;
import 'lathe.dart';
import 'shape.dart';
import 'package:vector_math/vector_math.dart';
import 'convex_polyhedron.dart';
/// Capsule class.
/// @example
///     const radiusTop = 0.5
///     const radiusBottom = 0.5
///     const height = 2
///     const numSegments = 12
///     const CapsuleShape = new CANNON.Capsule(radiusTop, radiusBottom, height, numSegments)
///     const CapsuleBody = new CANNON.Body({ mass: 1, shape: CapsuleShape })
///     world.addBody(CapsuleBody)
class Capsule extends ConvexPolyhedron {
  /// The radius of the top of the Capsule.
  double radiusTop;
  /// The radius of the bottom of the Capsule.
  double radiusBottom;
  ///The height of the Capsule.
  double height;
  /// The number of segments to build the Capsule out of.
  int numSegments;

  int numHeightSegments;

  Capsule({
    this.radiusTop = 1, 
    this.radiusBottom = 1, 
    this.height = 1,
    this.numSegments = 8,
    this.numHeightSegments = 4
  }):super(
    type: ShapeType.capsule
  ){
    if (radiusTop < 0) {
      throw('The Capsule radiusTop cannot be negative.');
    }

    if (radiusBottom < 0) {
      throw('The Capsule radiusBottom cannot be negative.');
    }

    vertices = [
      Vector3(-radiusBottom * math.sin(0), -height * 0.5, radiusBottom * math.cos(0)),
      Vector3(-radiusTop * math.sin(0), height * 0.5, radiusTop * math.cos(0))
    ];
    faces = [];

    List<Vector3> topVerts = [];
    List<Vector3> bottomVerts = [];

    final List<List<int>> grid = [];
    int index = 0;

    double phiStart = math.pi;
    double phiLength = 2*math.pi;
    double thetaStart = math.pi/2;
    double thetaLength = math.pi/2;

    Map<String,int> bs = {};
    Map<String,int> ts = {};

    for (int iy = 0; iy <= numHeightSegments; iy++) {
      final List<int> verticesRow = [];
      final v = iy / numHeightSegments;
      for (int ix = 0; ix <= numSegments; ix++) {
        final ub = ix/numSegments;
        final ut = (numSegments-ix)/numSegments;

        //create cylinder walls
        if(iy == 0 && ix < numSegments){
          final theta = ((2 * math.pi) / numSegments) * (ix + 1);
          if (ix < numSegments - 1) {
            // Bottom
            vertices.add(Vector3(-radiusBottom * math.sin(theta), -height * 0.5, radiusBottom * math.cos(theta)));
            // Top
            vertices.add(Vector3(-radiusTop * math.sin(theta), height * 0.5, radiusTop * math.cos(theta)));
            // Face
            faces.add([
              2 * ix, 
              2 * ix + 1, 
              2 * ix + 3, 
              2 * ix + 2
            ]);
          } 
          else {
            faces.add([
              2 * ix, 
              2 * ix + 1, 
              1, 
              0
            ]); // Connect
          }
        }

        //create hemisphere verts
        if(
          true ||
          height * 0.5 - radiusTop * math.cos( thetaStart + v * thetaLength ) != height * 0.5 &&
          -height * 0.5 + radiusTop * math.cos( thetaStart + v * thetaLength ) != -height * 0.5
        ){
          topVerts.add(
            Vector3(
              -radiusTop * math.cos(phiStart + ut * phiLength) * math.sin( thetaStart + v * thetaLength ), 
              height * 0.5 - radiusTop * math.cos( thetaStart + v * thetaLength ), 
              radiusTop * math.sin(phiStart + ut * phiLength) * math.sin( thetaStart + v * thetaLength )
            )
          );
          bottomVerts.add(
            Vector3(
              -radiusTop * math.cos(phiStart + ub * phiLength) * math.sin( thetaStart + v * thetaLength ), 
              -height * 0.5 + radiusTop * math.cos( thetaStart + v * thetaLength ), 
              radiusTop * math.sin(phiStart + ub * phiLength) * math.sin( thetaStart + v * thetaLength )
            )
          );
        }
        else{ //do this if has shared vert
          for(int k = 0; k < vertices.length;k++){
            final t = Vector3(
              -radiusTop * math.cos(phiStart + ut * phiLength) * math.sin( thetaStart + v * thetaLength ), 
              height * 0.5 - radiusTop * math.cos( thetaStart + v * thetaLength ), 
              radiusTop * math.sin(phiStart + ut * phiLength) * math.sin( thetaStart + v * thetaLength )
            );
            final b = Vector3(
              -radiusTop * math.cos(phiStart + ub * phiLength) * math.sin( thetaStart + v * thetaLength ), 
              -height * 0.5 + radiusTop * math.cos( thetaStart + v * thetaLength ), 
              radiusTop * math.sin(phiStart + ub * phiLength) * math.sin( thetaStart + v * thetaLength )
            );
            
            if(vertices[k] == t){
              ts['$iy $ix'] = k;
              break;
            }
            else if(vertices[k] == b){
              bs['$iy $ix'] = k;
              break;
            }
          }
        }
        verticesRow.add(index++);
      }
      grid.add( verticesRow );
    }

    int start1 = vertices.length;
    int start2 = vertices.length+topVerts.length;

		for ( int iy = 0; iy < numHeightSegments; iy ++ ) {
			for ( int ix = 0; ix < numSegments; ix ++ ) {
        final a1 = ts['$iy ${ix+1}'] ?? grid[ iy ][ ix + 1 ]+start1;
        final b1 = ts['$iy $ix'] ?? grid[ iy ][ ix ]+start1;
        final c1 = ts['${iy+1} $ix'] ?? grid[ iy + 1 ][ ix ]+start1;
        final d1 = ts['${iy+1} ${ix+1}'] ?? grid[ iy + 1 ][ ix + 1 ]+start1;
        faces.add([a1, b1, d1]);
        faces.add([b1, c1, d1]);

        final a2 = bs['$iy ${ix+1}'] ?? grid[ iy ][ ix + 1 ]+start2;
        final b2 = bs['$iy $ix'] ?? grid[ iy ][ ix ]+start2;
        final c2 = bs['${iy+1} $ix'] ?? grid[ iy + 1 ][ ix ]+start2;
        final d2 = bs['${iy+1} ${ix+1}'] ?? grid[ iy + 1 ][ ix + 1 ]+start2;
        faces.add([a2, b2, d2]);
        faces.add([b2, c2, d2]);
			}
		}
    vertices.addAll(topVerts);
    vertices.addAll(bottomVerts);

    init(vertices, faces);
  }
}
