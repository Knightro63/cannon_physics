import 'dart:math' as math;
import 'dart:ui';
import 'shape.dart';
import 'convex_polyhedron.dart';
import '../math/vec3.dart';
import '../collision/aabb.dart';
import 'package:vector_math/vector_math.dart';

class HeightfieldPillar{
  HeightfieldPillar(this.convex,this.offset);
  ConvexPolyhedron convex;
  Vector3 offset;
}

/// Heightfield shape class. Height data is given as an array. These data points are spread out evenly with a given distance.
/// @todo Should be possible to use along all axes, not just y
/// @todo should be possible to scale along all axes
/// @todo Refactor elementSize to elementSizeX and elementSizeY
///
/// @example
///     // Generate some height data (y-values).
///     final data = []
///     for (let i = 0; i < 1000; i++) {
///        final y = 0.5 * Math.cos(0.2 * i)
///         data.add(y)
///     }
///
///     // Create the heightfield shape
///     final heightfieldShape = CANNON.Heightfield(data, {
///         elementSize: 1 // Distance between the data points in X and Y directions
///     })
///     final heightfieldBody = CANNON.Body({ shape: heightfieldShape })
///     world.addBody(heightfieldBody)
class Heightfield extends Shape {
  /// An array of numbers, or height values, that are spread out along the x axis.
  late List<List<double>>data;

  /// Max value of the data points in the data array.
  double? maxValue;

  /// Minimum value of the data points in the data array.
  double? minValue;

  /// World spacing between the data points in X and Y direction.
  /// @todo elementSizeX and Y
  int elementSize;

  bool cacheEnabled = true;
  ConvexPolyhedron pillarConvex = ConvexPolyhedron(); 
  Vector3 pillarOffset = Vector3.zero();

  final Map<String,HeightfieldPillar> _cachedPillars = {};

  late Size size;
  late Size segments;

  /// @param data An array of numbers, or height values, that are spread out along the x axis.
  Heightfield(
    this.data,
    {
      this.maxValue,
      this.minValue,
      this.elementSize = 1
    }):super(type: ShapeType.heightfield){
    segments = Size(data.length.toDouble(),data[0].length.toDouble());
    size = Size((segments.width-1)*elementSize,(segments.height-1)*elementSize);
    if(minValue == null) {
      updateMinValue();
    }
    if (maxValue == null) {
      updateMaxValue();
    }
    updateBoundingSphereRadius();
  }

  List<int> getHeightAtIdx = [];
  final _getHeightAtWeights = Vector3.zero();
  final _getHeightAtA = Vector3.zero();
  final _getHeightAtB = Vector3.zero();
  final _getHeightAtC = Vector3.zero();

  final _getNormalAtA = Vector3.zero();
  final _getNormalAtB = Vector3.zero();
  final _getNormalAtC = Vector3.zero();
  final _getNormalAtE0 = Vector3.zero();
  final _getNormalAtE1 = Vector3.zero();

  /// Call whenever you change the data array.
  void update() {
    _cachedPillars.clear();
  }

  /// Update the `minValue` property
  void updateMinValue() {
    final data = this.data;
    double minValue = data[0][0];
    for (int i = 0; i != data.length; i++) {
      for (int j = 0; j != data[i].length; j++) {
        final v = data[i][j];
        if (v < minValue) {
          minValue = v;
        }
      }
    }
    this.minValue = minValue;
  }

  /// Update the `maxValue` property
  void updateMaxValue() {
    final data = this.data;
    double maxValue = data[0][0];
    for (int i = 0; i != data.length; i++) {
      for (int j = 0; j != data[i].length; j++) {
        final v = data[i][j];
        if (v > maxValue) {
          maxValue = v;
        }
      }
    }
    this.maxValue = maxValue;
  }

  /// Set the height value at an index. Don't forget to update maxValue and minValue after you're done.
  void setHeightValueAtIndex(int xi, int yi, double value){
    final data = this.data;
    data[xi][yi] = value;

    // Invalidate cache
    clearCachedConvexTrianglePillar(xi, yi, false);
    if (xi > 0) {
      clearCachedConvexTrianglePillar(xi - 1, yi, true);
      clearCachedConvexTrianglePillar(xi - 1, yi, false);
    }
    if (yi > 0) {
      clearCachedConvexTrianglePillar(xi, yi - 1, true);
      clearCachedConvexTrianglePillar(xi, yi - 1, false);
    }
    if (yi > 0 && xi > 0) {
      clearCachedConvexTrianglePillar(xi - 1, yi - 1, true);
    }
  }

  /// Get max/min in a rectangle in the matrix data
  /// @param result An array to store the results in.
  /// @return The result array, if it was passed in. Minimum will be at position 0 and max at 1.
  void getRectMinMax(int iMinX, int iMinY, int iMaxX, int iMaxY,List<double> result) {
    result.clear();
    // Get max and min of the data
    final data = this.data; // Set first value

    double max = minValue!;
    for (int i = iMinX; i <= iMaxX; i++) {
      for (int j = iMinY; j <= iMaxY; j++) {
        final height = data[i][j];
        if (height > max) {
          max = height;
        }
      }
    }

    result.addAll([minValue!,max]);
  }

  /// Get the index of a local position on the heightfield. The indexes indicate the rectangles, so if your terrain is made of N x N height data points, you will have rectangle indexes ranging from 0 to N-1.
  /// @param result Two-element array
  /// @param clamp If the position should be clamped to the heightfield edge.
  bool getIndexOfPosition(double x, double y, List<int> result, bool clamp) {
    // Get the index of the data points to test against
    final w = elementSize;
    final data = this.data;
    int xi = (x / w).floor();
    int yi = (y / w).floor();
    result.addAll([xi,yi]);

    if (clamp) {
      // Clamp index to edges
      if (xi < 0) {
        xi = 0;
      }
      if (yi < 0) {
        yi = 0;
      }
      if (xi >= data.length - 1) {
        xi = data.length - 1;
      }
      if (yi >= data[0].length - 1) {
        yi = data[0].length - 1;
      }
    }

    // Bail out if we are out of the terrain
    if (xi < 0 || yi < 0 || xi >= data.length - 1 || yi >= data[0].length - 1) {
      return false;
    }

    return true;
  }

  bool getTriangleAt(double x, double y, bool edgeClamp,Vector3 a,Vector3 b, Vector3 c) {
    final idx = getHeightAtIdx;
    idx.clear();
    getIndexOfPosition(x, y, idx, edgeClamp);
    int xi = idx[0];
    int yi = idx[1];

    final data = this.data;
    if (edgeClamp) {
      xi = math.min(data.length - 2, math.max(0, xi));
      yi = math.min(data[0].length - 2, math.max(0, yi));
    }

    final elementSize = this.elementSize;
    final lowerDist2 = math.pow((x / elementSize - xi),2) + math.pow((y / elementSize - yi),2);
    final upperDist2 = math.pow((x / elementSize - (xi + 1)), 2) + math.pow((y / elementSize - (yi + 1)),2);
    final upper = lowerDist2 > upperDist2;
    getTriangle(xi, yi, upper, a, b, c);
    return upper;
  }

  void getNormalAt(double x, double y, bool edgeClamp, Vector3 result) {
    final a = _getNormalAtA;
    final b = _getNormalAtB;
    final c = _getNormalAtC;
    final e0 = _getNormalAtE0;
    final e1 = _getNormalAtE1;
    getTriangleAt(x, y, edgeClamp, a, b, c);
    b.sub2(a, e0);
    c.sub2(a, e1);
    e0.cross2(e1, result);
    result.normalize();
  }

  /// Get an AABB of a square in the heightfield
  /// @param xi
  /// @param yi
  /// @param result
  void getAabbAtIndex(int xi, int yi, AABB aabb){//{ lowerBound, upperBound }: AABB) {
    final data = this.data;
    final elementSize = this.elementSize.toDouble();

    aabb.lowerBound.setValues(
      xi * elementSize, 
      yi * elementSize, 
      data[xi][yi]
    );
    aabb.upperBound.setValues(
      (xi + 1) * elementSize, 
      (yi + 1) * elementSize, 
      data[xi + 1][yi + 1]
    );
  }

  /// Get the height in the heightfield at a given position
  double getHeightAt(double x, double y, bool edgeClamp) {
    final data = this.data;
    final a = _getHeightAtA;
    final b = _getHeightAtB;
    final c = _getHeightAtC;
    final idx = getHeightAtIdx;

    getIndexOfPosition(x, y, idx, edgeClamp);
    int xi = idx[0];
    int yi = idx[1];
    if (edgeClamp) {
      xi = math.min(data.length - 2, math.max(0, xi));
      yi = math.min(data[0].length - 2, math.max(0, yi));
    }
    final upper = getTriangleAt(x, y, edgeClamp, a, b, c);
    barycentricWeights(x, y, a.x, a.y, b.x, b.y, c.x, c.y, _getHeightAtWeights);

    final w = _getHeightAtWeights;

    if (upper) {
      // Top triangle verts
      return data[xi + 1][yi + 1] * w.x + data[xi][yi + 1] * w.y + data[xi + 1][yi] * w.z;
    } else {
      // Top triangle verts
      return data[xi][yi] * w.x + data[xi + 1][yi] * w.y + data[xi][yi + 1] * w.z;
    }
  }

  String getCacheConvexTrianglePillarKey(int xi, int yi, bool getUpperTriangle) {
    return '${xi}_${yi}_${getUpperTriangle ? 1 : 0}';
  }

  HeightfieldPillar? getCachedConvexTrianglePillar(int xi, int yi, bool getUpperTriangle){
    return _cachedPillars[getCacheConvexTrianglePillarKey(xi, yi, getUpperTriangle)];
  }

  void setCachedConvexTrianglePillar(
    int xi,
    int yi,
    bool getUpperTriangle,
    ConvexPolyhedron convex,
    Vector3 offset
  ) {
    _cachedPillars[getCacheConvexTrianglePillarKey(xi, yi, getUpperTriangle)] = HeightfieldPillar(
      convex,
      offset,
    );
  }

  void clearCachedConvexTrianglePillar(
    int xi, 
    int yi, 
    bool getUpperTriangle
  ) {
    _cachedPillars.remove(getCacheConvexTrianglePillarKey(xi, yi, getUpperTriangle));//delete this._cachedPillars[getCacheConvexTrianglePillarKey(xi, yi, getUpperTriangle)];
  }

  /// Get a triangle from the heightfield
  void getTriangle(int xi, int yi, bool upper, Vector3 a, Vector3 b, Vector3 c) {
    final data = this.data;
    final elementSize = this.elementSize.toDouble();

    if (upper) {
      // Top triangle verts
      a.setValues((xi + 1) * elementSize, (yi + 1) * elementSize, data[xi + 1][yi + 1]);
      b.setValues(xi * elementSize, (yi + 1) * elementSize, data[xi][yi + 1]);
      c.setValues((xi + 1) * elementSize, yi * elementSize, data[xi + 1][yi]);
    } else {
      // Top triangle verts
      a.setValues(xi * elementSize, yi * elementSize, data[xi][yi]);
      b.setValues((xi + 1) * elementSize, yi * elementSize, data[xi + 1][yi]);
      c.setValues(xi * elementSize, (yi + 1) * elementSize, data[xi][yi + 1]);
    }
  }

  /// Get a triangle in the terrain in the form of a triangular convex shape.
  void getConvexTrianglePillar(int xi, int yi, bool getUpperTriangle) {
    ConvexPolyhedron result = pillarConvex;
    Vector3 offsetResult = pillarOffset;

    if (cacheEnabled) {
      final data = getCachedConvexTrianglePillar(xi, yi, getUpperTriangle);
      
      if(data != null){
        pillarConvex = data.convex;
        pillarOffset = data.offset;
        return;
      }

      result = ConvexPolyhedron();
      offsetResult = Vector3.zero();

      pillarConvex = result;
      pillarOffset = offsetResult;
    }

    final data = this.data;
    final elementSize = this.elementSize;

    // Reuse verts if possible
    if(result.vertices.isEmpty){
      for (int i = 0; i < 6; i++) {
        result.vertices.add(Vector3.zero());
      }
    }

    // Reuse faces if possible
    bool wasEmpty = result.faces.isEmpty;
    result.faces = wasEmpty? List.filled(5, []):result.faces;
    if(wasEmpty){
      for (int i = 0; i < 5; i++) {
        if (result.faces[i].isEmpty) {
          int len = 4;
          if(i < 2){
            len = 3;
          }
          result.faces[i] = List.filled(len, 0);
        }
      }
    }

    final verts = result.vertices;
    final faces = result.faces;

    final h =(
      math.min(
        math.min(
          data[xi][yi], 
          data[xi + 1][yi]
        ),
        math.min(
          data[xi][yi + 1], 
          data[xi + 1][yi + 1]
        )
      ) - minValue!
    ) / 2 +minValue!;

    if (!getUpperTriangle) {
      // Center of the triangle pillar - all polygons are given relative to this one
      offsetResult.setValues(
        (xi + 0.25) * elementSize, // sort of center of a triangle
        (yi + 0.25) * elementSize,
        h // vertical center
      );

      // Top triangle verts
      verts[0].setValues(-0.25 * elementSize, -0.25 * elementSize, data[xi][yi] - h);
      verts[1].setValues(0.75 * elementSize, -0.25 * elementSize, data[xi + 1][yi] - h);
      verts[2].setValues(-0.25 * elementSize, 0.75 * elementSize, data[xi][yi + 1] - h);

      // bottom triangle verts
      verts[3].setValues(-0.25 * elementSize, -0.25 * elementSize, -h - 1);
      verts[4].setValues(0.75 * elementSize, -0.25 * elementSize, -h - 1);
      verts[5].setValues(-0.25 * elementSize, 0.75 * elementSize, -h - 1);

      // top triangle
      faces[0][0] = 0;
      faces[0][1] = 1;
      faces[0][2] = 2;

      // bottom triangle
      faces[1][0] = 5;
      faces[1][1] = 4;
      faces[1][2] = 3;

      // -x facing quad
      faces[2][0] = 0;
      faces[2][1] = 2;
      faces[2][2] = 5;
      faces[2][3] = 3;

      // -y facing quad
      faces[3][0] = 1;
      faces[3][1] = 0;
      faces[3][2] = 3;
      faces[3][3] = 4;

      // +xy facing quad
      faces[4][0] = 4;
      faces[4][1] = 5;
      faces[4][2] = 2;
      faces[4][3] = 1;
    } 
    else {
      // Center of the triangle pillar - all polygons are given relative to this one
      offsetResult.setValues(
        (xi + 0.75) * elementSize, // sort of center of a triangle
        (yi + 0.75) * elementSize,
        h // vertical center
      );

      // Top triangle verts
      verts[0].setValues(0.25 * elementSize, 0.25 * elementSize, data[xi + 1][yi + 1] - h);
      verts[1].setValues(-0.75 * elementSize, 0.25 * elementSize, data[xi][yi + 1] - h);
      verts[2].setValues(0.25 * elementSize, -0.75 * elementSize, data[xi + 1][yi] - h);

      // bottom triangle verts
      verts[3].setValues(0.25 * elementSize, 0.25 * elementSize, -h - 1);
      verts[4].setValues(-0.75 * elementSize, 0.25 * elementSize, -h - 1);
      verts[5].setValues(0.25 * elementSize, -0.75 * elementSize, -h - 1);

      // Top triangle
      faces[0][0] = 0;
      faces[0][1] = 1;
      faces[0][2] = 2;

      // bottom triangle
      faces[1][0] = 5;
      faces[1][1] = 4;
      faces[1][2] = 3;

      // +x facing quad
      faces[2][0] = 2;
      faces[2][1] = 5;
      faces[2][2] = 3;
      faces[2][3] = 0;

      // +y facing quad
      faces[3][0] = 3;
      faces[3][1] = 4;
      faces[3][2] = 1;
      faces[3][3] = 0;

      // -xy facing quad
      faces[4][0] = 1;
      faces[4][1] = 4;
      faces[4][2] = 5;
      faces[4][3] = 2;
    }
    result.computeNormals();
    result.computeEdges();
    result.updateBoundingSphereRadius();
    setCachedConvexTrianglePillar(xi, yi, getUpperTriangle, result, offsetResult);
  }
  @override
  Vector3 calculateLocalInertia(double mass, [Vector3? target]) {
    target ??= Vector3.zero();
    target.setValues(0, 0, 0);
    return target;
  }
  @override
  double volume() {
    return double.infinity;
  }
  @override
  void calculateWorldAABB(Vector3 pos, Quaternion quat, Vector3 min, Vector3 max) {
    /** @TODO do it properly */
    min.setValues(-double.infinity, -double.infinity, -double.infinity);
    max.setValues(double.infinity, double.infinity, double.infinity);
  }
  @override
  void updateBoundingSphereRadius() {
    // Use the bounding box of the min/max values
    final data = this.data;

    final s = elementSize.toDouble();
    boundingSphereRadius = Vector3(
      data.length * s,
      data[0].length * s,
      math.max(maxValue!.abs(), minValue!.abs())
    ).length;
  }

  /// Sets the height values from an image. Currently only supported in browser.
  void setHeightsFromImage(Image image, Vector3 scale) async{
    final x = scale.x;
    final y = scale.y;
    final z = scale.z;
    // final canvas = document.createElement('canvas');
    // canvas.width = image.width;
    // canvas.height = image.height;
    // final context = canvas.getContext('2d')!;
    // context.drawImage(image, 0, 0);
    final toByteData = await image.toByteData();
    final imageData = toByteData!.buffer.asUint8List();//context.getImageData(0, 0, image.width, image.height);

    final matrix = data;
    matrix.clear();
    elementSize = x.abs() ~/ image.width;
    for (int i = 0; i < image.height; i++) {
      final List<double> row = [];
      for (int j = 0; j < image.width; j++) {
        final a = imageData[(i * image.height + j) * 4];
        final b = imageData[(i * image.height + j) * 4 + 1];
        final c = imageData[(i * image.height + j) * 4 + 2];
        final height = ((a + b + c) / 4 / 255) * z;
        if (x < 0) {
          row.add(height);
        } else {
          row.insert(0,height);
        }
      }
      if (y < 0) {
        matrix.insert(0,row);
      } else {
        matrix.add(row);
      }
    }
    updateMaxValue();
    updateMinValue();
    update();
  }

  // from https://en.wikipedia.org/wiki/Barycentric_coordinate_system
  void barycentricWeights(
    double x,
    double y,
    double ax,
    double ay,
    double bx,
    double by,
    double cx,
    double cy,
    Vector3 result
  ) {
    result.x = ((by - cy) * (x - cx) + (cx - bx) * (y - cy)) / ((by - cy) * (ax - cx) + (cx - bx) * (ay - cy));
    result.y = ((cy - ay) * (x - cx) + (ax - cx) * (y - cy)) / ((by - cy) * (ax - cx) + (cx - bx) * (ay - cy));
    result.z = 1 - result.x - result.y;
  }
}
