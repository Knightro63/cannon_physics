import 'dart:math' as math;
import '../collision/broadphase.dart';
import '../math/vec3.dart';
import '../shapes/shape.dart';
import '../objects/body.dart';
import '../shapes/sphere.dart';
import '../shapes/plane.dart';
import '../world/world_class.dart';

/**
 * Axis aligned uniform grid broadphase.
 * @todo Needs support for more than just planes and spheres.
 */
class GridBroadphase extends Broadphase {
  /// Number of boxes along x
  int nx;

  /// Number of boxes along y
  int ny;

  /// Number of boxes along z
  int nz;

  /// aabbMin
  late Vec3 aabbMin;

  /// aabbMax
  late Vec3 aabbMax;

  /// bins
  List<List<Body>> bins = [];

  /// binLengths
  List<int> binLengths = [];

  /// [nx] Number of boxes along x.
  /// [ny] Number of boxes along y.
  /// [nz] Number of boxes along z.
  GridBroadphase([
    Vec3? aabbMin,// = Vec3(100, 100, 100), 
    Vec3? aabbMax,// = Vec3(-100, -100, -100), 
    this.nx = 10, 
    this.ny = 10, 
    this.nz = 10
  ]):super(){
    this.aabbMin = aabbMin ?? Vec3(100, 100, 100);
    this.aabbMax = aabbMax ?? Vec3(-100, -100, -100);
    final nbins = nx * ny * nz;
    if (nbins <= 0) {
      throw"GridBroadphase: Each dimension's n must be >0";
    }
    bins.length = nbins;
    binLengths.length = nbins;
    for (int i = 0; i < nbins; i++) {
      bins[i] = [];
      binLengths[i] = 0;
    }
  }

  final _gridBroadphaseCollisionPairsD = Vec3();
  //final _gridBroadphaseCollisionPairsBinPos = Vec3();

  /// Get all the collision pairs in the physics world
  @override
  void collisionPairs(World world, List<Body> pairs1, List<Body> pairs2) {
    final N = world.bodies.length;
    final bodies = world.bodies;
    final max = aabbMax;
    final min = aabbMin;
    final nx = this.nx;
    final ny = this.ny;
    final nz = this.nz;

    final xstep = ny * nz;
    final ystep = nz;
    const zstep = 1;

    final xmax = max.x;
    final ymax = max.y;
    final zmax = max.z;
    final xmin = min.x;
    final ymin = min.y;
    final zmin = min.z;
    final xmult = nx / (xmax - xmin);
    final ymult = ny / (ymax - ymin);
    final zmult = nz / (zmax - zmin);
    final binsizeX = (xmax - xmin) / nx;
    final binsizeY = (ymax - ymin) / ny;
    final binsizeZ = (zmax - zmin) / nz;

    final binRadius = math.sqrt(binsizeX * binsizeX + binsizeY * binsizeY + binsizeZ * binsizeZ) * 0.5;
    final bins = this.bins;
    final binLengths = this.binLengths;
    final nBins = this.bins.length;

    // Reset bins
    for (int i = 0; i != nBins; i++) {
      binLengths[i] = 0;
    }

    void addBoxToBins(double x0, double y0,double z0,double x1,double y1,double z1,Body bi) {
      int xoff0 = ((x0 - xmin) * xmult).floor();
      int yoff0 = ((y0 - ymin) * ymult).floor();
      int zoff0 = ((z0 - zmin) * zmult).floor();
      int xoff1 = ((x1 - xmin) * xmult).ceil();
      int yoff1 = ((y1 - ymin) * ymult).ceil();
      int zoff1 = ((z1 - zmin) * zmult).ceil();

      if (xoff0 < 0) {
        xoff0 = 0;
      } else if (xoff0 >= nx) {
        xoff0 = nx - 1;
      }
      if (yoff0 < 0) {
        yoff0 = 0;
      } else if (yoff0 >= ny) {
        yoff0 = ny - 1;
      }
      if (zoff0 < 0) {
        zoff0 = 0;
      } else if (zoff0 >= nz) {
        zoff0 = nz - 1;
      }
      if (xoff1 < 0) {
        xoff1 = 0;
      } else if (xoff1 >= nx) {
        xoff1 = nx - 1;
      }
      if (yoff1 < 0) {
        yoff1 = 0;
      } else if (yoff1 >= ny) {
        yoff1 = ny - 1;
      }
      if (zoff1 < 0) {
        zoff1 = 0;
      } else if (zoff1 >= nz) {
        zoff1 = nz - 1;
      }

      xoff0 *= xstep;
      yoff0 *= ystep;
      zoff0 *= zstep;
      xoff1 *= xstep;
      yoff1 *= ystep;
      zoff1 *= zstep;

      for (int xoff = xoff0; xoff <= xoff1; xoff += xstep) {
        for (int yoff = yoff0; yoff <= yoff1; yoff += ystep) {
          for (int zoff = zoff0; zoff <= zoff1; zoff += zstep) {
            final idx = xoff + yoff + zoff;
            bins[idx][binLengths[idx]++] = bi;
          }
        }
      }
    }

    // Put all bodies into the bins
    for (int i = 0; i != N; i++) {
      final bi = bodies[i];
      final si = bi.shapes[0];

      switch (si.type) {
        case ShapeType.sphere: {
          final shape = si as Sphere;
          // Put in bin
          // check if overlap with other bins
          final x = bi.position.x;
          final y = bi.position.y;
          final z = bi.position.z;
          final r = shape.radius;

          addBoxToBins(x - r, y - r, z - r, x + r, y + r, z + r, bi);
          break;
        }
        case ShapeType.plane: {
          final shape = si as Plane;

          if (shape.worldNormalNeedsUpdate) {
            shape.computeWorldNormal(bi.quaternion);
          }
          final planeNormal = shape.worldNormal;

          //Relative position from origin of plane object to the first bin
          //Incremented as we iterate through the bins
          final xreset = xmin + binsizeX * 0.5 - bi.position.x;

          final yreset = ymin + binsizeY * 0.5 - bi.position.y;
          final zreset = zmin + binsizeZ * 0.5 - bi.position.z;

          final d = _gridBroadphaseCollisionPairsD;
          d.set(xreset, yreset, zreset);

          for (int xi = 0, xoff = 0; xi != nx; xi++, xoff += xstep, d.y = yreset, d.x += binsizeX) {
            for (int yi = 0, yoff = 0; yi != ny; yi++, yoff += ystep, d.z = zreset, d.y += binsizeY) {
              for (int zi = 0, zoff = 0; zi != nz; zi++, zoff += zstep, d.z += binsizeZ) {
                if (d.dot(planeNormal) < binRadius) {
                  final idx = xoff + yoff + zoff;
                  bins[idx][binLengths[idx]++] = bi;
                }
              }
            }
          }
          break;
        }
        default: {
          if (bi.aabbNeedsUpdate) {
            bi.updateAABB();
          }

          addBoxToBins(
            bi.aabb.lowerBound.x,
            bi.aabb.lowerBound.y,
            bi.aabb.lowerBound.z,
            bi.aabb.upperBound.x,
            bi.aabb.upperBound.y,
            bi.aabb.upperBound.z,
            bi
          );
          break;
        }
      }
    }

    // Check each bin
    for (int i = 0; i != nBins; i++) {
      final binLength = binLengths[i];
      //Skip bins with no potential collisions
      if (binLength > 1) {
        final bin = bins[i];

        // Do N^2 broadphase inside
        for (int xi = 0; xi != binLength; xi++) {
          final bi = bin[xi];
          for (int yi = 0; yi != xi; yi++) {
            final bj = bin[yi];
            if (needBroadphaseCollision(bi, bj)) {
              intersectionTest(bi, bj, pairs1, pairs2);
            }
          }
        }
      }
    }

    //	for (let zi = 0, zoff=0; zi < nz; zi++, zoff+= zstep) {
    //		console.log("layer "+zi);
    //		for (let yi = 0, yoff=0; yi < ny; yi++, yoff += ystep) {
    //			final row = '';
    //			for (let xi = 0, xoff=0; xi < nx; xi++, xoff += xstep) {
    //				final idx = xoff + yoff + zoff;
    //				row += ' ' + binLengths[idx];
    //			}
    //			console.log(row);
    //		}
    //	}

    makePairsUnique(pairs1, pairs2);
  }
}