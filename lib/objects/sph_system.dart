import 'dart:math' as math;
import '../math/vec3.dart';
import './body.dart';

/// Smoothed-particle hydrodynamics system
/// @todo Make parameters customizable in the constructor
class SPHSystem {
  /// The particles array.
  List<Body> particles = [];

  /// Density of the system (kg/m3).
  /// @default 1
  double density = 1;

  /// Distance below which two particles are considered to be neighbors.
  /// It should be adjusted so there are about 15-20 neighbor particles within this radius.
  /// @default 1
  double smoothingRadius = 1;
  double speedOfSound = 1;

  /// Viscosity of the system.
  /// @default 0.01
  double viscosity = 0.01;
  double eps = 0.00001;

  Map<int,double> pressures = {};
  Map<int,double> densities = {};
  List<List<Body>> neighbors = [];

  SPHSystem();

  final _sphSystemGetNeighborsDist = Vec3();

  // Temp vectors for calculation
  final _sphSystemUpdateDist = Vec3(); // Relative velocity

  final _sphSystemUpdateAPressure = Vec3();
  final _sphSystemUpdateAVisc = Vec3();
  final _sphSystemUpdateGradW = Vec3();
  final _sphSystemUpdateRVec = Vec3();
  final _sphSystemUpdateU = Vec3();

  /// Add a particle to the system.
  void add(Body particle) {
    particles.add(particle);
    if (neighbors.length < particles.length) {
      neighbors.add([]);
    }
  }

  /// Remove a particle from the system.
  void remove(Body particle) {
    final idx = particles.indexOf(particle);
    if (idx != -1) {
      particles.removeAt(idx);
      if (neighbors.length > particles.length) {
        neighbors.removeLast();
      }
    }
  }

  /// Get neighbors within smoothing volume, save in the array neighbors
  void getNeighbors(Body particle, List<Body> neighbors) {
    final N = particles.length;
    final id = particle.id;
    final r2 = smoothingRadius * smoothingRadius;
    final dist = _sphSystemGetNeighborsDist;
    for (int i = 0; i != N; i++) {
      final p = particles[i];
      p.position.vsub(particle.position, dist);
      if (id != p.id && dist.lengthSquared() < r2) {
        neighbors.add(p);
      }
    }
  }

  void update() {
    final N = particles.length;
    final dist = _sphSystemUpdateDist;
    final cs = speedOfSound;
    final eps = this.eps;

    for (int i = 0; i != N; i++) {
      final p = particles[i]; // Current particle
      final neighbors = this.neighbors[i];

      // Get neighbors
      neighbors.clear();
      getNeighbors(p, neighbors);
      neighbors.add(particles[i]); // Add current too
      final numNeighbors = neighbors.length;

      // Accumulate density for the particle
      double sum = 0.0;
      for (int j = 0; j != numNeighbors; j++) {
        //printf("Current particle has position %f %f %f\n",objects[id].pos.x(),objects[id].pos.y(),objects[id].pos.z());
        p.position.vsub(neighbors[j].position, dist);
        final len = dist.length();

        final weight = w(len);
        sum += neighbors[j].mass * weight;
      }

      // Save
      densities[i] = sum;
      pressures[i] = cs * cs * (densities[i]! - density);
    }

    // Add forces

    // Sum to these accelerations
    final aPressure = _sphSystemUpdateAPressure;
    final aVisc = _sphSystemUpdateAVisc;
    final gradW = _sphSystemUpdateGradW;
    final rVec = _sphSystemUpdateRVec;
    final u = _sphSystemUpdateU;

    for (int i = 0; i != N; i++) {
      final particle = particles[i];

      aPressure.set(0, 0, 0);
      aVisc.set(0, 0, 0);

      // Init vars
      double pij;
      double nabla;

      // Sum up for all other neighbors
      final neighbors = this.neighbors[i];
      final numNeighbors = neighbors.length;

      //printf("Neighbors: ");
      for (int j = 0; j != numNeighbors; j++) {
        final neighbor = neighbors[j];
        //printf("%d ",nj);

        // Get r once for all..
        particle.position.vsub(neighbor.position, rVec);
        final r = rVec.length();

        // Pressure contribution
        pij =
          -neighbor.mass *
          (pressures[i]! / (densities[i]! * densities[i]! + eps) +
            pressures[j]! / (densities[j]! * densities[j]! + eps));
        gradw(rVec, gradW);
        // Add to pressure acceleration
        gradW.scale(pij, gradW);
        aPressure.vadd(gradW, aPressure);

        // Viscosity contribution
        neighbor.velocity.vsub(particle.velocity, u);
        u.scale((1.0 / (0.0001 + densities[i]! * densities[j]!)) * viscosity * neighbor.mass, u);
        nabla = nablaw(r);
        u.scale(nabla, u);
        // Add to viscosity acceleration
        aVisc.vadd(u, aVisc);
      }

      // Calculate force
      aVisc.scale(particle.mass, aVisc);
      aPressure.scale(particle.mass, aPressure);

      // Add force to particles
      particle.force.vadd(aVisc, particle.force);
      particle.force.vadd(aPressure, particle.force);
    }
  }

  // Calculate the weight using the W(r) weightfunction
  double w(double r) {
    // 315
    final h = smoothingRadius;
    return (315.0 / (64.0 * math.pi * math.pow(h, 9))) * math.pow((h * h - r * r), 3).toDouble();
  }

  // calculate gradient of the weight function
  void gradw(Vec3 rVec,Vec3 resultVec) {
    final r = rVec.length();
    final h = smoothingRadius;
    rVec.scale(945.0 / (32.0 * math.pi * math.pow(h, 9)) * math.pow((h * h - r * r), 2).toDouble(), resultVec);
  }

  // Calculate nabla(W)
  double nablaw(double r) {
    final h = smoothingRadius;
    final nabla = (945.0 / (32.0 * math.pi * math.pow(h, 9))) * (h * h - r * r) * (7 * r * r - 3 * h * h);
    return nabla;
  }
}
