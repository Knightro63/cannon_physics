import 'package:cannon_physics/constraints/spring_constraint.dart';
import 'package:cannon_physics/math/vec3.dart';
import 'package:cannon_physics/objects/rigid_body.dart';
import 'package:cannon_physics/rigid_body_shapes/particle.dart';
import 'package:cannon_physics/rigid_body_shapes/shape.dart';
import 'dart:math';
import 'package:cannon_physics/objects/soft_body.dart';

/// Simple vehicle helper class with spherical rigid body wheels.
class BoxSoftBody extends SoftBody{
  Vec3 halfExtents;

  BoxSoftBody({
    required this.halfExtents,
    super.mass = 1,
    super.stiffness = 100,
    int numOfParticles = 18,
  }):super(shape: ShapeType.box){
    final Vec3 size = halfExtents;
    final mass = this.mass/numOfParticles;
    double maxV = size.x+size.y+size.z;
    final Nx = ((size.x/maxV)*numOfParticles).floor();
    final Ny = ((size.y/maxV)*numOfParticles).floor();
    final Nz = ((size.z/maxV)*numOfParticles).floor();
    double dist = numOfParticles/maxV;

    for (int i = 0; i < Nx; i++) {
      for (int j = 0; j < Ny; j++) {
        for (int k = 0; k < Nz; k++) {
          // Create a new body
          final body = Body( mass: mass );
          body.addShape(Particle());
          body.position.set(-dist * i, dist * k + dist * Nz * 0.3 + 1, dist * j);
          body.velocity.set(0, 0, (sin(i * 0.1) + sin(j * 0.1)) * 30);
          particleBodies['$i $j $k'] = body;
        }
      }
    }

    for (int i = 0; i < Nx; i++) {
      for (int j = 0; j < Ny; j++) {
        for (int k = 0; k < Nz; k++) {
          // normal directions
          if (i < Nx - 1) _connect(i, j, k, i + 1, j, k);
          if (j < Ny - 1) _connect(i, j, k, i, j + 1, k);
          if (k < Nz - 1) _connect(i, j, k, i, j, k + 1);

          // Diagonals
          if (i < Nx - 1 && j < Ny - 1 && k < Nz - 1) {
            // 3d diagonals
            _connect(i, j, k, i + 1, j + 1, k + 1);
            _connect(i + 1, j, k, i, j + 1, k + 1);
            _connect(i, j + 1, k, i + 1, j, k + 1);
            _connect(i, j, k + 1, i + 1, j + 1, k);
          }

          // 2d diagonals
          if (i < Nx - 1 && j < Ny - 1) {
            _connect(i + 1, j, k, i, j + 1, k);
            _connect(i, j + 1, k, i + 1, j, k);
          }
          if (i < Nx - 1 && k < Nz - 1) {
            _connect(i + 1, j, k, i, j, k + 1);
            _connect(i, j, k + 1, i + 1, j, k);
          }
          if (j < Ny - 1 && k < Nz - 1) {
            _connect(i, j + 1, k, i, j, k + 1);
            _connect(i, j, k + 1, i, j + 1, k);
          }
        }
      }
    }
  }

  void _connect(i1, j1, k1, i2, j2, k2) {
    final distanceConstraint = SpringConstraint(
      particleBodies['$i1 $j1 $k1']!,
      particleBodies['$i2 $j2 $k2']!,
      stiffness :stiffness
    );
    constraints.add(distanceConstraint);
  }
}