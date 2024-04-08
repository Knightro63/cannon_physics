import 'package:cannon_physics/constraints/spring_constraint.dart';
import 'package:cannon_physics/objects/rigid_body.dart';
import 'package:cannon_physics/rigid_body_shapes/particle.dart';
import 'package:cannon_physics/rigid_body_shapes/shape.dart';
import 'dart:math';
import 'package:cannon_physics/objects/soft_body.dart';

/// Simple vehicle helper class with spherical rigid body wheels.
class SphereSoftBody extends SoftBody{
  double radius;

  SphereSoftBody({
    required this.radius,
    super.mass = 1,
    super.stiffness = 100,
    int numOfParticles = 12,
  }):super(shape: ShapeType.box){
    final mass = this.mass/numOfParticles;

    int widthSegments = 4*numOfParticles~/8;//4*8
    int heightSegments = 4*numOfParticles~/4;//4*4

    widthSegments = max(3, widthSegments.floor());
    heightSegments = max(2,heightSegments.floor());
    
    for (var iy = 0; iy <= heightSegments; iy++) {
      double v = iy / heightSegments;
      for (var ix = 0; ix <= widthSegments; ix++) {
        var u = ix / widthSegments;
        final body = Body(mass: mass);
        body.addShape(Particle());
        body.position.setValues(
          -radius * cos(u * pi * 2) * sin(v * pi), 
          radius * cos(v * pi), 
          radius * sin( u * pi * 2) * sin(v * pi)
        );
        body.velocity.setValues(0, 0, (sin(iy * 0.1) + sin(ix * 0.1)) * 30);
        particleBodies['$iy $ix'] = body;
      }
    }

    for (var iy = 0; iy < heightSegments; iy++) {
      for (var ix = 0; ix < widthSegments; ix++) {
        if (iy != 0){
          _connect(iy, ix+1, iy, ix);
          _connect(iy, ix, iy + 1, ix+1);
        }
        if (iy != heightSegments - 1) {
          _connect(iy, ix, iy+1, ix);
          _connect(iy+1, ix, iy + 1, ix+1);
        }
      }
    }
  }

  void _connect(int i1, int j1, int i2, int j2) {
    final distanceConstraint = SpringConstraint(
      particleBodies['$i1 $j1']!,
      particleBodies['$i2 $j2']!,
      stiffness :stiffness
    );
    constraints.add(distanceConstraint);
  }
}