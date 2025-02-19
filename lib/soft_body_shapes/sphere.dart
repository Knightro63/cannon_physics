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
    
    for (int iy = 0; iy <= heightSegments; iy++) {
      final theta = ((2 * pi) / heightSegments) * (iy + 1);
      final thetaN = ((2 * pi) / heightSegments) * (iy + 0.5);
      final body1 = Body(mass: mass);
      body1.addShape(Particle());
      body1.position.setValues(
       -radius * sin(theta), 
       -radius * 0.55, 
       radius* cos(theta), 
      );
      particleBodies['$iy'] = body1;
      double v = iy / heightSegments * pi+pi/2;
      for (int ix = 0; ix <= widthSegments; ix++) {
        final u = ix / widthSegments * (pi);
        final body = Body(mass: mass);
        body.addShape(Particle());
        body.position.setValues(
          -radius * cos(u) * sin(v), 
          radius * 0.5-radius * cos(v), 
          radius * sin(u) * sin(v)
        );
        body.velocity.setValues(0, 0, (sin(iy * 0.1) + sin(ix * 0.1)) * 30);
        particleBodies['$iy $ix'] = body;
      }
    }

    for (var iy = 0; iy < 1; iy++) {
      _connect2(iy, iy+1);
      for (var ix = 0; ix < widthSegments; ix++) {
        //if (iy != 0){
          _connect(iy, ix+1, iy, ix);
          _connect(iy, ix, iy + 1, ix+1);
        //}
        //if (iy != heightSegments) {
          _connect(iy, ix, iy+1, ix);
          _connect(iy+1, ix, iy + 1, ix+1);
        //}
      }
    }
  }
  void _connect2(int i1, int i2) {
    final distanceConstraint = SpringConstraint(
      particleBodies['$i1']!,
      particleBodies['$i2']!,
      stiffness :stiffness
    );
    constraints.add(distanceConstraint);
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