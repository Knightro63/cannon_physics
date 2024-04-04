import 'package:cannon_physics/cannon_physics.dart';
import 'package:cannon_physics/rigid_body_shapes/shape.dart';

/// Simple vehicle helper class with spherical rigid body wheels.
class SoftBody {
  /// The bodies of the wheels.
  Map<String,Body> particleBodies = {};
  late Vec3 coordinateSystem;
  /// The constraints.
  List<Constraint> constraints = [];
  /// shape of the body
  ShapeType shape;
  /// total mass of the body
  double mass;

  double stiffness;
  
  SoftBody({
    required this.shape,
    this.mass = 1,
    this.stiffness = 100,
    int numOfParticles = 18,
  });

  void _connect(i1, j1, k1, i2, j2, k2, distance) {
    final distanceConstraint = SpringConstraint(
      particleBodies['$i1 $j1 $k1']!,
      particleBodies['$i2 $j2 $k2']!,
      stiffness :stiffness
    );
    constraints.add(distanceConstraint);
  }

  /// Add the vehicle including its constraints to the world.
  void addToWorld(World world) {
    for (final key in particleBodies.keys) {
      world.addBody(particleBodies[key]!);
    }
    for (int i = 0; i < constraints.length; i++) {
      world.addConstraint(constraints[i]);
    }
    //world.addEventListener('preStep', _update);
  }

  //void _update(event){}

  /// Remove the vehicle including its constraints from the world.
  void removeFromWorld(World world) {
    for(final key in particleBodies.keys){
      world.removeBody(particleBodies[key]!);
    }

    for (int i = 0; i < constraints.length; i++) {
      world.removeConstraint(constraints[i]);
    }
  }
}