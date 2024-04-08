import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

class Constraints extends StatefulWidget {
  const Constraints({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _ConstraintsState createState() => _ConstraintsState();
}

class _ConstraintsState extends State<Constraints> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -40,
        gz: 0,
      )
    );
    setupWorld();
    super.initState();
  }
  @override
  void dispose() {
    demo.dispose();
    super.dispose();
  }
  void setScene(){
    final world = demo.world;

    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.position.setValues(0, 1, 0);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);
  }

  void lockScene(){
    setScene();
    final world = demo.world;

    world.gravity.setValues(0, -10, 0);
    world.solver.iterations = 20;

    const size = 0.5;
    const mass = 1.0;
    const space = size * 0.1;

    final boxShape = cannon.Box(vmath.Vector3(size, size, size));

    const N = 10;
    cannon.Body? previous;
    for (int i = 0; i < N; i++) {
      // Create a box
      final boxBody = cannon.Body(
        mass: mass,
        shape: boxShape,
        position: vmath.Vector3(-(N - i - N / 2) * (size * 2 + 2 * space), size * 6 + space, 0),
      );
      world.addBody(boxBody);
      demo.addVisual(boxBody);

      if (previous != null) {
        // Connect the current body to the last one
        final lockConstraint = cannon.LockConstraint(
          boxBody, 
          previous
        );
        world.addConstraint(lockConstraint);
      }

      // To keep track of which body was added last
      previous = boxBody;
    }

    // Create stands
    final body1 = cannon.Body(
      mass: 0,
      shape: boxShape,
      position: vmath.Vector3(-(-N / 2 + 1) * (size * 2 + 2 * space), size * 3, 0),
    );
    world.addBody(body1);
    demo.addVisual(body1);

    final body2 = cannon.Body(
      mass: 0,
      shape: boxShape,
      position: vmath.Vector3(-(N / 2) * (size * 2 + space * 2), size * 3, 0),
    );
    world.addBody(body2);
    demo.addVisual(body2);
  }
  void linkScene(){
    setScene();
    final world = demo.world;
    world.gravity.setValues(0, -20, -1);

    const size = 1.0;
    double mass = 0;
    const space = size * 0.1;

    final boxShape = cannon.Box(vmath.Vector3(size, size, size * 0.1));

    const N = 10;
    cannon.Body? previous;
    for (int i = 0; i < N; i++) {
      // Create a box
      final boxBody = cannon.Body( mass:mass );
      boxBody.addShape(boxShape);
      boxBody.position.setValues(0, (N - i) * (size * 2 + space * 2) + size * 2 + space, 0);
      boxBody.linearDamping = 0.01; // Damping makes the movement slow down with time
      boxBody.angularDamping = 0.01;
      world.addBody(boxBody);
      demo.addVisual(boxBody);

      if (i != 0) {
        // Connect the current body to the last one
        // We connect two corner points to each other.
        final pointConstraint1 = cannon.PointToPointConstraint(
          boxBody,
          previous!,
          vmath.Vector3(size, size + space, 0),
          vmath.Vector3(size, -size - space, 0)
        );
        final pointConstraint2 = cannon.PointToPointConstraint(
          boxBody,
          previous,
          vmath.Vector3(-size, size + space, 0),
          vmath.Vector3(-size, -size - space, 0)
        );

        world.addConstraint(pointConstraint1);
        world.addConstraint(pointConstraint2);
      } else {
        // First body is now static. The rest should be dynamic.
        mass = 0.3;
      }

      // To keep track of which body was added last
      previous = boxBody;
    }
  }
  void clothOnSphere(){
    setScene();
    final world = demo.world;

    const dist = 0.2;
    const mass = 0.5;
    // To construct the cloth we need rows*cols particles.
    const rows = 15;
    const cols = 15;

    Map<String,cannon.Body> bodies = {}; // bodies['i j'] => particle
    for (int i = 0; i < cols; i++) {
      for (int j = 0; j < rows; j++) {
        // Create a new body
        final body = cannon.Body(mass: mass);
        body.addShape(cannon.Particle());
        body.position.setValues(-(i - cols * 0.5) * dist, 5, (j - rows * 0.5) * dist);
        bodies['$i $j'] = body;
        world.addBody(body);
        demo.addVisual(body);
      }
    }

    // To connect two particles, we use a distance constraint. This forces the particles to be at a constant distance from each other.
    void connect(i1, j1, i2, j2) {
      final distanceConstraint = cannon.DistanceConstraint(
        bodies['$i1 $j1']!, 
        bodies['$i2 $j2']!, 
        dist
      );
      world.addConstraint(distanceConstraint);
    }

    for (int i = 0; i < cols; i++) {
      for (int j = 0; j < rows; j++) {
        // Connect particle at position (i,j) to (i+1,j) and to (i,j+1).
        if (i < cols - 1) connect(i, j, i + 1, j);
        if (j < rows - 1) connect(i, j, i, j + 1);
      }
    }

    // Add the static sphere we throw the cloth on top of
    final sphere = cannon.Sphere(1.5);
    final body = cannon.Body(mass: 0 );
    body.addShape(sphere);
    body.position.setValues(0, 3.5, 0);
    world.addBody(body);
    demo.addVisual(body);
  }
  void spherePendulum(){
    setScene();
    final world = demo.world;

    const size = 1.0;
    const mass = 1.0;

    final sphereShape = cannon.Sphere(size);

    final spherebody = cannon.Body(mass:mass );
    spherebody.addShape(sphereShape);
    spherebody.position.setValues(0, size * 3, 0);
    spherebody.velocity.setValues(-5, 0, 0);
    spherebody.linearDamping = 0;
    spherebody.angularDamping = 0;
    world.addBody(spherebody);
    demo.addVisual(spherebody);

    final spherebody2 = cannon.Body(mass: 0 );
    spherebody2.addShape(sphereShape);
    spherebody2.position.setValues(0, size * 7, 0);
    world.addBody(spherebody2);
    demo.addVisual(spherebody2);

    // Connect this body to the last one
    final pointConstraint = cannon.PointToPointConstraint(
      spherebody,
      
      spherebody2,
      vmath.Vector3(0, size * 2, 0),
      vmath.Vector3(0, -size * 2, 0)
    );
    world.addConstraint(pointConstraint);
  }
  void sphereChain(){
    setScene();
    final world = demo.world;
    // world.solver.setSpookParams(1e20, 3)

    const size = 0.5;
    const dist = size * 2 + 0.12;
    const mass = 1.0;
    const N = 20;

    world.solver.iterations = N ;// To be able to propagate force throw the chain of N spheres, we need at least N solver iterations.

    final sphereShape = cannon.Sphere(size);

    cannon.Body? previous;
    for (int i = 0; i < N; i++) {
      // Create a new body
      final sphereBody = cannon.Body(mass: i == 0 ? 0 : mass);
      sphereBody.addShape(sphereShape);
      sphereBody.position.setValues(0, dist * (N - i), 0);
      sphereBody.velocity.x = -i*1.0;
      world.addBody(sphereBody);
      demo.addVisual(sphereBody);

      // Connect this body to the last one added
      if (previous != null) {
        final distanceConstraint = cannon.DistanceConstraint(sphereBody, previous, dist);
        world.addConstraint(distanceConstraint);
      }

      // Keep track of the lastly added body
      previous = sphereBody;
    }
  }
  void particleCloth(){
    setScene();
    final world = demo.world;
    // world.solver.setSpookParams(1e20, 3)
    world.solver.iterations = 18;

    const dist = 0.2;
    const mass = 0.5;
    const rows = 15;
    const cols = 15;

    Map<String,cannon.Body> bodies = {}; // bodies['i j'] => particle
    for (int i = 0; i < cols; i++) {
      for (int j = 0; j < rows; j++) {
        // Create a new body
        final body = cannon.Body(mass: j == rows - 1 ? 0 : mass);
        body.addShape(cannon.Particle());
        body.position.setValues(-dist * i, dist * j + 5, 0);
        body.velocity.setValues(0, 0, (Math.sin(i * 0.1) + Math.sin(j * 0.1)) * 3);
        bodies['$i $j'] = body;
        world.addBody(body);
        demo.addVisual(body);
      }
    }

    void connect(i1, j1, i2, j2) {
      final distanceConstraint = cannon.DistanceConstraint(
        bodies['$i1 $j1']!, 
        bodies['$i2 $j2']!, 
        dist
      );
      world.addConstraint(distanceConstraint);
    }

    for (int i = 0; i < cols; i++) {
      for (int j = 0; j < rows; j++) {
        if (i < cols - 1) connect(i, j, i + 1, j);
        if (j < rows - 1) connect(i, j, i, j + 1);
      }
    }
  }
  void clothStructure(){
    setScene();
    final world = demo.world;

    // Max solver iterations: Use more for better force propagation, but keep in mind that it's not very computationally cheap!
    world.solver.iterations = 10;

    const dist = 1.0;
    const mass = 1.0;
    const Nx = 6;
    const Ny = 3;
    const Nz = 3;

    Map<String,cannon.Body> bodies = {}; // bodies['i j k'] => particle
    for (int i = 0; i < Nx; i++) {
      for (int j = 0; j < Ny; j++) {
        for (int k = 0; k < Nz; k++) {
          // Create a new body
          final body = cannon.Body( mass:mass );
          body.addShape(cannon.Particle());
          body.position.setValues(-dist * i, dist * k + dist * Nz * 0.3 + 1, dist * j);
          body.velocity.setValues(0, 0, (Math.sin(i * 0.1) + Math.sin(j * 0.1)) * 30);
          bodies['$i $j $k'] = body;
          world.addBody(body);
          demo.addVisual(body);
        }
      }
    }

    void connect(i1, j1, k1, i2, j2, k2, distance) {
      final distanceConstraint = cannon.DistanceConstraint(
        bodies['$i1 $j1 $k1']!,
        bodies['$i2 $j2 $k2']!,
        distance
      );
      world.addConstraint(distanceConstraint);
    }

    for (int i = 0; i < Nx; i++) {
      for (int j = 0; j < Ny; j++) {
        for (int k = 0; k < Nz; k++) {
          // normal directions
          if (i < Nx - 1) connect(i, j, k, i + 1, j, k, dist);
          if (j < Ny - 1) connect(i, j, k, i, j + 1, k, dist);
          if (k < Nz - 1) connect(i, j, k, i, j, k + 1, dist);

          // Diagonals
          if (i < Nx - 1 && j < Ny - 1 && k < Nz - 1) {
            // 3d diagonals
            connect(i, j, k, i + 1, j + 1, k + 1, Math.sqrt(3) * dist);
            connect(i + 1, j, k, i, j + 1, k + 1, Math.sqrt(3) * dist);
            connect(i, j + 1, k, i + 1, j, k + 1, Math.sqrt(3) * dist);
            connect(i, j, k + 1, i + 1, j + 1, k, Math.sqrt(3) * dist);
          }

          // 2d diagonals
          if (i < Nx - 1 && j < Ny - 1) {
            connect(i + 1, j, k, i, j + 1, k, Math.sqrt(2) * dist);
            connect(i, j + 1, k, i + 1, j, k, Math.sqrt(2) * dist);
          }
          if (i < Nx - 1 && k < Nz - 1) {
            connect(i + 1, j, k, i, j, k + 1, Math.sqrt(2) * dist);
            connect(i, j, k + 1, i + 1, j, k, Math.sqrt(2) * dist);
          }
          if (j < Ny - 1 && k < Nz - 1) {
            connect(i, j + 1, k, i, j, k + 1, Math.sqrt(2) * dist);
            connect(i, j, k + 1, i, j + 1, k, Math.sqrt(2) * dist);
          }
        }
      }
    }
  }
  void setupWorld(){
    demo.addScene('Lock',lockScene);
    demo.addScene('Link',clothOnSphere);
    demo.addScene('Sphere Pendulum',spherePendulum);
    demo.addScene('Sphere Chain',sphereChain);
    demo.addScene('Particle Cloth',particleCloth);
    demo.addScene('Cloth Structure',clothStructure);
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}