import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

class SplitSolver extends StatefulWidget {
  const SplitSolver({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _SplitSolverState createState() => _SplitSolverState();
}

class _SplitSolverState extends State<SplitSolver> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -10,
        gz: 0,
        k: 1e7,
        d: 5
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
  void setScene(bool split){
    final world = demo.world;

    final solver = cannon.GSSolver();
    solver.iterations = 50;
    solver.tolerance = 0.0001;
    if (split) {
      world.solver = cannon.SplitSolver(solver);
    } else {
      world.solver = solver;
    }

    // Static ground plane
    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0 );
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    // Shape
    const size = 0.5;
    // const shape = cannon.Sphere(size)
    final shape = cannon.Box(vmath.Vector3(size * 0.5, size * 0.5, size * 0.5));

    // Shape on plane
    const N = 5;
    for (int i = 0; i < N; i++) {
      for (int j = 0; j < N; j++) {
        final shapeBody = cannon.Body( mass: 1 );
        shapeBody.addShape(shape);
        shapeBody.position.setValues((i - N * 0.5) * size * 2 * 1.1, size * 1.05, (j - N * 0.5) * size * 2 * 1.1);
        world.addBody(shapeBody);
        demo.addVisual(shapeBody);
      }
    }

    // Shape on top
    final shapeBody = cannon.Body(mass: 1 );
    shapeBody.addShape(shape);
    shapeBody.position.setValues(-size, size * 5, size);
    world.addBody(shapeBody);
    demo.addVisual(shapeBody)  ;
  }

  void setupWorld(){
    setScene(true);
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}