import 'package:flutter/material.dart';
import 'dart:math' as math;
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class SPH extends StatefulWidget {
  const SPH({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _SPHState createState() => _SPHState();
}

class _SPHState extends State<SPH> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -10,
        gz: 0,
        k: 1e11,
        d: 2,
        iterations: 10
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

    const width = 10;
    const height = 5;
    const mass = 0.01;

    final sph = cannon.SPHSystem();
    sph.density = 1;
    sph.viscosity = 0.03;
    sph.smoothingRadius = 1.0;
    world.subsystems.add(sph);

    // Same material for everything
    final material = cannon.Material();
    final material_material = cannon.ContactMaterial(material, material, 
      friction: 0.06,
      restitution: 0.0,
    );
    world.addContactMaterial(material_material);

    // Ground plane
    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0, material: material);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    // Plane -x
    final planeShapeXmin = cannon.Plane();
    final planeXmin = cannon.Body(mass: 0, material:material);
    planeXmin.addShape(planeShapeXmin);
    planeXmin.quaternion.setFromEuler(0, math.pi / 2, 0);
    planeXmin.position.setValues(-width * 0.5, 0, 0);
    world.addBody(planeXmin);

    // Plane +x
    final planeShapeXmax = cannon.Plane();
    final planeXmax = cannon.Body(mass: 0, material: material);
    planeXmax.addShape(planeShapeXmax);
    planeXmax.quaternion.setFromEuler(0, -math.pi / 2, 0);
    planeXmax.position.setValues(width * 0.5, 0, 0);
    world.addBody(planeXmax);

    // Plane -z
    final planeShapeZmin = cannon.Plane();
    final planeZmin = cannon.Body(mass: 0, material:material);
    planeZmin.addShape(planeShapeZmin);
    planeZmin.quaternion.setFromEuler(0, 0, 0);
    planeZmin.position.setValues(0, 0, -height * 0.5);
    world.addBody(planeZmin);

    // Plane +z
    final planeShapeZmax = cannon.Plane();
    final planeZmax = cannon.Body(mass: 0, material:material);
    planeZmax.addShape(planeShapeZmax);
    planeZmax.quaternion.setFromEuler(0, math.pi, 0);
    planeZmax.position.setValues(0, 0, height * 0.5);
    world.addBody(planeZmax);

    // Create particles
    const nx = 4;
    const ny = 15;
    const nz = 4;
    const randRange = 0.1;
    for (int i = 0; i < nx; i++) {
      for (int j = 0; j < nz; j++) {
        for (int k = 0; k < ny; k++) {
          final particle = cannon.Body(mass:mass, material:material);
          particle.addShape(cannon.Particle());
          particle.position.setValues(
            ((i + (math.Random().nextDouble() - 0.5) * randRange + 0.5) * width) / nx - width * 0.5,
            (k * height) / nz,
            ((j + (math.Random().nextDouble() - 0.5) * randRange + 0.5) * height) / nz - height * 0.5
          );
          world.addBody(particle);
          sph.add(particle);
          demo.addVisual(particle);
        }
      }
    }
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}