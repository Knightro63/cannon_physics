import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

class RigidVehicle extends StatefulWidget {
  const RigidVehicle({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _RigidVehicleState createState() => _RigidVehicleState();
}

class _RigidVehicleState extends State<RigidVehicle> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -30,
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

    // Build the car chassis
    final chassisShape = cannon.Box(cannon.Vec3(5, 0.5, 2));
    final chassisBody = cannon.Body(mass: 1);
    final centerOfMassAdjust = cannon.Vec3(0, -1, 0);
    chassisBody.addShape(chassisShape, centerOfMassAdjust);
    demo.addVisual(chassisBody);

    // Create the vehicle
    final vehicle = cannon.RigidVehicle(
      chassisBody:chassisBody,
    );

    const mass = 1.0;
    const axisWidth = 7;
    final wheelShape = cannon.Sphere(1.5);
    final wheelMaterial = cannon.Material(name:'wheel');
    final down = cannon.Vec3(0, -1, 0);

    final wheelBody1 = cannon.Body(mass:mass, material: wheelMaterial );
    wheelBody1.addShape(wheelShape);
    vehicle.addWheel(
      body: wheelBody1,
      position: cannon.Vec3(-5, 0, axisWidth / 2).vadd(centerOfMassAdjust),
      axis: cannon.Vec3(0, 0, 1),
      direction: down,
    );

    final wheelBody2 = cannon.Body( mass:mass, material: wheelMaterial );
    wheelBody2.addShape(wheelShape);
    vehicle.addWheel(
      body: wheelBody2,
      position: cannon.Vec3(-5, 0, -axisWidth / 2).vadd(centerOfMassAdjust),
      axis: cannon.Vec3(0, 0, -1),
      direction: down,
    );

    final wheelBody3 = cannon.Body( mass:mass, material: wheelMaterial );
    wheelBody3.addShape(wheelShape);
    vehicle.addWheel(
      body: wheelBody3,
      position: cannon.Vec3(5, 0, axisWidth / 2).vadd(centerOfMassAdjust),
      axis: cannon.Vec3(0, 0, 1),
      direction: down,
    );

    final wheelBody4 = cannon.Body(mass:mass, material: wheelMaterial );
    wheelBody4.addShape(wheelShape);
    vehicle.addWheel(
      body: wheelBody4,
      position: cannon.Vec3(5, 0, -axisWidth / 2).vadd(centerOfMassAdjust),
      axis: cannon.Vec3(0, 0, -1),
      direction: down,
    );

    vehicle.wheelBodies.forEach((wheelBody){
      // Some damping to not spin wheels too fast
      wheelBody.angularDamping = 0.4;

      // Add visuals
      demo.addVisual(wheelBody);
    });

    vehicle.addToWorld(world);

    // Add the ground
    const sizeX = 64;
    const sizeZ = sizeX;
    List<List<double>> matrix = [];
    for (int i = 0; i < sizeX; i++) {
      matrix.add([]);
      for (int j = 0; j < sizeZ; j++) {
        if (i == 0 || i == sizeX - 1 || j == 0 || j == sizeZ - 1) {
          const height = 6.0;
          matrix[i].add(height);
          continue;
        }

        double height = Math.sin((i / sizeX) * Math.PI * 7) * Math.sin((j / sizeZ) * Math.PI * 7) * 6 + 6;
        matrix[i].add(height);
      }
    }

    final groundMaterial = cannon.Material(name:'ground');
    final heightfieldShape = cannon.Heightfield(matrix, 
      elementSize: 300 ~/ sizeX,
    );
    final heightfieldBody = cannon.Body(mass: 0, material: groundMaterial);
    heightfieldBody.addShape(heightfieldShape);
    heightfieldBody.position.set(
      (-(sizeX - 1) * heightfieldShape.elementSize) / 2,
      -15,
      ((sizeZ - 1) * heightfieldShape.elementSize) / 2
    );
    heightfieldBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(heightfieldBody);
    demo.addVisual(heightfieldBody);

    // Define interactions between wheels and ground
    final wheel_ground = cannon.ContactMaterial(wheelMaterial, groundMaterial,
      friction: 0.3,
      restitution: 0,
      contactEquationStiffness: 1000,
    );
    world.addContactMaterial(wheel_ground);

    // final groundShape = cannon.Plane();
    // final groundBody = cannon.Body(mass: 0);
    // groundBody.addShape(groundShape);
    // groundBody.position.set(0, -1, 0);
    // groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    // world.addBody(groundBody);
    // demo.addVisual(groundBody);

    // // Define interactions between wheels and ground
    // final wheel_ground = cannon.ContactMaterial(wheelMaterial, groundMaterial, 
    //   friction: 0.3,
    //   restitution: 0,
    //   contactEquationStiffness: 1000,
    // );
    // world.addContactMaterial(wheel_ground);

    // Keybindings
    // Add force on keydown
    demo.addDomListener('keydown', (event){
      const maxSteerVal = Math.PI / 8;
      const maxSpeed = 10.0;
      const maxForce = 100.0;

      switch (event.keyId) {
        case 4294968068:
        case 119:
          vehicle.setWheelForce(maxForce, 2);
          vehicle.setWheelForce(-maxForce, 3);
          break;

        case 115:
        case 4294968065:
          vehicle.setWheelForce(-maxForce/2, 2);
          vehicle.setWheelForce(maxForce/2, 3);
          break;

        case 97:
        case 4294968066:
          vehicle.setSteeringValue(maxSteerVal, 0);
          vehicle.setSteeringValue(maxSteerVal, 1);
          break;

        case 4294968067:
        case 100:
          vehicle.setSteeringValue(-maxSteerVal, 0);
          vehicle.setSteeringValue(-maxSteerVal, 1);
          break;
      }
    });

    // Reset force on keyup
    demo.addDomListener('keyup', (event){
      switch (event.keyId) {
        case 4294968068:
        case 119:
          vehicle.setWheelForce(0, 2);
          vehicle.setWheelForce(0, 3);
          break;

        case 115:
        case 4294968065:
          vehicle.setWheelForce(0, 2);
          vehicle.setWheelForce(0, 3);
          break;

        case 97:
        case 4294968066:
          vehicle.setSteeringValue(0, 0);
          vehicle.setSteeringValue(0, 1);
          break;

        case 4294968067:
        case 100:
          vehicle.setSteeringValue(0, 0);
          vehicle.setSteeringValue(0, 1);
          break;
      }
    });
  }

  void setupWorld(){
    demo.world.broadphase = cannon.SAPBroadphase(demo.world);
    demo.world.defaultContactMaterial.friction = 0.2;
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}