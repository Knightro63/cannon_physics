import 'package:flutter/material.dart';
import 'dart:math' as math;
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

class RaycastVehicle extends StatefulWidget {
  const RaycastVehicle({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _RaycastVehicleState createState() => _RaycastVehicleState();
}

class _RaycastVehicleState extends State<RaycastVehicle> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -10,
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

    final chassisShape = cannon.Box(vmath.Vector3(2, 0.5, 1));
    final chassisBody = cannon.Body(mass: 150 );
    chassisBody.addShape(chassisShape);
    chassisBody.position.setValues(0, 4, 0);
    chassisBody.angularVelocity.setValues(0, 0.5, 0);
    demo.addVisual(chassisBody);

    // Create the vehicle
    final vehicle = cannon.RaycastVehicle(
      chassisBody: chassisBody,
    );

    final wheelOptions = cannon.WheelInfo(
      radius: 0.5,
      directionLocal: vmath.Vector3(0, -1, 0),
      suspensionStiffness: 30,
      suspensionRestLength: 0.3,
      frictionSlip: 1.4,
      dampingRelaxation: 2.3,
      dampingCompression: 4.4,
      maxSuspensionForce: 100000,
      rollInfluence: 0.01,
      axleLocal: vmath.Vector3(0, 0, 1),
      maxSuspensionTravel: 0.3,
      customSlidingRotationalSpeed: -30,
      useCustomSlidingRotationalSpeed: true,
    );

    vehicle.addWheel(wheelOptions.copy()..chassisConnectionPointLocal.setValues(-1, 0, 1));
    vehicle.addWheel(wheelOptions.copy()..chassisConnectionPointLocal.setValues(-1, 0, -1));
    vehicle.addWheel(wheelOptions.copy()..chassisConnectionPointLocal.setValues(1, 0, 1));
    vehicle.addWheel(wheelOptions.copy()..chassisConnectionPointLocal.setValues(1, 0, -1));

    vehicle.addToWorld(world);

    // Add the wheel bodies
    List<cannon.Body> wheelBodies = [];
    final wheelMaterial = cannon.Material(name: 'wheel');
    vehicle.wheelInfos.forEach((wheel){
      final cylinderShape = cannon.Cylinder(
        radiusTop:wheel.radius, 
        radiusBottom: wheel.radius, 
        height: wheel.radius / 2, 
        numSegments:20
      );
      final wheelBody = cannon.Body(
        mass: 0,
        material: wheelMaterial,
      );
      wheelBody.type = cannon.BodyTypes.kinematic;
      wheelBody.collisionFilterGroup = 0; // turn off collisions
      final quaternion = vmath.Quaternion(0,0,0,1).setFromEuler(-math.pi / 2, 0, 0);
      wheelBody.addShape(cylinderShape, vmath.Vector3.zero(), quaternion);
      wheelBodies.add(wheelBody);
      demo.addVisual(wheelBody);
      world.addBody(wheelBody);
    });

    // Update the wheel bodies
    world.addEventListener('postStep', (event){
      for (int i = 0; i < vehicle.wheelInfos.length; i++) {
        vehicle.updateWheelTransform(i);
        final transform = vehicle.wheelInfos[i].worldTransform;
        final wheelBody = wheelBodies[i];
        wheelBody.position.setFrom(transform.position);
        wheelBody.quaternion.setFrom(transform.quaternion);
      }
    });

    // Add the ground
    const sizeX = 64;
    const sizeZ = 64;
    List<List<double>> matrix = [];
    for (int i = 0; i < sizeX; i++) {
      matrix.add([]);
      for (int j = 0; j < sizeZ; j++) {
        if (i == 0 || i == sizeX - 1 || j == 0 || j == sizeZ - 1) {
          const height = 3.0;
          matrix[i].add(height);
          continue;
        }

        final height = math.cos((i / sizeX) * math.pi * 5) * math.cos((j / sizeZ) * math.pi * 5) * 2 + 2;
        matrix[i].add(height);
      }
    }

    final groundMaterial = cannon.Material(name: 'ground');
    final heightfieldShape = cannon.Heightfield(
      matrix, 
      elementSize: 100 ~/ sizeX,
    );
    final heightfieldBody = cannon.Body(mass: 0, material: groundMaterial);
    heightfieldBody.addShape(heightfieldShape);
    heightfieldBody.position.setValues(
      // -((sizeX - 1) * heightfieldShape.elementSize) / 2,
      -(sizeX * heightfieldShape.elementSize) / 2,
      -1,
      // ((sizeZ - 1) * heightfieldShape.elementSize) / 2
      (sizeZ * heightfieldShape.elementSize) / 2
    );
    heightfieldBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    world.addBody(heightfieldBody);
    demo.addVisual(heightfieldBody);

    // Define interactions between wheels and ground
    final wheelGround = cannon.ContactMaterial(wheelMaterial, groundMaterial, 
      friction: 0.3,
      restitution: 0,
      contactEquationStiffness: 1000,
    );
    world.addContactMaterial(wheelGround);

    // Keybindings
    // Add force on keydown
    demo.addDomListener('keydown', (event){
      const maxSteerVal = 0.5;
      const maxForce = 10000.0;
      const brakeForce = 10000.0;

      switch (event.keyId) {
        case 4294968068:
        case 119:
          vehicle.applyEngineForce(-maxForce, 2);
          vehicle.applyEngineForce(-maxForce, 3);
          break;

        case 115:
        case 4294968065:
          vehicle.applyEngineForce(maxForce, 2);
          vehicle.applyEngineForce(maxForce, 3);
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

        case 98:
          vehicle.setBrake(brakeForce, 0);
          vehicle.setBrake(brakeForce, 1);
          vehicle.setBrake(brakeForce, 2);
          vehicle.setBrake(brakeForce, 3);
          break;
      }
    });

    // Reset force on keyup
    demo.addDomListener('keyup', (event){
      switch (event.keyId) {
        case 4294968068:
        case 119:
          vehicle.applyEngineForce(0, 2);
          vehicle.applyEngineForce(0, 3);
          break;

        case 115:
        case 4294968065:
          vehicle.applyEngineForce(0, 2);
          vehicle.applyEngineForce(0, 3);
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

        case 98:
          vehicle.setBrake(0, 0);
          vehicle.setBrake(0, 1);
          vehicle.setBrake(0, 2);
          vehicle.setBrake(0, 3);
          break;
      }
    });

    demo.addAnimationEvent((p0){

    });
  }

  void setupWorld(){
    demo.world.broadphase = cannon.SAPBroadphase(demo.world);
    demo.world.defaultContactMaterial.friction = 0;
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}