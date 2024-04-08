import 'package:flutter/material.dart';
import 'package:three_dart/three_dart.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;

class Hinge extends StatefulWidget {
  const Hinge({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _HingeState createState() => _HingeState();
}

class _HingeState extends State<Hinge> {
  late Demo demo;

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      settings: DemoSettings(
        gx: 0,
        gy: -20,
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

    // Static ground plane
    final groundMaterial = cannon.Material(name: 'ground');
    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0, material: groundMaterial );
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    groundBody.position.y = -1.2;
    world.addBody(groundBody);
    demo.addVisual(groundBody);

    const mass = 1.0;

    // Wheels
    final wheelMaterial = cannon.Material(name:'wheel');
    final wheelShape = cannon.Sphere(1.2);
    final leftFrontWheel = cannon.Body(mass: mass, material: wheelMaterial);
    leftFrontWheel.addShape(wheelShape);
    final rightFrontWheel = cannon.Body(mass: mass, material: wheelMaterial);
    rightFrontWheel.addShape(wheelShape);
    final leftRearWheel = cannon.Body(mass: mass, material: wheelMaterial);
    leftRearWheel.addShape(wheelShape);
    final rightRearWheel = cannon.Body(mass: mass, material: wheelMaterial);
    rightRearWheel.addShape(wheelShape);

    final chassisShape = cannon.Box(vmath.Vector3(5, 0.5, 2));
    final chassis = cannon.Body(mass: mass);
    chassis.addShape(chassisShape);

    // Define interaction between ground and wheels
    final wheelGroundContactMaterial = cannon.ContactMaterial(groundMaterial, wheelMaterial,
      friction: 0.5,
      restitution: 0.3,
    );
    world.addContactMaterial(wheelGroundContactMaterial);

    // Position finalrain wheels
    leftFrontWheel.position.setValues(-5, 0, 5);
    rightFrontWheel.position.setValues(-5, 0, -5);
    leftRearWheel.position.setValues(5, 0, 5);
    rightRearWheel.position.setValues(5, 0, -5);

    // finalrain wheels
    List constraints = [];

    // Hinge the wheels
    final leftAxis = vmath.Vector3(0, 0, 1);
    final rightAxis = vmath.Vector3(0, 0, -1);
    // final leftFrontAxis = vmath.Vector3(0, 0, 1);
    // final rightFrontAxis = vmath.Vector3(0, 0, -1);
    final leftFrontAxis = vmath.Vector3(-0.3, 0, 0.7);
    final rightFrontAxis = vmath.Vector3(0.3, 0, -0.7);
    leftFrontAxis.normalize();
    rightFrontAxis.normalize();

    constraints.add(
      cannon.HingeConstraint(chassis, leftFrontWheel,
        pivotA: vmath.Vector3(-5, 0, 5),
        axisA: leftFrontAxis,
        axisB: leftAxis,
      )
    );
    constraints.add(
      cannon.HingeConstraint(chassis, rightFrontWheel,
        pivotA: vmath.Vector3(-5, 0, -5),
        axisA: rightFrontAxis,
        axisB: rightAxis,
      )
    );
    constraints.add(
      cannon.HingeConstraint(chassis, leftRearWheel,
        pivotA: vmath.Vector3(5, 0, 5),
        axisA: leftAxis,
        axisB: leftAxis,
      )
    );
    constraints.add(
      cannon.HingeConstraint(chassis, rightRearWheel,
        pivotA: vmath.Vector3(5, 0, -5),
        axisA: rightAxis,
        axisB: rightAxis,
      )
    );

    constraints.forEach((constraint){
      world.addConstraint(constraint);
    });

    final bodies = [chassis, leftFrontWheel, rightFrontWheel, leftRearWheel, rightRearWheel];
    bodies.forEach((body){
      world.addBody(body);
      demo.addVisual(body);
    });

    // Enable motors and set their velocities
    final frontLeftHinge = constraints[2];
    final frontRightHinge = constraints[3];
    frontLeftHinge.enableMotor();
    frontRightHinge.enableMotor();
    const velocity = -14.0;
    frontLeftHinge.setMotorSpeed(velocity);
    frontRightHinge.setMotorSpeed(-velocity);
  }

  void setScene2(){
    final world = demo.world;
     world.gravity.setValues(0, -20, 5);

    const size = 5.0;
    const distance = size * 0.1;

    final shape = cannon.Box(vmath.Vector3(size * 0.5, size * 0.5, size * 0.1));
    final hingedBody = cannon.Body(mass:1);
    hingedBody.addShape(shape);
    world.addBody(hingedBody);
    demo.addVisual(hingedBody);

    final staticBody = cannon.Body(mass: 0 );
    staticBody.addShape(shape);
    staticBody.position.y = size + distance * 2;
    world.addBody(staticBody);
    demo.addVisual(staticBody);

    // Hinge it
    final finalraint = cannon.HingeConstraint(staticBody, hingedBody,
      pivotA: vmath.Vector3(0, -size * 0.5 - distance, 0),
      axisA: vmath.Vector3(-1, 0, 0),
      pivotB: vmath.Vector3(0, size * 0.5 + distance, 0),
      axisB: vmath.Vector3(-1, 0, 0),
    );
    world.addConstraint(finalraint);
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}