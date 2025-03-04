import 'package:flutter/material.dart';
import '../src/demo.dart';
import 'package:three_js/three_js.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:vector_math/vector_math.dart' as vmath;
import 'dart:math' as math;

class Bowling extends StatefulWidget {
  const Bowling({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _BowlingState createState() => _BowlingState();
}

class _BowlingState extends State<Bowling>{
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

    // Static ground plane
    final groundMaterial = cannon.Material(name:'ground');
    final groundShape = cannon.Plane();
    final groundBody = cannon.Body(mass: 0, material: groundMaterial);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromEuler(-math.pi / 2, 0, 0);
    world.addBody(groundBody);
    demo.addVisual(
      groundBody,
      mesh:Mesh(
        PlaneGeometry(10,100)..translate(0, 30, 0),
        MeshPhongMaterial.fromMap({'shininess': 10, 'name':'sph', 'color':0xff00ff})
      )
    );

    const double mass = 10;
    const double size = 1;
    const double height = 5;
    const double damping = 0.01;

    double padding = 0.3;
    int i = 1;
    final slipperyMaterial = cannon.Material(name: 'slippery');
    while(i < 5){
      double acw = size*(i-1);
      for(int j = 0; j < i; j++){
        final sphereShape = cannon.Cylinder(
          radiusTop: size/2,
          radiusBottom: size/2,
          height: height
        );//BowlingPin(height: height);
        // Shape on plane
        final shapeBody1 = cannon.Body(
          mass: mass,
          material: slipperyMaterial,
          position: vmath.Vector3(
            2*size*j+(10-acw)-size+padding*(j-3)-8.75,//-size * 3, 
            height/2, 
            -(2*size+padding) * (i-1)-60,//size
          ),
        );
        shapeBody1.addShape(sphereShape);
        shapeBody1.linearDamping = damping;
        world.addBody(shapeBody1);
        demo.addVisual(shapeBody1);
      }
      i++;
    }

    final sphereShape = cannon.Sphere(size*1.5);
    final shapeBody3 = cannon.Body(
      mass: mass*20,
      material: slipperyMaterial,
      position: vmath.Vector3(0, height, size),
      velocity: vmath.Vector3(1,1,-35)
    );
    shapeBody3.addShape(sphereShape);
    shapeBody3.linearDamping = damping;
    world.addBody(shapeBody3);
    demo.addVisual(shapeBody3);

    // Adjust constraint equation parameters for ground/ground contact
    final ground_ground = cannon.ContactMaterial(groundMaterial, groundMaterial,
      friction: 0.6,
      restitution: 0.3,
      contactEquationStiffness: 1e8,
      contactEquationRelaxation: 3,
      frictionEquationStiffness: 1e8,
      frictionEquationRelaxation: 3,
    );

    // Add contact material to the world
    world.addContactMaterial(ground_ground);

    // The ContactMaterial defines what happens when two materials meet.
    // In this case we want friction coefficient = 0.0 when the slippery material touches ground.
    final slippery_ground = cannon.ContactMaterial(groundMaterial, slipperyMaterial,
      friction: 0.05,
      restitution: 0.1,
      contactEquationStiffness: 1e8,
      contactEquationRelaxation: 3,
    );

    // We must add the contact materials to the world
    world.addContactMaterial(slippery_ground);
  }

  void setupWorld(){
    setScene();
  }
  @override
  Widget build(BuildContext context) {
    return demo.threeDart();
  }
}

class BowlingPin extends cannon.LatheShape {
  ///The height of the Capsule.
  double height;

  factory BowlingPin({
    double height = 1,
    int numSegments = 8,
    int numHeightSegments = 4
  }){
    double radius = height/2;
    List<vmath.Vector2> ptsTop = [
      vmath.Vector2(0, height*0.5+radius),
      vmath.Vector2(radius, -height*0.5)
    ];
    
    return BowlingPin.fromPoints(
      height: height,
      numSegments: numSegments,
      points: ptsTop
    );
  }

  /// @param radiusTop The radius of the top of the Cylinder.
  /// @param radiusBottom The radius of the bottom of the Cylinder.
  /// @param height The height of the Cylinder.
  /// @param numSegments The number of segments to build the cylinder out of.
  BowlingPin.fromPoints({
    required super.points,
    this.height = 1,
    super.numSegments = 8,
  }):super(
    type: cannon.ShapeType.convex
  );
}