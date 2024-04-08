import 'package:flutter/material.dart';
import '../src/demo.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

import 'package:three_dart/three_dart.dart' as three;
import 'package:three_dart/three_dart.dart' hide Texture, Color;
import 'package:vector_math/vector_math.dart' as vmath;
import '../src/conversion_utils.dart';

class BasicPhysics extends StatefulWidget {
  const BasicPhysics({
    Key? key,
    this.offset = const Offset(0,0),
    this.settings
  }) : super(key: key);

  final Offset offset;
  final DemoSettings? settings;

  @override
  _BasicPhysicsState createState() => _BasicPhysicsState();
}

class _BasicPhysicsState extends State<BasicPhysics> {
  late Demo demo;
  Map<String,three.Material> mats = {};

  @override
  void initState() {
    demo = Demo(
      onSetupComplete: (){setState(() {});},
      updatePhysics: () => updateCannonPhysics(),
      settings: DemoSettings(
        tolerance: 0.1,
        gx: 0,
        gy: -200,
        gz: 0,
        iterations: 20,
        maxSubSteps: 1
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

  void populate(n) {
    final world = demo.world;
    int max = 200;
    int type = n;

    cannon.Body b1 = cannon.Body(
      shape: cannon.Box(vmath.Vector3(40.0/2, 40.0/2, 390.0/2)),
      position:vmath.Vector3(-180.0,20.0,0.0), 
    );
    world.addBody(b1);
    demo.addVisual(b1,material: mats['ground']);
    cannon.Body b2 = cannon.Body(
      shape: cannon.Box(vmath.Vector3(40.0/2, 40.0/2, 390.0/2)),
      position:vmath.Vector3(180.0,20.0,0.0), 
    );
    world.addBody(b2);
    demo.addVisual(b2,material: mats['ground']);
    cannon.Body b3 = cannon.Body(
      shape: cannon.Box(vmath.Vector3(400.0/2, 80.0/2, 400.0/2)),
      position:vmath.Vector3(0.0,-40.0,0.0), 
    );
    world.addBody(b3);
    demo.addVisual(b3,material: mats['ground']);

    //add object
    double x, y, z, w, h, d;
    int t;
    for(int i = 0; i < max;i++){
      if(type==4) {
        t = Math.floor(Math.random()*3)+1;
      }
      else {
        t = type;
      }
      x = -100 + Math.random()*200;
      z = -100 + Math.random()*200;
      y = 100 + Math.random()*1000;
      w = 10 + Math.random()*10;
      h = 10 + Math.random()*10;
      d = 10 + Math.random()*10;
      three.Color randColor = three.Color().setHex((Math.random() * 0xFFFFFF).toInt());

      if(t==1){
        three.Material mat = mats['sph']!;
        mat.color = randColor;
        cannon.Body sbody = cannon.Body(
          shape: cannon.Sphere(w*0.5),
          position:vmath.Vector3(x,y,z),
          mass: 1
        );
        world.addBody(sbody);
        demo.addVisual(sbody,material: mat);
      } 
      else if(t==2){
        three.Material mat = mats['box']!;
        mat.color = randColor;
        cannon.Body sbody = cannon.Body(
          shape: cannon.Box(vmath.Vector3(w/2,h/2,d/2)),
          position:vmath.Vector3(x,y,z),
          mass: 1
        );
        world.addBody(sbody);
        demo.addVisual(sbody,material: mat);
      } 
      else if(t==3){
        three.Material mat = mats['cyl']!;
        mat.color = randColor;
        cannon.Body sbody = cannon.Body(
          shape: cannon.Cylinder(radiusTop:w*0.5,radiusBottom:w*0.5,height: h),
          position:vmath.Vector3(x,y,z),
          mass: 1
        );
        world.addBody(sbody);
        demo.addVisual(sbody,material: mat);
      }
    }
  }

  void updateCannonPhysics() {
    demo.world.fixedStep();

    double x, y, z;
    Object3D mesh; 
    cannon.Body body;
    //print(bodys[0].getPosition());
    for(int i = 0; i < demo.bodies.length;i++){
      body = demo.bodies[i];
      mesh = demo.visuals[i];

      if(body.sleepState != cannon.BodySleepStates.sleeping){
        
        mesh.position.copy(body.position.toVector3());
        mesh.quaternion.copy(body.quaternion.toQuaternion());

        // change material
        if(mesh.material.name == 'sbox') mesh.material = mats['box'];
        if(mesh.material.name == 'ssph') mesh.material = mats['sph'];
        if(mesh.material.name == 'scyl') mesh.material = mats['cyl']; 

        // reset position
        if(mesh.position.y<-100){
          x = -100 + Math.random()*200;
          z = -100 + Math.random()*200;
          y = 100 + Math.random()*1000;
          body.position = vmath.Vector3(x,y,z);
        }
      } 
      else {
        if(mesh.material.name == 'box') mesh.material = mats['sbox'];
        if(mesh.material.name == 'sph') mesh.material = mats['ssph'];
        if(mesh.material.name == 'cyl') mesh.material = mats['scyl'];
      }
    }
  }
  void setupWorld(){
    final world = demo.world;
    world.broadphase = cannon.NaiveBroadphase();

    mats['sph']    = MeshPhongMaterial({'shininess': 10, 'name':'sph'});
    mats['box']    = MeshPhongMaterial({'shininess': 10, 'name':'box'});
    mats['cyl']    = MeshPhongMaterial({'shininess': 10, 'name':'cyl'});
    mats['ssph']   = MeshPhongMaterial({'shininess': 10, 'name':'ssph'});
    mats['sbox']   = MeshPhongMaterial({'shininess': 10, 'name':'sbox'});
    mats['scyl']   = MeshPhongMaterial({'shininess': 10, 'name':'scyl'});
    mats['ground'] = MeshPhongMaterial({'shininess': 10, 'color':0x3D4143, 'transparent':true, 'opacity':0.5});

    populate(4);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: demo.threeDart(),
    );
  }
}