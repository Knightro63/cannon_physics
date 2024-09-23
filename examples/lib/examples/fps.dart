import 'dart:async';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:three_js/three_js.dart' as three;
import 'package:vector_math/vector_math.dart' as vmath;
import '../src/conversion_utils.dart';

class FPSGame extends StatefulWidget {
  const FPSGame({
    Key? key,
    this.offset = const Offset(0,0)
  }) : super(key: key);

  final Offset offset;

  @override
  _FPSGamePageState createState() => _FPSGamePageState();
}

class _FPSGamePageState extends State<FPSGame> {
  late three.ThreeJS threeJs;
  late three.BufferGeometry clothGeometry;
  late three.Mesh floorGeometry;

  //cannon var
  late cannon.World world;
  late cannon.Shape sphereShape;
  late cannon.Body sphereBody;
  late cannon.Material physicsMaterial;
  late three.MeshLambertMaterial material;
  late double lastCallTime;
  List<cannon.Body> balls = [];
  List<three.Mesh> ballMeshes = [];
  List<cannon.Body> boxes = [];
  List<three.Mesh> boxMeshes = [];

  int mouseTime = 0;

  @override
  void initState() {
    threeJs = three.ThreeJS(
      onSetupComplete: (){setState(() {});},
      setup: setup
    );
    super.initState();
  }
  @override
  void dispose() {
    threeJs.dispose();
    super.dispose();
  }

  Future<void> setup() async {
    threeJs.scene = three.Scene();

    threeJs.camera = three.PerspectiveCamera(95, threeJs.width / threeJs.height, 0.1, 10000);
    threeJs.scene.add(three.AmbientLight( 0x111111 ) );
    
    three.DirectionalLight light = three.DirectionalLight( 0xffffff,1.4);
    light.position.setValues( 300, 1000, 500 );
    light.target!.position.setValues( 0, 0, 0 );
    // light.castShadow = true;

    // double d = 300;
    // light.shadow!.camera = three.OrthographicCamera( -d, d, d, -d,  500, 1600 );
    // light.shadow!.bias = 0.0001;
    // light.shadow!.mapSize.width = light.shadow!.mapSize.height = 1024;

    threeJs.scene.add( light );

    // floor
    three.PlaneGeometry floorGeometry = three.PlaneGeometry(300, 300, 50, 50);
    floorGeometry.applyMatrix4(three.Matrix4().makeRotationX( - math.pi / 2 ) );
    material = three.MeshLambertMaterial.fromMap({ 'color': 0xdddddd });
    three.Mesh floor = three.Mesh(floorGeometry, material);
    floor.receiveShadow = true;
    floor.castShadow = true;
    threeJs.scene.add(floor);

    initCannonPhysics();
    threeJs.addAnimationEvent((dt){
      world.step(1/60,);

      // Update ball positions
      for (int i = 0; i < balls.length; i++) {
        ballMeshes[i].position.setFrom(balls[i].position.toVector3());
        ballMeshes[i].quaternion.setFrom(balls[i].quaternion.toQuaternion());
      }

      // Update box positions
      for (int i = 0; i < boxes.length; i++) {
        boxMeshes[i].position.setFrom(boxes[i].position.toVector3());
        boxMeshes[i].quaternion.setFrom(boxes[i].quaternion.toQuaternion());
      }
    });
  }

  //----------------------------------
  //  cannon PHYSICS
  //----------------------------------
  void initCannonPhysics(){
    world = cannon.World();
    world.quatNormalizeSkip = 0;
    world.quatNormalizeFast = false;

    cannon.GSSolver solver = cannon.GSSolver();

    lastCallTime = world.performance.now().toDouble();
    world.defaultContactMaterial.contactEquationStiffness = 1e9;
    world.defaultContactMaterial.contactEquationRelaxation = 4;

    solver.iterations = 7;
    solver.tolerance = 0.1;
    world.solver = cannon.SplitSolver(solver);

    world.gravity.setValues(0, -20, 0);
    world.broadphase = cannon.NaiveBroadphase();

    // Create a slippery material (friction coefficient = 0.0)
    physicsMaterial = cannon.Material(name:'slipperyMaterial');
    cannon.ContactMaterial physics_physics = cannon.ContactMaterial(
      physicsMaterial, 
      physicsMaterial,
      friction: 0.0,
      restitution: 0.3,
    );

    // We must add the contact materials to the world
    world.addContactMaterial(physics_physics);

    // Create the user collision sphere
    const double mass = 5;
    const radius = 1.3;
    sphereShape = cannon.Sphere(radius);
    sphereBody = cannon.Body(mass: mass, material: physicsMaterial);
    sphereBody.addShape(sphereShape);
    sphereBody.position.setValues(0, 5, 0);
    sphereBody.linearDamping = 0.9;
    world.addBody(sphereBody);

    // Create the ground plane
    cannon.Plane groundShape = cannon.Plane();
    cannon.Body groundBody = cannon.Body(mass: 0);
    groundBody.addShape(groundShape);
    groundBody.quaternion.setFromAxisAngle(vmath.Vector3(1,0,0),(-math.pi / 2));
    world.addBody(groundBody);

    // Add boxes both in cannon.js and three.js
    vmath.Vector3 halfExtents = vmath.Vector3(1, 1, 1);
    cannon.Box boxShape = cannon.Box(halfExtents);
    three.BoxGeometry boxGeometry = three.BoxGeometry(halfExtents.x * 2, halfExtents.y * 2, halfExtents.z * 2);

    for (int i = 0; i < 7; i++) {
      cannon.Body boxBody = cannon.Body(mass: 5);
      boxBody.addShape(boxShape);
      three.Mesh boxMesh = three.Mesh(boxGeometry, material);

      double x = (math.Random().nextDouble() - 0.5) * 20;
      double y = (math.Random().nextDouble() - 0.5) * 1 + 1;
      double z = (math.Random().nextDouble() - 0.5) * 20;

      boxBody.position.setValues(x, y, z);
      boxMesh.position.setFrom(boxBody.position.toVector3());

      boxMesh.castShadow = true;
      boxMesh.receiveShadow = true;

      world.addBody(boxBody);
      threeJs.scene.add(boxMesh);
      boxes.add(boxBody);
      boxMeshes.add(boxMesh);
    }

    addBoxes();
  }
  void addBoxes(){
    // Add linked boxes
    var halfExtents = vmath.Vector3(1,1,1);
    var boxShape = cannon.Box(halfExtents);
    var boxGeometry = three.BoxGeometry(halfExtents.x*2,halfExtents.y*2,halfExtents.z*2);
    for(var i=0; i<7; i++){
      final double x = (math.Random().nextDouble()-0.5)*20;
      final double y = 1 + (math.Random().nextDouble()-0.5)*1;
      final double z = (math.Random().nextDouble()-0.5)*20;
      final boxBody = cannon.Body(mass: 5 );
      boxBody.addShape(boxShape);
      final boxMesh = three.Mesh( boxGeometry, material );
      world.addBody(boxBody);
      threeJs.scene.add(boxMesh);
      boxBody.position.setValues(x,y,z);
      boxMesh.position.setValues(x,y,z);
      boxMesh.castShadow = true;
      boxMesh.receiveShadow = true;
      boxes.add(boxBody);
      boxMeshes.add(boxMesh);
    }


    // Add linked boxes
    const size = 0.5;
    vmath.Vector3 he = vmath.Vector3(size,size,size*0.1);
    cannon.Box boxShape2 = cannon.Box(he);
    double mass = 0;
    const double space = 0.1 * size;
    var N = 5;
    late cannon.Body last;
    three.BoxGeometry boxGeometry2 = three.BoxGeometry(he.x*2,he.y*2,he.z*2);
    for(var i=0; i<N; i++){
      var boxbody = cannon.Body( mass: mass );
      boxbody.addShape(boxShape2);
      var boxMesh = three.Mesh(boxGeometry2, material);
      boxbody.position.setValues(5,(N-i)*(size*2+2*space) + size*2+space,0);
      boxbody.linearDamping = 0.01;
      boxbody.angularDamping = 0.01;
      // boxMesh.castShadow = true;
      boxMesh.receiveShadow = true;
      world.addBody(boxbody);
      threeJs.scene.add(boxMesh);
      boxes.add(boxbody);
      boxMeshes.add(boxMesh);

      if(i!=0){
        // Connect this body to the last one
        cannon.PointToPointConstraint c1 = cannon.PointToPointConstraint(
          boxbody,
          last,
          vmath.Vector3(-size,size+space,0),
          vmath.Vector3(-size,-size-space,0)
        );
        cannon.PointToPointConstraint c2 = cannon.PointToPointConstraint(
          boxbody,
          last,
          vmath.Vector3(size,size+space,0),
          vmath.Vector3(size,-size-space,0)
        );
        world.addConstraint(c1);
        world.addConstraint(c2);
      } else {
        mass=0.3;
      }
      last = boxbody;
    }
  }
  void throwBall() {
    const shootVelocity = 15;
    cannon.Sphere ballShape = cannon.Sphere(0.2);
    three.SphereGeometry ballGeometry = three.SphereGeometry(ballShape.radius, 32, 32);

    three.Vector3 getShootDirection() {
      three.Vector3 vector = three.Vector3(0, 0, 1);
      vector.unproject(threeJs.camera);
      three.Ray ray = three.Ray.originDirection(sphereBody.position.toVector3(), vector.sub(sphereBody.position.toVector3()).normalize());
      return ray.direction;
    }

    cannon.Body ballBody = cannon.Body(mass: 1);
    ballBody.addShape(ballShape);
    three.Mesh ballMesh = three.Mesh(ballGeometry, material);

    ballMesh.castShadow = true;
    ballMesh.receiveShadow = true;

    world.addBody(ballBody);
    threeJs.scene.add(ballMesh);
    balls.add(ballBody);
    ballMeshes.add(ballMesh);

    three.Vector3 shootDirection = getShootDirection();
    ballBody.velocity.setValues(
      shootDirection.x * shootVelocity,
      shootDirection.y * shootVelocity,
      shootDirection.z * shootVelocity
    );
    const radius = 1.3;
    // Move the ball outside the player sphere
    double x = sphereBody.position.x + shootDirection.x * (radius * 1.02 + ballShape.radius);
    double y = sphereBody.position.y + shootDirection.y * (radius * 1.02 + ballShape.radius);
    double z = sphereBody.position.z + shootDirection.z * (radius * 1.02 + ballShape.radius);
    ballBody.position.setValues(x, y, z);
    ballMesh.position.setFrom(ballBody.position.toVector3());
  }

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      height: double.infinity,
      width: double.infinity,
      child: Stack(
        children: [
          threeJs.build(),
        ],
      )
    );
  }
}