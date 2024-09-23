import 'dart:async';

import 'package:flutter/material.dart';
import 'dart:math' as math;

import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:three_js/three_js.dart' as three;
import 'package:three_js_geometry/three_js_geometry.dart';
import 'package:vector_math/vector_math.dart' as vmath;
import '../src/conversion_utils.dart';

class Cloth extends StatefulWidget {
  const Cloth({
    Key? key,
    this.offset = const Offset(0,0)
  }) : super(key: key);

  final Offset offset;

  @override
  _ClothPageState createState() => _ClothPageState();
}

class _ClothPageState extends State<Cloth> {
  late three.ThreeJS threeJs;
  late three.OrbitControls controls;
  late three.BufferGeometry clothGeometry;
  late three.Mesh sphereMesh;

  //cannon var
  late cannon.World world;
  List<cannon.Body> bodys = [];
  late cannon.Body sphereBody;
  double clothMass = 1; // 1 kg in total
  double clothSize = 1; // 1 meter
  int cols = 12; // number of horizontal particles in the cloth
  int rows = 12; // number of vertical particles in the cloth
  late double mass;
  late double restDistance;
  double sphereSize = 0.1;
  double movementRadius = 0.2;

  List<List<cannon.Body>> particles = [];

  @override
  void initState() {
    threeJs = three.ThreeJS(
      onSetupComplete: (){setState(() {});},
      setup: setup
    );
    mass = (clothMass / cols) * rows;
    restDistance = clothSize / cols;
    super.initState();
  }
  @override
  void dispose() {
    controls.dispose();
    threeJs.dispose();
    super.dispose();
  }

  Future<void> setup() async {
    threeJs.scene = three.Scene();

    threeJs.camera = three.PerspectiveCamera(30, threeJs.width / threeJs.height, 0.5, 10000);
    threeJs.camera.position.setValues(math.cos(math.pi/4) * 3,0,math.sin(math.pi/4) * 3);
    //camera.rotation.order = 'YXZ';

    controls = three.OrbitControls(threeJs.camera, threeJs.globalKey);
    //controls.target.set(0,20,0);
    //controls.update();
    
    threeJs.scene.add(three.AmbientLight( 0x3D4143 ) );
    three.DirectionalLight light = three.DirectionalLight( 0xffffff , 1.4);
    light.position.setValues( 300, 1000, 500 );
    light.target!.position.setValues( 0, 0, 0 );
    // light.castShadow = true;

    // double d = 300;
    // light.shadow!.camera =three.OrthographicCamera( -d, d, d, -d,  500, 1600 );
    // light.shadow!.bias = 0.0001;
    // light.shadow!.mapSize.width = light.shadow!.mapSize.height = 1024;

    threeJs.scene.add( light );

    // Cloth material
    three.Texture? clothTexture = await three.TextureLoader().fromAsset('assets/images/sunflower.jpg');
    clothTexture?.wrapS = three.RepeatWrapping;
    clothTexture?.wrapT = three.RepeatWrapping;
    clothTexture?.anisotropy = 16;
    clothTexture?.encoding = three.sRGBEncoding;

    three.MeshPhongMaterial clothMaterial = three.MeshPhongMaterial.fromMap({
      'map': clothTexture,
      'side': three.DoubleSide,
    });
    // Cloth geometry
    clothGeometry = ParametricGeometry(clothFunction, cols, rows);

    // Cloth mesh
    three.Mesh clothMesh = three.Mesh(clothGeometry, clothMaterial);
    //clothMesh.position.set(0,-1,0);
    threeJs.scene.add(clothMesh);
 
    // Sphere
    three.SphereGeometry sphereGeometry = three.SphereGeometry(sphereSize);
    three.MeshPhongMaterial sphereMaterial = three.MeshPhongMaterial.fromMap({'color': 0x888888 });

    sphereMesh = three.Mesh(sphereGeometry, sphereMaterial);
    threeJs.scene.add(sphereMesh);

    initCannonPhysics();
    threeJs.addAnimationEvent((dt){
      world.fixedStep();
      updateCannonPhysics();
      controls.update();
    });
  }

  //----------------------------------
  //  cannon PHYSICS
  //----------------------------------

  void initCannonPhysics(){
    world = cannon.World(
      allowSleep: false
    );
    world.gravity.setValues(0, -9.81, 0);

    // Max solver iterations: Use more for better force propagation, but keep in mind that it's not very computationally cheap!
    world.solver.iterations = 20;

    // Materials
    cannon.Material clothMaterial = cannon.Material(name: 'cloth');
    cannon.Material sphereMaterial = cannon.Material(name: 'sphere');
    cannon.ContactMaterial clothSphere = cannon.ContactMaterial(
      clothMaterial, 
      sphereMaterial,
      friction: 0,
      restitution: 0,
    );

    // Adjust constraint equation parameters
    // Contact stiffness - use to make softer/harder contacts
    clothSphere.contactEquationStiffness = 1e9;
    // Stabilization time in number of timesteps
    clothSphere.contactEquationRelaxation = 4;
    // Add contact material to the world
    world.addContactMaterial(clothSphere);

    // Create sphere
    // Make it a little bigger than the three.js sphere
    // so the cloth doesn't clip thruogh
    cannon.Sphere sphereShape = cannon.Sphere(sphereSize * 1.3);
    sphereBody = cannon.Body(
      //type: cannon.BodyTypes.kinematic,
      mass: 0
    );

    sphereBody.addShape(sphereShape);
    sphereMesh.position.setFrom(sphereBody.shapeOffsets[0].toVector3());
    sphereMesh.quaternion.setFrom(sphereBody.shapeOrientations[0].toQuaternion());
    world.addBody(sphereBody);

    // Create cannon particles
    for (int i = 0; i < cols + 1; i++) {
      particles.add([]);
      for (int j = 0; j < rows + 1; j++) {
        //int index = j * (cols + 1) + i;

        late final three.Vector3 point = three.Vector3();
        clothFunction(i / (cols + 1), j / (rows + 1), point);
        cannon.Body particle = cannon.Body(
          mass: j == rows ? 0 : mass,
        );
        particle.addShape(cannon.Particle());
        particle.linearDamping = 0.5;
        particle.position.setValues(point.x, point.y - rows * 0.9 * restDistance, point.z);
        particle.velocity.setValues(0, 0, -0.1 * (rows - j));

        particles[i].add(particle);
        world.addBody(particle);
      }
    }

    // Connect the particles with distance constraints
    void connect(int i1,int j1,int i2,int j2) {
      world.addConstraint(cannon.DistanceConstraint(particles[i1][j1], particles[i2][j2], restDistance));
    }
    for (int i = 0; i < cols + 1; i++) {
      for (int j = 0; j < rows + 1; j++) {
        if (i < cols) connect(i, j, i + 1, j);
        if (j < rows) connect(i, j, i, j + 1);
      }
    }
  }
  // Parametric function
  // https://threejs.org/docs/index.html#api/en/geometries/ParametricGeometry
  three.Vector3 clothFunction(double u, double v, three.Vector3 target) {
    double x = (u - 0.5) * restDistance * cols;
    double y = (v + 0.5) * restDistance * rows;
    double z = 0;

    target.setValues(x, y, z);

    return target;
  }

  void updateCannonPhysics() {
    // Make the three.js cloth follow the cannon.js particles
    for (int i = 0; i < cols + 1; i++) {
      for (int j = 0; j < rows + 1; j++) {
        int index = j * (cols + 1) + i;
        vmath.Vector3 v = particles[i][j].position;
        clothGeometry.attributes["position"].setXYZ(index, v.x, v.y, v.z);
      }
    }
    clothGeometry.attributes["position"].needsUpdate = true;

    clothGeometry.computeVertexNormals();
    clothGeometry.normalsNeedUpdate = true;
    clothGeometry.verticesNeedUpdate = true;

    // Move the ball in a circular motion
    double time = world.time;
    sphereBody.position.setValues(movementRadius * math.sin(time), 0, movementRadius * math.cos(time));

    // Make the three.js ball follow the cannon.js one
    // Copying quaternion is not needed since it's a sphere
    sphereMesh.position.setFrom(sphereBody.position.toVector3());
    sphereMesh.quaternion.setFrom(sphereBody.quaternion.toQuaternion());
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