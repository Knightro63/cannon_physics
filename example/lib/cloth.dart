import 'dart:async';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/foundation.dart';

import 'package:flutter_gl/flutter_gl.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:three_dart/three_dart.dart' as three;
import 'package:three_dart/three_dart.dart' hide Texture, Color;
import 'package:three_dart_jsm/three_dart_jsm.dart';

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
  FocusNode node = FocusNode();
  // gl values
  //late Object3D object;
  bool animationReady = false;
  late FlutterGlPlugin three3dRender;
  WebGLRenderTarget? renderTarget;
  WebGLRenderer? renderer;
  late OrbitControls controls;
  int? fboId;
  late double width;
  late double height;
  Size? screenSize;
  late Scene scene;
  late Camera camera;
  double dpr = 1.0;
  bool verbose = false;
  bool disposed = false;
  final GlobalKey<DomLikeListenableState> _globalKey = GlobalKey<DomLikeListenableState>();
  dynamic sourceTexture;

  late three.BufferGeometry clothGeometry;
  late three.Mesh sphereMesh;

  //cannon var
  late cannon.World world;
  List<cannon.Body> bodys = [];
  late cannon.Body sphereBody;
  double clothMass = 1; // 1 kg in total
  double clothSize = 1; // 1 meter
  int Nx = 12; // number of horizontal particles in the cloth
  int Ny = 12; // number of vertical particles in the cloth
  late double mass;
  late double restDistance;
  double sphereSize = 0.1;
  double movementRadius = 0.2;

  List<List<cannon.Body>> particles = [];

  @override
  void initState() {
    mass = (clothMass / Nx) * Ny;
    restDistance = clothSize / Nx;
    super.initState();
  }
  @override
  void dispose() {
    disposed = true;
    three3dRender.dispose();
    super.dispose();
  }
  void initScene() async{
    await initThree();
    initRenderer();
    initCannonPhysics();
    animate();
  }
  void initSize(BuildContext context) {
    if (screenSize != null) {
      return;
    }

    final mqd = MediaQuery.of(context);

    screenSize = mqd.size;
    dpr = mqd.devicePixelRatio;

    initPlatformState();
  }

  Future<void> initThree() async {
    scene = Scene();

    camera = PerspectiveCamera(30, width / height, 0.5, 10000);
    camera.position.set(Math.cos( Math.PI/4 ) * 3,
                                    0,
                                    Math.sin( Math.PI/4 ) * 3);
    camera.rotation.order = 'YXZ';

    controls = OrbitControls(camera, _globalKey);
    //controls.target.set(0,20,0);
    //controls.update();
    
    scene.add(AmbientLight( 0x3D4143 ) );
    DirectionalLight light = DirectionalLight( 0xffffff , 1.4);
    light.position.set( 300, 1000, 500 );
    light.target!.position.set( 0, 0, 0 );
    light.castShadow = true;

    int d = 300;
    light.shadow!.camera = OrthographicCamera( -d, d, d, -d,  500, 1600 );
    light.shadow!.bias = 0.0001;
    light.shadow!.mapSize.width = light.shadow!.mapSize.height = 1024;

    scene.add( light );

    // Cloth material
    three.Texture clothTexture = await three.TextureLoader(null).loadAsync('assets/images/sunflower.jpg');
    clothTexture.wrapS = three.RepeatWrapping;
    clothTexture.wrapT = three.RepeatWrapping;
    clothTexture.anisotropy = 16;
    clothTexture.encoding = three.sRGBEncoding;

    three.MeshPhongMaterial clothMaterial = three.MeshPhongMaterial({
      'map': clothTexture,
      'side': three.DoubleSide,
    });
    // Cloth geometry
    clothGeometry = three.ParametricGeometry(clothFunction, Nx, Ny);

    // Cloth mesh
    three.Mesh clothMesh = three.Mesh(clothGeometry, clothMaterial);
    //clothMesh.position.set(0,-1,0);
    scene.add(clothMesh);

    // Sphere
    three.SphereGeometry sphereGeometry = three.SphereGeometry(sphereSize, 20, 20);
    three.MeshPhongMaterial sphereMaterial = three.MeshPhongMaterial({'color': 0x888888 });

    sphereMesh = three.Mesh(sphereGeometry, sphereMaterial);
    scene.add(sphereMesh);

    animationReady = true;
  }

  //----------------------------------
  //  cannon PHYSICS
  //----------------------------------

  void initCannonPhysics(){
    world = cannon.World();
    world.gravity.set(0, -9.81, 0);

    // Max solver iterations: Use more for better force propagation, but keep in mind that it's not very computationally cheap!
    world.solver.iterations = 10;
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
    clothSphere.contactEquationRelaxation = 3;
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
    world.addBody(sphereBody);

    // Create cannon particles
    for (int i = 0; i < Nx + 1; i++) {
      particles.add([]);
      for (int j = 0; j < Ny + 1; j++) {
        int index = j * (Nx + 1) + i;

        three.Vector3 point = clothFunction(i / (Nx + 1), j / (Ny + 1), three.Vector3());
        cannon.Body particle = cannon.Body(
          // Fix in place the first row
          mass: j == Ny ? 0 : mass,
        );
        particle.addShape(cannon.Particle());
        particle.linearDamping = 1;
        particle.position.set(point.x, point.y - Ny * 0.9 * restDistance, point.z);
        particle.velocity.set(0, 0, -0.1 * (Ny - j));

        particles[i].add(particle);
        world.addBody(particle);
      }
    }

    // Connect the particles with distance constraints
    void connect(int i1,int j1,int i2,int j2) {
      world.addConstraint(cannon.DistanceConstraint(particles[i1][j1], particles[i2][j2], restDistance));
    }
    for (int i = 0; i < Nx + 1; i++) {
      for (int j = 0; j < Ny + 1; j++) {
        if (i < Nx) connect(i, j, i + 1, j);
        if (j < Ny) connect(i, j, i, j + 1);
      }
    }
  }
  // Parametric function
  // https://threejs.org/docs/index.html#api/en/geometries/ParametricGeometry
  three.Vector3 clothFunction(double u, double v, three.Vector3 target) {
    double x = (u - 0.5) * restDistance * Nx;
    double y = (v + 0.5) * restDistance * Ny;
    double z = 0;

    target.set(x, y, z);

    return target;
  }

  void updateCannonPhysics() {
    // Make the three.js cloth follow the cannon.js particles
    for (int i = 0; i < Nx + 1; i++) {
      for (int j = 0; j < Ny + 1; j++) {
        int index = j * (Nx + 1) + i;
        var v = particles[i][j].position;
        clothGeometry.attributes["position"].setXYZ(index, v.x, v.y, v.z);
      }
    }
    clothGeometry.attributes["position"].needsUpdate = true;

    clothGeometry.computeVertexNormals();
    clothGeometry.normalsNeedUpdate = true;
    clothGeometry.verticesNeedUpdate = true;

    // Move the ball in a circular motion
    double time = world.time;
    sphereBody.position.set(movementRadius * Math.sin(time), 0, movementRadius * Math.cos(time));

    // Make the three.js ball follow the cannon.js one
    // Copying quaternion is not needed since it's a sphere
    sphereMesh.position.copy(sphereBody.position);
  }
  void animate() {
    if (!mounted || disposed) {
      return;
    }

    render();

    Future.delayed(const Duration(milliseconds: 16), () {
      updateCannonPhysics();
      animate();
    });
  }
  void render() {
    final _gl = three3dRender.gl;
    renderer!.render(scene, camera);
    _gl.flush();
    world.fixedStep();
    controls.update();
    if(!kIsWeb) {
      three3dRender.updateTexture(sourceTexture);
    }
  }
  void initRenderer() {
    Map<String, dynamic> _options = {
      "width": width,
      "height": height,
      "gl": three3dRender.gl,
      "antialias": true,
      "canvas": three3dRender.element,
    };

    if(!kIsWeb && Platform.isAndroid){
      _options['logarithmicDepthBuffer'] = true;
    }

    renderer = WebGLRenderer(_options);
    renderer!.setPixelRatio(dpr);
    renderer!.setSize(width, height, false);
    renderer!.shadowMap.enabled = true;
    renderer!.shadowMap.type = three.PCFShadowMap;
    //renderer!.outputEncoding = three.sRGBEncoding;

    if(!kIsWeb){
      WebGLRenderTargetOptions pars = WebGLRenderTargetOptions({"format": RGBAFormat,"samples": 8});
      renderTarget = WebGLRenderTarget((width * dpr).toInt(), (height * dpr).toInt(), pars);
      renderer!.setRenderTarget(renderTarget);
      sourceTexture = renderer!.getRenderTargetGLTexture(renderTarget!);
    }
    else{
      renderTarget = null;
    }
  }

  Future<void> initPlatformState() async {
    width = screenSize!.width;
    height = screenSize!.height;

    three3dRender = FlutterGlPlugin();

    Map<String, dynamic> _options = {
      "antialias": true,
      "alpha": true,
      "width": width.toInt(),
      "height": height.toInt(),
      "dpr": dpr,
      'precision': 'highp'
    };
    await three3dRender.initialize(options: _options);

    setState(() {});

    // TODO web wait dom ok!!!
    Future.delayed(const Duration(milliseconds: 100), () async {
      await three3dRender.prepareContext();
      initScene();
    });
  }

  Widget threeDart() {
    return Builder(builder: (BuildContext context) {
      initSize(context);
      return Container(
        width: screenSize!.width,
        height: screenSize!.height,
        color: Theme.of(context).canvasColor,
        child: DomLikeListenable(
          key: _globalKey,
          builder: (BuildContext context) {
            FocusScope.of(context).requestFocus(node);
            return Container(
              width: width,
              height: height,
              color: Theme.of(context).canvasColor,
              child: Builder(builder: (BuildContext context) {
                if (kIsWeb) {
                  return three3dRender.isInitialized
                      ? HtmlElementView(
                          viewType:
                              three3dRender.textureId!.toString())
                      : Container();
                } else {
                  return three3dRender.isInitialized
                      ? Texture(textureId: three3dRender.textureId!)
                      : Container();
                }
              })
            );
          }
        ),
      );
    });
  }

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      height: double.infinity,
      width: double.infinity,
      child: Stack(
        children: [
          threeDart(),
        ],
      )
    );
  }
}