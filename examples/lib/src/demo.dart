import 'dart:async';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/foundation.dart';

import 'package:flutter_gl/flutter_gl.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:three_dart/three_dart.dart' as three;
import 'package:three_dart/three_dart.dart' hide Texture, Color;
import 'package:three_dart_jsm/three_dart_jsm.dart';
import 'conversion_utils.dart';
import 'package:flutter/services.dart';
import 'package:vector_math/vector_math.dart' as vmath;

enum RenderMode{solid,wireframe}

class DemoSettings{
  DemoSettings({
    this.stepFrequency = 60,
    this.quatNormalizeSkip = 2,
    this.quatNormalizeFast = true,
    this.gx = 0,
    this.gy = 0,
    this.gz = 0,
    this.iterations = 3,
    this.tolerance = 0.0001,
    this.k = 1e6,
    this.d = 3,
    this.scene= 0,
    this.paused= false,
    this.rendermode= RenderMode.solid,
    
    this.constraints= false,
    this.contacts= false, // Contact points
    this.cm2contact= false, // center of mass to contact points
    this.normals= false, // contact normals
    this.axes= false, // "local" frame axes
    this.shadows= false,
    this.aabbs= false,
    this.profiling= false,
    this.maxSubSteps = 20,
  });
  int stepFrequency;
  int quatNormalizeSkip;
  bool quatNormalizeFast;
  double gx;
  double gy;
  double gz;
  int iterations;
  double tolerance;
  double k;
  double d;
  int scene;
  bool paused;
  RenderMode rendermode;
  bool constraints;
  bool contacts;
  bool cm2contact;
  bool normals;
  bool axes;
  bool shadows;
  bool aabbs;
  bool profiling;
  int maxSubSteps;
}

/**
 * Demo utility class. If you want to learn how to connect cannon.js with three.js, please look at the examples/threejs_* instead.
 */
class Demo{
  void Function() onSetupComplete;
  Demo({required this.onSetupComplete,DemoSettings? settings, void Function()? updatePhysics}){
    this.settings = settings ?? DemoSettings();

    world = cannon.World();
    this.updatePhysics = updatePhysics ?? () => _updateCannonPhysics();
    

    world.gravity = vmath.Vector3(this.settings.gx,this.settings.gy,this.settings.gz);
    world.quatNormalizeSkip = this.settings.quatNormalizeSkip;
    world.quatNormalizeFast = this.settings.quatNormalizeFast;

    cannon.GSSolver solver = cannon.GSSolver();

    lastCallTime = world.performance.now().toDouble();
    world.defaultContactMaterial.contactEquationStiffness = this.settings.k;
    world.defaultContactMaterial.contactEquationRelaxation = this.settings.d;

    solver.iterations = this.settings.iterations;
    solver.tolerance = this.settings.tolerance;

    world.solver = solver;
    world.broadphase = cannon.NaiveBroadphase();

    if(this.settings.rendermode == RenderMode.solid){
      _currentMaterial = _solidMaterial;
    }
    else{
      _currentMaterial = _wireframeMaterial;
    }

    initGeometryCaches();
  }

  final GlobalKey<DomLikeListenableState> globalKey = GlobalKey<DomLikeListenableState>();
  DomLikeListenableState get domElement => globalKey.currentState!;

  Map<String,Function> domElements = {};

  late cannon.World world;
  List<cannon.Body> bodies = [];
  List<three.Object3D> visuals = [];
  late DemoSettings settings;

  bool animationReady = false;
  late FlutterGlPlugin three3dRender;
  WebGLRenderTarget? renderTarget;
  WebGLRenderer? renderer;
  late OrbitControls controls;

  late double width;
  late double height;
  Size? screenSize;
  Scene scene = Scene();
  late Camera camera;
  double dpr = 1.0;
  bool verbose = false;
  bool disposed = false;
  
  dynamic sourceTexture;
  
  double? lastCallTime;
  bool resetCallTime = false;
  double toRad = 0.0174532925199432957;

  late SpotLight _spotLight;
  late AmbientLight _ambientLight;

  Map<LogicalKeyboardKey,bool> keyStates = {
    LogicalKeyboardKey.keyW: false,
    LogicalKeyboardKey.keyA: false,
    LogicalKeyboardKey.keyS: false,
    LogicalKeyboardKey.keyD: false,
    LogicalKeyboardKey.space: false,

    LogicalKeyboardKey.arrowUp: false,
    LogicalKeyboardKey.arrowLeft: false,
    LogicalKeyboardKey.arrowDown: false,
    LogicalKeyboardKey.arrowRight: false,
  };
  int mouseTime = 0;
  bool mounted = false;

  MeshBasicMaterial _wireframeMaterial = MeshBasicMaterial({'color': 0xffffff, 'wireframe': true});
  MeshLambertMaterial _solidMaterial = MeshLambertMaterial({'color': 0xdddddd});
  late three.Material _currentMaterial;

  late void Function() updatePhysics;

  bool get pause => settings.paused;
  cannon.World get getWorld => world;
  set pause(bool p){
    settings.paused = p;
    resetCallTime = true;
  }

  late GeometryCache bboxMeshCache;
  late GeometryCache contactMeshCache;
  late GeometryCache cm2contactMeshCache;
  late GeometryCache distanceConstraintMeshCache;
  late GeometryCache normalMeshCache;
  late GeometryCache axesMeshCache;
  late GeometryCache p2pConstraintMeshCache;

  List<Function(double dt)> events = [];
  Map<String,dynamic> scenes = {};

  void dispose(){
    disposed = true;
    // renderTarget?.dispose();
    // renderer?.dispose();
    scene.dispose();
    three3dRender.dispose();
  }
  // void addEventListener(String type,Function(dynamic) listener){
  //   world.addEventListener('postStep', listener);
  // }
  void addAnimationEvent(Function(double dt) event){
    events.add(event);
  }
  void addDomListener(String type, Function function){
    domElements[type] = function;
  }
  void addScene(String name,newScene){
    scenes[name] = newScene;
  }
  void initGeometryCaches(){
    // Material
    int materialColor = 0xdddddd;
    if(settings.rendermode == RenderMode.solid){
      _currentMaterial = _solidMaterial;
    }
    else{
      _currentMaterial = _wireframeMaterial;
    }

    three.MeshBasicMaterial contactDotMaterial = three.MeshBasicMaterial({ 'color': 0xffffff });

    final contactPointGeometry = three.SphereGeometry(0.1, 6, 6);
    contactMeshCache = GeometryCache(scene, (){
      return three.Mesh(contactPointGeometry, contactDotMaterial);
    });

    cm2contactMeshCache = GeometryCache(scene, (){
      BufferGeometry geometry = BufferGeometry();
      geometry.setAttribute(
        'position',
          Float32BufferAttribute(Float32Array.from([0,0,0,1,1,1]), 3, false)
      );
      return three.Line(geometry, three.LineBasicMaterial({ 'color': 0xff0000 }));
    });

    three.BoxGeometry bboxGeometry = three.BoxGeometry(1, 1, 1);
    three.MeshBasicMaterial bboxMaterial = three.MeshBasicMaterial({
      'color': materialColor,
      'wireframe': true,
    });

    bboxMeshCache = GeometryCache(scene, (){
      return three.Mesh(bboxGeometry, bboxMaterial);
    });

    distanceConstraintMeshCache = GeometryCache(scene, (){
      BufferGeometry geometry = three.BufferGeometry();
      geometry.setAttribute(
        'position',
          Float32BufferAttribute(Float32Array.from([0,0,0,1,1,1]), 3, false)
      );
      return three.Line(geometry, three.LineBasicMaterial({ 'color': 0xff0000 }));
    });

    p2pConstraintMeshCache = GeometryCache(scene, () {
      BufferGeometry geometry = three.BufferGeometry();
      geometry.setAttribute(
        'position',
          Float32BufferAttribute(Float32Array.from([0,0,0,1,1,1]), 3, false)
      );
      return three.Line(geometry, three.LineBasicMaterial({ 'color': 0xff0000 }));
    });

    normalMeshCache = GeometryCache(scene, (){
      BufferGeometry geometry = three.BufferGeometry();
      geometry.setAttribute(
        'position',
          Float32BufferAttribute(Float32Array.from([0,0,0,1,1,1]), 3, false)
      );
      return three.Line(geometry, three.LineBasicMaterial({ 'color': 0x00ff00 }));
    });

    axesMeshCache = GeometryCache(scene, (){
      three.Object3D mesh = three.Object3D();
      List<double> origin = [0, 0, 0];
      BufferGeometry gX = BufferGeometry();
      BufferGeometry gY = BufferGeometry();
      BufferGeometry gZ = BufferGeometry();
      gX.setAttribute(
        'position',
          Float32BufferAttribute(Float32Array.from(origin+[1,0,0]), 3, false)
      );
      gY.setAttribute(
        'position',
          Float32BufferAttribute(Float32Array.from(origin+[0,1,0]), 3, false)
      );
      gZ.setAttribute(
        'position',
          Float32BufferAttribute(Float32Array.from(origin+[0,0,1]), 3, false)
      );
      three.Line lineX = three.Line(gX, three.LineBasicMaterial({ 'color': 0xff0000 }));
      three.Line lineY = three.Line(gY, three.LineBasicMaterial({ 'color': 0x00ff00 }));
      three.Line lineZ = three.Line(gZ, three.LineBasicMaterial({ 'color': 0x0000ff }));
      mesh.add(lineX);
      mesh.add(lineY);
      mesh.add(lineZ);
      return mesh;
    });
  }

  void restartGeometryCaches () {
    contactMeshCache.restart();
    contactMeshCache.hideCached();

    cm2contactMeshCache.restart();
    cm2contactMeshCache.hideCached();

    distanceConstraintMeshCache.restart();
    distanceConstraintMeshCache.hideCached();

    normalMeshCache.restart();
    normalMeshCache.hideCached();
  }
  void initSize(BuildContext context){
    if (screenSize != null) {
      return;
    }

    final mqd = MediaQuery.of(context);

    screenSize = mqd.size;
    dpr = mqd.devicePixelRatio;

   initPlatformState();
  }
  
  void animate() {
    if (!mounted || disposed) {
      return;
    }
    render();
    Future.delayed(const Duration(milliseconds: 1000~/60), () {
      if(!pause){
        updatePhysics();
        for(int i = 0; i < events.length;i++){
          events[i].call(world.dt);
        }
        updateVisuals();
      }
      animate();
    });
  }

  void initPage() async {
    scene.fog = three.Fog(0x222222, 1000, 2000);

    camera = PerspectiveCamera(24, width/height, 5, 2000);
    camera.position.set(0,20,40);
    camera.lookAt(three.Vector3(0, 0, 0));

    controls = OrbitControls(camera, globalKey);
    controls.rotateSpeed = 1.0;
    controls.zoomSpeed = 1.2;
    controls.enableDamping = true;
    controls.enablePan = false;
    controls.dampingFactor = 0.2;
    controls.minDistance = 10;
    controls.maxDistance = 500;

    _ambientLight = AmbientLight(0xffffff, 0.1);
    scene.add(_ambientLight);

    DirectionalLight directionalLight = DirectionalLight( 0xffffff , 0.15);
    directionalLight.position.set(-30, 40, 30);
    directionalLight.target!.position.set( 0, 0, 0 );
    scene.add(directionalLight);

    _spotLight = three.SpotLight(0xffffff, 0.9, 0.0, Math.PI / 8, 1.0);
    _spotLight.position.set(-30, 40, 30);
    _spotLight.target!.position.set(0, 0, 0);

    _spotLight.castShadow = true;

    _spotLight.shadow!.camera!.near = 10;
    _spotLight.shadow!.camera!.far = 100;
    _spotLight.shadow!.camera!.fov = 30;

    // spotLight.shadow.bias = -0.0001
    _spotLight.shadow!.mapSize.width = 2048;
    _spotLight.shadow!.mapSize.height = 2048;

    scene.add(_spotLight);

    for(String type in domElements.keys){
      domElement.addEventListener(type, domElements[type]!);
    }

    start();
  }
  void setRenderMode(RenderMode mode){
    switch(mode) {
      case RenderMode.solid:
        _currentMaterial = _solidMaterial;
        _spotLight.intensity = 1;
        _ambientLight.color!.setHex(0x222222);
        break;
      case RenderMode.wireframe:
        _currentMaterial = _wireframeMaterial;
        _spotLight.intensity = 0;
        _ambientLight.color!.setHex(0xffffff);
        break;
    }

    // set the materials
    visuals.forEach((visual){
      if (visual.material != null) {
        visual.material = _currentMaterial;
      }
      visual.traverse((child){
        if (child.material) {
          child.material = _currentMaterial;
        }
      });
    });

    settings.rendermode = mode;
  }

  void addVisual(cannon.Body body, {three.Mesh? mesh, three.Material? material}){
    MeshLambertMaterial particleMaterial = MeshLambertMaterial({ 'color': 0xff0000 });
    MeshBasicMaterial triggerMaterial = MeshBasicMaterial({ 'color': 0x00ff00, 'wireframe': true, 'wireframeLinewidth':1.0 });
    // if it's a particle paint it red, if it's a trigger paint it as green, otherwise just gray
    final isParticle = body.shapes.every((s) => s is cannon.Particle);
    final mat = material ?? (isParticle ? particleMaterial : body.isTrigger ? triggerMaterial : _currentMaterial);

    // get the correspondant three.js mesh
    final me = mesh?.clone() ?? ConversionUtils.bodyToMesh(body, mat);
    
    // enable shadows on every object
    // mesh.traverse((child){
    //   child.castShadow = true;
    //   child.receiveShadow = true;
    // });

    bodies.add(body);
    visuals.add(me);
    scene.add(me);
  }
  void addVisuals(List<cannon.Body> bodies) {
    bodies.forEach((body){
      addVisual(body);
    });
  }
  void removeVisual(cannon.Body body) {
    final index = bodies.indexOf(body);// .findIndex((b) => b.id === body.id);

    if (index == -1) {
      return;
    }

    final visual = visuals[index];

    bodies.splice(index, 1);
    visuals.splice(index, 1);

    scene.remove(visual);
  }
  void removeAllVisuals() {
    while (bodies.isNotEmpty) {
      removeVisual(bodies[0]);
    }
  }

  void start(){
    resetCallTime = true;
    if(scenes.keys.isNotEmpty){
      buildScene(scenes.keys.first);
    }
  }

  void _updateCannonPhysics() {
    // Step world
    final timeStep = 1 / settings.stepFrequency;
    final now = world.performance.now() / 1000;

    if (lastCallTime == null) {
      // last call time not saved, cant guess elapsed time. Take a simple step.
      world.step(timeStep);
      lastCallTime = now;
      return;
    }
    double timeSinceLastCall = now - lastCallTime!;
    if (resetCallTime) {
      timeSinceLastCall = 0;
      resetCallTime = false;
    }
    world.step(timeStep, timeSinceLastCall, settings.maxSubSteps);
    lastCallTime = now;
  }

  void render() {
    final _gl = three3dRender.gl;
    renderer!.render(scene, camera);
    _gl.flush();
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

  void initScene() async{
    initPage();
    initRenderer();
    // setupWorld();
    mounted = true;
    animate();
    onSetupComplete();
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

    // TODO web wait dom ok!!!
    Future.delayed(const Duration(milliseconds: 100), () async {
      await three3dRender.prepareContext();
      initScene();
    });
  }

  void updateVisuals(){
    // Copy position data into visuals
    for (int i = 0; i < bodies.length; i++) {
      final body = bodies[i];
      final visual = visuals[i];
      Object3D dummy = Object3D();

      // Interpolated or not?
      vmath.Vector3 position = body.interpolatedPosition;
      vmath.Quaternion quaternion = body.interpolatedQuaternion;
      if (settings.paused) {
        position = body.position;
        quaternion = body.quaternion;
      }

      if (visual is InstancedMesh) {
        dummy.position.copy(position.toVector3());
        dummy.quaternion.copy(quaternion.toQuaternion());

        dummy.updateMatrix();

        visual.setMatrixAt(body.index, dummy.matrix);
        visual.instanceMatrix!.needsUpdate = true;
      } else {
        visual.position.copy(position.toVector3());
        visual.quaternion.copy(quaternion.toQuaternion());
      }
    }

    // Render contacts
    contactMeshCache.restart();
    if (settings.contacts) {
      // if ci is even - use body i, else j
      for (int i = 0; i < world.contacts.length; i++) {
        cannon.ContactEquation contact = world.contacts[i];

        for (int ij = 0; ij < 2; ij++) {
          Object3D mesh = contactMeshCache.request();
          cannon.Body b = ij == 0 ? contact.bi : contact.bj;
          vmath.Vector3 r = ij == 0 ? contact.ri : contact.rj;
          mesh.position.set(b.position.x + r.x, b.position.y + r.y, b.position.z + r.z);
        }
      }
    }
    contactMeshCache.hideCached();

    // Lines from center of mass to contact point
    cm2contactMeshCache.restart();
    if (settings.cm2contact) {
      for (int i = 0; i < world.contacts.length; i++) {
        cannon.ContactEquation contact = world.contacts[i];

        for (int ij = 0; ij < 2; ij++) {
          Object3D line = cm2contactMeshCache.request();
          cannon.Body b = ij == 0 ? contact.bi : contact.bj;
          vmath.Vector3 r = ij == 0 ? contact.ri : contact.rj;
          line.scale.set(r.x, r.y, r.z);
          makeSureNotZero(line.scale);
          line.position.copy(b.position);
        }
      }
    }
    cm2contactMeshCache.hideCached();

    distanceConstraintMeshCache.restart();
    p2pConstraintMeshCache.restart();
    if (settings.constraints) {
      world.constraints.forEach((constraint){
        // Lines for distance constraints
        if(constraint is cannon.DistanceConstraint){
          constraint.equations.forEach((equation){
            cannon.Body bi = equation.bi;
            cannon.Body bj = equation.bj;

            Object3D line = distanceConstraintMeshCache.request();

            // Remember, bj is either a Vec3 or a Body.
            vmath.Vector3 vector = bj.position;

            line.scale.set(vector.x - bi.position.x, vector.y - bi.position.y, vector.z - bi.position.z);
            makeSureNotZero(line.scale);
            line.position.copy(bi.position);
          });
        }
        // Lines for point to point constraints
        else if(constraint is cannon.PointToPointConstraint) {
          constraint.equations.forEach((equation){
            cannon.Body bi = equation.bi;
            cannon.Body bj = equation.bj;

            Object3D relLine1 = p2pConstraintMeshCache.request();
            Object3D relLine2 = p2pConstraintMeshCache.request();
            Object3D diffLine = p2pConstraintMeshCache.request();
            if(equation is cannon.ContactEquation){
              relLine1.scale.set(equation.ri.x, equation.ri.y, equation.ri.z);
              relLine2.scale.set(equation.rj.x, equation.rj.y, equation.rj.z);
            }
            else if(equation is cannon.FrictionEquation){
              relLine1.scale.set(equation.ri.x, equation.ri.y, equation.ri.z);
              relLine2.scale.set(equation.rj.x, equation.rj.y, equation.rj.z);
            }

            // BUG this is not exposed anymore in the ContactEquation, this sections needs to be updated
            makeSureNotZero(relLine1.scale);
            makeSureNotZero(relLine2.scale);
            makeSureNotZero(diffLine.scale);
            relLine1.position.copy(bi.position);
            relLine2.position.copy(bj.position);

            if (equation is cannon.ContactEquation) {
              equation.bj.position.add2(equation.rj, diffLine.position.toVector3());
            }
            else if (equation is cannon.FrictionEquation) {
              equation.bj.position.add2(equation.rj, diffLine.position.toVector3());
            }
          });
        }
      });
    }
    p2pConstraintMeshCache.hideCached();
    distanceConstraintMeshCache.hideCached();

    // Normal lines
    normalMeshCache.restart();
    if (settings.normals) {
      for (int i = 0; i < world.contacts.length; i++) {
        cannon.ContactEquation constraint = world.contacts[i];

        cannon.Body bi = constraint.bi;
        //cannon.Body bj = constraint.bj;
        Object3D line = normalMeshCache.request();

        vmath.Vector3 constraintNormal = constraint.ni;
        cannon.Body body = bi;
        line.scale.set(constraintNormal.x, constraintNormal.y, constraintNormal.z);
        makeSureNotZero(line.scale);
        line.position.copy(body.position);
        constraint.ri.add2(line.position.toVector3(), line.position.toVector3());
      }
    }
    normalMeshCache.hideCached();

    // Frame axes for each body
    axesMeshCache.restart();
    if (settings.axes) {
      for (int i = 0; i < bodies.length; i++) {
        cannon.Body body = bodies[i];

        Object3D mesh = axesMeshCache.request();
        mesh.position.copy(body.position.toVector3());
        mesh.quaternion.copy(body.quaternion.toQuaternion());
      }
    }
    axesMeshCache.hideCached();

    // AABBs
    bboxMeshCache.restart();
    if (settings.aabbs) {
      for (int i = 0; i < bodies.length; i++) {
        cannon.Body body = bodies[i];

        if (body.aabbNeedsUpdate) {
          body.updateAABB();
        }

        // Todo: cap the infinite AABB to scene AABB, for now just dont render
        if (
          isFinite(body.aabb.lowerBound.x) &&
          isFinite(body.aabb.lowerBound.y) &&
          isFinite(body.aabb.lowerBound.z) &&
          isFinite(body.aabb.upperBound.x) &&
          isFinite(body.aabb.upperBound.y) &&
          isFinite(body.aabb.upperBound.z) &&
          body.aabb.lowerBound.x - body.aabb.upperBound.x != 0 &&
          body.aabb.lowerBound.y - body.aabb.upperBound.y != 0 &&
          body.aabb.lowerBound.z - body.aabb.upperBound.z != 0
        ) {
          Object3D mesh = bboxMeshCache.request();
          mesh.scale.set(
            body.aabb.lowerBound.x - body.aabb.upperBound.x,
            body.aabb.lowerBound.y - body.aabb.upperBound.y,
            body.aabb.lowerBound.z - body.aabb.upperBound.z
          );
          mesh.position.set(
            (body.aabb.lowerBound.x + body.aabb.upperBound.x) * 0.5,
            (body.aabb.lowerBound.y + body.aabb.upperBound.y) * 0.5,
            (body.aabb.lowerBound.z + body.aabb.upperBound.z) * 0.5
          );
        }
        
      }
    }
    bboxMeshCache.hideCached();
  }
  void makeSureNotZero(vector) {
    if (vector.x == 0) {
      vector.x = 1e-6;
    }
    if (vector.y == 0) {
      vector.y = 1e-6;
    }
    if (vector.z == 0) {
      vector.z = 1e-6;
    }
  }
  void changeScene(String n){
    settings.paused = false;
    buildScene(n);
  }
  void buildScene(n){
    // Remove current bodies
    bodies.forEach((body){
      world.removeBody(body);
    });

    // Remove all visuals
    removeAllVisuals();

    // Remove all constraints
    while (world.constraints.isNotEmpty) {
      world.removeConstraint(world.constraints[0]);
    }

    // Run the user defined "build scene" function
    scenes[n].call();

    // Read the newly set data to the gui
    settings.iterations = world.solver.iterations;
    settings.gx = world.gravity.x + 0.0;
    settings.gy = world.gravity.y + 0.0;
    settings.gz = world.gravity.z + 0.0;
    settings.quatNormalizeSkip = world.quatNormalizeSkip;
    settings.quatNormalizeFast = world.quatNormalizeFast;

    restartGeometryCaches();
  }
  void restartCurrentScene() {
    bodies.forEach((body){
      body.position.setFrom(body.initPosition);
      body.velocity.setFrom(body.initVelocity);
      //if(body.initAngularVelocity != null) {
        body.angularVelocity.setFrom(body.initAngularVelocity);
        body.quaternion.setFrom(body.initQuaternion);
      //}
    });
  }

  List<Widget> selectScene(BuildContext context){
    List<Widget> widgets = [];

    for(String key in scenes.keys){
      widgets.add(
        InkWell(
          onTap: (){
            changeScene(key);
          },
          child: Container(
            margin: const EdgeInsets.fromLTRB(5,5,5,0),
            height: 20,
            width: 120,
            decoration: BoxDecoration(
              borderRadius: BorderRadius.circular(2),
              border: Border.all(color: Theme.of(context).dividerColor)
            ),
            child: Text(
              key,
              style: Theme.of(context).primaryTextTheme.bodySmall,
            ),
          ),
        )
      );
    }

    return widgets;
  }

  Widget threeDart() {
    return Builder(builder: (BuildContext context) {
      initSize(context);
      return Stack(
        children:[
          Container(
            width: screenSize!.width,
            height: screenSize!.height,
            color: Theme.of(context).canvasColor,
            child: DomLikeListenable(
              key: globalKey,
              builder: (BuildContext context) {
                return Container(
                  width: width,
                  height: height,
                  color: Theme.of(context).canvasColor,
                  child: Builder(builder: (BuildContext context) {
                    if (kIsWeb) {
                      return three3dRender.isInitialized? HtmlElementView(viewType:three3dRender.textureId!.toString()):Container();
                    } 
                    else {
                      return three3dRender.isInitialized?Texture(textureId: three3dRender.textureId!):Container();
                    }
                  })
                );
              }
            ),
          ),
          if(scenes.isNotEmpty)Positioned(
            top: 20,
            right: 20,
            child: Container(
                width: 120,
                height: 120,
                decoration: BoxDecoration(
                  color: Theme.of(context).cardColor,
                  borderRadius: BorderRadius.circular(5)
                ),
                child: ListView(
                  children: selectScene(context),
                ),
              )
          ),
          Positioned(
            bottom: 20,
            right: 20,
            child: InkWell(
              onTap: (){
                restartCurrentScene();
              },
              child: Container(
                width: 45,
                height: 45,
                decoration: BoxDecoration(
                  color: Theme.of(context).secondaryHeaderColor,
                  borderRadius: BorderRadius.circular(45/2)
                ),
                child: const Icon(
                  Icons.refresh
                ),
              )
            )
          )
        ]
      );
    });
  }
}