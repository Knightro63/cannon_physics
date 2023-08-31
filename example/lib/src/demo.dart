// import { SmoothieChart, TimeSeries } from './smoothie.js'
// import { addTitle, addSourceButton } from './dom-utils.js'

import 'dart:math' as math;
import 'dart:async';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/foundation.dart';

import 'package:flutter_gl/flutter_gl.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:three_dart/three_dart.dart' as three;
import 'package:three_dart/three_dart.dart' hide Texture, Color;
import 'package:three_dart_jsm/three_dart_jsm.dart';

enum RenderMode{solid}

class DemoSettings{
  DemoSettings({
    this.stepFrequency= 60,
    this.quatNormalizeSkip= 2,
    quatNormalizeFast= true,
    this.gx= 0,
    this.gy= 0,
    this.gz= 0,
    this.iterations= 3,
    this.tolerance= 0.0001,
    this.k= 1e6,
    this.d= 3,
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
    //...options,
  });
  int stepFrequency;
  int quatNormalizeSkip;
  bool quatNormalizeFast;
  int gx;
  int gy;
  int gz;
  int iterations;
  double tolerance;
  int k;
  int d;
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
  //...options,
}
/**
 * Demo utility class. If you want to learn how to connect cannon.js with three.js, please look at the examples/threejs_* instead.
 */
class Demo extends cannon.EventTarget {
  //sceneFolder;
  List<three.Scene> scenes = [];
  Map listeners = {};

  // array used to keep in sync the visuals with the bodies
  // they will have always the same length
  List<cannon.Body> bodies = [];
  List visuals = [];

  //gui
  var smoothie;
  var smoothieCanvas;

  List<String> renderModes = ['solid', 'wireframe'];
  three.Object3D dummy = three.Object3D();
  DemoSettings settings = DemoSettings();
  cannon.World world = cannon.World();

  Demo(options):super() {
    if (settings.stepFrequency % 60 != 0) {
      throw('stepFrequency must be a multiple of 60.');
    }

    // Init three.js
    initThree();

    // Init the geometry caches
    this.initGeometryCaches();

    // Init stats.js
    this.initStats();

    // Init smoothie.js
    this.initSmoothie();

    // Start the loop!
    this.animate();

    // Attach listeners
    window.addEventListener('resize', this.resize);
    document.addEventListener('keypress', this.onKeyPress);
  }

  setRenderMode(mode){
    if (!this.renderModes.includes(mode)) {
      throw new Error(`Render mode ${mode} not found!`)
    }

    switch (mode) {
      case 'solid':
        this.currentMaterial = this.solidMaterial
        this.spotLight.intensity = 1
        this.ambientLight.color.setHex(0x222222)
        break
      case 'wireframe':
        this.currentMaterial = this.wireframeMaterial
        this.spotLight.intensity = 0
        this.ambientLight.color.setHex(0xffffff)
        break
    }

    // set the materials
    this.visuals.forEach((visual) => {
      if (visual.material) {
        visual.material = this.currentMaterial
      }
      visual.traverse((child) => {
        if (child.material) {
          child.material = this.currentMaterial
        }
      })
    })

    this.settings.rendermode = mode
  }

  initStats(){
    this.stats = new Stats();
    document.body.appendChild(this.stats.domElement);
  }

  /**
   * Add a scene to the demo app
   * @method addScene
   * @param {String} title Title of the scene
   * @param {Function} initfunc A function this takes one argument, app, and initializes a physics scene. The function runs app.setWorld(body), app.addVisual(body), app.removeVisual(body) etc.
   */
  addScene(String title, void Function() initfunc){
    this.scenes.add(initfunc);
    const index = this.scenes.length - 1;
    this.sceneFolder.add({ [title]: () => this.changeScene(index) }, title);
  }

  /**
   * Restarts the current scene
   * @method restartCurrentScene
   */
  restartCurrentScene (){
    this.bodies.forEach((body){
      body.position.copy(body.initPosition);
      body.velocity.copy(body.initVelocity);
      //if (body.initAngularVelocity) {
        body.angularVelocity.copy(body.initAngularVelocity);
        body.quaternion.copy(body.initQuaternion);
      //}
    });
  }

  updateVisuals(){
    // Copy position data into visuals
    for (int i = 0; i < this.bodies.length; i++) {
      cannon.Body body = this.bodies[i];
      three.Mesh visual = this.visuals[i];

      // Interpolated or not?
      cannon.Vec3 position = body.interpolatedPosition;
      cannon.Quaternion quaternion = body.interpolatedQuaternion;
      if (this.settings.paused) {
        position = body.position;
        quaternion = body.quaternion;
      }

      if (visual.isInstancedMesh) {
        this.dummy.position.copy(position);
        this.dummy.quaternion.copy(quaternion);

        this.dummy.updateMatrix();

        visual.setMatrixAt(body.instanceIndex, this.dummy.matrix);
        visual.instanceMatrix.needsUpdate = true;
      } 
      else {
        visual.position.copy(position);
        visual.quaternion.copy(quaternion);
      }
    }

    // Render contacts
    this.contactMeshCache.restart();
    if (this.settings.contacts) {
      // if ci is even - use body i, else j
      for (int i = 0; i < this.world.contacts.length; i++) {
        const contact = this.world.contacts[i]

        for (int ij = 0; ij < 2; ij++) {
          const mesh = this.contactMeshCache.request();
          const b = ij == 0 ? contact.bi : contact.bj;
          const r = ij == 0 ? contact.ri : contact.rj;
          mesh.position.set(b.position.x + r.x, b.position.y + r.y, b.position.z + r.z);
        }
      }
    }
    this.contactMeshCache.hideCached();

    // Lines from center of mass to contact point
    this.cm2contactMeshCache.restart();
    if (this.settings.cm2contact) {
      for (int i = 0; i < this.world.contacts.length; i++) {
        const contact = this.world.contacts[i]

        for (int ij = 0; ij < 2; ij++) {
          const line = this.cm2contactMeshCache.request();
          const b = ij == 0 ? contact.bi : contact.bj;
          const r = ij == 0 ? contact.ri : contact.rj;
          line.scale.set(r.x, r.y, r.z)
          makeSureNotZero(line.scale)
          line.position.copy(b.position)
        }
      }
    }
    this.cm2contactMeshCache.hideCached()

    this.distanceConstraintMeshCache.restart()
    this.p2pConstraintMeshCache.restart()
    if (this.settings.constraints) {
      this.world.constraints.forEach((constraint) => {
        switch (true) {
          // Lines for distance constraints
          case constraint instanceof CANNON.DistanceConstraint: {
            constraint.equations.forEach((equation) => {
              const { bi, bj } = equation

              const line = this.distanceConstraintMeshCache.request()

              // Remember, bj is either a Vec3 or a Body.
              const vector = bj.position || bj

              line.scale.set(vector.x - bi.position.x, vector.y - bi.position.y, vector.z - bi.position.z)
              makeSureNotZero(line.scale)
              line.position.copy(bi.position)
            })

            break
          }

          // Lines for point to point constraints
          case constraint instanceof CANNON.PointToPointConstraint: {
            constraint.equations.forEach((equation) => {
              const { bi, bj } = equation

              const relLine1 = this.p2pConstraintMeshCache.request()
              const relLine2 = this.p2pConstraintMeshCache.request()
              const diffLine = this.p2pConstraintMeshCache.request()
              if (equation.ri) {
                relLine1.scale.set(equation.ri.x, equation.ri.y, equation.ri.z)
              }
              if (equation.rj) {
                relLine2.scale.set(equation.rj.x, equation.rj.y, equation.rj.z)
              }
              // BUG this is not exposed anymore in the ContactEquation, this sections needs to be updated
              if (equation.penetrationVec) {
                diffLine.scale.set(-equation.penetrationVec.x, -equation.penetrationVec.y, -equation.penetrationVec.z)
              }
              makeSureNotZero(relLine1.scale)
              makeSureNotZero(relLine2.scale)
              makeSureNotZero(diffLine.scale)
              relLine1.position.copy(bi.position)
              relLine2.position.copy(bj.position)

              if (equation.bj && equation.rj) {
                equation.bj.position.vadd(equation.rj, diffLine.position)
              }
            })
            break
          }
        }
      })
    }
    this.p2pConstraintMeshCache.hideCached()
    this.distanceConstraintMeshCache.hideCached()

    // Normal lines
    this.normalMeshCache.restart()
    if (this.settings.normals) {
      for (let i = 0; i < this.world.contacts.length; i++) {
        const constraint = this.world.contacts[i]

        const bi = constraint.bi
        const bj = constraint.bj
        const line = this.normalMeshCache.request()

        const constraintNormal = constraint.ni
        const body = bi
        line.scale.set(constraintNormal.x, constraintNormal.y, constraintNormal.z)
        makeSureNotZero(line.scale)
        line.position.copy(body.position)
        constraint.ri.vadd(line.position, line.position)
      }
    }
    this.normalMeshCache.hideCached()

    // Frame axes for each body
    this.axesMeshCache.restart()
    if (this.settings.axes) {
      for (let i = 0; i < this.bodies.length; i++) {
        const body = this.bodies[i]

        const mesh = this.axesMeshCache.request()
        mesh.position.copy(body.position)
        mesh.quaternion.copy(body.quaternion)
      }
    }
    this.axesMeshCache.hideCached()

    // AABBs
    this.bboxMeshCache.restart()
    if (this.settings.aabbs) {
      for (let i = 0; i < this.bodies.length; i++) {
        const body = this.bodies[i]
        if (body.updateAABB) {
          if (body.aabbNeedsUpdate) {
            body.updateAABB()
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
            const mesh = this.bboxMeshCache.request()
            mesh.scale.set(
              body.aabb.lowerBound.x - body.aabb.upperBound.x,
              body.aabb.lowerBound.y - body.aabb.upperBound.y,
              body.aabb.lowerBound.z - body.aabb.upperBound.z
            )
            mesh.position.set(
              (body.aabb.lowerBound.x + body.aabb.upperBound.x) * 0.5,
              (body.aabb.lowerBound.y + body.aabb.upperBound.y) * 0.5,
              (body.aabb.lowerBound.z + body.aabb.upperBound.z) * 0.5
            )
          }
        }
      }
    }
    this.bboxMeshCache.hideCached()
  }

  changeScene = (n) => {
    this.dispatchEvent({ type: 'destroy' })

    // unbind all listeners
    Object.keys(this.listeners).forEach((event) => {
      this.listeners[event].forEach((callback) => {
        this.removeEventListener(event, callback)
      })
    })
    this.listeners = {}

    // clear any previous logs
    console.clear()

    this.settings.paused = false
    this.updateGui()
    this.buildScene(n)
  }

  start = () => {
    this.buildScene(0)
  }

  buildScene = (n) => {
    // Remove current bodies
    this.bodies.forEach((body) => this.world.removeBody(body))

    // Remove all visuals
    this.removeAllVisuals()

    // Remove all constraints
    while (this.world.constraints.length) {
      this.world.removeConstraint(this.world.constraints[0])
    }

    // Run the user defined "build scene" function
    this.scenes[n]()

    // Read the newly set data to the gui
    this.settings.iterations = this.world.solver.iterations
    this.settings.gx = this.world.gravity.x + 0.0
    this.settings.gy = this.world.gravity.y + 0.0
    this.settings.gz = this.world.gravity.z + 0.0
    this.settings.quatNormalizeSkip = this.world.quatNormalizeSkip
    this.settings.quatNormalizeFast = this.world.quatNormalizeFast
    this.updateGui()

    this.restartGeometryCaches()
  }

  initGeometryCaches = () => {
    // Material
    this.materialColor = 0xdddddd
    this.solidMaterial = new three.MeshLambertMaterial({ color: this.materialColor })
    //three.ColorUtils.adjustHSV( solidMaterial.color, 0, 0, 0.9 );
    this.wireframeMaterial = new three.MeshBasicMaterial({ color: 0xffffff, wireframe: true })
    this.currentMaterial = this.solidMaterial
    const contactDotMaterial = new three.MeshBasicMaterial({ color: 0xffffff })
    this.particleMaterial = new three.MeshLambertMaterial({ color: 0xff0000 })
    this.triggerMaterial = new three.MeshBasicMaterial({ color: 0x00ff00, wireframe: true })

    const contactPointGeometry = new three.SphereGeometry(0.1, 6, 6)
    this.contactMeshCache = new GeometryCache(this.scene, () => {
      return new three.Mesh(contactPointGeometry, contactDotMaterial)
    })

    this.cm2contactMeshCache = new GeometryCache(this.scene, () => {
      const geometry = new three.Geometry()
      geometry.vertices.push(new three.Vector3(0, 0, 0))
      geometry.vertices.push(new three.Vector3(1, 1, 1))
      return new three.Line(geometry, new three.LineBasicMaterial({ color: 0xff0000 }))
    })

    const bboxGeometry = new three.BoxGeometry(1, 1, 1)
    const bboxMaterial = new three.MeshBasicMaterial({
      color: this.materialColor,
      wireframe: true,
    })
    this.bboxMeshCache = new GeometryCache(this.scene, () => {
      return new three.Mesh(bboxGeometry, bboxMaterial)
    })

    this.distanceConstraintMeshCache = new GeometryCache(this.scene, () => {
      const geometry = new three.Geometry()
      geometry.vertices.push(new three.Vector3(0, 0, 0))
      geometry.vertices.push(new three.Vector3(1, 1, 1))
      return new three.Line(geometry, new three.LineBasicMaterial({ color: 0xff0000 }))
    })

    this.p2pConstraintMeshCache = new GeometryCache(this.scene, () => {
      const geometry = new three.Geometry()
      geometry.vertices.push(new three.Vector3(0, 0, 0))
      geometry.vertices.push(new three.Vector3(1, 1, 1))
      return new three.Line(geometry, new three.LineBasicMaterial({ color: 0xff0000 }))
    })

    this.normalMeshCache = new GeometryCache(this.scene, () => {
      const geometry = new three.Geometry()
      geometry.vertices.push(new three.Vector3(0, 0, 0))
      geometry.vertices.push(new three.Vector3(1, 1, 1))
      return new three.Line(geometry, new three.LineBasicMaterial({ color: 0x00ff00 }))
    })

    this.axesMeshCache = new GeometryCache(this.scene, () => {
      const mesh = new three.Object3D()
      const origin = new three.Vector3(0, 0, 0)
      const gX = new three.Geometry()
      const gY = new three.Geometry()
      const gZ = new three.Geometry()
      gX.vertices.push(origin)
      gY.vertices.push(origin)
      gZ.vertices.push(origin)
      gX.vertices.push(new three.Vector3(1, 0, 0))
      gY.vertices.push(new three.Vector3(0, 1, 0))
      gZ.vertices.push(new three.Vector3(0, 0, 1))
      const lineX = new three.Line(gX, new three.LineBasicMaterial({ color: 0xff0000 }))
      const lineY = new three.Line(gY, new three.LineBasicMaterial({ color: 0x00ff00 }))
      const lineZ = new three.Line(gZ, new three.LineBasicMaterial({ color: 0x0000ff }))
      mesh.add(lineX)
      mesh.add(lineY)
      mesh.add(lineZ)
      return mesh
    })
  }

  restartGeometryCaches = () => {
    this.contactMeshCache.restart()
    this.contactMeshCache.hideCached()

    this.cm2contactMeshCache.restart()
    this.cm2contactMeshCache.hideCached()

    this.distanceConstraintMeshCache.restart()
    this.distanceConstraintMeshCache.hideCached()

    this.normalMeshCache.restart()
    this.normalMeshCache.hideCached()
  }

  animate = () => {
    requestAnimationFrame(this.animate)
    if (!this.settings.paused) {
      this.updatePhysics()
      this.updateVisuals()
    }
    this.controls.update()
    this.renderer.render(this.scene, this.camera)
    this.stats.update()
  }

  lastCallTime = 0
  resetCallTime = false
  updatePhysics = () => {
    // Step world
    const timeStep = 1 / this.settings.stepFrequency

    const now = performance.now() / 1000

    if (!this.lastCallTime) {
      // last call time not saved, cant guess elapsed time. Take a simple step.
      this.world.step(timeStep)
      this.lastCallTime = now
      return
    }

    let timeSinceLastCall = now - this.lastCallTime
    if (this.resetCallTime) {
      timeSinceLastCall = 0
      this.resetCallTime = false
    }

    this.world.step(timeStep, timeSinceLastCall, this.settings.maxSubSteps)

    this.lastCallTime = now
  }

  resize = () => {
    this.camera.aspect = window.innerWidth / window.innerHeight
    this.camera.updateProjectionMatrix()
    this.renderer.setSize(window.innerWidth, window.innerHeight)
  }

  onKeyPress(event){
    switch (event.code) {
      case 'Space': // Space - restart
        this.restartCurrentScene()
        break;

      case 'KeyH': // h - toggle widgets
        if (this.stats.domElement.style.display == 'none') {
          this.stats.domElement.style.display = 'block';
          info.style.display = 'block';
        } else {
          this.stats.domElement.style.display = 'none';
          info.style.display = 'none';
        }
        break;

      case 'KeyA': // a - AABBs
        this.settings.aabbs = !this.settings.aabbs;
        this.updateGui();
        break;

      case 'KeyC': // c - constraints
        this.settings.constraints = !this.settings.constraints;
        this.updateGui();
        break;

      case 'KeyP': // p - pause
        this.settings.paused = !this.settings.paused;
        this.resetCallTime = true;
        this.updateGui();
        break;

      case 'KeyS': // s - step
        const timeStep = 1 / this.settings.stepFrequency;
        this.world.step(timeStep);
        this.updateVisuals();
        break;

      case 'KeyM': // m - toggle materials
        const index = this.renderModes.indexOf(this.settings.rendermode);
        index++;
        index = index % this.this.renderModes.length; // begin at 0 if we exceeded number of modes
        this.setRenderMode(this.renderModes[index]);
        this.updateGui();
        break;

      case 'Digit1':
      case 'Digit2':
      case 'Digit3':
      case 'Digit4':
      case 'Digit5':
      case 'Digit6':
      case 'Digit7':
      case 'Digit8':
      case 'Digit9':
        // Change scene
        const digit = Number(event.code.slice(-1)) - 1;
        // Only if scene is present and if no input field is active
        if (this.scenes.length > digit && !document.activeElement.localName.match(/input/)) {
          this.changeScene(digit);
        }
        break;
    }
  }

  initThree(){
    // Camera
    this.camera = new three.PerspectiveCamera(24, window.innerWidth / window.innerHeight, 5, 2000)

    this.camera.position.set(0, 20, 30);
    this.camera.lookAt(0, 0, 0);

    // Scene
    this.scene = new three.Scene();
    this.scene.fog = new three.Fog(0x222222, 1000, 2000);

    // Renderer
    this.renderer = new three.WebGLRenderer({ antialias: true });
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    document.body.appendChild(this.renderer.domElement);

    this.renderer.setClearColor(this.scene.fog.color, 1);

    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = three.PCFSoftShadowMap;

    // Lights
    this.ambientLight = new three.AmbientLight(0xffffff, 0.1);
    this.scene.add(this.ambientLight);

    this.spotLight = new three.SpotLight(0xffffff, 0.9, 0, Math.PI / 8, 1);
    this.spotLight.position.set(-30, 40, 30);
    this.spotLight.target.position.set(0, 0, 0);

    this.spotLight.castShadow = true;

    this.spotLight.shadow.camera.near = 10;
    this.spotLight.shadow.camera.far = 100;
    this.spotLight.shadow.camera.fov = 30;

    // spotLight.shadow.bias = -0.0001
    this.spotLight.shadow.mapSize.width = 2048;
    this.spotLight.shadow.mapSize.height = 2048;

    this.scene.add(this.spotLight);

    const directionalLight = new three.DirectionalLight(0xffffff, 0.15);
    directionalLight.position.set(-30, 40, 30);
    directionalLight.target.position.set(0, 0, 0);
    this.scene.add(directionalLight);

    // Orbit controls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.rotateSpeed = 1.0;
    this.controls.zoomSpeed = 1.2;
    this.controls.enableDamping = true;
    this.controls.enablePan = false;
    this.controls.dampingFactor = 0.2;
    this.controls.minDistance = 10;
    this.controls.maxDistance = 500;
  }

  initSmoothie(){
    this.smoothieCanvas = document.createElement('canvas');
    this.smoothieCanvas.width = window.innerWidth;
    this.smoothieCanvas.height = window.innerHeight;
    this.smoothieCanvas.style.opacity = 0.5;
    this.smoothieCanvas.style.position = 'absolute';
    this.smoothieCanvas.style.top = '0px';
    this.smoothieCanvas.style.zIndex = 1;
    document.body.appendChild(this.smoothieCanvas);

    this.smoothie = new SmoothieChart({
      labelOffsetY: 50,
      maxDataSetLength: 100,
      millisPerPixel: 2,
      grid: {
        strokeStyle: 'none',
        fillStyle: 'none',
        lineWidth: 1,
        millisPerLine: 250,
        verticalSections: 6,
      },
      labels: {
        fillStyle: 'rgb(180, 180, 180)',
      },
    });
    this.smoothie.streamTo(this.smoothieCanvas);

    // Create time series for each profile label
    const lines = {};
    const colors = [
      [255, 0, 0],
      [0, 255, 0],
      [0, 0, 255],
      [255, 255, 0],
      [255, 0, 255],
      [0, 255, 255],
    ];

    Object.keys(this.world.profile).forEach((label, i) => {
      const color = colors[i % colors.length];
      lines[label] = new TimeSeries({
        label,
        fillStyle: `rgb(${color[0]},${color[1]},${color[2]})`,
        maxDataLength: 500,
      })
    });

    // Add a random value to each line every second
    this.world.addEventListener('postStep', (){
      Object.keys(this.world.profile).forEach((label) => {
        lines[label].append(this.world.time * 1000, this.world.profile[label])
      });
    })

    // Add to SmoothieChart
    Object.keys(this.world.profile).forEach((label, i) => {
      const color = colors[i % colors.length]
      this.smoothie.addTimeSeries(lines[label], {
        strokeStyle: `rgb(${color[0]},${color[1]},${color[2]})`,
        lineWidth: 2,
      })
    });

    // Pause it
    this.world.doProfiling = false;
    this.smoothie.stop();
    this.smoothieCanvas.style.display = 'none';
  }

  setGlobalSpookParams(k, d, h) {
    // Set for all constraints
    for (int i = 0; i < this.world.constraints.length; i++) {
      const constraint = this.world.constraints[i];
      for (int j = 0; j < constraint.equations.length; j++) {
        const equation = constraint.equations[j];
        equation.setSpookParams(k, d, h);
      }
    }

    // Set for all contact materals
    for (int i = 0; i < this.world.contactmaterials.length; i++) {
      const contactMaterial = this.world.contactmaterials[i];
      contactMaterial.contactEquationStiffness = k;
      contactMaterial.frictionEquationStiffness = k;
      contactMaterial.contactEquationRelaxation = d;
      contactMaterial.frictionEquationRelaxation = d;
    }

    this.world.defaultContactMaterial.contactEquationStiffness = k;
    this.world.defaultContactMaterial.frictionEquationStiffness = k;
    this.world.defaultContactMaterial.contactEquationRelaxation = d;
    this.world.defaultContactMaterial.frictionEquationRelaxation = d;
  }

  getWorld() {
    return this.world;
  }

  addVisual(body) {
    if (!(body is cannon.Body)) {
      throw('The argument passed to addVisual() is not a body');
    }

    // if it's a particle paint it red, if it's a trigger paint it as green, otherwise just gray
    const isParticle = body.shapes.every((s) => s is cannon.Particle);
    const material = isParticle ? this.particleMaterial : body.isTrigger ? this.triggerMaterial : this.currentMaterial

    // get the correspondant three.js mesh
    const mesh = bodyToMesh(body, material);

    // enable shadows on every object
    mesh.traverse((child){
      child.castShadow = true;
      child.receiveShadow = true;
    });

    this.bodies.add(body);
    this.visuals.add(mesh);

    this.scene.add(mesh);
  }

  addVisuals(bodies) {
    bodies.forEach((body){
      this.addVisual(body);
    });
  }

  addVisualsInstanced(bodies) {
    if (
      !Array.isArray(bodies) ||
      !bodies.every((body) => body is cannon.Body && body.type == bodies[0].type)
    ) {
      throw new Error('The argument passed to addVisualsInstanced() is not an array of bodies of the same type')
    }

    // all bodies are the same, so pick the first
    const body = bodies[0];

    // if it's a particle paint it red, otherwise just gray
    const material = body.shapes.every((s) => s is cannon.Particle)
      ? this.particleMaterial
      : this.currentMaterial;

    // get the three.js mesh correspondant of the first body since they're of the same type
    const meshGroup = bodyToMesh(body, material);

    // extract the mesh from the group
    let mesh;
    meshGroup.traverse((child){
      if (child.isMesh) mesh = child;
    });

    // the clone is there because of this issue
    // https://github.com/mrdoob/three.js/issues/17701
    const instancedMesh = new three.InstancedMesh(mesh.geometry.clone(), mesh.material.clone(), bodies.length)
    instancedMesh.instanceMatrix.setUsage(three.DynamicDrawUsage); // will be updated every frame

    instancedMesh.receiveShadow = true;
    instancedMesh.castShadow = true;

    // Add bodies
    bodies.forEach((body, i){
      this.bodies.push(body);
      this.visuals.push(instancedMesh);
      body.instanceIndex = i;
    });

    this.scene.add(instancedMesh);
  }

  removeVisual(body) {
    const index = this.bodies.findIndex((b) => b.id == body.id);

    if (index == -1) {
      return;
    }

    const visual = this.visuals[index];

    this.bodies.splice(index, 1);
    this.visuals.splice(index, 1);

    this.scene.remove(visual);
  }

  removeAllVisuals() {
    while (this.bodies.length) {
      this.removeVisual(this.bodies[0]);
    }
  }

  addEventListener(event, callback) {
    if (this.listeners[event]) {
      this.listeners[event].push(callback)
    } else {
      this.listeners[event] = [callback]
    }

    super.addEventListener(event, callback)
  }
}

class GeometryCache {
  List<three.Mesh> geometries = [];
  List gone = [];
  three.Scene sceen;
  void Function() createFunc;

  GeometryCache(this.scene, this.createFunc);

  three.Mesh request(){
    final geometry = geometries.isNotEmpty?geometries.removeLast():createFunc();

    scene.add(geometry);
    gone.add(geometry);
    return geometry;
  }

  void restart(){
    while (gone.length) {
      geometries.add(this.gone.pop());
    }
  }

  hideCached(){
    geometries.forEach((geometry) => {
      scene.remove(geometry)
    });
  }
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
