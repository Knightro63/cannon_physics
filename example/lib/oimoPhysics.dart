import 'dart:async';

import 'package:flutter_gl/native-array/index.dart';
import 'package:three_dart/three_dart.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;

extension on cannon.Quaternion{
  Quaternion toQuaternion(){
    return Quaternion(x,y,z,w);
  }
}

class CannonPhysics{
  CannonPhysics();

  int frameRate = 60;
  List<Mesh> meshes = [];
  WeakMap meshMap = WeakMap();
  cannon.World world = cannon.World(); //2, cannon.Vec3( 0, - 9.8, 0 )

  cannon.Shape? getShape(BufferGeometry geometry ) {
    Map<String, dynamic> parameters = geometry.parameters!; // TODO change type to is*
    if (geometry.type == 'BoxGeometry') {
      double sx = (parameters['width'] ?? 1) /2;
      double sy = (parameters['height'] ?? 1) / 2;
      double sz = (parameters['depth'] ?? 1) / 2;
      return cannon.Box(cannon.ShapeConfig(),sx,sy,sz);
    }
    else if ( geometry.type == 'SphereGeometry' || geometry.type == 'IcosahedronGeometry') {
      double radius = parameters['radius'] ?? 1;
      return cannon.Sphere(cannon.ShapeConfig(),radius);
    }

    return null;
  }
  void addMesh(Mesh mesh, [double mass = 0]) {
    cannon.Shape? shape = getShape(mesh.geometry!);
    print(shape);
    if(shape != null) {
      if (mesh is InstancedMesh) {
        handleInstancedMesh(mesh, mass, shape.type);
      } 
      else if(mesh is Mesh) {
        handleMesh( mesh, mass, shape.type );
      }
    }
  }

  void handleMesh(Mesh mesh,double mass,cannon.Shapes shape) {
    // Is all the physics setting for rigidbody
    List<num> config = [
      100.0, // The density of the shape.
      0.4, // The coefficient of friction of the shape.
      0.2, // The coefficient of restitution of the shape.
      1<<0, // The bits of the collision groups to which the shape belongs.
      0xffffffff // The bits of the collision groups with which the shape collides.
    ];
    cannon.RigidBody body = world.add(cannon.ObjectConfigure(
      shapes:[shape], 
      size:[10.0,1.0,10.0], 
      position:[mesh.position.x, mesh.position.y, mesh.position.z], 
      //'move':mass != 0, 
      //world:world,
      //'config': config,
      name: 'floor'
    )) as cannon.RigidBody;
    if ( mass > 0 ) {
      meshes.add( mesh );
      meshMap.set( mesh, body );
    }
  }

  void handleInstancedMesh(Mesh mesh, double mass,cannon.Shapes shape) {
    NativeArray<num> array = mesh.instanceMatrix!.array;
    List<cannon.RigidBody> bodies = [];
    List<num> config = [
      10.0, // The density of the shape.
      0.4, // The coefficient of friction of the shape.
      0.2, // The coefficient of restitution of the shape.
      1<<1, // The bits of the collision groups to which the shape belongs.
      0xffffffff // The bits of the collision groups with which the shape collides.
    ];
    for (int i = 0; i < mesh.count!; i++) {
      int index = i * 16;
      cannon.RigidBody body = world.add(cannon.ObjectConfigure(
        shapes:[shape], 
        size:[0.1,0.1,0.1], 
        position:[array[ index + 12 ].toDouble(), array[ index + 13 ].toDouble(), array[ index + 14 ].toDouble()], 
        move:mass != 0, 
        //'sleep': false,
        //'world':world,
        //'config': config,
        //'name': 'object_$i'
      )) as cannon.RigidBody;
      bodies.add(body);
    }
    if ( mass > 0 ) {
      meshes.add( mesh );
      meshMap.set( mesh, bodies );
    }
  } 


  void setMeshPosition(Mesh mesh, Vector3 position, [int index = 0]) {
    if(mesh is InstancedMesh) {
      List<cannon.RigidBody>? bodies = meshMap.get( mesh );//WeakMap<dynamic, dynamic>
      if(bodies != null){
        cannon.RigidBody body = bodies[ index ];
        body.setPosition(cannon.Vec3( position.x, position.y, position.z ));
      }
    }
    else if(mesh is Mesh) {
      dynamic body = meshMap.get( mesh );
      body.setPosition( cannon.Vec3( position.x, position.y, position.z ) );
    }
  } //

  int lastTime = 0;

  void step() {
    int time = DateTime.now().millisecondsSinceEpoch;

    if ( lastTime > 0 ) {
      // console.time( 'world.step' );
      world.step(); // console.timeEnd( 'world.step' ); //1/frameRate
    }

    lastTime = time; //

    for(int i = 0; i < meshes.length;i++){
      Mesh mesh = meshes[i];
      List<cannon.RigidBody> bodies = meshMap.get(mesh);
      for (int j = 0; j < bodies.length; j++){
        cannon.RigidBody body = bodies[j];
        
        if(!body.sleeping){
          mesh.position.copy(body.getPosition());
          mesh.quaternion.copy(body.getQuaternion().toQuaternion());

          // reset position
          if(mesh.position.y<-10){
            body.resetPosition(0, Math.random() + 1, 0 );
          }
        }
      }
    }
  }
}