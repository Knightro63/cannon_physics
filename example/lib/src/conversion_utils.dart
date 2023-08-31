import 'dart:async';
import 'dart:io';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter/foundation.dart';

import 'package:flutter_gl/flutter_gl.dart';
import 'package:cannon_physics/cannon_physics.dart' as cannon;
import 'package:three_dart/three_dart.dart' as three;
import 'package:three_dart/three_dart.dart' hide Texture, Color;
import 'package:three_dart_jsm/three_dart_jsm.dart';

extension on cannon.Quaternion{
  Quaternion toQuaternion(){
    return Quaternion(x,y,z,w);
  }
}
extension on cannon.Vec3{
  Vector3 toVector3(){
    return Vector3(x,y,z);
  }
}

class ConversionUtils{
  static three.BufferGeometry shapeToGeometry(cannon.Shape shape,{bool flatShading = true }) {
    switch (shape.type) {
      case cannon.ShapeType.sphere: {
        shape as cannon.Sphere;
        return three.SphereGeometry(shape.radius, 8, 8);
      }

      case cannon.ShapeType.particle: {
        return three.SphereGeometry(0.1, 8, 8);
      }

      case cannon.ShapeType.plane: {
        return three.PlaneGeometry(500, 500, 4, 4);
      }

      case cannon.ShapeType.box: {
        shape as cannon.Box;
        return three.BoxGeometry(shape.halfExtents.x * 2, shape.halfExtents.y * 2, shape.halfExtents.z * 2);
      }

      case cannon.ShapeType.cylinder: {
        shape as cannon.Cylinder;
        return three.CylinderGeometry(shape.radiusTop, shape.radiusBottom, shape.height, shape.numSegments);
      }

      case cannon.ShapeType.convex: {
        shape as cannon.ConvexPolyhedron;
        List<three.Vector3> vertices = [];
        // Add vertices
        for (int i = 0; i < shape.vertices.length; i++) {
          final vertex = shape.vertices[i];
          vertices.add(vertex.toVector3());
        }
        final geometry = three.ConvexGeometry(vertices);
        geometry.computeBoundingSphere();

        if (flatShading) {
          geometry.computeFaceNormals();
        } else {
          geometry.computeVertexNormals();
        }

        return three.ConvexGeometry(vertices);
      }

      case cannon.ShapeType.heightfield: {
        shape as cannon.Heightfield;
        final geometry = three.BufferGeometry();

        final v0 = cannon.Vec3();
        final v1 = cannon.Vec3();
        final v2 = cannon.Vec3();

        List<double> vertices = [];  
        List<num> indices = []; 
        for (int xi = 0; xi < shape.data.length - 1; xi++) {
          for (int yi = 0; yi < shape.data[xi].length - 1; yi++) {
            for (int k = 0; k < 2; k++) {
              shape.getConvexTrianglePillar(xi, yi, k == 0);
              v0.copy(shape.pillarConvex.vertices[0]);
              v1.copy(shape.pillarConvex.vertices[1]);
              v2.copy(shape.pillarConvex.vertices[2]);
              v0.vadd(shape.pillarOffset, v0);
              v1.vadd(shape.pillarOffset, v1);
              v2.vadd(shape.pillarOffset, v2);
              vertices.addAll([
                v0.x, v0.y, v0.z,
                v1.x, v1.y, v1.z,
                v2.x, v2.y, v2.z
              ]);
              final i = vertices.length - 3;
              indices.addAll([i, i + 1, i + 2]);
            }
          }
        }

        geometry.setIndex(indices);
        geometry.setAttribute(
          'position',
            Float32BufferAttribute(Float32Array.from(vertices), 3, false)
        );

        geometry.computeBoundingSphere();

        if (flatShading) {
          geometry.computeFaceNormals();
        } else {
          geometry.computeVertexNormals();
        }

        return geometry;
      }

      case cannon.ShapeType.trimesh: {
        shape as cannon.Trimesh;
        final geometry = three.BufferGeometry();

        List<num> indices = shape.indices;
        List<double> vertices = shape.vertices;

        geometry.setIndex(indices);
        geometry.setAttribute(
            'position', Float32BufferAttribute(Float32Array.from(vertices), 3));

        geometry.computeBoundingSphere();

        if (flatShading) {
          geometry.computeFaceNormals();
        } else {
          geometry.computeVertexNormals();
        }

        return geometry;
      }

      default: {
        throw('Shape not recognized: "${shape.type}"');
      }
    }
  }

  static three.Group bodyToMesh(cannon.Body body, material) {
    final group = three.Group();

    group.position.copy(body.position.toVector3());
    group.quaternion.copy(body.quaternion.toQuaternion());

    final meshes = body.shapes.map((shape){
      final geometry = shapeToGeometry(shape);

      return three.Mesh(geometry, material);
    });
    int i = 0;
    meshes.forEach((three.Mesh mesh){
      final offset = body.shapeOffsets[i];
      final orientation = body.shapeOrientations[i];
      mesh.position.copy(offset);
      mesh.quaternion.copy(orientation.toQuaternion());

      group.add(mesh);
      i++;
    });

    return group;
  }
}