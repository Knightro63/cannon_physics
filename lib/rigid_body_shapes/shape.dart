import '../math/vec3.dart';
import '../math/quaternion.dart';
import '../objects/rigid_body.dart';
import '../material/material.dart';

/// The available shape types.
enum ShapeType{
  sphere,
  plane,
  box,
  convex,
  heightfield,
  particle,
  cylinder,
  trimesh,
}

/**
 * ShapeType
 */
//export type ShapeType = typeof SHAPE_TYPES[keyof typeof SHAPE_TYPES]

//export type ShapeOptions = ConstructorParameters<typeof Shape>[0]

/// Base class for shapes
class Shape {
  /// Identifier of the Shape.
  late int id;

  ///The type of this shape. Must be set to an int > 0 by subclasses.
  ShapeType type = ShapeType.sphere;

  /// The local bounding sphere radius of this shape.
  double boundingSphereRadius;

  /// Whether to produce contact forces when in contact with other bodies. Note that contacts will be generated, but they will be disabled.
  bool collisionResponse;

  int collisionFilterGroup;
  int collisionFilterMask;

  /// Optional material of the shape that regulates contact properties.
  Material? material;

  /// The body to which the shape is added to.
  Body? body;

  static int idCounter = 0;

  Shape({
    /**
     * The type of this shape.
     */
    this.type =ShapeType.sphere,
    /**
     * Whether to produce contact forces when in contact with other bodies.
     * @default true
     */
    this.collisionResponse = true,
    /**
     * @default 1
     */
    this.collisionFilterGroup = -1,
    /**
     * @default -1
     */
    this.collisionFilterMask = -1,
    /**
     * Optional material of the shape that regulates contact properties.
     * @default null
     * @todo check this, the material is passed to the body, right?
     */
    this.boundingSphereRadius = 0,
    this.material,
    this.body
  }) {
    id = Shape.idCounter++;
  }

  /// Computes the bounding sphere radius.
  /// The result is stored in the property `.boundingSphereRadius`
  void updateBoundingSphereRadius() {
    throw'computeBoundingSphereRadius() not implemented for shape type $type';
  }

  /// Get the volume of this shape
  double volume() {
    throw 'volume() not implemented for shape type $type';
  }

  /// Calculates the inertia in the local frame for this shape.
  /// @see http://en.wikipedia.org/wiki/List_of_moments_of_inertia
  Vec3 calculateLocalInertia(double mass, Vec3 target) {
    throw 'calculateLocalInertia() not implemented for shape type $type';
  }

  /// @todo use abstract for these kind of methods
  void calculateWorldAABB(Vec3 pos, Quaternion quat, Vec3 min, Vec3 max) {
    throw'calculateWorldAABB() not implemented for shape type $type';
  }
}
