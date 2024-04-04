# cannon_physics

[![Pub Version](https://img.shields.io/pub/v/cannon_physics)](https://pub.dev/packages/cannon_physics)
[![analysis](https://github.com/Knightro63/cannon_physics/actions/workflows/flutter.yml/badge.svg)](https://github.com/Knightro63/cannon_physics/actions/)
[![License: BSD](https://img.shields.io/badge/license-MIT-purple.svg)](https://opensource.org/licenses/BSD)

A 3D physics engine for dart (based on cannon.js) that allows users to add physics support to their 3d projects.

<picture>
  <img alt="Image of ball hitting a cloth." src="https://github.com/Knightro63/cannon_physics/blob/de7b2dad5cf7e94c13c8ca96dbc887e4b834d7f9/examples/assets/images/cloth.png">
</picture>

This is a dart conversion of cannon.js and cannon-es, originally created by Stefan Hedman [@schteppe](https://github.com/schteppe) and has a maintaind fork by Poimandres [@pmndrs](https://github.com/pmndrs).

### Getting started

To get started add cannon_physics, three_dart, and three_dart_jsm to your pubspec.yaml file.

The Cannon World is the main scene that has all of the objects that will be manipulated to the scene. To get started add the cannon world then all of the objects in it.

If there is no shapes or type in the ObjectConfigure class it will not work. If you need a RigidBody use shapes. If you need a Joint use type.

```dart
    world = cannon.World();

    const mass = 1.0;
    const size = 0.25;
    final cylinderShape = cannon.Cylinder(
        radiusTop: size, 
        radiusBottom: size, 
        height: size * 2, 
        numSegments: 10
    );
    final cylinderBody = cannon.Body(mass:mass);
    cylinderBody.addShape(cylinderShape);
    cylinderBody.position.set(size * 2, size + 1, size * 2);
    world.addBody(cylinderBody);
```

## Usage

This project is a basic physics engine for three_dart. This package includes RigidBodies, Constraints, and Joints.

## Features

* Rigid body dynamics
* Discrete collision detection
* Contacts, friction and restitution
* Constraints
   * PointToPoint (a.k.a. ball/socket joint)
   * Distance
   * Hinge (with optional motor)
   * Lock
   * ConeTwist
* Gauss-Seidel constraint solver and an island split algorithm
* Collision filters
* Body sleeping
* Experimental SPH / fluid support
* Various shapes and collision algorithms (see table below)

|             | Sphere | Plane | Box | Convex | Particle | Heightfield | Trimesh |
| :-----------|:------:|:-----:|:---:|:------:|:--------:|:-----------:|:-------:|
| Sphere      | Yes    | Yes   | Yes | Yes    | Yes      | Yes         | Yes     |
| Plane       | -      | -     | Yes | Yes    | Yes      | -           | Yes     |
| Box         | -      | -     | Yes | Yes    | Yes      | Yes         | (todo)  |
| Cylinder    | -      | -     | Yes | Yes    | Yes      | Yes         | (todo)  |
| Convex      | -      | -     | -   | Yes    | Yes      | Yes         | (todo)  |
| Particle    | -      | -     | -   | -      | -        | Yes         | Yes     |
| Heightfield | -      | -     | -   | -      | -        | -           | (todo)  |
| Trimesh     | -      | -     | -   | -      | -        | -           | (todo)  |

## Example

Find the example app [here](https://github.com/Knightro63/cannon_physics/tree/main/example) and the current web version of the code [here](https://knightro63.github.io/cannon_physics/).

## Contributing

Contributions are welcome.
In case of any problems look at [existing issues](https://github.com/Knightro63/cannon_physics/issues), if you cannot find anything related to your problem then open an issue.
Create an issue before opening a [pull request](https://github.com/Knightro63/cannon_physics/pulls) for non trivial fixes.
In case of trivial fixes open a [pull request](https://github.com/Knightro63/cannon_physics/pulls) directly.

## Additional Information

This plugin is only for performing basic physics. While this can be used as a standalone project it does not render scenes.