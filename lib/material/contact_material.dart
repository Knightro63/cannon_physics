import '../material/material.dart';

/**
 * Defines what happens when two materials meet.
 * @todo Refactor materials to materialA and materialB
 */
class ContactMaterial {
  /**
   * Identifier of this material.
   */
  late int id;
  /**
   * Participating materials.
   */
  late List<Material> materials;
  /**
   * Friction coefficient.
   * @default 0.3
   */
  double friction;
  /**
   * Restitution coefficient.
   * @default 0.3
   */
  double restitution;
  /**
   * Stiffness of the produced contact equations.
   * @default 1e7
   */
  double contactEquationStiffness;
  /**
   * Relaxation time of the produced contact equations.
   * @default 3
   */
  double contactEquationRelaxation;
  /**
   * Stiffness of the produced friction equations.
   * @default 1e7
   */
  double frictionEquationStiffness;
  /**
   * Relaxation time of the produced friction equations
   * @default 3
   */
  double frictionEquationRelaxation;

  static int idCounter = 0;

  ContactMaterial(
    Material m1,
    Material m2,
    {
      /**
       * Friction coefficient.
       * @default 0.3
       */
      this.friction = 0.3,
      /**
       * Restitution coefficient.
       * @default 0.3
       */
      this.restitution = 0.3,
      /**
       * Stiffness of the produced contact equations.
       * @default 1e7
       */
      this.contactEquationStiffness = 1e7,
      /**
       * Relaxation time of the produced contact equations.
       * @default 3
       */
      this.contactEquationRelaxation = 3,
      /**
       * Stiffness of the produced friction equations.
       * @default 1e7
       */
      this.frictionEquationStiffness = 1e7,
      /**
       * Relaxation time of the produced friction equations
       * @default 3
       */
      this.frictionEquationRelaxation = 3
    }
  ) {
    id = ContactMaterial.idCounter++;
    materials = [m1, m2];
  }
}
