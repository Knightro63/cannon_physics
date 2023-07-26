/**
 * Defines a physics material.
 */
class Material {
  /**
   * Material name.
   * If options is a string, name will be set to that string.
   * @todo Deprecate this
   */
  String name = '';
  /** Material id. */
  late int id;
  /**
   * Friction for this material.
   * If non-negative, it will be used instead of the friction given by ContactMaterials. If there's no matching ContactMaterial, the value from `defaultContactMaterial` in the World will be used.
   */
  num friction;
  /**
   * Restitution for this material.
   * If non-negative, it will be used instead of the restitution given by ContactMaterials. If there's no matching ContactMaterial, the value from `defaultContactMaterial` in the World will be used.
   */
  num restitution;

  static int idCounter = 0;

  Material({
    /**
     * Friction for this material.
     * If non-negative, it will be used instead of the friction given by ContactMaterials. If there's no matching ContactMaterial, the value from `defaultContactMaterial` in the World will be used.
     */
    this.friction = -1,
    /**
     * Restitution for this material.
     * If non-negative, it will be used instead of the restitution given by ContactMaterials. If there's no matching ContactMaterial, the value from `defaultContactMaterial` in the World will be used.
     */
    this.restitution = -1,
    this.name = ''
  }) {
    this.id = Material.idCounter++;
  }
}
