/// Defines a physics material.
class Material {
  /// Material name.
  /// If options is a string, name will be set to that string.
  /// @todo Deprecate this
  String name = '';
  /// Material id. */
  late int id;

  /// Friction for this material.
  /// If non-negative, it will be used instead of the friction given by ContactMaterials. If there's no matching ContactMaterial, the value from `defaultContactMaterial` in the World will be used.
  double friction;
  /// Restitution for this material.
  /// If non-negative, it will be used instead of the restitution given by ContactMaterials. If there's no matching ContactMaterial, the value from `defaultContactMaterial` in the World will be used.
  double restitution;

  static int idCounter = 0;

  Material({
    this.friction = -1,
    this.restitution = -1,
    this.name = ''
  }) {
    id = Material.idCounter++;
  }
}
