enum AxisIndex{x,y,z}

class Utils {
  /**
   * Extend an options object with default values.
   * @param options The options object. May be falsy: in this case, a new object is created and returned.
   * @param defaults An object containing default values.
   * @return The modified options object.
   */
  static Map<String,dynamic> getDefaults(Map<String,dynamic> options, Map<String,dynamic> defaults){
    for (String key in defaults.keys) {
      if (options.containsKey(key)) {
        options[key] = defaults[key];
      }
    }

    return options;
  }
}
