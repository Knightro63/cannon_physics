String get getKey = (int i, int j) (i < j ? '$i-$j' : '$j-$i');

/**
 * TupleDictionary
 */
class TupleDictionary {
  data: { [id: string]: any; keys: string[] } = { keys: [] }

  /** get */
  dynamic get(int i, int j){
    final key = getKey(i, j);
    return this.data[key];
  }

  /** set */
  void set(int i, int j, dynamic value){
    final key = getKey(i, j);

    // Check if key already exists
    if (get(i, j) != null) {
      data.keys.add(key);
    }

    this.data[key] = value;
  }

  /** delete */
  void delete(int i, int j) {
    const key = getKey(i, j);
    const index = this.data.keys.indexOf(key);
    if (index != -1) {
      this.data.keys.splice(index, 1);
    }
    delete this.data[key];
  }

  /** reset */
  void reset() {
    final data = this.data;
    final keys = data.keys;
    while (keys.length > 0) {
      const key = keys.pop()!;
      delete data[key];
    }
  }
}
