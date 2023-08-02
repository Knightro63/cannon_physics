String getKey(int i, int j){
  return (i < j ? '$i-$j' : '$j-$i');
}

class TupleDictionary {
  Map<String,dynamic> data = {};//: { [id: string]: any; keys: string[] } = { keys: [] }

  dynamic get(int i, int j){
    final key = getKey(i, j);
    return data[key];
  }

  void set(int i, int j, dynamic value){
    final key = getKey(i, j);

    // Check if key already exists
    // if (get(i, j) != null) {
    //   data.keys.add(key);
    // }

    data[key] = value;
  }

  void delete(int i, int j) {
    final key = getKey(i, j);
    // final index = data.keys.indexOf(key);
    // if (index != -1) {
    //   data.keys.splice(index, 1);
    // }
    data.remove(key);//delete this.data[key];
  }

  void reset() {
    final data = this.data;
    //final keys = data.keys;
    data.clear();
    // while (keys.isNotEmpty) {
    //   final key = keys.last;
    //   data.remove(key);//delete data[key];
    // }
  }
}
