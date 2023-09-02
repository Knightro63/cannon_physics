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
    data[key] = value;
  }

  void delete(int i, int j) {
    final key = getKey(i, j);
    data.remove(key);
  }

  void reset() {
    final data = this.data;
    data.clear();
  }
}
