class OverlapKeeper {
  List<int> current = [];
  List<int> previous = [];

  OverlapKeeper([List<int>? current,List<int>? previous]){
    this.current = current != null?List.from(current):List.empty(growable: true);
    this.previous = previous != null?List.from(previous):List.empty(growable: true);
  }

  /// getKey
  int getKey(int i, int j) {
    if (j < i) {
      final temp = j;
      j = i;
      i = temp;
    }
    return (i << 16) | j;
  }

  /// set
  void set(int i,int j){
    // Insertion sort. This way the diff will have linear complexity.
    final key = getKey(i, j);
    final current = this.current;
    int index = 0;
    if(current.isNotEmpty){
      while (key > current[index]) {
        index++;
      }
      if (key == current[index]) {
        return; // Pair was already added
      }
      for (int j = current.length - 1; j >= index; j--) {
        current[j + 1] = current[j];
      }
      current[index] = key;
    }
  }

  /// tick
  void tick() {
    final tmp = current;
    current = previous;
    previous = tmp;
    current.clear();
  }

  /// getDiff
  void getDiff(List<int> additions, List<int> removals) {
    final a = current;
    final b = previous;
    final al = a.length;
    final bl = b.length;

    int j = 0;
    for (int i = 0; i < al; i++) {
      bool found = false;
      final keyA = a[i];
      while (keyA > b[j]) {
        j++;
      }
      found = keyA == b[j];

      if (!found) {
        print('here');
        _unpackAndPush(additions, keyA);
      }
    }
    j = 0;
    for (int i = 0; i < bl; i++) {
      bool found = false;
      final keyB = b[i];
      while (keyB > a[j]) {
        j++;
      }
      found = a[j] == keyB;

      if (!found) {
        print('here');
        _unpackAndPush(removals, keyB);
      }
    }
  }
  void _unpackAndPush(List<int> array, int key) {
    array.addAll([(key & 0xffff0000) >> 16, key & 0x0000ffff]);
  }
}
