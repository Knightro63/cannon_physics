/**
 * Base class for objects that dispatches events.
 */
class EventTarget {
  Map<String,List<Function>>? _listeners;//private _listeners: Record<string, Function[]> | null;

  /**
   * Add an event listener
   * @return The self object, for chainability.
   */
  EventTarget addEventListener(String type, void Function() listener) {
    if (_listeners == null) {
      _listeners = {};
    }
    final listeners = _listeners;
    if (listeners?[type] == null) {
      listeners![type] = [];
    }
    if (!listeners![type].contains(listener)) {
      listeners[type] = listener;
    }
    return this;
  }

  /**
   * Check if an event listener is added
   */
  bool hasEventListener(String type, void Function() listener) {
    if (_listeners == null) {
      return false;
    }
    final listeners = _listeners;
    if (listeners[type] != null && listeners[type].contains(listener)) {
      return true;
    }
    return false;
  }

  /**
   * Check if any event listener of the given type is added
   */
  bool hasAnyEventListener(String type){
    if (this._listeners == null) {
      return false;
    }
    final listeners = _listeners;
    return listeners?[type] != null;
  }

  /**
   * Remove an event listener
   * @return The self object, for chainability.
   */
  EventTarget removeEventListener(String type, void Function() listener){
    if (_listeners == null) {
      return this;
    }
    final listeners = _listeners;
    if (listeners?[type] == null) {
      return this;
    }
    final index = listeners[type].indexOf(listener);
    if (index != -1) {
      listeners[type].splice(index, 1);
    }
    return this;
  }

  /**
   * Emit an event.
   * @return The self object, for chainability.
   */
  EventTarget dispatchEvent(event){
    if (_listeners == null) {
      return this;
    }
    final listeners = _listeners;
    final listenerArray = listeners?[event.type];
    if (listenerArray != null) {
      event.target = this;
      for (int i = 0, l = listenerArray.length; i < l; i++) {
        listenerArray[i].call(this, event);
      }
    }
    return this;
  }
}
