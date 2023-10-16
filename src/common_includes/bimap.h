#ifndef BIMAP_H
#define BIMAP_H

#include <map>
#include <unordered_map>

template <class T, class F>
class bimap {
public:
  void add(T t, F f) {
    _map[t] = f;
    _invMap[f] = t;
  }

  T getLeft(F f) {return _invMap[f];}

  F getRight(T t) {return _map[t];}

  bool containsLeft(T t) { return _map.count(t) > 0;}

  bool containsRight(F f) {return _invMap.count(f) > 0;}

  void remove(T t, F f) {
    _map.erase(t);
    _invMap.erase(f);
  }

  void removeByLeft(T t) {remove(t, _map[t]);}

  void removeByRight(F f) {remove(_invMap[f], f);}

private:
  std::map<T, F> _map;
  std::map<F, T> _invMap;
};

template <class T, class F>
class unordered_bimap {
public:
  void add(T t, F f) {
    _map[t] = f;
    _invMap[f] = t;
  }

  T getLeft(F f) {return _invMap[f];}

  F getRight(T t) {return _map[t];}

  bool containsLeft(T t) { return _map.count(t) > 0;}

  bool containsRight(F f) {return _invMap.count(f) > 0;}

  void remove(T t, F f) {
    _map.erase(t);
    _invMap.erase(f);
  }

  void removeByLeft(T t) {remove(t, _map[t]);}

  void removeByRight(F f) {remove(_invMap[f], f);}

private:
  std::unordered_map<T, F> _map;
  std::unordered_map<F, T> _invMap;
};

#endif