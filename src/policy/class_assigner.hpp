// Interface and helpers for assigning traffic classes at injection.
#ifndef _CLASS_ASSIGNER_HPP_
#define _CLASS_ASSIGNER_HPP_

#include <functional>

#include "policy/telemetry.hpp"

class ClassAssigner {
public:
  virtual ~ClassAssigner() {}
  virtual int Assign(int src, int dest, int suggested_class,
                     const PolicyTelemetry &t) = 0;
};

class StaticClassAssigner : public ClassAssigner {
public:
  explicit StaticClassAssigner(int classes) : _classes(classes) {}
  int Assign(int, int, int suggested_class,
             const PolicyTelemetry &) override {
    if (suggested_class < 0) return 0;
    if (suggested_class >= _classes) return _classes - 1;
    return suggested_class;
  }

private:
  int _classes;
};

class CustomClassAssigner : public ClassAssigner {
public:
  explicit CustomClassAssigner(
      std::function<int(int, int, int, const PolicyTelemetry &)> fn)
      : _fn(fn) {}
  int Assign(int src, int dest, int suggested_class,
             const PolicyTelemetry &t) override {
    return _fn ? _fn(src, dest, suggested_class, t) : suggested_class;
  }

private:
  std::function<int(int, int, int, const PolicyTelemetry &)> _fn;
};

#endif
