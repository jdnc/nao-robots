#ifndef COORDINATES_H
#define COORDINATES_H

struct Coordinates {
  int x;
  int y;
  Coordinates(int x, int y) : x(x), y(y) { }
  Coordinates() : x(0), y(0) { }
};

#endif
