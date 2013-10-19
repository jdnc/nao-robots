#include <vision/structures/Blob.h>

bool sortBlobAreaPredicate(const Blob& left, const Blob& right) {
  return left.dx * left.dy < right.dx * right.dy;
}

bool areaOutOfRangePredicate(const Blob& b){
   double area = b.dx * b.dy;
   return area <= 350 || area >= 17000;
}
