#include <vision/structures/Blob.h>

bool sortBlobAreaPredicate(const Blob& left, const Blob& right) {
  return left.dx * left.dy < right.dx * right.dy;
}
