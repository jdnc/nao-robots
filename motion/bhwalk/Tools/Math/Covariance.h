
#pragma once

#include "Matrix2x2.h"
#include <cmath>

namespace Covariance
{
/**
 * Creates a covariance of 2 independend random variables.
 * @param dev A vector containing the standard deviations of the random
 *            variables.
 */
inline const Matrix2x2BH<> create(const Vector2BH<> dev)
{
  return Matrix2x2BH<>(Vector2BH<>(dev.x * dev.x, 0.0f), Vector2BH<>(0.0f, dev.y * dev.y));
}

/**
 * Creates a covariance of 2 dependend random variables with known rotation.
 * @param dev A vector containing the standard deviations of the random
 *            variables.
 * @param angle The rotation in radians.
 */
inline const Matrix2x2BH<> create(const Vector2BH<>& dev, const float angle)
{
  const float sinRotation = sin(angle);
  const float cosRotation = cos(angle);
  const Matrix2x2BH<> r = Matrix2x2BH<>(
                          Vector2BH<>(cosRotation, sinRotation),
                          Vector2BH<>(-sinRotation, cosRotation));
  return r * create(dev) * r.transpose();
}

/**
 * Creates a covariance of 2 dependend random variables with known rotation.
 * @param xDev The standard deviations of the random variables in x direction.
 * @param yDev The standard deviations of the random variables in y direction.
 * @param angle The rotation in radians.
 */
inline const Matrix2x2BH<> create(const float xDev, const float yDev, const float angle)
{
  const float sinRotation = sin(angle);
  const float cosRotation = cos(angle);
  const Matrix2x2BH<> r = Matrix2x2BH<>(
                          Vector2BH<>(cosRotation, sinRotation),
                          Vector2BH<>(-sinRotation, cosRotation));
  return r * Matrix2x2BH<>(Vector2BH<>(xDev * xDev, 0.0f), Vector2BH<>(0.0f, yDev * yDev)) * r.transpose();
}

/**
 * Calculates an ellipse of equiprobable points of a zero centered covariance.
 * This is usually used for debug drawings.
 * @param covariance The covariance matrix.
 * @param axis1 The major axis of the corresponding ellipse.
 * @param axis2 The minor axis of the corresponding ellipse.
 * @param angle The rotation of the ellipse.
 * @param factor A scaling factor for the axes.
 */
inline void errorEllipse(
  const Matrix2x2BH<>& covariance,
  float& axis1,
  float& axis2,
  float& angle,
  const float factor = 1.0f)
{
  const float cov012 = covariance.c[0][1] * covariance.c[0][1];
  const float varianceDiff = covariance.c[0][0] - covariance.c[1][1];
  const float varianceDiff2 = varianceDiff * varianceDiff;
  const float varianceSum = covariance.c[0][0] + covariance.c[1][1];
  const float root = sqrt(varianceDiff2 + 4.0f * cov012);
  const float eigenValue1 = 0.5f * (varianceSum + root);
  const float eigenValue2 = 0.5f * (varianceSum - root);

  angle = 0.5f * atan2(2.0f * covariance.c[0][1], varianceDiff);
  axis1 = 2.0f * sqrt(factor * eigenValue1);
  axis2 = 2.0f * sqrt(factor * eigenValue2);
}

/**
 * The cholesky decomposition L of a covariance matrix C such that LL^t = C.
 */
inline Matrix2x2BH<> choleskyDecomposition(const Matrix2x2BH<>& c)
{
  Matrix2x2BH<> L;
  L[0][0] = sqrtf(c[0][0]);
  L[0][1] = c[0][1] / L[0][0];
  L[1][1] = sqrtf(c[1][1] - L[0][1] * L[0][1]);
  return L;
}

/**
 * The squared Mahalanobis distance between two points a and b. The
 * Mahalanobis distance is the euclidean distance with the components
 * weighted by a covariance matrix.
 */
inline float squaredMahalanobisDistance(const Vector2BH<>& a, const Matrix2x2BH<>& c, const Vector2BH<>& b)
{
  const Vector2BH<> diff(a - b);
  return diff * (c.invert() * diff);
}
};
