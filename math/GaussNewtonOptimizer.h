#include <Eigen/Core>
#include <vector>
#ifndef GAUSS_NEWTON_OPTIMIZER_H
#define GAUSS_NEWTON_OPTIMIZER_H

using namespace std;

  template <class M, class C>
  class GaussNewtonOptimizer
  {
  private:
    const unsigned int numOfMeasurements; /**< The number of measurements. */
    const unsigned int numOfParameters; /**< The number of parameters. */
    Eigen::VectorXf currentParameters; /**< The vector (Nx1-matrix) containing the current parameters. */
    Eigen::VectorXf currentValues; /**< The vector (Nx1-matrix) containing the current error values for all measurements. */
    const vector<M>& measurements; /**< A reference to the vector containing all measurements. */
    C& object; /**< The object used to call the error function. */
    float(C::*pFunction)(const M& measurement, const Eigen::VectorXf& parameters);  /**< A pointer to the error function. */
    const float delta; /**< The delta used to approximate the partial derivatives of the Jacobian. */

  public:
	GaussNewtonOptimizer();
    GaussNewtonOptimizer(const Eigen::VectorXf& parameters, const vector<M>& measurements, C& object, float(C::*pFunction)(const M& measurement, const Eigen::VectorXf& parameters));

    /**
     * This method executes one iteration of the Gauss-Newton algorithm.
     * The new parameter vector is computed by a_i+1 = a_i - (D^T * D)^-1 * D^T * r
     * where D is the Jacobian, a is the parameter vector and r is the vector containing the current error values.
     * @return The sum of absolute differences between the old and the new parameter vector.
     */
	 float iterate();

    /**
     * The method returns the current parameter vector.
     * @return The current parameter vector.
     */
   Eigen::VectorXf getParameters() const;
  };

	template <class M, class C>
    GaussNewtonOptimizer<M,C>::GaussNewtonOptimizer(const Eigen::VectorXf& parameters, const vector<M>& measurements, C& object, float(C::*pFunction)(const M& measurement, const Eigen::VectorXf& parameters))
      : numOfMeasurements(measurements.size()), numOfParameters(parameters.size()), currentParameters(numOfParameters, 1), currentValues(numOfMeasurements, 1), measurements(measurements), object(object), pFunction(pFunction), delta(0.1f)
    {
      currentParameters = parameters;
      
      for(unsigned int i = 0; i < numOfMeasurements; ++i)
      {
        currentValues[i] = (object.*pFunction)(measurements[i], getParameters());
      }
    }

	template <class M, class C>
	float GaussNewtonOptimizer<M,C>::iterate()
    {
      // build jacobi matrix
      Eigen::MatrixXf jacobiMatrix(numOfMeasurements, numOfParameters);
      for(unsigned int j = 0; j < numOfParameters; ++j)
      {
        // the first derivative is approximated using values slightly above and below the current value
        const float oldParameter = currentParameters[j];
        const float parameterAbove = oldParameter + delta;
        const float parameterBelow = oldParameter - delta;
        for(unsigned int i = 0; i < numOfMeasurements; ++i)
        {
          // approximate first derivation numerically
          currentParameters[j] = parameterAbove;
          const float valueAbove = (object.*pFunction)(measurements[i], getParameters());
          currentParameters[j] = parameterBelow;
          const float valueBelow = (object.*pFunction)(measurements[i], getParameters());
          const float derivation = (valueAbove - valueBelow) / (2.0f * delta);
          jacobiMatrix(i, j) = derivation;
        }
        currentParameters[j] = oldParameter;
      }

      try
      {
        Eigen::VectorXf result = (jacobiMatrix.transpose() * jacobiMatrix).inverse() * jacobiMatrix.transpose() * currentValues;
        currentParameters -= (result * .001f);

        for(unsigned int i = 0; i < numOfMeasurements; ++i)
        {
          currentValues[i] = (object.*pFunction)(measurements[i], currentParameters);
        }

        float sum = 0;
        for(unsigned int i = 0; i < numOfParameters; ++i)
        {
          sum += abs(result[i]);
        }
        return sum;
      }
      catch(...)
      {
        return 0.0f;
      }

    }

	template <class M, class C>
    Eigen::VectorXf GaussNewtonOptimizer<M,C>::getParameters() const {
      return currentParameters;
    }

#endif
