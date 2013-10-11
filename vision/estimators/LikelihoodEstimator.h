#ifndef LIKELIHOOD_ESTIMATOR_H
#define LIKELIHOOD_ESTIMATOR_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>
#include <stdarg.h>
#include <memory/TextLogger.h>

#define normpdf(x,u,E,k) ( pow(2 * M_PI, -k / 2.0) * 1.0 / sqrt(E.determinant()) * exp(-.5 *\
      (double)((x - u).transpose() * E.inverse() * (x - u))\
      )\
    )

template<int N>
class LikelihoodEstimator {
  private:
    typedef Eigen::Matrix<double, N, 1> Vector;
    typedef Eigen::Matrix<double, N, N> Matrix;
    double prob_;
    Vector mean_, measurement_, stddev_;
    Matrix cov_;

  protected:
    std::vector<std::string> names_;
    virtual void setNames() = 0;

  public:
    void setMean(double first, ...) {
      va_list arguments;
      va_start(arguments, first);
      mean_[0] = first;
      for ( int i = 1; i < N; i++ )
        mean_[i] = va_arg(arguments, double);
      va_end(arguments);
    }
    void setStdDev(double first, ...) {
      Vector v;
      va_list arguments;
      va_start(arguments, first);
      v[0] = first * first;
      stddev_[0] = first;
      for ( int i = 1; i < N; i++ ) {
        v[i] = va_arg(arguments, double);
        stddev_[i] = v[i];
        v[i] *= v[i];
      }
      va_end(arguments);
      cov_ = v.asDiagonal();
    }
    double getLikelihood(double first, ...) {
      Vector v;
      va_list arguments;
      va_start(arguments, first);
      v[0] = first;
      for ( int i = 1; i < N; i++ )
        v[i] = va_arg(arguments, double);
      va_end(arguments);
      measurement_ = v;
      double base = normpdf(mean_, mean_, cov_, N);
      prob_ = normpdf(v, mean_, cov_, N);
      prob_ /= base;
      if(isnan(prob_)) prob_ = 0.0;
      return prob_;
    }
    void printLast() {
      for(unsigned int i = 0; i < names_.size(); i++) {
        printf("%s: %05.2f (u=%2.1f,s=%2.1f),", names_[i].c_str(), measurement_[i], mean_[i], stddev_[i]);
      }
      printf("p: %.2f\n", prob_);
    }

    void logLast(int num, TextLogger *textlogger) {
#ifndef ALLOW_DEBUG_LOG
      return;
#endif
      for(unsigned int i = 0; i < names_.size(); i++) {
        visionLog((num,"%s: %05.2f,", names_[i].c_str(), measurement_[i]));
      }
      visionLog((num,"p: %.2f", prob_));
    }
};

#endif

