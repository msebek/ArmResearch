#ifndef _MULTIVARIATE_GAUSSIAN_H_
#define _MULTIVARIATE_GAUSSIAN_H_

#include <Eigen/Core>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

template <int N, class T = double>
class MVGaussianDistribution {
public:
    typedef Eigen::Matrix<T, N, 1> Rn;
    typedef Eigen::Matrix<T, N, N> Rnn;
    
    typedef boost::mt19937 RandEngine;
    typedef boost::normal_distribution<T> UnivariateNormal;
    typedef boost::variate_generator<RandEngine&, UnivariateNormal> RandAdapter;
    
    MVGaussianDistribution(Rn &u, Rnn &c, RandEngine &eng);
    ~MVGaussianDistribution();

    MVGaussianDistribution::Rn Sample();
    
protected:

    Rn mean;
    Rnn cov;
    Rnn A; // Used to produce sample
    RandEngine gaussianEngine;
    UnivariateNormal gaussianDistribution;
    RandAdapter gaussianAdapter;
    
};

template <class T>
class MVGaussianDistribution<1, T> {
public:

    typedef boost::mt19937 RandEngine;
    typedef boost::normal_distribution<T> UnivariateNormal;
    typedef boost::variate_generator<RandEngine&, UnivariateNormal> RandAdapter;
    
    MVGaussianDistribution(T u, T c, RandEngine &eng);
    ~MVGaussianDistribution();
    
    T Sample();
    
protected:
    
    T mean;
    T cov;
    T A;
    
    RandEngine gaussianEngine;
    UnivariateNormal gaussianDistribution;
    RandAdapter gaussianAdapter;
    
};

#include "MVGaussianDistribution.cc"

#endif