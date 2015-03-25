// Implementation for MVGaussianDistribution

#include <Eigen/Cholesky>
#include <boost/random/uniform_01.hpp>

template <int N, class T>
MVGaussianDistribution<N, T>::MVGaussianDistribution(Rn &u, Rnn &c, RandEngine &eng) :
    mean(u), cov(c), A(cov.llt().matrixL()), gaussianEngine(eng),
    gaussianDistribution(0.0, 1.0), gaussianAdapter(gaussianEngine, gaussianDistribution) {}

template <int N, class T>
MVGaussianDistribution<N, T>::~MVGaussianDistribution() {}

template <int N, class T>
typename MVGaussianDistribution<N, T>::Rn MVGaussianDistribution<N, T>::Sample() {
    
    Rn samples;
    for(unsigned int i = 0; i < N; i++) {
        samples(i) = gaussianAdapter();
    }

    Rn result = mean + A*samples;
    return result;
    
}

template <class T>
MVGaussianDistribution<1, T>::MVGaussianDistribution(T u, T c, RandEngine &eng) :
    mean(u), cov(c), A(sqrt(c)), gaussianEngine(eng), gaussianDistribution(0.0, 1.0),
    gaussianAdapter(gaussianEngine, gaussianDistribution) {}

template <class T>
MVGaussianDistribution<1, T>::~MVGaussianDistribution() {}

template <class T>
T MVGaussianDistribution<1, T>::Sample() {

    return A*gaussianAdapter() + mean;
    
}