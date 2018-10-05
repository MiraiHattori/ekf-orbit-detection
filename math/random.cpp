#include "math/random.hpp"

namespace Math
{
double normalRand(const double& mean, const double& stddev)
{
  std::random_device rd{};
  std::mt19937 gen{ rd() };
  std::normal_distribution<> d{ mean, stddev };
  return d(gen);
}
double impulsiveNoise(const double& mean, const double& stddev, const double& prob)
{
  std::random_device rd{};
  std::mt19937 gen{ rd() };
  std::uniform_real_distribution<> d{ 0.0, 1.0 };
  if (d(gen) <= prob)
  {
    return normalRand(mean, stddev);
  }
  else
  {
    return 0.0;
  }
}
}  // namespace Math
