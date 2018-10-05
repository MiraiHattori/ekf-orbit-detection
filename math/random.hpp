#pragma once

#include <random>

namespace Math
{
double normalRand(const double& mean, const double& stddev);
// probの確率で強烈なノイズが乗る
double impulsiveNoise(const double& mean, const double& stddev, const double& prob);
}  // namespace Math
