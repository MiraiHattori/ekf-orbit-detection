#include <iomanip>
#include <iostream>
#include <Eigen/Core>

int main()
{
  Eigen::MatrixXd p(3, 4);
  // clang-format off
  p << 1710.009813,    0.000000,  717.047562,    0.000000,
          0.000000, 1710.009813,  435.945057,    0.000000,
          0.000000,    0.000000,    1.000000,    0.000000;
  // clang-format on
  Eigen::MatrixXd rp(3, 4);
  // clang-format off
  /*
  rp << 1710.009813,    0.000000,  717.047562, -157.965126,
           0.000000, 1710.009813,  435.945057,    0.000000,
           0.000000,    0.000000,    1.000000,    0.000000;
  */
  // double camera_distance = 157.965126 / 1710.009813; // カメラ間距離[m]
  double camera_distance = 0.1; // カメラ間距離[m]
  rp << 1710.009813,    0.000000,  717.047562, -1710.009813 * camera_distance,
           0.000000, 1710.009813,  435.945057,     0.000000,
           0.000000,    0.000000,    1.000000,     0.000000;
  // clang-format on

  // 1mから5mまで
  for (double Z = 1.0; Z < 5.0; Z += 0.1)
  {
    Eigen::VectorXd ball(4);
    // zが奥行き、xが左、yが下
    ball << 0.0, 0.0, Z, 1.0;
    Eigen::VectorXd pixel = p * ball;
    Eigen::VectorXd rpixel = rp * ball;
    pixel /= pixel[2];
    rpixel /= rpixel[2];
    // clang-format off
    std::cout
        << "Z: " << std::left << std::setw(7) << Z
        << " l: " << std::left << "( " << std::setw(7) << pixel[0] << " , " << pixel[1] << " )"
        << " r: " << std::left << "( " << std::setw(7) << rpixel[0] << " , " << pixel[1] << " )"
        << " dr: " << std::left << "( " << std::setw(7) << 1710.009813 * camera_distance / (Z * Z) * 0.1 << " , " << 0 << " )"
        << std::endl;
    // clang-format on
  }
  return 0;
}

/*
ボールの座標(X, Y, Z, 1)、pixel(x*w, y*w, w), 最終的にはw=Zになる
x * Z / Z = 1710.009813 * X / Z + 717.047562
y * Z / Z = 1710.009813 * Y / Z + 435.945057
rx * Z / Z = 1710.009813 * X / Z + 717.047562 - 1710.009813 * camera_distance / Z
ry * Z / Z = 1710.009813 * Y / Z + 435.945057

X = Y = 0なら
dx/dZ = 0
dy/dZ = 0
drx/dZ = 1710.009813 * camera_distance / (Z * Z)
dry/dZ = 0

*/
