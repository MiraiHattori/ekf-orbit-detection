#include "filter/ekf.hpp"

#include <cmath>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>

#include "math/random.hpp"

int main()
{
  Eigen::VectorXd x_init(6);
  // 雑な値を入れておく
  x_init << 4.0, 0.0, 0.0, -1.0, -1.0, 2.0;
  // 雑な値を入れておいたので増やしておく
  Eigen::MatrixXd P_init(6, 6);
  // clang-format off
  P_init << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 2.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 2.0;
  // clang-format on
  Filter::EKF ekf(x_init, P_init);

  // 放物線を描くボール
  // 重力は適当
  const Eigen::VectorXd GRAVITY((Eigen::VectorXd(3) << 0, 0, -9.80665).finished());

  /*
   * 単位はm,k,s,rad
   * xが前, zが上
   * xy平面上に軸wを取る。
   * 軸wをxy平面上の方向ベクトルで表すと(p,
   * q)^Tである(p^2+q^2=1に正規化されている前提)
   * ボールはwz平面上を運動する(z = -aw^2 + bw + cのような放物線)
   * ボールの初期位置は(x0, y0, z0), 初期速度は(p * v_w, q * v_w, v_z)^T,
   * 加速度はGRAVITY
   * ボールの位置はx = x0+p*v_w*t, y = y0+q*v_w*t, z = z0+v_z*t+GRAVITY*t^2/2.0
   */

  double delta_t = 0.01;

  Eigen::MatrixXd PL(3, 4);
  Eigen::MatrixXd PR(3, 4);
  // clang-format off
  PL << 1710.009813,    0.000000, 717.047562,    0.000000,
           0.000000, 1710.009813, 435.945057,    0.000000,
           0.000000,    0.000000,   1.000000,    0.000000;
  PR << 1710.009813,    0.000000, 717.047562, -157.965126,
           0.000000, 1710.009813, 435.945057,    0.000000,
           0.000000,    0.000000,   1.000000,    0.000000;
  // clang-format on

  double sim_x0 = 5.0;
  double sim_y0 = 1.0;
  double sim_z0 = 0.0;
  double sim_v_w = -5.0;
  double sim_v_z = 3.0;
  double sim_p = 0.9;                           // 0.0 <= p <= 1.0である必要がある
  double sim_q = std::sqrt(1 - sim_p * sim_p);  // p^2 + q^2 = 1
  double sim_t = 0.0;

  while (true)
  {
    // {{{ simulator calc start
    Eigen::VectorXd pos3d(3);
    pos3d << sim_x0 + sim_p * sim_v_w * sim_t, sim_y0 + sim_q * sim_v_w * sim_t,
        sim_z0 + sim_v_z * sim_t + GRAVITY[2] * sim_t * sim_t / 2.0;

    std::cout << "sim_time_and_pos: " << sim_t << " " << pos3d[0] << " " << pos3d[1] << " " << pos3d[2] << std::endl;

    Eigen::VectorXd homo_pos4d(4);  // 同次座標系, かつ光学座標系(奥がz)
    homo_pos4d << -pos3d[1], -pos3d[2], pos3d[0], 1.0;
    Eigen::VectorXd tmp_l = PL * homo_pos4d;
    // 左右ステレオカメラ上のボールの画像重心ピクセル値
    Eigen::VectorXd pixel_l(2);
    pixel_l << tmp_l[0] / tmp_l[2] + Math::normalRand(0.0, 1.0) + Math::impulsiveNoise(0.0, 0.0, 0.0),
        tmp_l[1] / tmp_l[2] + Math::normalRand(0.0, 1.0) + Math::impulsiveNoise(0.0, 0.0, 0.0);
    Eigen::VectorXd tmp_r = PR * homo_pos4d;
    Eigen::VectorXd pixel_r(2);
    pixel_r << tmp_r[0] / tmp_r[2] + Math::normalRand(0.0, 1.0) + Math::impulsiveNoise(0.0, 0.0, 0.0),
        tmp_r[1] / tmp_r[2] + Math::normalRand(0.0, 1.0) + Math::impulsiveNoise(0.0, 0.0, 0.0);
    if (0.0 <= pixel_l[0] and pixel_l[0] <= 1280.0 and 0.0 <= pixel_l[1] and pixel_l[1] < 1280.0 and
        0.0 <= pixel_r[0] and pixel_r[0] <= 1280.0 and 0.0 <= pixel_r[1] and pixel_r[1] <= 1024.0)
    {
      std::cout << "pixel: " << pixel_l[0] << " " << pixel_l[1] << " " << pixel_r[0] << " " << pixel_r[1] << std::endl;
    }
    // }}} simulator calc end
    // {{{ user program start
    float pd[12] = {
      static_cast<float>(PL(0, 0)), static_cast<float>(PL(0, 1)), static_cast<float>(PL(0, 2)),
      static_cast<float>(PL(0, 3)), static_cast<float>(PL(1, 0)), static_cast<float>(PL(1, 1)),
      static_cast<float>(PL(1, 2)), static_cast<float>(PL(1, 3)), static_cast<float>(PL(2, 0)),
      static_cast<float>(PL(2, 1)), static_cast<float>(PL(2, 2)), static_cast<float>(PL(2, 3)),
    };
    cv::Mat p(cv::Size(4, 3), CV_32F, pd);
    std::vector<cv::Point2f> xy;
    xy.push_back(cv::Point2f(static_cast<float>(pixel_l[0]), static_cast<float>(pixel_l[1])));
    float rpd[12] = {
      static_cast<float>(PR(0, 0)), static_cast<float>(PR(0, 1)), static_cast<float>(PR(0, 2)),
      static_cast<float>(PR(0, 3)), static_cast<float>(PR(1, 0)), static_cast<float>(PR(1, 1)),
      static_cast<float>(PR(1, 2)), static_cast<float>(PR(1, 3)), static_cast<float>(PR(2, 0)),
      static_cast<float>(PR(2, 1)), static_cast<float>(PR(2, 2)), static_cast<float>(PR(2, 3)),
    };
    cv::Mat rp(cv::Size(4, 3), CV_32F, rpd);
    std::vector<cv::Point2f> rxy;
    rxy.push_back(cv::Point2f(static_cast<float>(pixel_r[0]), static_cast<float>(pixel_r[1])));
    float resultd[4] = { 0.0, 0.0, 0.0, 0.0 };
    cv::Mat result(cv::Size(1, 4), CV_32F, resultd);
    cv::triangulatePoints(p, rp, xy, rxy, result);
    Eigen::VectorXd point(3);
    point << result.at<float>(0, 0) / result.at<float>(3, 0), result.at<float>(1, 0) / result.at<float>(3, 0),
        result.at<float>(2, 0) / result.at<float>(3, 0);

    if (0.0 <= pixel_l[0] and pixel_l[0] <= 1280.0 and 0.0 <= pixel_l[1] and pixel_l[1] < 1280.0 and
        0.0 <= pixel_r[0] and pixel_r[0] <= 1280.0 and 0.0 <= pixel_r[1] and pixel_r[1] <= 1024.0)
    {
      std::cout << "measured: " << point[2] << " " << -point[0] << " " << -point[1] << std::endl;
    }

    // 状態xはカメラリンク座標系でのボールの位置と速度を6次元並べたもの
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F.block(0, 3, 3, 3) = delta_t * Eigen::MatrixXd::Identity(3, 3);

    std::function<Eigen::VectorXd(Eigen::VectorXd)> f = [F](Eigen::VectorXd x) { return F * x; };
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd Q = 0.01 * Eigen::MatrixXd::Identity(6, 6);  // なんとなく誤差を入れた
    Eigen::VectorXd u(6);
    u.block(0, 0, 3, 1) = GRAVITY * delta_t * delta_t / 2.0;
    u.block(3, 0, 3, 1) = GRAVITY * delta_t;
    Eigen::VectorXd z(4);
    z << pixel_l[0], pixel_l[1], pixel_r[0], pixel_r[1];
    std::function<Eigen::VectorXd(Eigen::VectorXd)> h = [PL, PR](Eigen::VectorXd x) {
      Eigen::VectorXd z_(4);
      double X = x[0];
      double Y = x[1];
      double Z = x[2];
      z_ << -PL(0, 0) * Y / X + PL(0, 2), -PL(1, 1) * Z / X + PL(1, 2), -PR(0, 0) * Y / X + PR(0, 2) + PR(0, 3) / X,
          -PR(1, 1) * Z / X + PR(1, 2);
      return z_;
    };
    std::function<Eigen::MatrixXd(Eigen::VectorXd)> dh = [PL, PR](Eigen::VectorXd x_filtered_pre) {
      double X = x_filtered_pre[0];
      double Y = x_filtered_pre[1];
      double Z = x_filtered_pre[2];
      Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 6);
      H(0, 0) = PL(0, 0) * Y / (X * X);
      H(0, 1) = -PL(0, 0) / X;
      H(1, 0) = PL(1, 1) * Z / (X * X);
      H(1, 2) = -PL(1, 1) / X;
      H(2, 0) = PR(0, 0) * Y / (X * X) - PR(0, 3) / (X * X);
      H(2, 1) = -PR(0, 0) / X;
      H(3, 0) = PR(1, 1) * Z / (X * X);
      H(3, 2) = -PR(1, 1) / X;
      return H;
    };
    // 画素のばらつき
    Eigen::MatrixXd R = 10.0 * Eigen::MatrixXd::Identity(4, 4);

    std::pair<Eigen::VectorXd, Eigen::MatrixXd> value = ekf.update(f, F, G, Q, u, z, h, dh, R);

    if (0.0 <= pixel_l[0] and pixel_l[0] <= 1280.0 and 0.0 <= pixel_l[1] and pixel_l[1] <= 1024.0 and
        0.0 <= pixel_r[0] and pixel_r[0] <= 1280.0 and 0.0 <= pixel_r[1] and pixel_r[1] <= 1024.0)
    {
      /*
    std::cout << "estimated: " << (value.first)[0] << " " << (value.first)[1] << " " << (value.first)[2] << " "
              << (value.first)[3] << " " << (value.first)[4] << " " << (value.first)[5] << std::endl;
              */
      std::cout << "estimated: " << (value.first)[0] << " " << (value.first)[1] << " " << (value.first)[2] << " "
                << (value.first)[3] << " " << (value.first)[4] << " " << (value.first)[5] << std::endl;
    }

    // }}} user program end
    // {{{ simulator update start
    if (pos3d[2] < 0.0)
    {
      break;
    }
    sim_t += delta_t;
    // }}} simulator update end
  }
}
