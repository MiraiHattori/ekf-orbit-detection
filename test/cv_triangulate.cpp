#include <array>
#include <cmath>
#include <iostream>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <memory>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <chrono>

#include <Eigen/Geometry>
#include <opencv2/calib3d/calib3d.hpp>
#include "filter/ekf.hpp"
#include "math/random.hpp"


int main(int argc, char** argv)
{

    // 放物線を描くボール
    // 重力は適当
    const Eigen::VectorXd GRAVITY((Eigen::VectorXd(3) << 0, 0, -9.80665).finished());

    Eigen::MatrixXd PL(3, 4);
    Eigen::MatrixXd PR(3, 4);
    PL << 880.808505, 0.000000, 618.013809, 0.000000, 0.000000, 880.808505, 516.929707, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000;
    PR << 880.808505, 0.000000, 618.013809, -203.457281, 0.000000, 880.808505, 516.929707, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000;
    // clang-format on

    Eigen::Vector3d pos3d;
    pos3d << 3.71614, 1.25774, 1.48539;
    std::cout << "sim_pos: " << pos3d[0] << " " << pos3d[1] << " " << pos3d[2] << std::endl;

    // mechanical parameter
    Eigen::Vector3d pos_camera(0.0824143, -0.0461265, 1.51731);
    Eigen::Vector3d pos_camera_inv = -pos_camera;
    // TODO use tf2::Quaternion
    Eigen::Quaterniond q_camera(0.985996, 0.0988545, 0.0612321, 0.119544);  // w, x, y, z
    Eigen::Quaterniond q_camera_inv = q_camera.inverse();
    Eigen::Vector3d q_pos(pos3d[0], pos3d[1], pos3d[2]);                // カメラの姿勢を考慮していないxyz座標, 絶対座標系
    Eigen::Vector3d q_pos_rot = q_camera_inv * q_pos + pos_camera_inv;  // カメラの姿勢を考慮したxyz座標, 絶対座標系
    Eigen::VectorXd homo_pos4d(4);  // 同次座標系, かつ光学座標系(奥がz)
    homo_pos4d << -q_pos_rot[1], -q_pos_rot[2], q_pos_rot[0], 1.0;
    Eigen::VectorXd tmp_l = PL * homo_pos4d;
    // 左右ステレオカメラ上のボールの画像重心ピクセル値
    Eigen::VectorXd pixel_l(2);
    pixel_l << tmp_l[0] / tmp_l[2],
        tmp_l[1] / tmp_l[2];
    Eigen::VectorXd tmp_r = PR * homo_pos4d;
    Eigen::VectorXd pixel_r(2);
    pixel_r << tmp_r[0] / tmp_r[2],
        tmp_r[1] / tmp_r[2];
    // std::cout << "pixel: " << pixel_l[0] << " " << pixel_l[1] << " " << pixel_r[0] << " " << pixel_r[1] << std::endl;

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
    // 光学座標系でのボール位置
    Eigen::VectorXd point_opt(3);
    // リンク座標系でのボール位置
    Eigen::VectorXd point(3);
    Eigen::VectorXd point_rot(3);
    point_opt << result.at<float>(0, 0) / result.at<float>(3, 0), result.at<float>(1, 0) / result.at<float>(3, 0),
        result.at<float>(2, 0) / result.at<float>(3, 0);
    point << point_opt[2], -point_opt[0], -point_opt[1];
    point_rot = q_camera * (point + pos_camera);
    // std::cout << "opt: " << point_opt[0] << " " << point_opt[1] << " " << point_opt[2] << std::endl;
    std::cout << "point: " << point[0] << " " << point[1] << " " << point[2] << std::endl;
    std::cout << "point_rot: " << point_rot[0] << " " << point_rot[1] << " " << point_rot[2] << std::endl;

  return 0;
}
