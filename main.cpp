#include <array>
#include <cmath>
#include <iostream>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <memory>
#include <unistd.h>
#include <thread>
#include <mutex>

#include <Eigen/Geometry>
#include <opencv2/calib3d/calib3d.hpp>
#include "filter/ekf.hpp"
#include "math/random.hpp"

// {{{ class Window
class Window
{
public:
  Window(int* argcp, char** argvp)
  {
    glutInit(argcp, argvp);
    glutInitWindowPosition(window_pos_x, window_pos_y);
    glutInitWindowSize(window_width, window_height);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutCreateWindow(window_title);
    // 背景色
    glClearColor(bg_r, bg_g, bg_b, bg_a);
    // デプスバッファを使用：glutInitDisplayMode() で GLUT_DEPTH を指定する
    glEnable(GL_DEPTH_TEST);
  }
  // 初期化
  static void init()
  {
    glutDisplayFunc(displayAll);
    glutReshapeFunc(reshapeFunc);
    // glutMotionFunc(dragFunc);
    glutKeyboardFunc(normalKeyboardFunc);
    // glutSpecialFunc(specialKeyboardFunc);
    // glutMouseFunc(mouseFunc);
    glutIdleFunc(refreshFunc);
  }
  // コールバック開始
  static void start()
  {
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
    glutMainLoop();
  }
  // 表示
  static void displayAll()
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(
        7000.0, 0.0, 300.0,
        6000.0, 0.0, 200.0,
        0.0, 0.0, 1.0);
    /*
    // 横視点
    gluLookAt(
        2000.0, 8000.0, 300.0,
        1000.0, 0.0, 200.0,
        0.0, 0.0, 1.0);
    */

    displayGround();
    double x, y, z;
    {
      std::lock_guard<std::mutex> lock(m_mtx);
      x = m_ball_x;
      y = m_ball_y;
      z = m_ball_z;
    }
    glTranslated(x, y, z);
    glColor4d(ball_r, ball_g, ball_b, ball_a);
    glutSolidSphere(60.0, 100, 100);

    glTranslated(-x, -y, -z);
    {
      std::lock_guard<std::mutex> lock(m_mtx);
      x = m_ball_est_x;
      y = m_ball_est_y;
      z = m_ball_est_z;
    }
    glTranslated(x, y, z);
    glColor4d(ball_est_r, ball_est_g, ball_est_b, ball_est_a);
    glutSolidSphere(60.0, 100, 100);
    glTranslated(-x, -y, -z);
    glColor4d(1.0, 1.0, 1.0, 1.0);

    static double x_, y_, vx_, vy_, vz_;
    static bool initialized = false;
    if (not initialized)
    {
      std::lock_guard<std::mutex> lock(m_mtx);
      x_ = m_ball_x;
      y_ = m_ball_y;
      vx_ = m_ball_vx;
      vy_ = m_ball_vy;
      vz_ = m_ball_vz;
      initialized = true;
    }
    glTranslated(x_ + vx_ * 2.0 * vz_ / (9.80665 * 1000.0), y_ + vy_ * 2.0 * vz_ / (9.80665 * 1000.0), 0.0);
    glutSolidCylinder(100.0, 10.0, 10, 10);
    glTranslated(-x_ - vx_ * 2.0 * vz_ / (9.80665 * 1000.0), -y_ - vy_ * 2.0 * vz_ / (9.80665 * 1000.0), 0.0);

    /*
    // ストライクゾーンの描画
    glColor4d(0.2, 0.2, 0.2, 1.0);
    glTranslated(0.0, 0.0, 1000.0);
    glRotated(90.0, 0.0, 1.0, 0.0);
    glutSolidCylinder(10000.0, 10.0, 10, 10);
    glRotated(-90.0, 0.0, 1.0, 0.0);
    glTranslated(0.0, 0.0, -1000.0);
    */

    displayMarker();
    glutSwapBuffers();  // double buffering
  }
  // 大地の描画
  static void displayGround()
  {
    glTranslated(0.0, 0.0, -2.0);
    // quads
    glBegin(GL_QUADS);
    glColor4d(ground_r, ground_g, ground_b, ground_a);  // 盤面の色
    glVertex3d(-ground_max_x / 2.0, -ground_max_y / 2.0, 0.0);
    glColor4d(ground_r, ground_g, ground_b, ground_a);  // 盤面の色
    glVertex3d(ground_max_x / 2.0, -ground_max_y / 2.0, 0.0);
    glColor4d(ground_r, ground_g, ground_b, ground_a);  // 盤面の色
    glVertex3d(ground_max_x / 2.0, ground_max_y / 2.0, 0.0);
    glColor4d(ground_r, ground_g, ground_b, ground_a);  // 盤面の色
    glVertex3d(-ground_max_x / 2.0, ground_max_y / 2.0, 0.0);
    glTranslated(0.0, 0.0, 2.0);
    glEnd();
  }

  // 放物線や落下点や通過点の描画
  static void displayMarker()
  {
    double x0, y0, z0, vx0, vy0, vz0;
    {
      std::lock_guard<std::mutex> lock(m_mtx);
      x0 = m_ball_est_x;
      y0 = m_ball_est_y;
      z0 = m_ball_est_z;
      vx0 = m_ball_est_vx;
      vy0 = m_ball_est_vy;
      vz0 = m_ball_est_vz;
    }
    glColor4d(1.0, 1.0, 1.0, 1.0);
    const Eigen::VectorXd GRAVITY((Eigen::VectorXd(3) << 0, 0, -9.80665 * 1000.0).finished());
    for (double t = 0.0; t < 10.0; t += 0.01)
    {
      double x = x0 + vx0 * t;
      double y = y0 + vy0 * t;
      double z = z0 + vz0 * t + GRAVITY[2] * t * t / 2.0;
      // x=0にある垂直な壁との推定通過点
      if (-30.0 < x and x < 30.0)
      {
        glTranslated(10.0, y, z);
        glRotated(90.0, 0.0, 1.0, 0.0);
        glutSolidCylinder(100.0, 10.0, 10, 10);
        glRotated(-90.0, 0.0, 1.0, 0.0);
        glTranslated(-10.0, -y, -z);
      }
      // z=0にある地面への落下点
      else if (-30.0 < z and z < 30.0)
      {
        glTranslated(x, y, 0);
        glutSolidCylinder(100.0, 10.0, 10, 10);
        glTranslated(-x, -y, -0);
      }
      else
      {
        glTranslated(x, y, z);
        glutSolidCylinder(10.0, 10.0, 10, 10);
        glTranslated(-x, -y, -z);
      }
    }
    // x=0にある垂直な壁とボールの通過点
    static bool init = false;
    static double x, y, z, vx, vy, vz;
    if (not init)
    {
      std::lock_guard<std::mutex> lock(m_mtx);
      x = m_ball_x;
      y = m_ball_y;
      z = m_ball_z;
      vx = m_ball_vx;
      vy = m_ball_vy;
      vz = m_ball_vz;
      init = true;
    }
    static double t = -x / vx;
    glTranslated(0, y + vy * t, z + vz * t + GRAVITY[2] * t * t / 2.0);
    glRotated(90.0, 0.0, 1.0, 0.0);
    glutSolidCylinder(100.0, 10.0, 10, 10);
    glRotated(-90.0, 0.0, 1.0, 0.0);
    glTranslated(0, -y - vy * t, -z - vz * t - GRAVITY[2] * t * t / 2.0);
  }

  // 画面サイズ変更時
  static void reshapeFunc(int w, int h)
  {
    std::cerr << "reshape " << w << " " << h << std::endl;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // 透視投影法の視体積gluPerspective(th, w/h, near, far);
    gluPerspective(50.0, 1.0, 0.1, 20000.0f);
    glMatrixMode(GL_MODELVIEW);
  }
  // ドラッグ時
  static void dragFunc(int x, int y)
  {
    std::cerr << "drag " << x << " " << y << std::endl;
  }
  // 通常キー押下時
  static void normalKeyboardFunc(unsigned char key, int x, int y)
  {
    if (key == 'q')
    {
      std::cerr << "keyboard " << key << " " << x << " " << y << std::endl;
      glutLeaveMainLoop();
    }
  }
  // 特殊キー押下時
  static void specialKeyboardFunc(int key, int x, int y)
  {
    std::cerr << "keyboard " << key << " " << x << " " << y << std::endl;
  }
  // マウス処理
  static void mouseFunc(int button, int state, int x, int y)
  {
    switch (button)
    {
      case GLUT_LEFT_BUTTON:
        if (state == GLUT_DOWN)
          glutIdleFunc(refreshFunc);
        break;
      case GLUT_MIDDLE_BUTTON:
        if (state == GLUT_DOWN)
          glutIdleFunc(NULL);
        break;
      case GLUT_RIGHT_BUTTON:
        if (state == GLUT_DOWN)
          glutIdleFunc(refreshFunc);
        break;
      default:
        break;
    }
    std::cerr << "mouse" << x << " " << y << std::endl;
  }
  // 他の処理が終わった後の更新時
  static void refreshFunc()
  {
    glutPostRedisplay();
  }

  static void setRealBallState(const double& x, const double& y, const double& z,
                               const double& vx, const double& vy, const double& vz)
  {
    std::lock_guard<std::mutex> lock(m_mtx);
    m_ball_x = x * 1000.0;
    m_ball_y = y * 1000.0;
    m_ball_z = z * 1000.0;
    m_ball_vx = vx * 1000.0;
    m_ball_vy = vy * 1000.0;
    m_ball_vz = vz * 1000.0;
  }

  static void setEstimatedBallState(const double& x, const double& y, const double& z,
                                    const double& vx, const double& vy, const double& vz)
  {
    std::lock_guard<std::mutex> lock(m_mtx);
    m_ball_est_x = x * 1000.0;
    m_ball_est_y = y * 1000.0;
    m_ball_est_z = z * 1000.0;
    m_ball_est_vx = vx * 1000.0;
    m_ball_est_vy = vy * 1000.0;
    m_ball_est_vz = vz * 1000.0;
  }

private:
  static constexpr int window_pos_x = 100;
  static constexpr int window_pos_y = 100;
  static constexpr int window_width = 720;
  static constexpr int window_height = 720;
  static constexpr float bg_r = 0.0f;
  static constexpr float bg_g = 0.765f;
  static constexpr float bg_b = 1.0f;
  static constexpr float bg_a = 1.0f;

  // 盤面
  static constexpr double ground_max_x = 10000.0;
  static constexpr double ground_max_y = 10000.0;
  static constexpr float ground_r = 0.0f;
  static constexpr float ground_g = 0.0f;
  static constexpr float ground_b = 0.0f;
  static constexpr float ground_a = 1.0f;
  static constexpr float ball_r = 1.0f;
  static constexpr float ball_g = 0.2f;
  static constexpr float ball_b = 0.2f;
  static constexpr float ball_a = 1.0f;
  static constexpr float ball_est_r = 0.2f;
  static constexpr float ball_est_g = 0.2f;
  static constexpr float ball_est_b = 1.0f;
  static constexpr float ball_est_a = 1.0f;

  static std::mutex m_mtx;

  static double m_ball_x;
  static double m_ball_y;
  static double m_ball_z;
  static double m_ball_vx;
  static double m_ball_vy;
  static double m_ball_vz;
  static double m_ball_est_x;
  static double m_ball_est_y;
  static double m_ball_est_z;
  static double m_ball_est_vx;
  static double m_ball_est_vy;
  static double m_ball_est_vz;

  const char* window_title = "sim";
};

double Window::m_ball_x = 0.0;
double Window::m_ball_y = 0.0;
double Window::m_ball_z = 0.0;
double Window::m_ball_vx = 0.0;
double Window::m_ball_vy = 0.0;
double Window::m_ball_vz = 0.0;
double Window::m_ball_est_x = 0.0;
double Window::m_ball_est_y = 0.0;
double Window::m_ball_est_z = 0.0;
double Window::m_ball_est_vx = 0.0;
double Window::m_ball_est_vy = 0.0;
double Window::m_ball_est_vz = 0.0;
std::mutex Window::m_mtx;

// }}}

// {{{ simulation thread
void simulate(const std::unique_ptr<Window>& window)
{
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

  double sim_x0 = 6.0;
  double sim_y0 = 1.0;
  double sim_z0 = 0.0;
  double sim_v_w = -10.0;
  double sim_v_z = 3.3;
  double sim_p = 0.9;                           // 0.0 <= p <= 1.0である必要がある
  double sim_q = std::sqrt(1 - sim_p * sim_p);  // p^2 + q^2 = 1
  double sim_t = 0.0;
  bool is_ekf_initialized = false;
  Filter::EKF ekf{ Eigen::VectorXd(6), Eigen::MatrixXd(6, 6) };

  while (true)
  {
    // {{{ simulator calc start
    Eigen::VectorXd pos3d(3);
    Eigen::VectorXd vel3d(3);
    double throw_start_t = 0.0;
    // しばらく投げない
    if (sim_t < throw_start_t)
    {
      pos3d << sim_x0, sim_y0, sim_z0;
      vel3d << 0.0, 0.0, 0.0;
    }
    else
    {
      pos3d << sim_x0 + sim_p * sim_v_w * (sim_t - throw_start_t),
          sim_y0 + sim_q * sim_v_w * (sim_t - throw_start_t),
          sim_z0 + sim_v_z * (sim_t - throw_start_t) +
              GRAVITY[2] * (sim_t - throw_start_t) * (sim_t - throw_start_t) / 2.0;
      vel3d << sim_p * sim_v_w, sim_q * sim_v_w, sim_v_z;
    }

    std::cout << "sim_time_and_pos: " << sim_t << " " << pos3d[0] << " " << pos3d[1] << " " << pos3d[2] << std::endl;
    window->setRealBallState(pos3d[0], pos3d[1], pos3d[2], vel3d[0], vel3d[1], vel3d[2]);

    // mechanical parameter
    // TODO use tf2::Quaternion
    Eigen::Quaterniond q_camera(0.9987503, 0.0, -0.0499792, 0.0);  // w, x, y, z
    Eigen::Quaterniond q_camera_inv = q_camera.inverse();

    Eigen::Vector3d q_pos(pos3d[0], pos3d[1], pos3d[2]);  // カメラの姿勢を考慮していないxyz座標, 絶対座標系
    Eigen::Vector3d q_pos_rot = q_camera_inv * q_pos;     // カメラの姿勢を考慮したxyz座標, 絶対座標系

    Eigen::VectorXd homo_pos4d(4);  // 同次座標系, かつ光学座標系(奥がz)
    homo_pos4d << -q_pos_rot[1], -q_pos_rot[2], q_pos_rot[0], 1.0;
    Eigen::VectorXd tmp_l = PL * homo_pos4d;
    // 左右ステレオカメラ上のボールの画像重心ピクセル値
    Eigen::VectorXd pixel_l(2);
    pixel_l << tmp_l[0] / tmp_l[2] + Math::normalRand(0.0, 2.0) + Math::impulsiveNoise(0.0, 0.0, 0.0),
        tmp_l[1] / tmp_l[2] + Math::normalRand(0.0, 0.0) + Math::impulsiveNoise(0.0, 0.0, 0.0);
    Eigen::VectorXd tmp_r = PR * homo_pos4d;
    Eigen::VectorXd pixel_r(2);
    pixel_r << tmp_r[0] / tmp_r[2] + Math::normalRand(0.0, 2.0) + Math::impulsiveNoise(0.0, 0.0, 0.0),
        tmp_r[1] / tmp_r[2] + Math::normalRand(0.0, 2.0) + Math::impulsiveNoise(0.0, 0.0, 0.0);
    if (0.0 <= pixel_l[0] and pixel_l[0] <= 1280.0 and 0.0 <= pixel_l[1] and pixel_l[1] < 1280.0 and
        0.0 <= pixel_r[0] and pixel_r[0] <= 1280.0 and 0.0 <= pixel_r[1] and pixel_r[1] <= 1024.0)
    {
      std::cout << "pixel: " << pixel_l[0] << " " << pixel_l[1] << " " << pixel_r[0] << " " << pixel_r[1] << std::endl;
    }
    // }}} simulator calc end
    // {{{ user program start
    // {{{ getting raw ball point
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
    Eigen::VectorXd point_opt(3);
    Eigen::VectorXd point(3);
    Eigen::VectorXd point_rot(3);
    point_opt << result.at<float>(0, 0) / result.at<float>(3, 0), result.at<float>(1, 0) / result.at<float>(3, 0),
        result.at<float>(2, 0) / result.at<float>(3, 0);
    point << point_opt[2], -point_opt[0], -point_opt[1];
    point_rot = q_camera * point;

    if (0.0 <= pixel_l[0] and pixel_l[0] <= 1280.0 and 0.0 <= pixel_l[1] and pixel_l[1] < 1280.0 and
        0.0 <= pixel_r[0] and pixel_r[0] <= 1280.0 and 0.0 <= pixel_r[1] and pixel_r[1] <= 1024.0)
    {
      std::cout << "measured: " << point_rot[0] << " " << point_rot[1] << " " << point_rot[2] << std::endl;
    }
    // }}}

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
    std::function<Eigen::VectorXd(Eigen::VectorXd)> h = [PL, PR, q_camera_inv](Eigen::VectorXd x) {
      Eigen::VectorXd z_(4);
      Eigen::VectorXd x_ = q_camera_inv * x;
      double X = x_[0];
      double Y = x_[1];
      double Z = x_[2];
      z_ << -PL(0, 0) * Y / X + PL(0, 2), -PL(1, 1) * Z / X + PL(1, 2), -PR(0, 0) * Y / X + PR(0, 2) + PR(0, 3) / X,
          -PR(1, 1) * Z / X + PR(1, 2);
      return z_;
    };
    std::function<Eigen::MatrixXd(Eigen::VectorXd)> dh = [PL, PR, q_camera_inv](Eigen::VectorXd x_filtered_pre) {
      Eigen::VectorXd x_ = q_camera_inv * x_filtered_pre;
      double X = x_[0];
      double Y = x_[1];
      double Z = x_[2];
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

    if (not is_ekf_initialized)
    {
      Eigen::VectorXd x_init(6);
      // 雑な値を入れておく(位置は最初に見つけたところ，速度は45度射出時に原点に向かうところ)
      // v0*t=distance, v0*t-g*t^2/2=-zより，v0=d*g^0.5/(2d+2z)^0.5 (g=-GRAVITY[2])
      double distance = std::hypot(point_rot[0], point_rot[1]);
      double v0 = distance * std::sqrt(-GRAVITY[2]) / std::sqrt(2 * (std::abs(distance + point_rot[2])));
      x_init << point_rot[0], point_rot[1], point_rot[2],                    // 位置
          -v0 * point_rot[0] / distance, -v0 * point_rot[1] / distance, v0;  // 速度
      // 雑な値を入れておいたので増やしておく
      Eigen::MatrixXd P_init(6, 6);
      // clang-format off
      P_init << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 2.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 10.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 10.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 30.0;
      // clang-format on
      is_ekf_initialized = true;
      ekf.reset(x_init, P_init);
    }

    if (is_ekf_initialized)
    {
      std::pair<Eigen::VectorXd, Eigen::MatrixXd> value = ekf.update(f, F, G, Q, u, z, h, dh, R);
      if (0.0 <= pixel_l[0] and pixel_l[0] <= 1280.0 and 0.0 <= pixel_l[1] and pixel_l[1] <= 1024.0 and
          0.0 <= pixel_r[0] and pixel_r[0] <= 1280.0 and 0.0 <= pixel_r[1] and pixel_r[1] <= 1024.0)
      {
        std::cout << "estimated: " << (value.first)[0] << " " << (value.first)[1] << " " << (value.first)[2] << " "
                  << (value.first)[3] << " " << (value.first)[4] << " " << (value.first)[5] << std::endl;
        std::cout << "coeff: " << (value.second)(0, 0) << " " << (value.second)(1, 1) << " " << (value.second)(2, 2) << " "
                  << (value.second)(3, 3) << " " << (value.second)(4, 4) << " " << (value.second)(5, 5) << std::endl;

        window->setEstimatedBallState(
            (value.first)[0],
            (value.first)[1],
            (value.first)[2],
            (value.first)[3],
            (value.first)[4],
            (value.first)[5]);
      }
    }

    // }}} user program end
    // {{{ simulator update start
    if (pos3d[2] < 0.0)
    {
      break;
    }
    usleep(100000);
    sim_t += delta_t;
    // }}} simulator update end
  }
}
// }}}

int main(int argc, char** argv)
{
  std::unique_ptr<Window> window = nullptr;
  window.reset(new Window(&argc, argv));

  std::thread thread_simulation([&] { simulate(window); });
  thread_simulation.detach();

  window->init();
  window->start();

  return 0;
}
