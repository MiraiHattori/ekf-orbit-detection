#include <iostream>
#include <GL/glut.h>
#include <memory>

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
    glutMotionFunc(dragFunc);
    glutSpecialFunc(specialKeyboardFunc);
    glutMouseFunc(mouseFunc);
  }
  // コールバック開始
  static void start()
  {
    glutMainLoop();
  }
  // 表示
  static void displayAll()
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(
        0.0, 0.0, 6000.0,
        0.0, 0.0, 0.0,
        0.0, 1.0, 0.0);
    displayGround();
    glColor4f(ball_r, ball_g, ball_b, ball_a);
    glutSolidCube(100);
    glutSwapBuffers();  // double buffering
  }
  // 大地の描画
  static void displayGround()
  {
    glTranslatef(0.0, 0.0, -2.0);
    // quads
    glBegin(GL_QUADS);
    glColor4f(ground_r, ground_g, ground_b, ground_a);  // 盤面の色
    glVertex3f(-ground_max_x / 2.0, -ground_max_y / 2.0, 0.0);
    glColor4f(ground_r, ground_g, ground_b, ground_a);  // 盤面の色
    glVertex3f(ground_max_x / 2.0, -ground_max_y / 2.0, 0.0);
    glColor4f(ground_r, ground_g, ground_b, ground_a);  // 盤面の色
    glVertex3f(ground_max_x / 2.0, ground_max_y / 2.0, 0.0);
    glColor4f(ground_r, ground_g, ground_b, ground_a);  // 盤面の色
    glVertex3f(-ground_max_x / 2.0, ground_max_y / 2.0, 0.0);
    glTranslatef(0.0, 0.0, 2.0);
    glEnd();
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
    //std::cout << "refresh" << std::endl;
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
  static constexpr double ground_max_x = 5400.0;
  static constexpr double ground_max_y = 5400.0;
  static constexpr float ground_r = 0.0f;
  static constexpr float ground_g = 0.0f;
  static constexpr float ground_b = 0.0f;
  static constexpr float ground_a = 1.0f;
  static constexpr float ball_r = 1.0f;
  static constexpr float ball_g = 0.2f;
  static constexpr float ball_b = 0.2f;
  static constexpr float ball_a = 1.0f;

  const char* window_title = "sim";
};

int main(int argc, char** argv)
{
  std::unique_ptr<Window> window = nullptr;
  window.reset(new Window(&argc, argv));
  window->init();
  window->start();
  return 0;
}
