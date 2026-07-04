#include <chrono>
#include <cstring>
#include <print>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

#include "kinematic.hpp"

const char *model = "./lib/universal_robots_ur5e/scene.xml";

// https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
std::array<float, 6> theta = {0.0f, 0, 0, 0.0f, 0, 0.0f};
std::array<float, 6> home = {0, -M_PI_2, M_PI_2, 0, M_PI_2, 0.0f};
std::array<float, 6> theta2 = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f};
std::array<float, 6> a = {0.0f, -0.425, -0.3922, 0.f, 0.0f, 0.0f};
std::array<float, 6> alpha = {M_PI_2, 0.0f, 0.0f, M_PI_2, -M_PI_2, 0.0f};
std::array<float, 6> d_ = {0.1625f, 0.0f, 0.0f, 0.1333f, 0.0997f, 0.0996f};
std::array<float, 6> offset = {0.0f, 0.0f, 0.0f, 0.f, 0.f, 0.f};

MDH mdh(theta, a, alpha, d_, offset);
Kinematic kinematic(mdh);

std::array<float, 6> test_angles = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
std::array<float, 3> xyz = {0.29f, -0.13f, -0.36f}; // 平移向量
std::array<float, 3> rpy = {0.f, 0.f, 1.5f};        // 欧拉角 (roll, pitch, yaw)
std::array<float, 6> pose = {-0.29f, -0.13f, -0.36f,
                             0.f,    0.f,    1.5f}; // 初始末端位置 xyz,rpy

// MuJoCo data structures
mjModel *m = NULL; // MuJoCo model
mjData *d = NULL;  // MuJoCo data
mjvCamera cam;     // abstract camera
mjvOption opt;     // visualization options
mjvScene scn;      // abstract scene
mjrContext con;    // custom GPU context
mjUI ui = {};
mjuiState ui_state;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}

void UpdateMjuiState(GLFWwindow *window) {
  // mouse buttons
  ui_state.left =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  ui_state.right =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
  ui_state.middle =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);

  // keyboard modifiers
  ui_state.control = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                     glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
  ui_state.shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                   glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
  ui_state.alt = glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS ||
                 glfwGetKey(window, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS;

  // swap left and right if Alt
  if (ui_state.alt) {
    int tmp = ui_state.left;
    ui_state.left = ui_state.right;
    ui_state.right = tmp;
  }

  // get mouse position, scale by buffer-to-window ratio
  double x, y = 0;
  int w, h = 0;
  int fw, fh = 0;
  glfwGetCursorPos(window, &x, &y);
  glfwGetFramebufferSize(window, &fw, &fh);
  glfwGetWindowSize(window, &w, &h);
  const double buffer_window_ratio = static_cast<double>(fw) / w;
  x *= buffer_window_ratio;
  y *= buffer_window_ratio;

  // invert y to match OpenGL convention
  y = ui_state.rect[0].height - y;

  // save
  ui_state.dx = x - ui_state.x;
  ui_state.dy = y - ui_state.y;
  ui_state.x = x;
  ui_state.y = y;

  // find mouse rectangle
  ui_state.mouserect = mjr_findRect(mju_round(x), mju_round(y),
                                    ui_state.nrect - 1, ui_state.rect + 1) +
                       1;
}
mjtButton TranslateMouseButton(int button) {
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    return mjBUTTON_LEFT;
  } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
    return mjBUTTON_RIGHT;
  } else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
    return mjBUTTON_MIDDLE;
  }
  return mjBUTTON_NONE;
}
// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods) {
  // update button state
  button_left =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  mjtButton mj_button = TranslateMouseButton(button);

  UpdateMjuiState(window);

  // press
  if (act == GLFW_PRESS) {

    double now = std::chrono::duration<double>(
                     std::chrono::steady_clock::now().time_since_epoch())
                     .count();

    // detect doubleclick: 250 ms
    if (mj_button == ui_state.button && now - ui_state.buttontime < 0.25) {
      ui_state.doubleclick = 1;
    } else {
      ui_state.doubleclick = 0;
    }

    // set info
    ui_state.type = mjEVENT_PRESS;
    ui_state.button = mj_button;
    ui_state.buttontime = now;

    // start dragging
    if (ui_state.mouserect) {
      ui_state.dragbutton = ui_state.button;
      ui_state.dragrect = ui_state.mouserect;
    }
  }

  // release
  else {
    ui_state.type = mjEVENT_RELEASE;
  }

  mjuiItem *it = mjui_event(&ui, &ui_state, &con);

  // stop dragging after application processing
  if (ui_state.type == mjEVENT_RELEASE) {
    ui_state.dragrect = 0;
    ui_state.dragbutton = 0;
  }

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  UpdateMjuiState(window);

  // set move info
  ui_state.type = mjEVENT_MOVE;

  // ui events handle
  if ((ui_state.dragrect == ui.rectid) ||
      (ui_state.dragrect == 0 && ui_state.mouserect == ui.rectid) ||
      ui_state.type == mjEVENT_KEY) {
    mjuiItem *it = mjui_event(&ui, &ui_state, &con);
  } else {
    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right) {
      action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left) {
      action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
      action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
  }
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void UiLayout(mjuiState *state) {
  mjrRect *rect = state->rect;

  // set number of rectangles
  state->nrect = 4;

  rect[1].left = 0;
  rect[1].width = ui.width;
  rect[1].bottom = 0;
  rect[1].height = rect[0].height;
}

void UiModify(mjUI *ui, mjuiState *state, mjrContext *con) {
  mjui_resize(ui, con);

  // remake aux buffer only if missing or different
  int id = ui->auxid;
  if (con->auxFBO[id] == 0 || con->auxFBO_r[id] == 0 ||
      con->auxColor[id] == 0 || con->auxColor_r[id] == 0 ||
      con->auxWidth[id] != ui->width || con->auxHeight[id] != ui->maxheight ||
      con->auxSamples[id] != ui->spacing.samples) {
    mjr_addAux(id, ui->width, ui->maxheight, ui->spacing.samples, con);
  }

  UiLayout(state);
  mjui_update(-1, -1, ui, state, con);
}

int UiPredicate(int category, void *userdata) {

  switch (category) {
  case 2: // require model
    std::print("require model");
    break;

  case 3: // require model and nkey
    std::print("require model and nkey");
    break;

  case 4: // require model and paused
    std::print("require model and paused");
    break;

  case 5: // require model and fully managed mode
    std::print("require model and fully managed mode");
    break;

  default:
    return 1;
  }
  return 1;
}

int main() {

  char error[1000] = "Could not load binary model";
  m = mj_loadXML(model, 0, error, 1000);
  if (!m) {
    mju_error("Load model error: %s", error);
  }

  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow *window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  // user

  // opt.frame = mjFRAME_BODY;

  // mjuiDef defControl[] = {
  //     {mjITEM_SECTION, "Control", mjPRESERVE, nullptr, "AJ"}, {mjITEM_END}};
  // mjuiDef defSlider[] = {
  //     {mjITEM_SLIDERNUM, "posSlider", 2, d->ctrl, "-3.14 3.14"},
  //     {mjITEM_END}};
  // for (int i = 0; i < 6; i++) {
  //   defSlider[0].pdata = &d->ctrl[i];
  //   std::sprintf(defSlider[0].name, "joint %d", i);
  //   std::memcpy(defSlider[0].other, "-3.14 3.14", 11);
  // }
  // mjui_add(&ui, defControl);
  // mjui_add(&ui, defSlider);
  //
  // UiModify(&ui, &ui_state, &con);

  int buf_width;
  int buf_height;
  glfwGetFramebufferSize(window, &buf_width, &buf_height);
  ui_state.nrect = 1;
  ui_state.rect[0].width = buf_width;
  ui_state.rect[0].height = buf_height;

  ui.spacing = mjui_themeSpacing(0);
  ui.color = mjui_themeColor(0);
  // ui.predicate = UiPredicate;
  ui.rectid = 1;
  ui.auxid = 0;
  ui.sect[0].state = 1;
  ui.sect[1].state = 1;
  ui.sect[2].state = 1;

  mjuiDef defTCP[] = {{mjITEM_SECTION, "TCP", mjPRESERVE, nullptr, "AT"},
                      {mjITEM_END}};
  mjui_add(&ui, defTCP);
  for (int i = 0; i < 6; i++) {
    mjuiDef defSliderT[] = {{mjITEM_SLIDERNUM, "", 2, nullptr, "0 1"},
                            {mjITEM_END}};
    defSliderT[0].state = 4;
    defSliderT[0].pdata = &pose[i];
    std::sprintf(defSliderT[0].name, "tcp pos %d", i);
    std::strcpy(defSliderT[0].other, "-3.1416 3.1416");
    mjui_add(&ui, defSliderT);
  }
  UiModify(&ui, &ui_state, &con);

  mjuiDef defJoint[] = {{mjITEM_SECTION, "Joint", mjPRESERVE, nullptr, "AJ"},
                        {mjITEM_END}};
  mjui_add(&ui, defJoint);
  for (int i = 0; i < 6; i++) {
    mjuiDef defSlider[] = {{mjITEM_SLIDERNUM, "", 2, nullptr, "0 1"},
                           {mjITEM_END}};
    defSlider[0].state = 4;
    defSlider[0].pdata = &d->qpos[i];
    std::sprintf(defSlider[0].name, "joint pos %d", i);
    std::strcpy(defSlider[0].other, "-3.1416 3.1416");

    mjui_add(&ui, defSlider);
  }

  mjuiDef defControl[] = {
      {mjITEM_SECTION, "Control", mjPRESERVE, nullptr, "AC"}, {mjITEM_END}};
  mjui_add(&ui, defControl);
  for (int i = 0; i < 6; i++) {
    mjuiDef defSliderC[] = {{mjITEM_SLIDERNUM, "", 2, nullptr, "0 1"},
                            {mjITEM_END}};
    defSliderC[0].state = 2;
    defSliderC[0].pdata = &d->ctrl[i];
    std::sprintf(defSliderC[0].name, "ctrl pos %d", i);
    std::strcpy(defSliderC[0].other, "-3.1416 3.1416");
    mjui_add(&ui, defSliderC);
  }
  UiModify(&ui, &ui_state, &con);

  m->qpos0 = (mjtNum *)&theta;
  for (int i = 0; i < 6; i++)
    std::println("theta1:{}", home[i]);
  for (int i = 0; i < 6; i++)
    std::println("qpos:{}", m->qpos0[i]);

  auto fk = kinematic.fkine2pose(home);
  std::println("roll:{},pitch:{},yaw:{},x:{},y:{},z:{}", fk.rpy.roll,
               fk.rpy.pitch, fk.rpy.yaw, fk.xyz.x, fk.xyz.y, fk.xyz.z);

  // auto ik = kinematic.ikinePose({pose[0], pose[1], pose[2]},
  //                               {pose[3], pose[4], pose[5]},
  //                               *(std::array<float, 6> *)d->qpos);
  // auto ik = kinematic.ikinePose({fk.xyz.x, fk.xyz.y, fk.xyz.z},
  //                               {fk.rpy.roll, fk.rpy.pitch, fk.rpy.yaw},
  //                               *(std::array<float, 6> *)m->qpos0);
  auto fkt = kinematic.fkine2T(theta2);
  auto ik = kinematic.ikine(fkt, theta2);

  auto fkpose = kinematic.fkine2pose(home);
  auto ikpose = kinematic.ikinePose(
      {fkpose.xyz.x, fkpose.xyz.y, fkpose.xyz.z},
      {fkpose.rpy.roll, fkpose.rpy.pitch, fkpose.rpy.yaw}, theta2);

  std::println("ik:{}", ik);
  std::println("ik:{}", ikpose);

  for (int i = 0; i < 6; i++) {
    d->ctrl[i] = home[i];
  }
  pose = {fkpose.xyz.x,    fkpose.xyz.y,     fkpose.xyz.z,
          fkpose.rpy.roll, fkpose.rpy.pitch, fkpose.rpy.yaw};

  for (int i = 0; i < 6; i++)
    std::println("dpos:{}", d->qpos[i]);

  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window)) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually
    //  can, this loop will finish on time for the next frame to be rendered
    //  at 60 fps. Otherwise add a cpu timer and exit this loop when it is
    //  time to render.
    mjtNum simstart = d->time;
    while (d->time - simstart < 1.0 / 60.0) {
      mj_step(m, d);
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // kinematic
    // auto fkpose = kinematic.fkine2pose(*(std::array<float, 6> *)d->qpos);
    // pose = {fkpose.xyz.x,    fkpose.xyz.y,     fkpose.xyz.z,
    //         fkpose.rpy.roll, fkpose.rpy.pitch, fkpose.rpy.yaw};
    const std::array<float, 3> cxyz = {pose[0], pose[1], pose[2]};
    const std::array<float, 3> crpy = {pose[3], pose[4], pose[5]};
    auto ik = kinematic.ikinePose(cxyz, crpy, *(std::array<float, 6> *)d->qpos);
    for (int i = 0; i < 6; i++) {
      d->ctrl[i] = ik[i];
    }

    // ui
    UiModify(&ui, &ui_state, &con);
    // mjui_update(1, -1, &ui, &ui_state, &con);
    mjui_render(&ui, &ui_state, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);
}
