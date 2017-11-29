#include "gl.h"
#include <GLFW/glfw3.h>

#include <cmath>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <cstdint>

#include <vecmath.h>
#include <nanogui/nanogui.h>

#include "starter_util.h"
#include "camera.h"
#include "vertexrecorder.h"
#include "skeletalmodel.h"

using namespace std;
// Note: using namespace nanogui not possible due to naming conflicts
namespace ng = ::nanogui;

namespace
{
// Constants 
const int NJOINTS = 18;
const string jointNames[NJOINTS] = { "Root", "Chest", "Waist", "Neck",
                                 "Right hip", "Right leg", "Right knee", "Right foot",
                                 "Left hip", "Left leg", "Left knee", "Left foot",
                                 "Right collarbone", "Right shoulder", "Right elbow", "Left collarbone", "Left shoulder", "Left elbow" };

// Global variables here.
GLFWwindow* window;
ng::Screen *screen;
Vector3f g_jointangles[NJOINTS];
string basepath;
bool jumping;

class glfwtimer {
public:
    void set() {
        freq = glfwGetTimerFrequency();
        start = glfwGetTimerValue();
    }
    // return number of seconds elapsed
    float elapsed() {
        uint64_t now = glfwGetTimerValue();
        return (float)(now - start) / freq;
    }

    uint64_t freq;
    uint64_t start;

};
glfwtimer timer;


    // This assignment uses a useful camera implementation
Camera camera;
SkeletalModel* skeleton;

// most curves are drawn with constant color, and no lighting
GLuint program_color;

// These are state variables for the UI
bool gMousePressed = false;
bool gDrawSkeleton = true;
bool gDrawAxisAlways = false;

// Declarations of functions whose implementations occur later.
void drawAxis(void);
void freeSkeleton(void);
void loadSkeleton(const std::string& basepath);

static void keyCallback(GLFWwindow* window, int key,
    int scancode, int action, int mods)
{
    if (action == GLFW_RELEASE) { // only handle PRESS and REPEAT
        return;
    }

    // Special keys (arrows, CTRL, ...) are documented
    // here: http://www.glfw.org/docs/latest/group__keys.html
    switch (key) {
    case GLFW_KEY_ESCAPE: // Escape key
        exit(0);
        break;
    case ' ':
    {
        Matrix4f eye = Matrix4f::identity();
        camera.SetRotation(eye);
        camera.SetDistance(1.5);
        camera.SetCenter(Vector3f(-0.5, -0.5, -0.5));
        break;
    }
    case 'S': {
        gDrawSkeleton = !gDrawSkeleton;
        break;
    }
    case 'A': {
        gDrawAxisAlways = !gDrawAxisAlways;
        break;
    }
    case 'R': {
        cout << "Resetting simulation\n";
        freeSkeleton();
        loadSkeleton(basepath);
        timer.set();
        break;
    }
    case 'T': {
        cout << "Toggle movements \n";
        freeSkeleton();
        loadSkeleton(basepath);
        timer.set();
        jumping = !jumping;
        break;
    }
    default:
        cout << "Unhandled key press " << key << "." << endl;
    }
}

static void mouseCallback(GLFWwindow* window, int button, int action, int mods) {
    double xd, yd;
    glfwGetCursorPos(window, &xd, &yd);
    int x = (int)xd;
    int y = (int)yd;

    int lstate = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    int rstate = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
    int mstate = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE);
    if (lstate == GLFW_PRESS) {
        gMousePressed = true;
        camera.MouseClick(Camera::LEFT, x, y);
    }
    else if (rstate == GLFW_PRESS) {
        gMousePressed = true;
        camera.MouseClick(Camera::RIGHT, x, y);
    }
    else if (mstate == GLFW_PRESS) {
        gMousePressed = true;
        camera.MouseClick(Camera::MIDDLE, x, y);
    }
    else {
        gMousePressed = true;
        camera.MouseRelease(x, y);
        gMousePressed = false;
    }
}

static void motionCallback(GLFWwindow* window, double x, double y)
{
    if (!gMousePressed) {
        return;
    }
    camera.MouseDrag((int)x, (int)y);
}

void setViewport(GLFWwindow* window)
{
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);

    camera.SetDimensions(w, h);
    camera.SetViewport(0, 0, w, h);
    camera.ApplyViewport();
}

void drawAxis()
{
    glUseProgram(program_color);
    Matrix4f M = Matrix4f::translation(camera.GetCenter()).inverse();
    camera.SetUniforms(program_color, M);

    const Vector3f DKRED(1.0f, 0.5f, 0.5f);
    const Vector3f DKGREEN(0.5f, 1.0f, 0.5f);
    const Vector3f DKBLUE(0.5f, 0.5f, 1.0f);
    const Vector3f GREY(0.5f, 0.5f, 0.5f);

    const Vector3f ORGN(0, 0, 0);
    const Vector3f AXISX(5, 0, 0);
    const Vector3f AXISY(0, 5, 0);
    const Vector3f AXISZ(0, 0, 5);

    VertexRecorder recorder;
    recorder.record_poscolor(ORGN, DKRED);
    recorder.record_poscolor(AXISX, DKRED);
    recorder.record_poscolor(ORGN, DKGREEN);
    recorder.record_poscolor(AXISY, DKGREEN);
    recorder.record_poscolor(ORGN, DKBLUE);
    recorder.record_poscolor(AXISZ, DKBLUE);

    recorder.record_poscolor(ORGN, GREY);
    recorder.record_poscolor(-AXISX, GREY);
    recorder.record_poscolor(ORGN, GREY);
    recorder.record_poscolor(-AXISY, GREY);
    recorder.record_poscolor(ORGN, GREY);
    recorder.record_poscolor(-AXISZ, GREY);

    glLineWidth(3);
    recorder.draw(GL_LINES);
}

void initRendering()
{
    // Clear to black
    glClearColor(0, 0, 0, 1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void updateMesh()
{
    // Update the bone to world transforms for SSD.
    skeleton->updateCurrentJointToWorldTransforms();
    // update the mesh given the new skeleton
    skeleton->updateMesh();
}

/*
   initializes a simple NanoGUI-based UI
   must call freeGUI() when done.

   This function implements a simple GUI with three sliders
   for each joint. You won't have to touch it, but feel free
   to add your own features.

   The GUI is drawn in the same window as the main application.
   Any mouse and keyboard events, we first send to the GUI. If the
   GUI didn't handle the event, we forward it to the event handler
   functions above.

   Once initialized, the GUI is drawn in the main loop of the application
   The GUI is drawn in the same window as the main application.
   Any mouse and keyboard events, we first send to the GUI. If the
   GUI didn't handle the event, we forward it to the event handler
   functions above.

   Once initialized, the GUI is drawn in the main loop of the
   application.
*/
void initGUI(GLFWwindow* glfwwin) {
    screen = new ng::Screen();
    screen->initialize(glfwwin, false);

    screen->performLayout();

    // nanoGUI wants to handle events.
    // We forward GLFW events to nanoGUI first. If nanoGUI didn't handle
    // the event, we pass it to the handler routine.
    glfwSetCursorPosCallback(glfwwin,
        [](GLFWwindow* window, double x, double y) {
        if (gMousePressed) {
            // sticky mouse gestures
            motionCallback(window, x, y);
            return;
        }
        if (screen->cursorPosCallbackEvent(x, y)) {
            return;
        }
        motionCallback(window, x, y);
    }
    );

    glfwSetMouseButtonCallback(glfwwin,
        [](GLFWwindow* window, int button, int action, int modifiers) {
        if (screen->mouseButtonCallbackEvent(button, action, modifiers)) {
            return;
        }
        mouseCallback(window, button, action, modifiers);
    }
    );

    glfwSetKeyCallback(glfwwin,
        [](GLFWwindow* window, int key, int scancode, int action, int mods) {
        if (screen->keyCallbackEvent(key, scancode, action, mods)) {
            return;
        }
        keyCallback(window, key, scancode, action, mods);
    }
    );

    glfwSetCharCallback(glfwwin,
        [](GLFWwindow *, unsigned int codepoint) {
        screen->charCallbackEvent(codepoint);
    }
    );

    glfwSetDropCallback(glfwwin,
        [](GLFWwindow *, int count, const char **filenames) {
        screen->dropCallbackEvent(count, filenames);
    }
    );

    glfwSetScrollCallback(glfwwin,
        [](GLFWwindow *, double x, double y) {
        screen->scrollCallbackEvent(x, y);
    }
    );

    glfwSetFramebufferSizeCallback(glfwwin,
        [](GLFWwindow *, int width, int height) {
        screen->resizeCallbackEvent(width, height);
    }
    );
}
void freeGUI() {
    delete screen;
    screen = nullptr;
}

void loadSkeleton(const std::string& basepath) {
    skeleton = new SkeletalModel();
    string skelfile = basepath + ".skel";
    string objfile = basepath + ".obj";
    string attachfile = basepath + ".attach";
    skeleton->load(skelfile.c_str(), objfile.c_str(), attachfile.c_str());
}


void jumpingSkeleton(float elapsed_s) {
    float y = (2.0f*elapsed_s - 2.0f*pow(elapsed_s,2));
    if (y > 0) {
        Matrix4f m = Matrix4f::translation(0, y, 0);
        skeleton->translateSkeleton(m);
    }
    updateMesh();
}

void runningSkeleton(float elapsed_s) {

    // shoulders
//    float rs = abs(cosf(elapsed_s/2.0f))*M_PI*0.5f;
//    float ls = abs(sinf(elapsed_s/2.0f))*M_PI*0.5f;
    skeleton->setJointTransform(12, M_PI*0.35f, 0, 0);
    skeleton->setJointTransform(16, M_PI*0.35f, 0, 0);

//    // hips
//    skeleton->setJointTransform(3, -M_PI*0.2f, 0, 0);

    // legs
    g_jointangles[5][0] = -(cosf(elapsed_s)+0.2)*M_PI*0.4f;
    skeleton->setJointTransform(5, g_jointangles[5].x(), g_jointangles[5].y(), g_jointangles[5].z());

    g_jointangles[9][0] = +(cosf(elapsed_s)-0.2)*M_PI*0.4f;
    skeleton->setJointTransform(9, g_jointangles[9].x(), g_jointangles[9].y(), g_jointangles[9].z());


    //knees
    g_jointangles[6][0] = abs(cosf(elapsed_s/2.0f))*M_PI*0.5f;
    skeleton->setJointTransform(6, g_jointangles[6].x(), g_jointangles[6].y(), g_jointangles[6].z());

    g_jointangles[10][0] = abs(sinf(elapsed_s/2.0f))*M_PI*0.5f;
    skeleton->setJointTransform(10, g_jointangles[10].x(), g_jointangles[10].y(), g_jointangles[10].z());
    updateMesh();
}

void updateSkeleton() {
    if (skeleton) {


        // update animation
        float elapsed_s = timer.elapsed();

        // for a skelet0ne that jumps
        if (jumping) {
            jumpingSkeleton(elapsed_s);
        } else {
            // and a skelet0ne that runs
            runningSkeleton(elapsed_s);
        }

    }
}
void freeSkeleton() {
    delete skeleton;
    skeleton = nullptr;
}


}


int main(int argc, char** argv)
{
    if (argc < 2)
    {
        cout << "Usage: " << argv[0] << " PREFIX" << endl;
        cout << "For example, if you're trying to load data/Model1.skel, data/Model1.obj, and data/Model1.attach, run with: " << argv[0] << " data/Model1" << endl;
        return -1;
    }
    basepath = argv[1];

    window = createOpenGLWindow(1024, 1024, "Assignment 2");

    initGUI(window);
    initRendering();

    // The program object controls the programmable parts
    // of OpenGL. All OpenGL programs define a vertex shader
    // and a fragment shader.
    program_color = compileProgram(c_vertexshader, c_fragmentshader_color);
    if (!program_color) {
        printf("Cannot compile program\n");
        return -1;
    }

    camera.SetPerspective(50);
    camera.SetDistance(1.5);
    camera.SetCenter(Vector3f(-0.5, -0.5, -0.5));


    loadSkeleton(basepath);

    timer.set();
    // Main Loop
    while (!glfwWindowShouldClose(window)) {

        // Clear the rendering window
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw nanogui
        screen->drawContents();
        screen->drawWidgets();
        glEnable(GL_DEPTH_TEST);

        setViewport(window);

        if (gDrawAxisAlways || gMousePressed) {
            drawAxis();
        }

        updateSkeleton();
        skeleton->draw(camera, gDrawSkeleton);

        // Make back buffer visible
        glfwSwapBuffers(window);

        // Check if any input happened during the last frame
        glfwPollEvents();
    }
    freeSkeleton();

    // All OpenGL resource that are created with
    // glGen* or glCreate* must be freed.
    freeGUI();
    glDeleteProgram(program_color);

    glfwTerminate(); // destroy the window
    return 0;
}
