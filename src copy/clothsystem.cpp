#include "clothsystem.h"
#include "vertexrecorder.h"
#include "math.h"

 // your system should at least contain 8x8 particles.
const int W = 12;
const int H = 10;
const float springConstant = 180.0f;
const float springLength = 0.25f;
const float GRAVITY_CONSTANT = -9.81f;
const float DRAG_CONSTANT = 4.8f;

int indexOf(int i, int j);

bool breezeOn = false;
float t = 0.0f;

ClothSystem::ClothSystem() {

    Vector3f O(2, 3, 0);
    // TODO 5. Initialize m_vVecState with cloth particles. 
    // You can again use rand_uniform(lo, hi) to make things a bit more interesting
    //particleSprings;
    for (int i = 0; i < W*H; i++) {
        int x = (i%W)*springLength;
        int y = -floor(i*1.0f/H)*springLength;
        m_vVecState.push_back(O+Vector3f(x+rand_uniform(-0.1f, 0.1f), rand_uniform(-0.1f, 0.1f), rand_uniform(0.0f, 0.5f)));
        m_vVecState.push_back(Vector3f(rand_uniform(-0.1f, 0.1f), rand_uniform(-0.1f, 0.1f), rand_uniform(-0.1f, 0.1f)));
    }

    m_vVecState[0] = O;
    m_vVecState[2*(W-1)] = O+Vector3f(springLength*(W-3), 0, 0.1f);

    m_vVecState[0+1] = Vector3f();
    m_vVecState[2*(W-1)+1] = Vector3f();

    // STRUCTURAL SPRINGS: horizontal springs first
    for (int i=0; i < W-1; i++) {
        for (int j=0; j < H; j++) {
            int p1 = indexOf(i, j);
            int p2 = indexOf(i+1, j);
            particleSprings.push_back(Vector4f(p1, p2, springConstant, springLength));
        }
    }

    // vertical springs next
    for (int i=0; i < W; i++) {
        for (int j=0; j < H-1; j++) {
            int p1 = indexOf(i, j);
            int p2 = indexOf(i, j+1);
            particleSprings.push_back(Vector4f(p1, p2, springConstant, springLength));
        }
    }

    //SHEAR SPRINGS
    for (int i=0; i < W-1; i++) {
        for (int j=0; j < H-1; j++) {
            int p1 = indexOf(i, j);
            int p2 = indexOf(i+1, j);
            int p3 = indexOf(i, j+1);
            int p4 = indexOf(i+1, j+1);
            particleSprings.push_back(Vector4f(p1, p4, springConstant, sqrt(2)*springLength));
            particleSprings.push_back(Vector4f(p2, p3, springConstant, sqrt(2)*springLength));
        }
    }

    // FLEX SPRINGS: horizontal springs first
    for (int i=0; i < W-2; i++) {
        for (int j=0; j < H; j++) {
            int p1 = indexOf(i, j);
            int p2 = indexOf(i+2, j);
            particleSprings.push_back(Vector4f(p1, p2, springConstant, 2.0f*springLength));
        }
    }

    // vertical springs next
    for (int i=0; i < W; i++) {
        for (int j=0; j < H-2; j++) {
            int p1 = indexOf(i, j);
            int p2 = indexOf(i, j+2);
            particleSprings.push_back(Vector4f(p1, p2, springConstant, 2.0f*springLength));
        }
    }


}

int indexOf(int i, int j) {
    if (i < W && j < H) {
        return 2*(W*j + i);
    } else {
        return -1;
    }
}

Vector3f evalGrav(Vector3f pos) {
    Vector3f f = Vector3f(0, GRAVITY_CONSTANT, 0);
    return f;
}

Vector3f evalDra(Vector3f pos, Vector3f vel, float dragConstant) {
    Vector3f f = -vel*dragConstant;
    return f;
}

Vector3f evalSpr(Vector3f pos, Vector3f pos2, float springConstant, float length) {
    Vector3f d = pos-pos2;
    // k(||d|| - r) * unit(d);
    if (springConstant == 0) {
        return Vector3f();
    }

    Vector3f f = -springConstant*(d.abs()-length)*d.normalized();
    return f;

}

void ClothSystem::toggleBreeze() {
    breezeOn = !breezeOn;
}

std::vector<Vector3f> ClothSystem::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f(state.size());
    // TODO 5. implement evalF
    // - gravity
    // - viscous drag
    // - structural springs
    // - shear springs
    // - flexion springs

    // skip the first particle, since it's fixed

    for (int i=0; i < state.size()-1; i+=2) {
        Vector3f pos = state[i];
        Vector3f vel = state[i+1];
        Vector3f gravF = evalGrav(pos);
        Vector3f dragF = evalDra(pos, vel, DRAG_CONSTANT);

        f[i] = vel;
        f[i+1] = gravF+dragF;

        // if breeze is on
        if (breezeOn) {
            f[i+1] += cos(t)*Vector3f(0, 0, rand_uniform(0.0f, 10.0f)) + Vector3f(0, 0, rand_uniform(-5.0f, 15.0f));
        }
    }

    if (breezeOn) {
        t+=0.005;
    }

    //update the springs
    for (Vector4f spring : particleSprings) {
        Vector3f springF = evalSpr(state[spring[0]], state[spring[1]], spring[2], spring[3]);
        f[spring[0]+1] += springF;
        f[spring[1]+1] -= springF;
    }

    // fix the top left/right corners:
    int pLeft = indexOf(0, 0);
    int pRight = indexOf(W-1, 0);

    f[pLeft] = Vector3f();
    f[pRight] = Vector3f();
    return f;
}

void drawNormals(std::vector<Vector3f> state) {
    VertexRecorder rec;
    for (int i = 0; i < W-1; ++i) {
        for (int j=0; j < H-1; ++j) {
            Vector3f v0 = state[indexOf(i, j)];
            Vector3f v1 = state[indexOf(i+1, j)];
            Vector3f v2 = state[indexOf(i, j+1)];
            Vector3f v3 = state[indexOf(i+1, j+1)];

            Vector3f N1 = Vector3f::cross(v2 - v0, v1 - v0).normalized();
            Vector3f N2 = Vector3f::cross(v2 - v1, v3 - v1).normalized();

            rec.record(v0, N1);
            rec.record(v1, N1);
            rec.record(v2, N1);

            rec.record(v1, N2);
            rec.record(v2, N2);
            rec.record(v3, N2);
        }

    }
    rec.draw();
}


void ClothSystem::draw(GLProgram& gl)
{
    //TODO 5: render the system 
    //         - ie draw the particles as little spheres
    //         - or draw the springs as little lines or cylinders
    //         - or draw wireframe mesh

    const Vector3f CLOTH_COLOR(0.9f, 0.9f, 0.9f);
    gl.updateMaterial(CLOTH_COLOR);

    gl.updateModelMatrix(Matrix4f::identity());
    drawNormals(m_vVecState);

//    float w = 1.0f;
//
//    for (int i=0; i < W*H; i++) {
//        gl.updateModelMatrix(Matrix4f::translation(m_vVecState[2*i]));
//        drawSphere(0.04f, 8, 8);
//    }
//    VertexRecorder rec;
//    gl.disableLighting();
//    gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
//
//    // draw STRUCTURAL SPRINGS: horizontal springs first
//    for (int i=0; i < W-1; i++) {
//        for (int j=0; j < H; j++) {
//            int p1 = indexOf(i, j);
//            int p2 = indexOf(i+1, j);
//            rec.record(m_vVecState[p1], CLOTH_COLOR);
//            rec.record(m_vVecState[p2], CLOTH_COLOR);
//        }
//    }
//
//    // vertical springs next
//    for (int i=0; i < W; i++) {
//        for (int j=0; j < H-1; j++) {
//            int p1 = indexOf(i, j);
//            int p2 = indexOf(i, j+1);
//            rec.record(m_vVecState[p1], CLOTH_COLOR);
//            rec.record(m_vVecState[p2], CLOTH_COLOR);
//        }
//    }
//
//    glLineWidth(3.0f);
//    rec.draw(GL_LINES);
//    gl.enableLighting();
}

