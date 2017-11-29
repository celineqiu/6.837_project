#include "pendulumsystem.h"

#include <cassert>
#include "camera.h"
#include "vertexrecorder.h"

// TODO adjust to number of particles.
const int NUM_PARTICLES = 4;
const float GRAVITY_CONSTANT = -9.81f;
const float DRAG_CONSTANT = 0.3f;

PendulumSystem::PendulumSystem()
{

    // TODO 4.2 Add particles for simple pendulum
//    m_vVecState.push_back(Vector3f(-0.5, 1.0, 0));
//    m_vVecState.push_back(Vector3f());
//
//    m_vVecState.push_back(Vector3f(rand_uniform(-0.5f, 0.5f), rand_uniform(-0.5f, 0.5f), rand_uniform(-0.5f, 0.5f)));
//    m_vVecState.push_back(Vector3f(rand_uniform(-1.0, 1.0f), rand_uniform(-1.0, 1.0f), rand_uniform(-1.0, 1.0f)));
//    particleSprings.push_back(Vector4f(0, 2, 32, 0.15));

    // TODO 4.3 Extend to multiple particles
    // fix a point at the origin
    m_vVecState.push_back(Vector3f(-0.5, 1.0, 0));
    m_vVecState.push_back(Vector3f());

    for (int i=0; i < NUM_PARTICLES; i++) {
        m_vVecState.push_back(Vector3f(rand_uniform(-0.1f, 0.1f), rand_uniform(-0.1f, 0.1f), rand_uniform(-0.1f, 0.1f)));
        m_vVecState.push_back(Vector3f(rand_uniform(-0.1f, 0.1f), rand_uniform(-0.1f, 0.1f), rand_uniform(-0.1f, 0.1f)));

        // spring between last particle and here
        // <pos1, pos2, spring constant, spring length>
        particleSprings.push_back(Vector4f(2*i, 2*i+2, 32, 0.15));
    }
}

Vector3f evalGravity(Vector3f pos) {
    Vector3f f = Vector3f(0, GRAVITY_CONSTANT, 0);
    return f;
}

Vector3f evalDrag(Vector3f pos, Vector3f vel, float dragConstant) {
    Vector3f f = -vel*dragConstant;
    return f;
}

Vector3f evalSpring(Vector3f pos, Vector3f pos2, float springConstant, float length) {
    Vector3f d = pos-pos2;
    // k(||d|| - r) * unit(d);
    if (springConstant == 0) {
        return Vector3f();
    }

    Vector3f f = -springConstant*(d.abs()-length)*d.normalized();
    return f;

}

std::vector<Vector3f> PendulumSystem::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f(state.size());
    // TODO 4.1: implement evalF
    //  - gravity
    //  - viscous drag
    //  - springs

    // skip the first particle, since it's fixed
    for (int i=2; i < state.size()-1; i+=2) {
        Vector3f pos = state[i];
        Vector3f vel = state[i+1];
        Vector3f gravF = evalGravity(pos);
        Vector3f dragF = evalDrag(pos, vel, DRAG_CONSTANT);

        f[i] = vel;
        f[i+1] = gravF+dragF;
    }

    //update the springs
    for (Vector4f spring : particleSprings) {
        Vector3f springF = evalSpring(state[spring[0]], state[spring[1]], spring[2], spring[3]);
        f[spring[0]+1] += springF;
        f[spring[1]+1] -= springF;
    }

    return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw(GLProgram& gl)
{
    const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
    gl.updateMaterial(PENDULUM_COLOR);

    // TODO 4.2, 4.3
    // example code. Replace with your own drawing  code

    for (int i = 0; i < m_vVecState.size()-1; i+=2) {
        gl.updateModelMatrix(Matrix4f::translation(m_vVecState[i]));
        drawSphere(0.075f, 10, 10);
    }

}
