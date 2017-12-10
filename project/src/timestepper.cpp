//
// Created by Helen Ho on 11/28/17.
//

#include "timestepper.h"

#include <cstdio>

void ForwardEuler::takeStep(Mesh* m_mesh, float stepSize)
{
    //TODO: See handout 3.1
    std::vector<Vector3f> viss = m_mesh->getVisState();
    std::vector<Vector3f> vel = m_mesh->getVelocities();
    std::vector<Vector3f> f = m_mesh->evalF(viss);
    std::vector<Vector3f> newState;
    std::vector<Vector3f> newVel;
    for (int i = 0; i < viss.size(); i++) {
        newState.push_back(viss[i] + vel[i]*stepSize);
        newVel.push_back(vel[i] + f[i]*stepSize);
    }

    m_mesh->setState(newState, newVel);
}

void Trapezoidal::takeStep(Mesh* m_mesh, float stepSize)
{
    //TODO: See handout 3.1
    std::vector<Vector3f> viss = m_mesh->getVisState();
    std::vector<Vector3f> vel = m_mesh->getVelocities();
    std::vector<Vector3f> f0 = m_mesh->evalF(viss);

    std::vector<Vector3f> nextVel;
    std::vector<Vector3f> nextState;
    for (int i = 0; i < viss.size(); i++) {
        nextState.push_back(viss[i] + vel[i]*stepSize);
        nextVel.push_back(vel[i] + f0[i]*stepSize);
    }

    std::vector<Vector3f> f1 = m_mesh->evalF(nextState);

    std::vector<Vector3f> newState;
    std::vector<Vector3f> newVel;
    for (int i = 0; i < viss.size(); i++) {
        newState.push_back(viss[i] + (vel[i]+nextVel[i])*stepSize/2.);
        newVel.push_back(vel[i] + (f0[i]+f1[i])*stepSize/2);
    }
    m_mesh->setState(newState, newVel);

}


void RK4::takeStep(Mesh* m_mesh, float stepSize)
{
    std::vector<Vector3f> s = m_mesh->getVisState();
    std::vector<Vector3f> vel = m_mesh->getVelocities();
    std::vector<Vector3f> k1 = m_mesh->evalF(s);

    std::vector<Vector3f> next;
    for (int i = 0; i < s.size(); i++) {
        next.push_back(s[i] + k1[i]*stepSize/2.);
    }
    std::vector<Vector3f> k2 =  m_mesh->evalF(next);

    for (int i = 0; i < s.size(); i++) {
        next[i] = s[i] + k2[i]*stepSize/2.;
    }
    std::vector<Vector3f> k3 =  m_mesh->evalF(next);

    for (int i = 0; i < s.size(); i++) {
        next[i] = s[i] + k3[i]*stepSize/2.;
    }
    std::vector<Vector3f> k4 =  m_mesh->evalF(next);

    std::vector<Vector3f> newState;
    std::vector<Vector3f> newVel;
    for (int i = 0; i < s.size(); i++) {
        newVel.push_back(vel[i] + (stepSize/6.)*(k1[i]+2*k2[i]+2*k3[i]+k4[i]));

        newState.push_back(s[i] + vel[i]*stepSize/2.);
    }

    m_mesh->setState(newState, newVel);

}

