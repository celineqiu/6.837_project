#include "timestepper.h"

#include <cstdio>

void ForwardEuler::takeStep(ParticleSystem* particleSystem, float stepSize)
{
   //TODO: See handout 3.1
    std::vector<Vector3f> s = particleSystem->getState();
    std::vector<Vector3f> f = particleSystem->evalF(s);
    std::vector<Vector3f> newState;
    for (int i = 0; i < s.size(); i++) {
        newState.push_back(s[i] + f[i]*stepSize);
}

    particleSystem->setState(newState);
}

void Trapezoidal::takeStep(ParticleSystem* particleSystem, float stepSize)
{
   //TODO: See handout 3.1
    std::vector<Vector3f> s = particleSystem->getState();
    std::vector<Vector3f> f0 = particleSystem->evalF(s);

    std::vector<Vector3f> nextState;
    for (int i = 0; i < s.size(); i++) {
        nextState.push_back(s[i] + f0[i]*stepSize);
    }

    std::vector<Vector3f> f1 = particleSystem->evalF(nextState);

    std::vector<Vector3f> newState;
    for (int i = 0; i < s.size(); i++) {
        newState.push_back(s[i] + (f0[i]+f1[i])*stepSize/2.);
    }
    particleSystem->setState(newState);

}


void RK4::takeStep(ParticleSystem* particleSystem, float stepSize)
{
    std::vector<Vector3f> s = particleSystem->getState();
    std::vector<Vector3f> k1 = particleSystem->evalF(s);

    std::vector<Vector3f> next;
    for (int i = 0; i < s.size(); i++) {
        next.push_back(s[i] + k1[i]*stepSize/2.);
    }
    std::vector<Vector3f> k2 = particleSystem->evalF(next);

    for (int i = 0; i < s.size(); i++) {
        next[i] = s[i] + k2[i]*stepSize/2.;
    }
    std::vector<Vector3f> k3 = particleSystem->evalF(next);

    for (int i = 0; i < s.size(); i++) {
        next[i] = s[i] + k3[i]*stepSize/2.;
    }
    std::vector<Vector3f> k4 = particleSystem->evalF(next);

    std::vector<Vector3f> newState;
    for (int i = 0; i < s.size(); i++) {
        newState.push_back(s[i] + (stepSize/6.)*(k1[i]+2*k2[i]+2*k3[i]+k4[i]));
    }

    particleSystem->setState(newState);

}

