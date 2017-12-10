//
// Created by Helen Ho on 11/28/17.
//

#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "vecmath.h"
#include <vector>
#include "mesh.h"

class TimeStepper
{
public:
    virtual ~TimeStepper() {}
    virtual void takeStep(Mesh* m_mesh, float stepSize) = 0;
};

//IMPLEMENT YOUR TIMESTEPPERS

class ForwardEuler : public TimeStepper
{
    void takeStep(Mesh* m_mesh, float stepSize) override;
};

class Trapezoidal : public TimeStepper
{
    void takeStep(Mesh* m_mesh, float stepSize) override;
};

class RK4 : public TimeStepper
{
    void takeStep(Mesh* m_mesh, float stepSize) override;
};


/////////////////////////
#endif
