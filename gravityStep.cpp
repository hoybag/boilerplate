/*
* gravityStep.cpp
*
* Takes in the current position, velocity, acceleration, mass of N bodies
* as well as the step size to output the next pos, vel, and accel.
* Uses the velocity Verlet integrator:
* 1. v(t + delta_t/2) = v(t) + 1/2*a(t)*delta_t
* 2. x(t + delta_t) = x(t) + v(t + delta_t/2)*delta_t
* 3. call calculateGravity() to replace a(t) with a(t + delta_t)
* 4. v(t + delta_t) = v(t + delta_t/2) + 1/2*a(t + delta_t)*delta_t
* 
* calculateGravity defined in gravityNewton.h is used to calculate the acceleration matrix
* 
* Currently the radis is just ignored, but
* should the time allows, implement collision detection and response later.
* 
* pos: N by 3 matrix of current 3D positions
* vel: N by 3 matrix of 3D velocities
* accel: N by 3 matrix of 3D accelerations
* mass: vector with length N containing each body's mass
* 
* pos_stepped: N by 3 matrix of 3D positions after delta_t
* vel_stepped: N by 3 matrix of 3D velocities after delta_t
* accel_stepped: N by 3 matrix of 3D accelerations after delta_t, due to gravity
*
* Usage : from MATLAB
*         >> [pos_stepped, vel_stepped, accel_stepped] = gravityStep(pos, vel, accel, delta_t, mass, radius)
*
* This is a C++ MEX-file for MATLAB.
*
*/

#include "mex.hpp"
#include "mexAdapter.hpp"

#include "gravityNewton.h"

class MexFunction : public matlab::mex::Function {
public:
    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);
        matlab::data::TypedArray<double> pos = std::move(inputs[0]);
        matlab::data::TypedArray<double> vel = std::move(inputs[1]);
        matlab::data::TypedArray<double> accel = std::move(inputs[2]);
        double delta_t = inputs[3][0];
        matlab::data::TypedArray<double> mass = std::move(inputs[4]);
        //matlab::data::TypedArray<double> radii = std::move(inputs[5]);

        velocityVerlet(pos, vel, accel, delta_t, mass);
        outputs[0] = std::move(pos);
        outputs[1] = std::move(vel);
        outputs[2] = std::move(accel);
    }

    void velocityVerlet(matlab::data::TypedArray<double>& pos,
                        matlab::data::TypedArray<double>& vel,
                        matlab::data::TypedArray<double>& accel,
                        matlab::data::TypedArray<double>& mass,
                        double delta_t) {
        size_t N = pos.getDimensions()[0];
        for (int i = 0; i < N; i++) {
            // step 1
            // overwrite vel matrix to store v(t + delta_t/2)
            vel[i][0] += accel[i][0] * delta_t / 2; // v_x
            vel[i][1] += accel[i][0] * delta_t / 2; // v_y
            vel[i][2] += accel[i][0] * delta_t / 2; // v_z

            // step 2
            // overwrite the pos matrix to store pos(t + delta_t)
            pos[i][0] += vel[i][0] * delta_t;
            pos[i][1] += vel[i][1] * delta_t;
            pos[i][2] += vel[i][2] * delta_t;

            // step 3
            // replace accel at time = t with accel at time = (t + delta_t)
            calculateGravity(pos, mass, accel);

            // step 4
            // overwrite v(t + delta_t/2) with v(t + delta_t)
            vel[i][0] += accel[i][0] * delta_t / 2;
            vel[i][1] += accel[i][1] * delta_t / 2;
            vel[i][2] += accel[i][2] * delta_t / 2;
        }

    }

    void checkArguments(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        /*std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        matlab::data::ArrayFactory factory;

        if (inputs.size() != 2) {
            matlabPtr->feval(u"error",
                0, std::vector<matlab::data::Array>({ factory.createScalar("Two inputs required") }));
        }

        if (inputs[0].getNumberOfElements() != 1) {
            matlabPtr->feval(u"error",
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input multiplier must be a scalar") }));
        }

        if (inputs[0].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[0].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error",
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input multiplier must be a noncomplex scalar double") }));
        }

        if (inputs[1].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[1].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error",
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input matrix must be type double") }));
        }

        if (inputs[1].getDimensions().size() != 2) {
            matlabPtr->feval(u"error",
                0, std::vector<matlab::data::Array>({ factory.createScalar("Input must be m-by-n dimension") }));
        }
        */
    }
};