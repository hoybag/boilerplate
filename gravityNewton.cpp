#include "gravityNewton.h"

/*
* gravityNewton.cpp
*
* Takes an N by 3 matrix representing 3D coordinates of N bodies
* and the corresponding vector of length N representing their masses
* and returns an N by 3 matrix of the net acceleration due to gravity
*
* Usage : from MATLAB
*         >> outMatrix = gravityNewton(pos, mass)
*
* This is a C++ MEX-file for MATLAB.
*
*/

class MexFunction : public matlab::mex::Function {
public:
    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        // prepare inputs
        checkArguments(outputs, inputs);
        matlab::data::TypedArray<double> pos = std::move(inputs[0]);
        matlab::data::TypedArray<double> mass = std::move(inputs[1]);

        // create an empty matrix to hold acceleration results
        size_t N = pos.getDimensions()[0];
        matlab::data::ArrayDimensions dims = { N, 3 };
        matlab::data::ArrayFactory factory;
        matlab::data::TypedArray<double> accel = factory.createArray<double>(dims);

        // populate accel and return
        calculateGravity(pos, mass, accel);
        outputs[0] = std::move(accel);
    }

    void checkArguments(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        matlab::data::ArrayFactory factory;

        if (inputs.size() != 2) {
            matlabPtr->feval(u"error",
                0, std::vector<matlab::data::Array>({ factory.createScalar("Two inputs required") }));
        }
    }
};