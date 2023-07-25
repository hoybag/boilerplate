#pragma once
#ifndef GRAVITY_NEWTON_H
#define GRAVITY_NEWTON_H
/*
* Helper function that calculates the acceleration due to gravity for each body.
* F = G * m1 * m2 / r^2
*/

#include "mex.hpp"
#include "mexAdapter.hpp"

#include <cmath>

const double G = 1.0; // Gravitational constant

/*  caculateGravity
*   Implementation uses brute force pairwise calculations.
*   If there is time, improve this by using Barnes-Hut later.
* 
*   pos: N x 3 matrix that represents the spatial coordinates of N bodies in 3-dimensions
*   mass: 1D array of length N that represents each body's mass
*   accel: N x 3 matrix whose values will be overwritten. Each row will be the acceleration due to gravity. 
*/
void calculateGravity(const matlab::data::TypedArray<double>& pos,
                      const matlab::data::TypedArray<double>& mass,
                      matlab::data::TypedArray<double>& accel) {

    size_t N = pos.getDimensions()[0]; // get # of bodies

    // For each ith body, calculate the acceleration vector due to gravity: a_vec_i
    for (int i = 0; i < N; i++) {
        // 1. Clear the acceleration data
        accel[i][0] = 0.0;
        accel[i][1] = 0.0;
        accel[i][2] = 0.0;

        // 2. Calculate a_vec_ij for all j (except j = i) and sum them up to get a_vec_i.
        for (int j = 0; j < N; j++) {
            if (i == j) continue; // don't calculate gravity for a body with itself

            // calculate r_vec_ij, the position vector pointing from body i to body j
            double dx = pos[j][0] - pos[i][0];
            double dy = pos[j][1] - pos[i][1];
            double dz = pos[j][2] - pos[i][2];

            // a_vec_ij = (G * m_j / ||r_vec_ij||^3) r_vec_ij (Newton's universal law of gravitation)
            double r = std::sqrt(dx * dx + dy * dy + dz * dz);
            double gFactor = G * mass[j] / (r * r * r);

            // update a_vec_i with a_vec_ij contribution
            accel[i][0] += gFactor * dx;
            accel[i][1] += gFactor * dy;
            accel[i][2] += gFactor * dz;
        }
    }
}

#endif