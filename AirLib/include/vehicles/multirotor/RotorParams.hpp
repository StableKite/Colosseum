// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_RotorParams_hpp
#define msr_airlib_RotorParams_hpp

#include "common/Common.hpp"

namespace msr
{
namespace airlib
{

    //In NED system, +ve torque would generate clockwise rotation
    enum class RotorTurningDirection : int
    {
        RotorTurningDirectionCCW = -1,
        RotorTurningDirectionCW = 1
    };

    struct RotorParams
    {
        /*
            Ref: http://physics.stackexchange.com/a/32013/14061
            force in Newton = C_T * \rho * n^2 * D^4
            torque in N.m = C_P * \rho * n^2 * D^5 / (2*pi)
            where,
            \rho = air density (1.225 kg/m^3)
            n = revolutions per sec
            D = propeller diameter in meters
            C_T, C_P = dimensionless constants available at
            propeller performance database http://m-selig.ae.illinois.edu/props/propDB.html

            We use values for GWS 9X5 propeller for which,
            C_T = 0.109919, C_P = 0.040164 @ 6396.667 RPM
        */
       
        real_T C_T; // the thrust co-efficient
        real_T C_P; // the torque co-efficient
        real_T air_density; // kg/m^3
        real_T max_rpm; // revolutions per minute
        real_T propeller_diameter; //diameter in meters
        real_T propeller_height; //height of cylindrical area when propeller rotates
        real_T control_signal_filter_tc; //time constant for low pass filter

        real_T revolutions_per_second; // RPM
        real_T max_speed; // in radians per second
        real_T max_speed_square; // max speed in square
        real_T max_thrust; //computed from above formula for the given constants
        real_T max_torque; //computed from above formula

        // call this method to recalculate thrust if you want to use different numbers for C_T, C_P, max_rpm, etc.
        void calculateMaxThrust()
        {
            revolutions_per_second = max_rpm / 60;
            max_speed = revolutions_per_second * 2 * M_PIf; // radians / sec
            max_speed_square = pow(max_speed, 2.0f);

            real_T nsquared = revolutions_per_second * revolutions_per_second;
            max_thrust = C_T * air_density * nsquared * static_cast<real_T>(pow(propeller_diameter, 4));
            max_torque = C_P * air_density * nsquared * static_cast<real_T>(pow(propeller_diameter, 5)) / (2 * M_PIf);
        }
    };
}
} //namespace
#endif
