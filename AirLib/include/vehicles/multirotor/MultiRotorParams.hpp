// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_MultiRotorParameters_hpp
#define msr_airlib_MultiRotorParameters_hpp

#include "common/Common.hpp"
#include "RotorParams.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"

namespace msr
{
namespace airlib
{

    class MultiRotorParams
    {
        //All units are SI
    public: //types
        struct RotorPose
        {
            Vector3r position; //relative to center of gravity of vehicle body
            Vector3r normal;
            RotorTurningDirection direction;

            RotorPose()
            {
            }
            RotorPose(const Vector3r& position_val, const Vector3r& normal_val, RotorTurningDirection direction_val)
                : position(position_val), normal(normal_val), direction(direction_val)
            {
            }
        };

        struct Params
        {
            /*********** required parameters ***********/
            uint rotor_count;
            vector<RotorPose> rotor_poses;
            real_T mass;
            Matrix3x3r inertia;
            Vector3r body_box;

            /*********** optional parameters with defaults ***********/
            real_T linear_drag_coefficient;
            real_T angular_drag_coefficient;
            real_T restitution;
            real_T friction;
            RotorParams rotor_params;
        };

    protected: //must override by derived class
        virtual void setupParams() = 0;
        virtual const SensorFactory* getSensorFactory() const = 0;

    public: //interface
        virtual std::unique_ptr<MultirotorApiBase> createMultirotorApi() = 0;

        virtual ~MultiRotorParams() = default;
        virtual void initialize(const AirSimSettings::VehicleSetting* vehicle_setting)
        {
            sensor_storage_.clear();
            sensors_.clear();

            setupParams();

            addSensorsFromSettings(vehicle_setting);
        }

        const Params& getParams() const
        {
            return params_;
        }
        Params& getParams()
        {
            return params_;
        }
        SensorCollection& getSensors()
        {
            return sensors_;
        }
        const SensorCollection& getSensors() const
        {
            return sensors_;
        }

        void addSensorsFromSettings(const AirSimSettings::VehicleSetting* vehicle_setting)
        {
            const auto& sensor_settings = vehicle_setting->sensors;

            getSensorFactory()->createSensorsFromSettings(sensor_settings, sensors_, sensor_storage_);
        }

    protected: //static utility functions for derived classes to use
        /// Initialize the rotor_poses given the rotor_count, the arm lengths and the arm angles (relative to forwards vector).
        /// Also provide the direction you want to spin each rotor and the z-offset of the rotors relative to the center of gravity.
        static void initializeRotors(
            vector<RotorPose>& rotor_poses,
            uint rotor_count,
            const std::vector<real_T>& arm_lengths, 
            const std::vector<real_T>& arm_angles, 
            const std::vector<RotorTurningDirection>& rotor_directions,
            real_T rotor_z /* z relative to center of gravity */
        )
        {
            Vector3r unit_z(0, 0, -1); //NED frame
            rotor_poses.clear();
            for (uint i = 0; i < rotor_count; i++)
            {
                // vectors below are rotated according to NED left hand rule (so the vectors are rotated counter clockwise)
                Quaternionr angle(AngleAxisr(-arm_angles[i] * M_PIf / 180, unit_z));
                rotor_poses.emplace_back(
                    VectorMath::rotateVector(
                        Vector3r(arm_lengths[i], 0, rotor_z),
                        angle,
                        true
                    ),
                    unit_z,
                    rotor_directions[i]
                );
            }
        }

        static void computeInertiaMatrix(Matrix3x3r& inertia, const Vector3r& body_box, const vector<RotorPose>& rotor_poses,
                                         real_T box_mass, real_T motor_assembly_weight)
        {
            inertia = Matrix3x3r::Zero();

            //http://farside.ph.utexas.edu/teaching/336k/Newtonhtml/node64.html
            inertia(0, 0) = box_mass / 12.0f * (body_box.y() * body_box.y() + body_box.z() * body_box.z());
            inertia(1, 1) = box_mass / 12.0f * (body_box.x() * body_box.x() + body_box.z() * body_box.z());
            inertia(2, 2) = box_mass / 12.0f * (body_box.x() * body_box.x() + body_box.y() * body_box.y());

            for (size_t i = 0; i < rotor_poses.size(); ++i) {
                const auto& pos = rotor_poses.at(i).position;
                inertia(0, 0) += (pos.y() * pos.y() + pos.z() * pos.z()) * motor_assembly_weight;
                inertia(1, 1) += (pos.x() * pos.x() + pos.z() * pos.z()) * motor_assembly_weight;
                inertia(2, 2) += (pos.x() * pos.x() + pos.y() * pos.y()) * motor_assembly_weight;
            }
        }

        // Some Frame types which can be used by different firmwares
        // Specific frame configurations, modifications can be done in the Firmware Params

        void setupFrame(
            Params& params,
            const AirSimSettings::MultirotorPhysicsSettings& setup_params
        )
        {
            /*
            Motor placements:
            RotorCount 4:
                 x
            (2)  |   (0)
                 |
            ------------ y
                 |
            (1)  |   (3)
                 |
            
            RotorCount 6:
                x-axis
             (2)    (4)
                \  /
                 \/
            (1)-------(0) y-axis
                 /\
                /  \
              (5)  (3)

            RotorCount 8:
                    x-axis
                 (4)  |  (0) 
                      |
            (6)       |       (2)
            __________|__________  y-axis
                      |
            (5)       |       (7)
                      |
                 (1)  |  (3)
            */

            //set up arm lengths
            params.rotor_count = static_cast<unsigned int>(setup_params.rotor_count);
 
            std::vector<real_T> arm_lengths(setup_params.arm_lengths.begin(), setup_params.arm_lengths.end());

            Vector3r unit_z(0, 0, -1); // NED frame

            std::vector<real_T> arm_angles(setup_params.arm_angles.begin(), setup_params.arm_angles.end());

            // quad X pattern
            std::vector<RotorTurningDirection> rotor_directions;
            for (uint i = 0; i < setup_params.rotor_directions.size(); i++)
            {
                if (Utils::toLower(setup_params.rotor_directions[i]) == "cw") rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCW);
                else
                {
                    if (Utils::toLower(setup_params.rotor_directions[i]) == "ccw") rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCCW);
                    else throw std::invalid_argument("Invaid multirotor direction: " + setup_params.rotor_directions[i]);
                }
            }

            //set up mass
            params.mass = setup_params.mass;
            real_T motor_assembly_weight = setup_params.motor_assembly_weight;
            real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

            // setup rotor params
            params.rotor_params.C_T = setup_params.rotor_params.C_T; // the thrust co-efficient
            params.rotor_params.C_P = setup_params.rotor_params.C_P; // the torque co-efficient
            params.rotor_params.air_density = setup_params.rotor_params.air_density; // kg/m^3
            params.rotor_params.max_rpm = setup_params.rotor_params.max_rpm; // revolutions per minute
            params.rotor_params.propeller_diameter = setup_params.rotor_params.propeller_diameter; // diameter in meters
            params.rotor_params.propeller_height = setup_params.rotor_params.propeller_height; // height of cylindrical area when propeller rotates
            params.rotor_params.control_signal_filter_tc = setup_params.rotor_params.control_signal_filter_tc; // time constant for low pass filter
            params.rotor_params.calculateMaxThrust();
            if (setup_params.rotor_params.max_thrust > 0) params.rotor_params.max_thrust = setup_params.rotor_params.max_thrust;
            if (setup_params.rotor_params.max_torque > 0) params.rotor_params.max_torque = setup_params.rotor_params.max_torque;

            //set up dimensions of core body box or abdomen (not including arms)
            params.body_box.x() = setup_params.body_box[0];
            params.body_box.y() = setup_params.body_box[1];
            params.body_box.z() = setup_params.body_box[2];
            real_T rotor_z = setup_params.rotor_z;

            //computer rotor poses
            initializeRotors(params.rotor_poses, params.rotor_count, arm_lengths, arm_angles, rotor_directions, rotor_z);

            //compute inertia matrix
            computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
        }

    private:
        Params params_;
        SensorCollection sensors_; //maintains sensor type indexed collection of sensors
        vector<shared_ptr<SensorBase>> sensor_storage_; //RAII for created sensors
    };
}
} //namespace
#endif
