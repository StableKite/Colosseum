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

            /*********** optional parameters ***********/
            real_T linear_drag_coefficient;
            //sample value 1.3 from http://klsin.bpmsg.com/how-fast-can-a-quadcopter-fly/, but divided by 4 to account
            // for nice streamlined frame design and allow higher top speed which is more fun.
            //angular coefficient is usually 10X smaller than linear, however we should replace this with exact number
            //http://physics.stackexchange.com/q/304742/14061
            real_T angular_drag_coefficient;
            real_T restitution; // value of 1 would result in perfectly elastic collisions, 0 would be completely inelastic.
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
        /// Initializes 4 rotors in the usual QuadX pattern:  http://ardupilot.org/copter/_images/MOTORS_QuadX_QuadPlus.jpg
        /// which shows that given an array of 4 motors, the first is placed top right and flies counter clockwise (CCW) and
        /// the second is placed bottom left, and also flies CCW.  The third in the array is placed top left and flies clockwise (CW)
        /// while the last is placed bottom right and also flies clockwise.  This is how the 4 items in the arm_lengths and
        /// arm_angles arrays will be used.  So arm_lengths is 4 numbers (in meters) where four arm lengths, 0 is top right,
        /// 1 is bottom left, 2 is top left and 3 is bottom right.  arm_angles is 4 numbers (in degrees)  relative to forward vector (0,1),
        /// provided in the same order where 0 is top right, 1 is bottom left, 2 is top left and 3 is bottom right, so for example,
        /// the angles for a regular symmetric X pattern would be 45, 225, 315, 135.  The rotor_z is the offset of each motor upwards
        /// relative to the center of mass (in meters).

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
                Quaternionr angle(AngleAxisr(-arm_angles[i] * M_PIf / 180, unit_z));
                rotor_poses.emplace_back(
                    VectorMath::rotateVector(Vector3r(arm_lengths[i], 0, rotor_z), angle, true),
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
                    else throw std::invalid_argument("Invalid multirotor direction: " + setup_params.rotor_directions[i]);
                }
            }

            // set up physics
            params.linear_drag_coefficient = setup_params.linear_drag_coefficient;
            params.angular_drag_coefficient = setup_params.angular_drag_coefficient;
            params.restitution = setup_params.restitution; // value of 1 would result in perfectly elastic collisions, 0 would be completely inelastic
            params.friction = setup_params.friction;

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

            std::string arm_lengths_str = "";
            std::string arm_angles_str = "";
            std::string rotor_directions_str = "";
            for (uint i = 0; i < arm_lengths.size(); i++) arm_lengths_str += std::to_string(arm_lengths[i]);
            for (uint i = 0; i < arm_angles.size(); i++) arm_angles_str += std::to_string(arm_angles[i]);
            for (uint i = 0; i < setup_params.rotor_directions.size(); i++) rotor_directions_str += " " + setup_params.rotor_directions[i];

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
