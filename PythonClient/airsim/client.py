from __future__ import print_function
from sys import stderr
from warnings import warn
from dataclasses import asdict
from msgpackrpc.future import Future
from msgpackrpc import Client, Address
from typing import Optional, Literal, List
from .types import RgbaType, ImageRequest, ImageResponse, MeshPositionVertexBuffersResponse, \
    DrivetrainType, ImageType, YawMode, GeoPoint, Pose, Vector3r, WeatherParameter, MessageSeverity, \
    CollisionInfo, DetectionInfo, CameraInfo, DistortionParams, \
    KinematicsState, EnvironmentState, MultirotorState, RotorStates, CarState, CarControls, \
    ImuData, BarometerData, MagnetometerData, GpsData, DistanceSensorData, LidarData, RCData, \
    AngleRateControllerGains, AngleLevelControllerGains, VelocityControllerGains, PositionControllerGains


class VehicleClient:
    def __init__(
            self,
            ip: str = "127.0.0.1",
            port: int = 41451,
            timeout_value: Optional[int] = 3600,
            reconnect_limit: int = 5
        ):
        """
        Initializing connection to simulator

        Args:
            ip (str): Simulator IP address
            port (int): Simulator port
            timeout_value (Optional[int]): Timeout, in seconds
            reconnect_limit (int): Limit of reconnection attempts
        """

        self.client = Client(
            Address(ip, port),
            timeout = timeout_value,
            reconnect_limit = reconnect_limit,
            pack_encoding = "utf-8",
            unpack_encoding = "utf-8"
        )


# Common vehicle APIs:
    def reset(self):
        """
        Reset the vehicle to its original starting state

        Note that you must call `enable_api_control` and `arm_disarm`
        again after the call to reset
        """

        self.client.call("reset")


    def ping(self):
        """
        If connection is established then this call
        will return true otherwise it will be blocked until timeout

        Returns:
            is_connected (bool): Is connected
        """

        return self.client.call("ping")


    def get_client_version(self) -> int:
        return 1  # sync with C++ client


    def get_server_version(self):
        return self.client.call("getServerVersion")


    def get_min_required_server_version(self) -> int:
        return 1  # sync with C++ client


    def get_min_required_client_version(self):
        return self.client.call("getMinRequiredClientVersion")


# Basic flight control:
    def enable_api_control(
            self,
            is_enabled: bool,
            vehicle_name: str = ""
        ):
        """
        Enables or disables API control
        for vehicle corresponding to vehicle_name

        Args:
            is_enabled (bool):
                True to enable, False to disable API control
            vehicle_name (str):
                Name of the vehicle to send this command to
        """

        self.client.call("enableApiControl", is_enabled, vehicle_name)


    def is_api_control_enabled(self, vehicle_name: str = ""):
        """
        Returns true if API control is established.

        If false (which is default) then API calls would be ignored.
        After a successful call to `enableApiControl`,
        `isApiControlEnabled` should return True

        Args:
            vehicle_name (str): Name of the vehicle

        Returns:
            bool: If API control is enabled
        """

        return self.client.call("isApiControlEnabled", vehicle_name)


    def arm_disarm(self, arm: bool, vehicle_name: str = ""):
        """
        Arms or disarms vehicle

        Args:
            arm (bool): True to arm, False to disarm the vehicle
            vehicle_name (str):
                Name of the vehicle to send this command to

        Returns:
            bool: Success
        """

        return self.client.call("armDisarm", arm, vehicle_name)


    def sim_pause(self, is_paused: bool):
        """
        Pauses simulation

        Args:
            is_paused (bool):
                True to pause the simulation, False to release
        """

        self.client.call("simPause", is_paused)


    def sim_is_pause(self) -> bool:
        """
        Returns true if the simulation is paused

        Returns:
            is_pause (bool): Is the simulation on pause
        """

        return self.client.call("simIsPaused")


    def sim_continue_for_time(self, seconds: float):
        """
        Continue the simulation for the specified number of seconds

        Args:
            seconds (float): Time to run the simulation for
        """

        self.client.call("simContinueForTime", seconds)


    def sim_continue_for_frames(self, frames: int):
        """
        Continue (or resume if paused) the simulation
        for the specified number of frames,
        after which the simulation will be paused.

        Args:
            frames (int): Frames to run the simulation for
        """

        self.client.call("simContinueForFrames", frames)


    def get_home_geo_point(self, vehicle_name: str = "") -> GeoPoint:
        """
        Get the Home location of the vehicle

        Args:
            vehicle_name (str): Name of vehicle to get home location of

        Returns:
            GeoPoint: Home location of the vehicle
        """

        return GeoPoint.from_msgpack(self.client.call("getHomeGeoPoint", vehicle_name))


    def confirm_connection(self):
        """
        Checks state of connection every 1 sec and reports it in Console
        so user can see the progress for connection
        """

        if self.ping():
            print("Connected!")
        else:
            print("Ping returned false!")
        
        server_ver = self.get_server_version()
        client_ver = self.get_client_version()
        server_min_ver = self.get_min_required_server_version()
        client_min_ver = self.get_min_required_client_version()

        ver_info = "Client Ver:" + str(client_ver) + " (Min Req: " + str(client_min_ver) + \
              "), Server Ver:" + str(server_ver) + " (Min Req: " + str(server_min_ver) + ")"

        if server_ver < server_min_ver:
            print(ver_info, file = stderr)
            print(
                "AirSim server is of older version and not supported by this client. \
                Please upgrade!"
            )
        elif client_ver < client_min_ver:
            print(ver_info, file = stderr)
            print(
                "AirSim client is of older version and not supported by this server. \
                Please upgrade!"
            )
        else:
            print(ver_info)
        print("")


    def sim_set_light_intensity(self, light_name: str, intensity: float) -> bool:
        """
        Change intensity of named light

        Args:
            light_name (str): Name of light to change
            intensity (float): New intensity value

        Returns:
            status (bool): True if successful, otherwise False
        """

        return self.client.call("simSetLightIntensity", light_name, intensity)


    def sim_swap_textures(
            self,
            tags: str,
            tex_id: int = 0,
            component_id: int = 0,
            material_id: int = 0
        ) -> List[str]:
        """
        Runtime Swap Texture API.
        See https://microsoft.github.io/AirSim/retexturing/ for details

        Args:
            tags (str):
                String of "," or ", " delimited tags to identify
                on which actors to perform the swap
            tex_id (int):
                Indexes the array of textures assigned to each actor
                undergoing a swap. If out-of-bounds for some object's
                texture set, it will be taken modulo the number
                of textures that were available
            component_id (int): Id of component
            material_id (int): Id of component matherial

        Returns:
            swapped_list (List[str]):
                List of objects which matched the provided tags
                and had the texture swap perfomed
        """

        return self.client.call("simSwapTextures", tags, tex_id, component_id, material_id)


    def sim_set_object_material(
            self,
            object_name: str,
            material_name: str,
            component_id: int = 0
        ) -> bool:
        """
        Runtime Swap Texture API
        See https://microsoft.github.io/AirSim/retexturing/ for details
        Args:
            object_name (str): Name of object to set material for
            material_name (str): Name of material to set for object
            component_id (int): Index of material elements

        Returns:
            is_matherial_set (bool): True if material was set
        """

        return self.client.call("simSetObjectMaterial", object_name, material_name, component_id)


    def sim_set_object_material_from_texture(
            self,
            object_name: str,
            texture_path: str,
            component_id: int = 0
        ) -> bool:
        """
        Runtime Swap Texture API
        See https://microsoft.github.io/AirSim/retexturing/ for details
        Args:
            object_name (str): Name of object to set material for
            texture_path (str): Path to texture to set for object
            component_id (int): Index of material elements

        Returns:
            is_matherial_set (bool): True if material was set
        """

        return self.client.call("simSetObjectMaterialFromTexture", object_name, texture_path, component_id)


# Time-of-day control:
    def sim_set_time_of_day(
            self,
            is_enabled: bool,
            start_datetime: str = "",
            is_start_datetime_dst: bool = False,
            celestial_clock_speed: float = 1.,
            update_interval_secs: float = 60.,
            move_sun: bool = True
        ):
        """
        Control the position of Sun in the environment.
        Sun's position is computed using the coordinates specified
        in `OriginGeopoint` in settings for the date-time specified
        in the argument, else if the string is empty,
        current date & time is used

        Args:
            is_enabled (bool):
                True to enable time-of-day effect,
                False to reset the position to original
            start_datetime (str):
                Date & Time in %Y-%m-%d %H:%M:%S format,
                e.g. `2018-02-12 15:20:00`
            is_start_datetime_dst (bool):
                True to adjust for Daylight Savings Time
            celestial_clock_speed (float):
                Run celestial clock fasteror slower than simulation
                clock. E.g. Value 100 means for every 1 second of
                simulation clock, Sun's position is advanced
                by 100 seconds so Sun will move in sky much faster
            update_interval_secs (float):
                Interval to update the Sun's position
            move_sun (bool): Whether or not to move the Sun
        """

        self.client.call(
            "simSetTimeOfDay",
            is_enabled,
            start_datetime,
            is_start_datetime_dst,
            celestial_clock_speed,
            update_interval_secs,
            move_sun
        )


# Weather:
    def sim_enable_weather(self, enable: bool):
        """
        Enable Weather effects.
        Needs to be called before using `sim_set_weather_parameter` API

        Args:
            enable (bool): True to enable, False to disable
        """

        self.client.call("simEnableWeather", enable)


    def sim_set_weather_parameter(self, param: WeatherParameter, val: float):
        """
        Enable various weather effects

        Args:
            param (WeatherParameter): Weather effect to be enabled
            val (float): Intensity of the effect, Range 0-1
        """

        self.client.call("simSetWeatherParameter", param, val)


# Camera control:
    def sim_get_image(
            self,
            camera_name: str,
            image_type: ImageType,
            vehicle_name: str = "",
            external: bool = False
        ) -> bytes:
        """
        Get a single image.
        Returns bytes of png format image which can be dumped
        into abinary file to create .png image
        See https://microsoft.github.io/AirSim/image_apis/ for details

        Args:
            camera_name (str):
                Name of the camera
            image_type (ImageType): Type of image required
            vehicle_name (str): Name of the vehicle with the camera
            external (bool): Whether the camera is an External Camera

        Returns:
            binary_png_image (bytes):
                Binary string literal of compressed png image
        """

        # Because this method returns std::vector<uint8>,
        # msgpack decides to encode it as a string unfortunately
        result = self.client.call("simGetImage", camera_name, image_type, vehicle_name, external)
        if (result == "" or result == "\0"):
            return None
        return result


    def sim_get_images(
            self,
            requests: List[ImageRequest],
            vehicle_name: str = "",
            external: bool = False
        ) -> List[ImageResponse]:
        """
        Get multiple images.
        See https://microsoft.github.io/AirSim/image_apis/
        for details and examples

        Args:
            requests (List[ImageRequest]): Images required
            vehicle_name (str):
                Name of vehicle associated with the camera
            external (bool): Whether the camera is an External Camera

        Returns:
            images_list (List[ImageResponse]): List of sim images
        """

        responses_raw = self.client.call("simGetImages", requests, vehicle_name, external)
        return [ImageResponse.from_msgpack(response_raw) for response_raw in responses_raw]


# CinemAirSim:
    def sim_get_preset_lens_settings(
            self,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ) -> List[str] | None:
        result = self.client.call("simGetPresetLensSettings", camera_name, vehicle_name, external)
        if (result == "" or result == "\0"):
            return None
        return result


    def sim_get_lens_settings(
            self,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ) -> str:  
        result = self.client.call("simGetLensSettings", camera_name, vehicle_name, external)
        if (result == "" or result == "\0"):
            return None
        return result


    def sim_set_preset_lens_settings(
            self,
            preset_lens_settings,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ):  
        self.client.call(
            "simSetPresetLensSettings",
            preset_lens_settings,
            camera_name,
            vehicle_name,
            external
        )


    def sim_get_preset_filmback_settings(
            self,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ):  
        result = self.client.call(
            "simGetPresetFilmbackSettings",
            camera_name,
            vehicle_name,
            external
        )
        if (result == "" or result == "\0"):
            return None
        return result


    def sim_set_preset_filmback_settings(
            self,
            preset_filmback_settings,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ):  
        self.client.call(
            "simSetPresetFilmbackSettings",
            preset_filmback_settings,
            camera_name,
            vehicle_name,
            external
        )


    def sim_get_filmback_settings(
            self,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ) -> str:  
        result = self.client.call("simGetFilmbackSettings", camera_name, vehicle_name, external)
        if (result == "" or result == "\0"):
            return None
        
        return result


    def sim_set_filmback_settings(
            self,
            sensor_width: int,
            sensor_height: int,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ):  
        return self.client.call(
            "simSetFilmbackSettings",
            sensor_width,
            sensor_height,
            camera_name,
            vehicle_name,
            external
        )


    def sim_get_focal_length(
            self,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ) -> float:  
        return self.client.call("simGetFocalLength", camera_name, vehicle_name, external)


    def sim_set_focal_length(
            self,
            focal_length: float,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ):  
        self.client.call("simSetFocalLength", focal_length, camera_name, vehicle_name, external)


    def sim_enable_manual_focus(
            self,
            enable: bool,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ):  
        self.client.call("simEnableManualFocus", enable, camera_name, vehicle_name, external)


    def sim_get_focus_distance(
            self,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ) -> float:  
        return self.client.call("simGetFocusDistance", camera_name, vehicle_name, external)


    def sim_set_focus_distance(
            self,
            focus_distance: float,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ):  
        self.client.call("simSetFocusDistance", focus_distance, camera_name, vehicle_name, external)


    def sim_get_focus_aperture(
            self,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ) -> float:  
        return self.client.call("simGetFocusAperture", camera_name, vehicle_name, external)


    def sim_set_focus_aperture(
            self,
            focus_aperture: float,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ):  
        self.client.call("simSetFocusAperture", focus_aperture, camera_name, vehicle_name, external)


    def sim_enable_focus_plane(
            self,
            enable: bool,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ):  
        self.client.call("simEnableFocusPlane", enable, camera_name, vehicle_name, external)


    def sim_get_current_field_of_view(
            self,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ) -> str:  
        return self.client.call("simGetCurrentFieldOfView", camera_name, vehicle_name, external)
# End CinemAirSim
    

    def sim_test_line_of_sight_to_point(self, point: GeoPoint, vehicle_name: str = "") -> bool:
        """
        Returns whether the target point is visible
        from the perspective of the inputted vehicle

        Args:
            point (GeoPoint): Target point
            vehicle_name (str): Name of vehicle

        Returns:
            Is_success (bool): Is success
        """

        return self.client.call("simTestLineOfSightToPoint", point, vehicle_name)


    def sim_test_line_of_sight_between_points(self, point1: GeoPoint, point2: GeoPoint) -> bool:
        """
        Returns whether the target point is visible
        from the perspective of the source point

        Args:
            point1 (GeoPoint): Source point
            point2 (GeoPoint): Target point

        Returns:
            is_success (bool): Is success
        """

        return self.client.call("simTestLineOfSightBetweenPoints", point1, point2)


    def sim_get_world_extents(self) -> List[GeoPoint]:
        """
        Returns a list of GeoPoints representing the minimum
        and maximum extents of the world

        Returns:
            geo_points_list (List[GeoPoint]): A list of GeoPoints
        """

        responses_raw = self.client.call("simGetWorldExtents")
        return [GeoPoint.from_msgpack(response_raw) for response_raw in responses_raw]


    def sim_run_console_command(self, command: str):
        """
        Allows the client to execute a command in
        Unreal's native console, via an API.
        Affords access to the countless built-in commands such as
        "stat unit", "stat fps", "open [map]",
        adjust any config settings, etc. etc.
        Allows the user to create bespoke APIs very easily,
        by adding a custom event to the level blueprint,
        and then calling the console command "ce MyEventName [args]".
        No recompilation of AirSim needed!

        Args:
            command (str): Desired Unreal Engine Console command to run

        Returns:
            is_success (bool): Is success
        """

        return self.client.call("simRunConsoleCommand", command)


    def sim_get_mesh_position_vertex_buffers(self) -> List[MeshPositionVertexBuffersResponse]:
        """
        Gets the static meshes in the UE scene.
        See https://microsoft.github.io/AirSim/meshes/
        for details and how to use this

        Returns:
            meshes_list (List[MeshPositionVertexBuffersResponse]):
                Returns the static meshes that make up the scene
        """

        responses_raw = self.client.call("simGetMeshPositionVertexBuffers")
        return [MeshPositionVertexBuffersResponse.from_msgpack(response_raw) for response_raw in responses_raw]


    def sim_get_collision_info(self, vehicle_name: str = "") -> CollisionInfo:
        """
        Args:
            vehicle_name (str): Name of the Vehicle to get the info of

        Returns:
            collision_info (CollisionInfo):
                Information about vehile collisions
        """

        return CollisionInfo.from_msgpack(self.client.call("simGetCollisionInfo", vehicle_name))


    def sim_set_vehicle_pose(
            self,
            pose: Pose,
            ignore_collision: bool,
            vehicle_name: str = ""
        ):
        """
        Set the pose of the vehicle.
        If you don't want to change position (or orientation)
        then just set components of position (or orientation)
        to floating point nan values

        Args:
            pose (Pose): Desired Pose pf the vehicle
            ignore_collision (bool):
                Whether to ignore any collision or not
            vehicle_name (str): Name of the vehicle to move
        """

        self.client.call("simSetVehiclePose", pose, ignore_collision, vehicle_name)


    def sim_get_vehicle_pose(self, vehicle_name: str = "") -> Pose:
        """
        The position inside the returned Pose is in the frame
        of the vehicle's starting point
        
        Args:
            vehicle_name (str): Name of the vehicle to get the Pose of

        Returns:
            vehile_pose (Pose): Vehile pose
        """

        pose = self.client.call("simGetVehiclePose", vehicle_name)
        return Pose.from_msgpack(pose)


    def sim_set_trace_line(
            self,
            color_rgba: RgbaType,
            thickness: float = 1.0,
            vehicle_name: str = ""
        ):
        """
        Modify the color and thickness of the line
        when Tracing is enabled.
        Tracing can be enabled by pressing T in the Editor or setting
        `EnableTrace` to `True` in the Vehicle Settings

        Args:
            color_rgba (RgbaType): Desired RGBA values from 0.0 to 1.0
            thickness (float): Thickness of the line
            vehicle_name (string):
                Name of the vehicle to set Trace line values for
        """

        self.client.call("simSetTraceLine", color_rgba, thickness, vehicle_name)


    def sim_get_object_pose(self, object_name: str) -> Pose:
        """
        The position inside the returned Pose is in the world frame

        Args:
            object_name (str): Object to get the Pose of

        Returns:
            object_pose (Pose): Object pose
        """

        pose = self.client.call("simGetObjectPose", object_name)
        return Pose.from_msgpack(pose)


    def sim_set_object_pose(
            self,
            object_name: str,
            pose: Pose,
            teleport: bool = True
        ) -> bool:
        """
        Set the pose of the object (actor) in the environment.
        The specified actor must have Mobility set to movable,
        otherwise there will be undefined behaviour.
        See https://www.unrealengine.com/en-US/blog/moving-physical-objects
        for details on how to set Mobility
        and the effect of Teleport parameter

        Args:
            object_name (str): Name of the object(actor) to move
            pose (Pose): Desired Pose of the object
            teleport (bool):
                Whether to move the object immediately
                without affecting their velocity

        Returns:
            is_success (bool): If the move was successful
        """

        return self.client.call("simSetObjectPose", object_name, pose, teleport)


    def sim_get_object_scale(self, object_name: str) -> Vector3r:
        """
        Gets scale of an object in the world

        Args:
            object_name (str): Object to get the scale of

        Returns:
            scale (Vector3r): Scale of object
        """

        scale = self.client.call("simGetObjectScale", object_name)
        return Vector3r.from_msgpack(scale)


    def sim_set_object_scale(self, object_name: str, scale_vector: Vector3r) -> bool:
        """
        Sets scale of an object in the world

        Args:
            object_name (str): Object to set the scale of
            scale_vector (Vector3r): Desired scale of object

        Returns:
            bool: True if scale change was successful
        """

        return self.client.call("simSetObjectScale", object_name, scale_vector)


    def sim_list_scene_objects(self, name_regex: str = ".*") -> List[str]:
        """
        Lists the objects present in the environment.
        Default behaviour is to list all objects, regex can be used
        to return smaller list of matching objects or actors

        Args:
            name_regex (str):
                String to match actor names against, e.g. "Cylinder.*"

        Returns:
            objects_list (List[str]): List containing all the names
        """

        return self.client.call("simListSceneObjects", name_regex)


    def sim_list_scene_objects_by_tag(self, tag_regex: str = ".*") -> List[str]:
        """
        Lists the objects present in the environment
        by searching their tags.
        Default behaviour is to list all objects, regex can be used
        to return smaller list of matching objects or actors

        Args:
            tag_regex (str): String to match actor tags against, e.g. "Tag.*"

        Returns:
            objects_list (List[str]): List containing all the names
        """

        return self.client.call("simListSceneObjectsByTag", tag_regex)


    def sim_load_level(self, level_name: str) -> bool:
        """
        Loads a level specified by its name

        Args:
            level_name (str): Name of the level to load

        Returns:
            is_success (bool): True if the level was successfully loaded
        """

        return self.client.call("simLoadLevel", level_name)


    def sim_list_assets(self) -> List[str]:
        """
        Lists all the assets present in the Asset Registry

        Returns:
            assets_list (List[str]): Names of all the assets
        """

        return self.client.call("simListAssets")


    def sim_spawn_object(
            self,
            object_name: str,
            asset_name: str,
            pose: Pose,
            scale: Vector3r,
            physics_enabled: bool = False,
            is_blueprint: bool = False
        ) -> str:
        """
        Spawned selected object in the world

        Args:
            object_name (str): Desired name of new object
            asset_name (str):
                Name of asset(mesh) in the project database
            pose (Pose): Desired pose of object
            scale (Vector3r): Desired scale of object
            physics_enabled (bool):
                Whether to enable physics for the object
            is_blueprint (bool):
                Whether to spawn a blueprint or an actor

        Returns:
            object_name (str):
                Name of spawned object, in case it had to be modified
        """

        return self.client.call(
            "simSpawnObject",
            object_name,
            asset_name,
            pose,
            scale,
            physics_enabled,
            is_blueprint
        )


    def sim_destroy_object(self, object_name: str) -> bool:
        """
        Removes selected object from the world

        Args:
            object_name (str): Name of object to be removed

        Returns:
            is_success (bool): True if object is queued up for removal
        """

        return self.client.call("simDestroyObject", object_name)


    def sim_set_segmentation_object_id(
            self,
            mesh_name: str,
            object_id: int,
            is_name_regex: bool = False
        ) -> bool:
        """
        Set segmentation ID for specific objects.
        See https://microsoft.github.io/AirSim/image_apis/#segmentation
        for details

        Args:
            mesh_name (str):
                Name of the mesh to set the ID of (supports regex)
            object_id (int):
                Object ID to be set, range 0-255.
                RBG values for IDs can be seen at
                https://microsoft.github.io/AirSim/seg_rgbs.txt
            is_name_regex (bool): Whether the mesh name is a regex

        Returns:
            is_mesh_found (bool): If the mesh was found
        """

        return self.client.call("simSetSegmentationObjectID", mesh_name, object_id, is_name_regex)


    def sim_get_segmentationObjectID(self, mesh_name: str) -> int:
        """
        Returns Object ID for the given mesh name.
        Mapping of Object IDs to RGB values can be seen at
        https://microsoft.github.io/AirSim/seg_rgbs.txt

        Args:
            mesh_name (str): Name of the mesh to get the ID of

        Returns:
            id (int): ID of object
        """

        return self.client.call("simGetSegmentationObjectID", mesh_name)


    def sim_add_detection_filter_mesh_name(
            self,
            camera_name: str,
            image_type: ImageType,
            mesh_name: str,
            vehicle_name: str = "",
            external: bool = False
        ):
        """
        Add mesh name to detect in wild card format.
        For example: simAddDetectionFilterMeshName("Car_*")
        will detect all instance named "Car_*"

        Args:
            camera_name (str):
                Name of the camera
            image_type (ImageType): Type of image required
            mesh_name (str): Mesh name in wild card format
            vehicle_name (str):
                Vehicle which the camera is associated with
            external (bool): Whether the camera is an External Camera
        """

        self.client.call(
            "simAddDetectionFilterMeshName",
            camera_name,
            image_type,
            mesh_name,
            vehicle_name,
            external
        )


    def sim_set_detection_filter_radius(
            self,
            camera_name: str,
            image_type: ImageType,
            radius_cm: int,
            vehicle_name: str = "",
            external: bool = False
        ):
        """
        Set detection radius for all cameras

        Args:
            camera_name (str):
                Name of the camera
            image_type (ImageType): Type of image required
            radius_cm (int): Radius in [cm]
            vehicle_name (str):
                Vehicle which the camera is associated with
            external (bool): Whether the camera is an External Camera
        """

        self.client.call(
            "simSetDetectionFilterRadius",
            camera_name,
            image_type,
            radius_cm,
            vehicle_name,
            external
        )


    def sim_clear_detection_mesh_names(
            self,
            camera_name: str,
            image_type: ImageType,
            vehicle_name: str = "",
            external: bool = False
        ):
        """
        Clear all mesh names from detection filter

        Args:
            camera_name (str):
                Name of the camera
            image_type (ImageType): Type of image required
            vehicle_name (str):
                Vehicle which the camera is associated with
            external (bool): Whether the camera is an External Camera
        """

        self.client.call(
            "simClearDetectionMeshNames",
            camera_name,
            image_type,
            vehicle_name,
            external
        )


    def sim_get_detections(
            self,
            camera_name,
            image_type,
            vehicle_name = "",
            external = False
        ) -> List[DetectionInfo]:
        """
        Get current detections

        Args:
            camera_name (str):
                Name of the camera
            image_type (ImageType): Type of image required
            vehicle_name (str):
                Vehicle which the camera is associated with
            external (bool): Whether the camera is an External Camera

        Returns:
            detections_list (List[DetectionInfo]): Detections list
        """

        responses_raw = self.client.call(
            "simGetDetections",
            camera_name,
            image_type,
            vehicle_name,
            external
        )
        return [DetectionInfo.from_msgpack(response_raw) for response_raw in responses_raw]


    def sim_print_log_message(
            self,
            message: str,
            message_param: str = "",
            severity: MessageSeverity = MessageSeverity.FATAL
        ):
        """
        Prints the specified message in the simulator's window.
        If message_param is supplied, then it"s printed next
        to the message and in that case if this API is called
        with same message value but different message_param again
        then previous line is overwritten with new line
        (instead of API creating new line on display).
        For example, `simPrintLogMessage("Iteration: ", to_string(i))`
        keeps updating same line on display when API
        is called with different values of i.
        The valid values of severity parameter is 0 to 3 inclusive
        that corresponds to different colors.

        Args:
            message (str): Message to be printed
            message_param (str):
                Parameter to be printed next to the message
            severity (MessageSeverity):
                Range 0-3, inclusive, corresponding to the
                severity of the message
        """

        self.client.call("simPrintLogMessage", message, message_param, severity)


    def sim_get_camera_info(
            self,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ) -> CameraInfo:
        """
        Get details about the camera

        Args:
            camera_name (str):
                Name of the camera
            vehicle_name (str):
                Vehicle which the camera is associated with
            external (bool): Whether the camera is an External Camera

        Returns:
            camera_info (CameraInfo): Details about the camera
        """

        return CameraInfo.from_msgpack(
            self.client.call("simGetCameraInfo", camera_name, vehicle_name, external)
        )


    def sim_get_distortion_params(
            self,
            camera_name: str,
            vehicle_name: str = "",
            external: bool = False
        ) -> DistortionParams:
        """
        Get camera distortion parameters

        Args:
            camera_name (str):
                Name of the camera
            vehicle_name (str):
                Vehicle which the camera is associated with
            external (bool):
                Whether the camera is an External Camera

        Returns:
            distortion_params (DistortionParams):
                Distortion parameter values K1, K2, K3, P1, P2
        """

        return DistortionParams(
            *self.client.call("simGetDistortionParams", str(camera_name), vehicle_name, external)
        )


    def sim_set_distortion_params(
            self,
            camera_name: str,
            distortion_params: DistortionParams,
            vehicle_name: str = "",
            external: bool = False
        ):
        """
        Set camera distortion parameters

        Args:
            camera_name (str):
                Name of the camera
            distortion_params (DistortionParams):
                Distortion params with values K1, K2, K3, P1, P2
            vehicle_name (str):
                Vehicle which the camera is associated with
            external (bool): Whether the camera is an External Camera
        """

        for param_name, value in asdict(distortion_params).items():
            self.sim_set_distortion_param(
                camera_name,
                param_name.upper(),
                value,
                vehicle_name,
                external
            )


    def sim_set_distortion_param(
            self,
            camera_name: str,
            param_name: Literal["K1", "K2", "K3", "P1", "P2"],
            value: float,
            vehicle_name: str = "",
            external: bool = False
        ):
        """
        Set single camera distortion parameter

        Args:
            camera_name (str):
                Name of the camera
            param_name (str): Name of distortion parameter
            value (float): Value of distortion parameter
            vehicle_name (str):
                Vehicle which the camera is associated with
            external (bool): Whether the camera is an External Camera
        """

        self.client.call(
            "simSetDistortionParam",
            str(camera_name),
            param_name,
            value,
            vehicle_name,
            external
        )


    def sim_set_camera_pose(
            self,
            camera_name: str,
            pose: Pose,
            vehicle_name: str = "",
            external: bool = False
        ):
        """
        Control the pose of a selected camera

        Args:
            camera_name (str): Name of the camera to be controlled
            pose (Pose):
                Pose representing the desired position
                and orientation of the camera
            vehicle_name (str):
                Name of vehicle which the camera corresponds to
            external (bool): Whether the camera is an External Camera
        """

        self.client.call("simSetCameraPose", camera_name, pose, vehicle_name, external)


    def sim_set_camera_fov(
            self,
            camera_name: str,
            fov_degrees: float,
            vehicle_name: str = "",
            external: bool = False
        ):
        """
        Control the field of view of a selected camera

        Args:
            camera_name (str): Name of the camera to be controlled
            fov_degrees (float): Value of field of view in degrees
            vehicle_name (str):
                Name of vehicle which the camera corresponds to
            external (bool): Whether the camera is an External Camera
        """

        self.client.call("simSetCameraFov", camera_name, fov_degrees, vehicle_name, external)


    def sim_get_ground_truth_kinematics(self, vehicle_name: str = "") -> KinematicsState:
        """
        Get Ground truth kinematics of the vehicle.
        The position inside the returned KinematicsState
        is in the frame of the vehicle"s starting point

        Args:
            vehicle_name (str): Name of the vehicle

        Returns:
            kinematics_state (KinematicsState):
                Ground truth of the vehicle
        """

        kinematics_state = self.client.call("simGetGroundTruthKinematics", vehicle_name)
        return KinematicsState.from_msgpack(kinematics_state)


    def sim_set_kinematics(
            self,
            state: KinematicsState,
            ignore_collision: bool,
            vehicle_name: str = ""
        ):
        """
        Set the kinematics state of the vehicle.
        If you don't want to change position (or orientation)
        then just set components of position (or orientation)
        to floating point nan values

        Args:
            state (KinematicsState): Desired Pose pf the vehicle
            ignore_collision (bool): Whether to ignore any collision or not
            vehicle_name (str): Name of the vehicle to move
        """

        self.client.call("simSetKinematics", state, ignore_collision, vehicle_name)


    def sim_get_ground_truth_environment(self, vehicle_name: str = "") -> EnvironmentState:
        """
        Get ground truth environment state.
        The position inside the returned EnvironmentState
        is in the frame of the vehicle's starting point

        Args:
            vehicle_name (str): Name of the vehicle

        Returns:
            EnvironmentState: Ground truth environment state
        """

        env_state = self.client.call("simGetGroundTruthEnvironment", vehicle_name)
        return EnvironmentState.from_msgpack(env_state)


# Sensor APIs:
    def get_imu_data(self, imu_name: str = "", vehicle_name: str = "") -> ImuData:
        """
        Args:
            imu_name (str):
                Name of IMU to get data from, specified in settings.json
            vehicle_name (str):
                Name of vehicle to which the sensor corresponds to

        Returns:
            imu_data (ImuData): Data of vehicle IMU
        """
        
        return ImuData.from_msgpack(self.client.call("getImuData", imu_name, vehicle_name))


    def get_barometer_data(self, barometer_name: str = "", vehicle_name: str = "") -> BarometerData:
        """
        Args:
            barometer_name (str):
                Name of Barometer to get data from,
                specified in settings.json
            vehicle_name (str):
                Name of vehicle to which the sensor corresponds to

        Returns:
            barometer_data (BarometerData): Data of vehicle barometer
        """

        return BarometerData.from_msgpack(self.client.call("getBarometerData", barometer_name, vehicle_name))


    def get_magnetometer_data(
            self,
            magnetometer_name: str = "",
            vehicle_name: str = ""
        ) -> MagnetometerData:
        """
        Args:
            magnetometer_name (str):
                Name of Magnetometer to get data from,
                specified in settings.json
            vehicle_name (str):
                Name of vehicle to which the sensor corresponds to

        Returns:
            magnetometer_data (MagnetometerData):
                Data of vehicle magnetometer
        """

        return MagnetometerData.from_msgpack(
            self.client.call("getMagnetometerData", magnetometer_name, vehicle_name)
        )


    def get_gps_data(self, gps_name: str = "", vehicle_name: str = "") -> GpsData:
        """
        Args:
            gps_name (str):
                Name of GPS to get data from, specified in settings.json
            vehicle_name (str):
                Name of vehicle to which the sensor corresponds to

        Returns:
            gps_data (GpsData): Data of vehicle GPS
        """

        return GpsData.from_msgpack(self.client.call("getGpsData", gps_name, vehicle_name))


    def get_distance_sensor_data(self, distance_sensor_name: str = "", vehicle_name: str = "") -> DistanceSensorData:
        """
        Args:
            distance_sensor_name (str):
                Name of Distance Sensor to get data from,
                specified in settings.json
            vehicle_name (str):
                Name of vehicle to which the sensor corresponds to

        Returns:
            distance_sensor_data (DistanceSensorData): Data of vehicle distance sensor
        """

        return DistanceSensorData.from_msgpack(
            self.client.call("getDistanceSensorData", distance_sensor_name, vehicle_name)
        )


    def get_lidar_data(self, lidar_name: str = "", vehicle_name: str = "") -> LidarData:
        """
        Args:
            lidar_name (str):
                Name of Lidar to get data from,
                specified in settings.json
            vehicle_name (str):
                Name of vehicle to which the sensor corresponds to

        Returns:
            lidar_data (LidarData): Data of vehicle distance lidar
        """

        return LidarData.from_msgpack(self.client.call("getLidarData", lidar_name, vehicle_name))


    def sim_get_lidar_segmentation(self, lidar_name: str = "", vehicle_name: str = "") -> List[int]:
        """
        NOTE: Deprecated API, use `get_lidar_data()` API instead
        Returns Segmentation ID of each point's collided object
        in the last Lidar update

        Args:
            lidar_name (str): Name of Lidar sensor
            vehicle_name (str): Name of the vehicle wth the sensor

        Returns:
            segmentation_id_list (List[int]):
                Segmentation IDs of the objects
        """

        warn(
            "sim_get_lidar_segmentation API is deprecated, \
            use get_lidar_data() API instead"
        )

        return self.getLidarData(lidar_name, vehicle_name).segmentation


# Plotting APIs:
    def sim_flush_persistent_markers(self):
        """
        Clear any persistent markers - those plotted with setting
        `is_persistent = True` in the APIs below
        """

        self.client.call("simFlushPersistentMarkers")


    def sim_plot_points(
            self,
            points: List[Vector3r],
            color_rgba: RgbaType = [1.0, 0.0, 0.0, 1.0],
            size: float = 10.0,
            duration: float = -1.0,
            is_persistent: bool = False
        ):
        """
        Plot a list of 3D points in World NED frame

        Args:
            points (List[Vector3r]): List of Vector3r objects
            color_rgba (list): desired RGBA values from 0.0 to 1.0
            size (float): Size of plotted point
            duration (float): Duration (seconds) to plot for
            is_persistent (bool):
                If set to True, the desired object will be plotted
                for infinite time
        """

        self.client.call("simPlotPoints", points, color_rgba, size, duration, is_persistent)


    def sim_plot_line_strip(
            self,
            points: List[Vector3r],
            color_rgba: RgbaType = [1.0, 0.0, 0.0, 1.0],
            thickness: float = 5.0,
            duration: float = -1.0,
            is_persistent: bool = False
        ):
        """
        Plots a line strip in World NED frame, defined from points[0]
        to points[1], points[1] to points[2], ... ,
        points[n-2] to points[n-1]

        Args:
            points (List[Vector3r]):
                List of 3D locations of line start and end points,
                specified as Vector3r objects
            color_rgba (RgbaType): Desired RGBA values from 0.0 to 1.0
            thickness (float): Thickness of line
            duration (float): Duration (seconds) to plot for
            is_persistent (bool):
                If set to True, the desired object will be plotted
                for infinite time
        """

        self.client.call("simPlotLineStrip", points, color_rgba, thickness, duration, is_persistent)


    def sim_plot_line_list(
            self,
            points: List[Vector3r],
            color_rgba: RgbaType = [1.0, 0.0, 0.0, 1.0],
            thickness: float = 5.0,
            duration: float = -1.0,
            is_persistent: bool = False
        ):
        """
        Plots a line strip in World NED frame, defined from points[0]
        to points[1], points[2] to points[3], ... ,
        points[n-2] to points[n-1]

        Args:
            points (List[Vector3r]):
                List of 3D locations of line start and end points,
                specified as Vector3r objects. Must be even
            color_rgba (RgbaType): Desired RGBA values from 0.0 to 1.0
            thickness (float): Thickness of line
            duration (float): Duration (seconds) to plot for
            is_persistent (bool):
                If set to True, the desired object will be plotted
                for infinite time
        """

        self.client.call("simPlotLineList", points, color_rgba, thickness, duration, is_persistent)


    def sim_plot_arrows(
            self,
            points_start: List[Vector3r],
            points_end: List[Vector3r],
            color_rgba: RgbaType = [1.0, 0.0, 0.0, 1.0],
            thickness: float = 5.0,
            arrow_size: float = 2.0,
            duration: float = -1.0,
            is_persistent: bool = False
        ):
        """
        Plots a list of arrows in World NED frame,
        defined from points_start[0] to points_end[0], points_start[1]
        to points_end[1], ... , points_start[n-1] to points_end[n-1]

        Args:
            points_start (List[Vector3r]):
                List of 3D start positions of arrow start positions,
                specified as Vector3r objects
            points_end (List[Vector3r]):
                List of 3D end positions of arrow start positions,
                specified as Vector3r objects
            color_rgba (RgbaType): Desired RGBA values from 0.0 to 1.0
            thickness (float): Thickness of line
            arrow_size (float): Size of arrow head
            duration (float): Duration (seconds) to plot for
            is_persistent (bool):
                If set to True, the desired object will be plotted
                for infinite time
        """

        self.client.call(
            "simPlotArrows",
            points_start,
            points_end,
            color_rgba,
            thickness,
            arrow_size,
            duration,
            is_persistent
        )


    def sim_plot_strings(
            self,
            strings: List[str],
            positions: List[Vector3r],
            scale: float = 5.,
            color_rgba: RgbaType = [1.0, 0.0, 0.0, 1.0],
            duration: float = -1.0
        ):
        """
        Plots a list of strings at desired positions in World NED frame

        Args:
            strings (List[str]): List of strings to plot
            positions (List[Vector3r]):
                List of positions where the strings should be plotted.
                Should be in one-to-one correspondence with the
                strings' list
            scale (float): Font scale of transform name
            color_rgba (RgbaType): desired RGBA values from 0.0 to 1.0
            duration (float): Duration (seconds) to plot for
        """

        self.client.call("simPlotStrings", strings, positions, scale, color_rgba, duration)


    def sim_plot_transforms(
            self,
            poses: List[Pose],
            scale: float = 5.0,
            thickness: float = 5.0,
            duration: float = -1.0,
            is_persistent: bool = False
        ):
        """
        Plots a list of transforms in World NED frame

        Args:
            poses (List[Pose]):
                List of Pose objects representing the transforms to plot
            scale (float): Length of transforms" axes
            thickness (float): Thickness of transforms" axes
            duration (float): Duration (seconds) to plot for
            is_persistent (bool):
                If set to True, the desired object will be plotted
                for infinite time
        """

        self.client.call("simPlotTransforms", poses, scale, thickness, duration, is_persistent)


    def sim_plot_transforms_with_names(
            self,
            poses: List[Pose],
            names: List[str],
            tf_scale: float = 5.0,
            tf_thickness: float = 5.0,
            text_scale: float = 10.0,
            text_color_rgba: RgbaType = [1.0, 0.0, 0.0, 1.0],
            duration: float = -1.0
        ):
        """
        Plots a list of transforms with their names in World NED frame

        Args:
            poses (List[Pose]):
                List of Pose objects representing the transforms to plot
            names (List[str]):
                List of strings with one-to-one correspondence
                to list of poses
            tf_scale (float): Length of transforms" axes
            tf_thickness (float): Thickness of transforms" axes
            text_scale (float): Font scale of transform name
            text_color_rgba (list):
                Desired RGBA values from 0.0 to 1.0
                for the transform name
            duration (float): Duration (seconds) to plot for
        """

        self.client.call(
            "simPlotTransformsWithNames",
            poses,
            names,
            tf_scale,
            tf_thickness,
            text_scale,
            text_color_rgba,
            duration
        )


    def cancel_last_task(self, vehicle_name: str = ""):
        """
        Cancel previous Async task

        Args:
            vehicle_name (str): Name of the vehicle
        """

        self.client.call("cancelLastTask", vehicle_name)


# Recording APIs:
    def start_recording(self):
        """
        Start Recording

        Recording will be done according to the settings
        """

        self.client.call("startRecording")


    def stop_recording(self):
        """
        Stop Recording
        """

        self.client.call("stopRecording")


    def is_recording(self) -> bool:
        """
        Whether Recording is running or not

        Returns:
            is_recording (bool): True if Recording, else False
        """

        return self.client.call("isRecording")


    def sim_set_wind(self, wind: Vector3r):
        """
        Set simulated wind, in World frame, NED direction, m/s

        Args:
            wind (Vector3r): Wind, in World frame, NED direction, in m/s
        """

        self.client.call("simSetWind", wind)


    def sim_create_voxel_grid(
            self,
            position: Vector3r,
            x: int,
            y: int,
            z: int,
            res: float,
            of: str
        ):
        """
        Construct and save a binvox-formatted voxel grid of environment

        Args:
            position (Vector3r):
                Position around which voxel grid is centered in m
            x, y, z (int): Size of each voxel grid dimension in m
            res (float): Resolution of voxel grid in m
            of (str): Name of output file to save voxel grid as

        Returns:
            is_success (bool):
                True if output written to file successfully, else False
        """

        return self.client.call("simCreateVoxelGrid", position, x, y, z, res, of)


# Add new vehicle via RPC:
    def sim_add_vehicle(
            self,
            vehicle_name: str,
            vehicle_type: str,
            pose: Pose,
            pawn_path: str = ""
        ):
        """
        Create vehicle at runtime

        Args:
            vehicle_name (str): Name of the vehicle being created
            vehicle_type (str): Type of vehicle, e.g. "simpleflight"
            pose (Pose): Initial pose of the vehicle
            pawn_path (str):
                Vehicle blueprint path, default empty
                wbich uses the default blueprint for the vehicle type

        Returns:
            is_success (bool): Whether vehicle was created
        """

        return self.client.call("simAddVehicle", vehicle_name, vehicle_type, pose, pawn_path)


    def list_vehicles(self) -> List[str]:
        """
        Lists the names of current vehicles

        Returns:
            vehicles_list (List[str]):
                List containing names of all vehicles
        """

        return self.client.call("listVehicles")


    def get_settings_string(self, settings_string: str) -> str:
        """
        Fetch the settings text being used by AirSim

        Returns:
            settings_string (str): Settings text in JSON format
        """

        return self.client.call("getSettingsString", settings_string)


    def sim_set_ext_force(self, ext_force: Vector3r):
        """
        Set arbitrary external forces, in World frame, NED direction.
        Can be used for implementing simple payloads

        Args:
            ext_force (Vector3r):
                Force, in World frame, NED direction, in N
        """

        self.client.call("simSetExtForce", ext_force)


    def sim_find_look_at_rotation(self, object_name: str, vehicle_name: str = ""):
        return self.client.call("simFindLookAtRotation", vehicle_name, object_name)



class MultirotorClient(VehicleClient, object):
    """
    Multirotor APIs
    """

    def __init__(
            self,
            ip: str = "127.0.0.1",
            port: int = 41451,
            timeout_value: int = 3600,
            reconnect_limit: int = 5
        ):

        super(MultirotorClient, self).__init__(ip, port, timeout_value, reconnect_limit)


    def takeoff_async(self, timeout_sec: int = 20, vehicle_name: str = "") -> Future:
        """
        Takeoff vehicle to 3m above ground.
        Vehicle should not be moving when this API is used

        Args:
            timeout_sec (int):
                Timeout for the vehicle to reach desired altitude
            vehicle_name (str):
                Name of the vehicle to send this command to

        Returns:
            call_result (Future):
                Future call .join() to wait for method to finish.
                Example: client.METHOD().join()
        """

        return self.client.call_async("takeoff", timeout_sec, vehicle_name)


    def land_async(self, timeout_sec: int = 60, vehicle_name: str = "") -> Future:
        """
        Land the vehicle

        Args:
            timeout_sec (int): Timeout for the vehicle to land
            vehicle_name (str):
                Name of the vehicle to send this command to

        Returns:
            call_result (Future):
                Future call .join() to wait for method to finish.
                Example: client.METHOD().join()
        """

        return self.client.call_async("land", timeout_sec, vehicle_name)


    def go_home_async(self, timeout_sec: int = 3e+38, vehicle_name: str = "") -> Future:
        """
        Return vehicle to Home i.e. Launch location

        Args:
            timeout_sec (int):
                Timeout for the vehicle to reach desired altitude
            vehicle_name (str):
                Name of the vehicle to send this command to

        Returns:
            call_result (Future):
                Future call .join() to wait for method to finish.
                Example: client.METHOD().join()
        """

        return self.client.call_async("goHome", timeout_sec, vehicle_name)


# APIs for control:
    def move_by_velocity_body_frame_async(
            self,
            vx: float,
            vy: float,
            vz: float,
            duration: float,
            drivetrain: DrivetrainType = DrivetrainType.MAX_DEGREE_OF_FREEDOM,
            yaw_mode: YawMode = YawMode(),
            vehicle_name: str = ""
        ) -> Future:
        """
        Args:
            vx (float):
                Desired velocity in the X, Y, Z axis of the vehicle's
                local NED frame
            vy (float):
                Desired velocity in the Y axis of the vehicle's
                local NED frame
            vz (float):
                Desired velocity in the Z axis of the vehicle's
                local NED frame.
            duration (float):
                Desired amount of time (seconds),
                to send this command for
            drivetrain (DrivetrainType):
                When FORWARD_ONLY, vehicle rotates itself so that
                its front is always facing the direction of travel.
                If MaxDegreeOfFreedom then it doesn"t do that
                (crab-like movement)
            yaw_mode (YawMode):
                Specifies if vehicle should face at given angle
                (is_rate == False) or should be rotating around its axis
                at given rate (is_rate == True)
            vehicle_name (str):
                Name of the multirotor to send this command to

        Returns:
            call_result (Future):
                Future call .join() to wait for method to finish.
                Example: client.METHOD().join()
        """

        return self.client.call_async(
            "moveByVelocityBodyFrame",
            vx,
            vy,
            vz,
            duration,
            drivetrain,
            yaw_mode,
            vehicle_name
        )


    def move_by_velocity_z_body_frame_async(
            self,
            vx: float,
            vy: float,
            z: float,
            duration: float,
            drivetrain: DrivetrainType = DrivetrainType.MAX_DEGREE_OF_FREEDOM,
            yaw_mode: YawMode = YawMode(),
            vehicle_name: str = ""
        ) -> Future:
        """
        Args:
            vx (float):
                Desired velocity in the X axis of the vehicle's
                local NED frame
            vy (float):
                Desired velocity in the Y axis of the vehicle's
                local NED frame
            z (float):
                Desired Z value (in local NED frame of the vehicle)
            duration (float):
                Desired amount of time (seconds),
                to send this command for
            drivetrain (DrivetrainType):
                When FORWARD_ONLY, vehicle rotates itself so that
                its front is always facing the direction of travel.
                If MaxDegreeOfFreedom then it doesn"t do that
                (crab-like movement)
            yaw_mode (YawMode):
                Specifies if vehicle should face at given angle
                (is_rate == False) or should be rotating around its axis
                at given rate (is_rate == True)
            vehicle_name (str):
                Name of the multirotor to send this command to

        Returns:
            call_result (Future):
                Future call .join() to wait for method to finish.
                Example: client.METHOD().join()
        """

        return self.client.call_async(
            "moveByVelocityZBodyFrame",
            vx,
            vy,
            z,
            duration,
            drivetrain,
            yaw_mode,
            vehicle_name
        )


    def move_by_angle_z_async(
            self,
            pitch,
            roll,
            z,
            yaw,
            duration,
            vehicle_name = ""
        ):

        warn(
            "move_by_angle_z_async API is deprecated, \
             use move_by_roll_pitch_yaw_z_async() API instead"
        )

        return self.client.call_async(
            "moveByRollPitchYawZ",
            roll,
            -pitch,
            -yaw,
            z,
            duration,
            vehicle_name
        )


    def move_by_angle_throttle_async(
            self,
            pitch: float,
            roll: float,
            throttle: float,
            yaw_rate: float,
            duration: float,
            vehicle_name: str = ""
        ):

        warn(
            "move_by_angle_throttle_async API is deprecated, \
             use move_by_roll_pitch_yawrate_throttle_async() API instead"
        )

        return self.client.call_async(
            "moveByRollPitchYawrateThrottle",
            roll,
            -pitch,
            -yaw_rate,
            throttle,
            duration,
            vehicle_name
        )


    def move_by_velocity_async(
            self,
            vx: float,
            vy: float,
            vz: float,
            duration: float,
            drivetrain: DrivetrainType = DrivetrainType.MAX_DEGREE_OF_FREEDOM,
            yaw_mode: YawMode = YawMode(),
            vehicle_name: str = ""
        ) -> Future:
        """
        Args:
            vx (float): Desired velocity in world (NED) X axis
            vy (float): Desired velocity in world (NED) Y axis
            vz (float): Desired velocity in world (NED) Z axis
            duration (float):
                Desired amount of time (seconds),
                to send this command for
            drivetrain (DrivetrainType):
                When FORWARD_ONLY, vehicle rotates itself so that
                its front is always facing the direction of travel.
                If MaxDegreeOfFreedom then it doesn"t do that
                (crab-like movement)
            yaw_mode (YawMode):
                Specifies if vehicle should face at given angle
                (is_rate == False) or should be rotating around its axis
                at given rate (is_rate == True)
            vehicle_name (str):
                Name of the multirotor to send this command to

        Returns:
            call_result (Future):
                Future call .join() to wait for method to finish.
                Example: client.METHOD().join()
        """

        return self.client.call_async(
            "moveByVelocity",
            vx,
            vy,
            vz,
            duration,
            drivetrain,
            yaw_mode,
            vehicle_name
        )


    def move_by_velocity_z_async(
            self,
            vx: float,
            vy: float,
            z: float,
            duration: float,
            drivetrain: DrivetrainType = DrivetrainType.MAX_DEGREE_OF_FREEDOM,
            yaw_mode: YawMode = YawMode(),
            vehicle_name: str = ""
        ):
        
        return self.client.call_async(
            "moveByVelocityZ",
            vx,
            vy,
            z,
            duration,
            drivetrain,
            yaw_mode,
            vehicle_name
        )


    def move_on_path_async(
            self,
            path: Vector3r,
            velocity: float,
            timeout_sec: int = 3e+38,
            drivetrain: DrivetrainType = DrivetrainType.MAX_DEGREE_OF_FREEDOM,
            yaw_mode: YawMode = YawMode(),
            lookahead: int = -1,
            adaptive_lookahead: int = 1,
            vehicle_name: str = ""
        ):
        
        return self.client.call_async(
            "moveOnPath",
            path,
            velocity,
            timeout_sec,
            drivetrain,
            yaw_mode,
            lookahead,
            adaptive_lookahead,
            vehicle_name
        )


    def move_to_position_async(
            self,
            x: float,
            y: float,
            z: float,
            velocity: float,
            timeout_sec: int = 3e+38,
            drivetrain: DrivetrainType = DrivetrainType.MAX_DEGREE_OF_FREEDOM,
            yaw_mode: YawMode = YawMode(),
            lookahead: int = -1,
            adaptive_lookahead: int = 1,
            vehicle_name: str = ""
        ):
        
        return self.client.call_async(
            "moveToPosition",
            x,
            y,
            z,
            velocity,
            timeout_sec,
            drivetrain,
            yaw_mode,
            lookahead,
            adaptive_lookahead,
            vehicle_name
        )


    def move_to_gps_async(
            self,
            latitude: float, 
            longitude: float,
            altitude: float,
            velocity: float,
            timeout_sec: int = 3e+38,
            drivetrain: DrivetrainType = DrivetrainType.MAX_DEGREE_OF_FREEDOM,
            yaw_mode: YawMode = YawMode(),
            lookahead: int = -1,
            adaptive_lookahead: int = 1,
            vehicle_name: str = ""
        ):

        return self.client.call_async(
            "moveToGPS",
            latitude,
            longitude,
            altitude,
            velocity,
            timeout_sec,
            drivetrain,
            yaw_mode,
            lookahead,
            adaptive_lookahead,
            vehicle_name
        )


    def move_to_z_async(
            self,
            z: float,
            velocity: float,
            timeout_sec: int = 3e+38,
            yaw_mode: YawMode = YawMode(),
            lookahead: int = -1,
            adaptive_lookahead: int = 1,
            vehicle_name: str = ""
        ):

        return self.client.call_async(
            "moveToZ",
            z,
            velocity,
            timeout_sec,
            yaw_mode,
            lookahead,
            adaptive_lookahead,
            vehicle_name
        )


    def move_by_manual_async(
            self,
            vx_max: float,
            vy_max: float,
            z_min: float,
            duration: float,
            drivetrain: DrivetrainType = DrivetrainType.MAX_DEGREE_OF_FREEDOM,
            yaw_mode: YawMode = YawMode(),
            vehicle_name: str = ""
        ) -> Future:
        """
        - Read current RC state and use it to control the vehicles.

        Parameters sets up the constraints on velocity and
        minimum altitude while flying.
        If RC state is detected to violate these constraints
        then that RC state would be ignored.

        Args:
            vx_max (float): Max velocity allowed in x direction
            vy_max (float): Max velocity allowed in y direction
            vz_max (float): Max velocity allowed in z direction
            z_min (float): Min z allowed for vehicle position
            duration (float):
                After this duration vehicle would switch back to
                non-manual mode
            drivetrain (DrivetrainType):
                When FORWARD_ONLY, vehicle rotates itself so that
                its front is always facing the direction of travel.
                If MaxDegreeOfFreedom then it doesn"t do that
                (crab-like movement)
            yaw_mode (YawMode):
                Specifies if vehicle should face at given angle
                (is_rate == False) or should be rotating around its axis
                at given rate (is_rate == True)
            vehicle_name (str):
                Name of the multirotor to send this command to
        Returns:
            call_result (Future):
                Future call .join() to wait for method to finish.
                Example: client.METHOD().join()
        """

        return self.client.call_async(
            "moveByManual",
            vx_max,
            vy_max,
            z_min,
            duration,
            drivetrain,
            yaw_mode,
            vehicle_name
        )


    def rotate_to_yaw_async(
            self,
            yaw: float,
            timeout_sec: int = 3e+38,
            margin: int = 5,
            vehicle_name: str = ""
        ):

        return self.client.call_async("rotateToYaw", yaw, timeout_sec, margin, vehicle_name)


    def rotate_by_yaw_rate_async(self, yaw_rate: float, duration: float, vehicle_name: str = ""):
        return self.client.call_async("rotateByYawRate", yaw_rate, duration, vehicle_name)


    def hover_async(self, vehicle_name: str = ""):
        return self.client.call_async("hover", vehicle_name)


    def move_by_rc(self, rcdata: RCData = RCData(), vehicle_name: str = ""):
        return self.client.call("moveByRC", rcdata, vehicle_name)


# Low - level control API:
    def move_by_motor_pwms_async(
            self,
            front_right_pwm: float,
            rear_left_pwm: float,
            front_left_pwm: float,
            rear_right_pwm: float,
            duration: float,
            vehicle_name: str = ""
        ) -> Future:
        """
        - Directly control the motors using PWM values

        Args:
            front_right_pwm (float):
                PWM value for the front right motor (between 0.0 to 1.0)
            rear_left_pwm (float):
                PWM value for the rear left motor (between 0.0 to 1.0)
            front_left_pwm (float):
                PWM value for the front left motor (between 0.0 to 1.0)
            rear_right_pwm (float):
                PWM value for the rear right motor (between 0.0 to 1.0)
            duration (float):
                Desired amount of time (seconds),
                to send this command for
            vehicle_name (str):
                Name of the multirotor to send this command to
        Returns:
            call_result (Future):
                Future call .join() to wait for method to finish.
                Example: client.METHOD().join()
        """

        return self.client.call_async(
            "moveByMotorPWMs",
            front_right_pwm,
            rear_left_pwm,
            front_left_pwm,
            rear_right_pwm,
            duration,
            vehicle_name
        )


    def move_by_roll_pitch_yaw_z_async(
            self,
            roll: float,
            pitch: float,
            yaw: float,
            z: float,
            duration: float,
            vehicle_name: str = ""
        ) -> Future:
        """
        - z is given in local NED frame of the vehicle
        - Roll angle, pitch angle, and yaw angle set points are given in
        **radians**, in the body frame
        - The body frame follows the Front Left Up (FLU) convention,
        and right-handedness

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor

            | Clockwise rotation about this axis defines a positive
            **roll** angle
            | Hence, rolling with a positive angle is equivalent to
            translating in the **right** direction,
            w.r.t. our FLU body frame

            - Y axis is along the **Left** direction of the quadrotor

            | Clockwise rotation about this axis defines a positive
            **pitch** angle
            | Hence, pitching with a positive angle is equivalent to
            translating in the **front** direction,
            w.r.t. our FLU body frame

            - Z axis is along the **Up** direction

            | Clockwise rotation about this axis defines a positive
            **yaw** angle
            | Hence, yawing with a positive angle is equivalent to
            rotated towards the **left** direction wrt our FLU
            body frame. Or in an anticlockwise fashion in the body
            XY / FL plane

        Args:
            roll (float): Desired roll angle, in radians
            pitch (float): Desired pitch angle, in radians
            yaw (float): Desired yaw angle, in radians
            z (float):
                Desired Z value (in local NED frame of the vehicle)
            duration (float):
                Desired amount of time (seconds),
                to send this command for
            vehicle_name (str):
                Name of the multirotor to send this command to

        Returns:
            call_result (Future):
                Future call .join() to wait for method to finish.
                Example: client.METHOD().join()
        """

        return self.client.call_async(
            "moveByRollPitchYawZ",
            roll,
            -pitch,
            -yaw,
            z,
            duration,
            vehicle_name
        )


    def move_by_roll_pitch_yaw_throttle_async(
            self,
            roll: float,
            pitch: float,
            yaw: float,
            throttle: float,
            duration: float,
            vehicle_name: str = ""
        ) -> Future:
        """
        - Desired throttle is between 0.0 to 1.0
        - Roll angle, pitch angle, and yaw angle are given in
        **degrees** when using PX4 and in **radians** when using
        SimpleFlight, in the body frame
        - The body frame follows the Front Left Up (FLU) convention,
        and right-handedness

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor

            | Clockwise rotation about this axis defines a positive
            **roll** angle
            | Hence, rolling with a positive angle is equivalent to
            translating in the **right** direction,
            w.r.t. our FLU body frame

            - Y axis is along the **Left** direction of the quadrotor

            | Clockwise rotation about this axis defines a positive
            **pitch** angle
            | Hence, pitching with a positive angle is equivalent to
            translating in the **front** direction,
            w.r.t. our FLU body frame

            - Z axis is along the **Up** direction.

            | Clockwise rotation about this axis defines a positive
            **yaw** angle
            | Hence, yawing with a positive angle is equivalent to
            rotated towards the **left** direction wrt our FLU
            body frame. Or in an anticlockwise fashion in the body
            XY / FL plane

        Args:
            roll (float): Desired roll angle
            pitch (float): Desired pitch angle
            yaw (float): Desired yaw angle
            throttle (float): Desired throttle (between 0.0 to 1.0)
            duration (float):
                Desired amount of time (seconds),
                to send this command for
            vehicle_name (str): 
                Name of the multirotor to send this command to

        Returns:
            call_result (Future):
                Future call .join() to wait for method to finish.
                Example: client.METHOD().join()
        """

        return self.client.call_async(
            "moveByRollPitchYawThrottle",
            roll,
            -pitch,
            -yaw,
            throttle,
            duration,
            vehicle_name
        )


    def move_by_roll_pitch_yawrate_throttle_async(
            self,
            roll: float,
            pitch: float,
            yaw_rate: float,
            throttle: float,
            duration: float,
            vehicle_name: str = ""
        ) -> Future:
        """
        - Desired throttle is between 0.0 to 1.0
        - Roll angle, pitch angle, and yaw rate set points are given
        in **radians**, in the body frame
        - The body frame follows the Front Left Up (FLU) convention,
        and right-handedness

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor

            | Clockwise rotation about this axis defines a positive
            **roll** angle
            | Hence, rolling with a positive angle is equivalent to
            translating in the **right** direction,
            w.r.t. our FLU body frame

            - Y axis is along the **Left** direction of the quadrotor

            | Clockwise rotation about this axis defines a positive
            **pitch** angle
            | Hence, pitching with a positive angle is equivalent to
            translating in the **front** direction,
            w.r.t. our FLU body frame

            - Z axis is along the **Up** direction

            | Clockwise rotation about this axis defines a positive
            **yaw** angle
            | Hence, yawing with a positive angle is equivalent to
            rotated towards the **left** direction wrt our FLU
            body frame. Or in an anticlockwise fashion in the body
            XY / FL plane

        Args:
            roll (float): Desired roll angle, in radians
            pitch (float): Desired pitch angle, in radians
            yaw_rate (float): Desired yaw rate, in radian per second
            throttle (float): Desired throttle (between 0.0 to 1.0)
            duration (float):
                Desired amount of time (seconds),
                to send this command for
            vehicle_name (str):
                Name of the multirotor to send this command to

        Returns:
            call_result (Future):
                Future call .join() to wait for method to finish.
                Example: client.METHOD().join()
        """

        return self.client.call_async(
            "moveByRollPitchYawrateThrottle",
            roll,
            -pitch,
            -yaw_rate,
            throttle,
            duration,
            vehicle_name
        )


    def move_by_roll_pitch_yawrate_z_async(
            self,
            roll: float,
            pitch: float,
            yaw_rate: float,
            z: float,
            duration: float,
            vehicle_name: str = ""
        ) -> Future:
        """
        - z is given in local NED frame of the vehicle
        - Roll angle, pitch angle, and yaw rate set points are given in
        **radians**, in the body frame
        - The body frame follows the Front Left Up (FLU) convention,
        and right-handedness

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor

            | Clockwise rotation about this axis defines a positive
            **roll** angle
            | Hence, rolling with a positive angle is equivalent
            to translating in the **right** direction, w.r.t. our
            FLU body frame

            - Y axis is along the **Left** direction of the quadrotor

            | Clockwise rotation about this axis defines a positive
            **pitch** angle
            | Hence, pitching with a positive angle is equivalent
            to translating in the **front** direction, w.r.t. our
            FLU body frame

            - Z axis is along the **Up** direction

            | Clockwise rotation about this axis defines a positive
            **yaw** angle
            | Hence, yawing with a positive angle is equivalent
            to rotated towards the **left** direction wrt our FLU
            body frame. Or in an anticlockwise fashion in the body
            XY / FL plane

        Args:
            roll (float): Desired roll angle, in radians
            pitch (float): Desired pitch angle, in radians
            yaw_rate (float): Desired yaw rate, in radian per second
            z (float):
                Desired Z value (in local NED frame of the vehicle)
            duration (float):
                Desired amount of time (seconds),
                to send this command for
            vehicle_name (str):
                Name of the multirotor to send this command to

        Returns:
            call_result (Future):
                Future call .join() to wait for method to finish.
                Example: client.METHOD().join()
        """

        return self.client.call_async(
            "moveByRollPitchYawrateZ",
            roll,
            -pitch,
            -yaw_rate,
            z,
            duration,
            vehicle_name
        )


    def move_by_angle_rates_z_async(
            self,
            roll_rate: float,
            pitch_rate: float,
            yaw_rate: float,
            z: float,
            duration: float,
            vehicle_name: str = ""
        ) -> Future:
        """
        - z is given in local NED frame of the vehicle
        - Roll rate, pitch rate, and yaw rate set points are given in
        **radians**, in the body frame
        - The body frame follows the Front Left Up (FLU) convention,
        and right-handedness

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor

            | Clockwise rotation about this axis defines a positive
            **roll** angle
            | Hence, rolling with a positive angle is equivalent
            to translating in the **right** direction, w.r.t. our
            FLU body frame

            - Y axis is along the **Left** direction of the quadrotor

            | Clockwise rotation about this axis defines a positive
            **pitch** angle
            | Hence, pitching with a positive angle is equivalent
            to translating in the **front** direction, w.r.t. our
            FLU body frame

            - Z axis is along the **Up** direction

            | Clockwise rotation about this axis defines a positive
            **yaw** angle
            | Hence, yawing with a positive angle is equivalent to
            rotated towards the **left** direction wrt our FLU
            body frame. Or in an anticlockwise fashion in the body
            XY / FL plane

        Args:
            roll_rate (float): Desired roll rate, in radians / second
            pitch_rate (float): Desired pitch rate, in radians / second
            yaw_rate (float): Desired yaw rate, in radians / second
            z (float):
                Desired Z value (in local NED frame of the vehicle)
            duration (float):
                Desired amount of time (seconds),
                to send this command for
            vehicle_name (str):
                Name of the multirotor to send this command to

        Returns:
            call_result (Future):
                Future call .join() to wait for method to finish.
                Example: client.METHOD().join()
        """

        return self.client.call_async(
            "moveByAngleRatesZ",
            roll_rate,
            -pitch_rate,
            -yaw_rate,
            z,
            duration,
            vehicle_name
        )


    def move_by_angle_rates_throttle_async(
            self,
            roll_rate: float,
            pitch_rate: float,
            yaw_rate: float,
            throttle: float,
            duration: float,
            vehicle_name: str = ""
        ) -> Future:
        """
        - Desired throttle is between 0.0 to 1.0
        - Roll rate, pitch rate, and yaw rate set points are given in
        **radians**, in the body frame
        - The body frame follows the Front Left Up (FLU) convention,
        and right-handedness

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor

            | Clockwise rotation about this axis defines a positive
            **roll** angle
            | Hence, rolling with a positive angle is equivalent
            to translating in the **right** direction, w.r.t.
            our FLU body frame

            - Y axis is along the **Left** direction of the quadrotor.

            | Clockwise rotation about this axis defines a positive
            **pitch** angle.
            | Hence, pitching with a positive angle is equivalent
            to translating in the **front** direction, w.r.t. our
            FLU body frame

            - Z axis is along the **Up** direction.

            | Clockwise rotation about this axis defines a positive
            **yaw** angle
            | Hence, yawing with a positive angle is equivalent
            to rotated towards the **left** direction wrt our FLU
            body frame. Or in an anticlockwise fashion in the body
            XY / FL plane

        Args:
            roll_rate (float): Desired roll rate, in radians / second
            pitch_rate (float): Desired pitch rate, in radians / second
            yaw_rate (float): Desired yaw rate, in radians / second
            throttle (float): Desired throttle (between 0.0 to 1.0)
            duration (float):
                Desired amount of time (seconds),
                to send this command for
            vehicle_name (str):
                Name of the multirotor to send this command to

        Returns:
            call_result (Future):
                Future call .join() to wait for method to finish.
                Example: client.METHOD().join()
        """

        return self.client.call_async(
            "moveByAngleRatesThrottle",
            roll_rate,
            -pitch_rate,
            -yaw_rate,
            throttle,
            duration,
            vehicle_name
        )


    def set_angle_rate_controller_gains(
            self,
            angle_rate_gains: AngleRateControllerGains = AngleRateControllerGains(),
            vehicle_name: str = ""
        ):
        """
        - Modifying these gains will have an affect on *ALL* move*()
        APIs. This is because any velocity setpoint is converted
        to an angle level setpoint which is tracked with an angle
        level controllers. That angle level setpoint is itself tracked
        with and angle rate controller
        - This function should only be called if the default
        angle rate control PID gains need to be modified.

        Args:
            angle_rate_gains (AngleRateControllerGains):
                - Correspond to the roll, pitch, yaw axes,
                defined in the body frame
                - Pass AngleRateControllerGains() to reset gains
                to default recommended values
            vehicle_name (str):
                Name of the multirotor to send this command to
        """

        self.client.call("setAngleRateControllerGains", *angle_rate_gains.to_lists(), vehicle_name)


    def set_angle_level_controller_gains(
            self,
            angle_level_gains: AngleLevelControllerGains = AngleLevelControllerGains(),
            vehicle_name = ""
        ):
        """
        - Sets angle level controller gains (used by any API setting
        angle references - for ex: move_by_roll_pitch_yaw_z_async(),
        move_by_roll_pitch_yaw_throttle_async(), etc.)
        - Modifying these gains will also affect the behaviour of
        move_by_velocity_async() API. This is because the AirSim
        flight controller will track velocity setpoints by converting
        them to angle set points
        - This function should only be called if the default
        angle level control PID gains need to be modified
        - Passing AngleLevelControllerGains() sets gains
        to default airsim values

        Args:
            angle_level_gains (AngleLevelControllerGains):
                - Correspond to the roll, pitch, yaw axes,
                defined in the body frame
                - Pass AngleLevelControllerGains() to reset gains
                to default recommended values
            vehicle_name (str):
                Name of the multirotor to send this command to
        """

        self.client.call("setAngleLevelControllerGains", *angle_level_gains.to_lists(), vehicle_name)


    def set_velocity_controller_gains(
            self,
            velocity_gains: VelocityControllerGains = VelocityControllerGains(),
            vehicle_name: str = ""
        ):
        """
        - Sets velocity controller gains for moveByVelocityAsync()
        - This function should only be called if the default velocity
        control PID gains need to be modified
        - Passing VelocityControllerGains() sets gains
        to default airsim values

        Args:
            velocity_gains (VelocityControllerGains):
                - Correspond to the world X, Y, Z axes
                - Pass VelocityControllerGains() to reset gains
                to default recommended values
                - Modifying velocity controller gains will have
                an affect on the behavior of move_on_spline_async() and
                move_on_spline_vel_constraints_async(), as they both use
                velocity control to track the trajectory
            vehicle_name (str):
                Name of the multirotor to send this command to
        """

        self.client.call("setVelocityControllerGains", *velocity_gains.to_lists(), vehicle_name)


    def set_position_controller_gains(
            self,
            position_gains: PositionControllerGains = PositionControllerGains(),
            vehicle_name: str = ""
        ):
        """
        Sets position controller gains for moveByPositionAsync.
        This function should only be called if the default position
        control PID gains need to be modified

        Args:
            position_gains (PositionControllerGains):
                - Correspond to the X, Y, Z axes
                - Pass PositionControllerGains() to reset gains
                to default recommended values
            vehicle_name (str):
                Name of the multirotor to send this command to
        """

        self.client.call("setPositionControllerGains", *position_gains.to_lists(), vehicle_name)


    def get_multirotor_state(self, vehicle_name: str = "") -> MultirotorState:
        """
        The position inside the returned MultirotorState is in the frame
        of the vehicle's starting point

        Args:
            vehicle_name (str): Vehicle to get the state of

        Returns:
            multirotor_state (MultirotorState): Multirotor state
        """

        return MultirotorState.from_msgpack(self.client.call("getMultirotorState", vehicle_name))


    def get_rotor_states(self, vehicle_name: str = "") -> RotorStates:
        """
        Used to obtain the current state of all a multirotor's rotors.
        The state includes the speeds,
        thrusts and torques for all rotors

        Args:
            vehicle_name (str): Vehicle to get the rotor state of

        Returns:
            rotor_states (RotorStates):
                Containing a timestamp and the RPM speed of all rotors
        """

        return RotorStates.from_msgpack(self.client.call("getRotorStates", vehicle_name))



class CarClient(VehicleClient, object):
    """
    Car APIs
    """

    def __init__(
            self,
            ip: str = "127.0.0.1",
            port: int = 41451,
            timeout_value: int = 3600,
            reconnect_limit: int = 5
        ):

        super(CarClient, self).__init__(ip, port, timeout_value, reconnect_limit)


    def set_car_controls(self, controls: CarControls, vehicle_name: str = ""):
        """
        Control the car using throttle, steering, brake, etc.

        Args:
            controls (CarControls): Struct containing control values
            vehicle_name (str): Name of vehicle to be controlled
        """

        self.client.call("setCarControls", controls, vehicle_name)


    def get_car_state(self, vehicle_name: str = ""):
        """
        The position inside the returned CarState is in the frame
        of the vehicle's starting point

        Args:
            vehicle_name (str): Name of vehicle

        Returns:
            car_state (CarState): State of the car
        """

        state_raw = self.client.call("getCarState", vehicle_name)

        return CarState.from_msgpack(state_raw)


    def get_car_controls(self, vehicle_name: str = "") -> CarControls:
        """
        Args:
            vehicle_name (str): Name of vehicle

        Returns:
            car_controls (CarControls): Car controls
        """

        controls_raw = self.client.call("getCarControls", vehicle_name)

        return CarControls.from_msgpack(controls_raw)