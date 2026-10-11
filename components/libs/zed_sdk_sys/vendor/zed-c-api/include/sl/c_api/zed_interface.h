#ifndef __INTERFACE_ZED_H
#define __INTERFACE_ZED_H

#include "types_c.h"
#include <stdbool.h>
#include <cuda.h>
#include <cuda_runtime.h>

/**
 * @file
 * */

 //#define DEBUG
#ifdef _WIN32
#ifdef INTERFACE_NOEXPORT
#define INTERFACE_API
#else
#ifdef INTERFACE_EXPORT
#define INTERFACE_API __declspec(dllexport)
#else
#define INTERFACE_API __declspec(dllimport)
#endif
#endif
#else
#define INTERFACE_API
#endif

#ifdef __cplusplus
extern "C" {
#endif


	INTERFACE_API void sl_free(void* ptr);

    /**
    \brief Forces unload of all instances.
    */
    INTERFACE_API void sl_unload_all_instances();

    /**
    \brief Forces unload of one instance.
    \param camera_id : id of the instance to unload.
    */
    INTERFACE_API void sl_unload_instance(int camera_id);

    /**
    * \brief Creates a camera with resolution mode, fps and id for linux.
    * \param camera_id : id of the camera to be added.
    * \param verbose : Enable verbose mode.
    * \return true if the camera has been created successfully.
    */
    INTERFACE_API bool sl_create_camera(int camera_id);

    /**
    \brief Reports if the camera has been successfully opened.
    \param camera_id : Id of the camera.
    \return true if the ZED camera is already setup, otherwise false.
    */
    INTERFACE_API bool sl_is_opened(int camera_id);

    /**
    \brief Opens the ZED camera from the provided SL_InitParameters.
    \param camera_id : Id of the camera to open.
    \param init_parameters : A structure containing all the initial parameters. Default: a preset of SL_InitParameters.
    \param serial_number : Serial number of the camera to open.
    \param path_svo : Filename of the svo to read (for SVO input).
    \param ip : IP of the camera to open (for Stream input).
    \param stream_port : Port of the camera to open (for Stream input).
    \param gmsl_port : GMSL port number for camera selection (only used when input_type is GMSL). Default: -1 (do nothing).
	\param bus_type : Whether the camera is a USB or a GMSL camera (when opening with camera ID).
    \param output_file : ZED SDK verbose log file. Redirect the SDK verbose message to the file.
    \param opt_settings_path[optional] : Settings path.
    \param opencv_calib_path[optional] : openCV calibration file.

    \return An error code giving information about the internal process. If \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" (0) is returned, the camera is ready to use. Every other code indicates an error and the program should be stopped.
    */
    INTERFACE_API int sl_open_camera(int camera_id, struct SL_InitParameters* init_parameters, const unsigned int serial_number,  const char* path_svo, const char* ip, int stream_port, int gmsl_port, const char* output_file, const char* opt_settings_path, const char* opencv_calib_path);

    /**
	\brief Opens the ZED camera from the provided SL_InitParameters using its  camera ID.
	\param camera_id : Id of the camera to open.
	\param init_parameters : A structure containing all the initial parameters. Default: a preset of SL_InitParameters.
	\param output_file : ZED SDK verbose log file. Redirect the SDK verbose message to the file.
	\param opt_settings_path[optional] : Settings path.
	\param opencv_calib_path[optional] : openCV calibration file.
	\return An error code giving information about the internal process. If \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" (0) is returned, the camera is ready to use. Every other code indicates an error and the program should be stopped.
    */
    INTERFACE_API int sl_open_camera_from_camera_id(int camera_id, struct SL_InitParameters* init_parameters, const char* output_file, const char* opt_settings_path, const char* opencv_calib_path);

    /**
	\brief Opens the ZED camera from the provided SL_InitParameters using its serial number.
	\param camera_id : Id of the camera to open.
	\param init_parameters : A structure containing all the initial parameters. Default: a preset of SL_InitParameters.
	\param serial_number : Serial number of the camera to open.
	\param output_file : ZED SDK verbose log file. Redirect the SDK verbose message to the file.
	\param opt_settings_path[optional] : Settings path.
	\param opencv_calib_path[optional] : openCV calibration file.
	\return An error code giving information about the internal process. If \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" (0) is returned, the camera is ready to use. Every other code indicates an error and the program should be stopped.
    */
	INTERFACE_API int sl_open_camera_from_serial_number(int camera_id, struct SL_InitParameters* init_parameters, const unsigned int serial_number, const char* output_file, const char* opt_settings_path, const char* opencv_calib_path);

    /**
	\brief Opens the ZED camera from the provided SL_InitParameters using an SVO file.
	\param camera_id : Id of the camera to open.
	\param init_parameters : A structure containing all the initial parameters. Default: a preset of SL_InitParameters.
	\param path_svo : Filename of the svo to read (for SVO input).
	\param output_file : ZED SDK verbose log file. Redirect the SDK verbose message to the file.
	\param opt_settings_path[optional] : Settings path.
	\param opencv_calib_path[optional] : openCV calibration file.
	\return An error code giving information about the internal process. If \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" (0) is returned, the camera is ready to use. Every other code indicates an error and the program should be stopped.
    */
	INTERFACE_API int sl_open_camera_from_svo_file(int camera_id, struct SL_InitParameters* init_parameters, const char* path_svo, const char* output_file, const char* opt_settings_path, const char* opencv_calib_path);

    /**
	\brief Opens the ZED camera from the provided SL_InitParameters using its IP and stream port.
	\param camera_id : Id of the camera to open.
	\param init_parameters : A structure containing all the initial parameters. Default: a preset of SL_InitParameters.
	\param ip : IP of the camera to open (for Stream input).
	\param stream_port : Port of the camera to open (for Stream input).
	\param output_file : ZED SDK verbose log file. Redirect the SDK verbose message to the file.
	\param opt_settings_path[optional] : Settings path.
	\param opencv_calib_path[optional] : openCV calibration file.
	\return An error code giving information about the internal process. If \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" (0) is returned, the camera is ready to use. Every other code indicates an error and the program should be stopped.
    */
	INTERFACE_API int sl_open_camera_from_stream(int camera_id, struct SL_InitParameters* init_parameters, const char* ip, int stream_port, const char* output_file, const char* opt_settings_path, const char* opencv_calib_path);
    /**
    \brief Set this camera as a data provider for the Fusion module.

    Metadata is exchanged with the Fusion.
    \param camera_id : Id of the camera instance.
    \param configuration : A structure containing all the initial parameters. Default: a preset of \ref SL_CommunicationParameters.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if everything went fine, \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" otherwise.
     */
    INTERFACE_API enum SL_ERROR_CODE sl_start_publishing(int camera_id, struct SL_CommunicationParameters* comm_params);

    /**
    \brief Set this camera as normal camera(without data providing).
    \n Stop to send camera data to fusion.
    \return \ref SUCCESS if everything went fine, \ref SL_ERROR_CODE_FAILURE otherwise
    */
    INTERFACE_API enum SL_ERROR_CODE sl_stop_publishing(int camera_id);

    /**
    \brief Gets the Camera-created CUDA context for sharing it with other CUDA-capable libraries.

    This can be useful for sharing GPU memories.
    \param camera_id : Id of the camera instance.
    */
    INTERFACE_API CUcontext sl_get_cuda_context(int camera_id);

    /**
    \brief Returns the SL_InitParameters used.

    It corresponds to the structure given as argument to the sl_open_camera() function.
    \param camera_id : Id of the camera instance.
    \return SL_InitParameters containing the parameters used to initialize the camera.
    */
    INTERFACE_API struct SL_InitParameters* sl_get_init_parameters(int camera_id);

    /**
    \brief Returns the SL_RuntimeParameters used.

    It corresponds to the structure given as argument to the sl_grab() function.
    \param camera_id : id of the camera instance.
    \return SL_RuntimeParameters containing the parameters that define the behavior of the \ref sl_grab function.
    */
    INTERFACE_API struct SL_RuntimeParameters* sl_get_runtime_parameters(int camera_id);

    /**
    \brief Returns the \ref SL_PositionalTrackingParameters used.

    It corresponds to the structure given as argument to the \ref sl_enable_positional_tracking() method.
    \param camera_id : Id of the camera instance.
    \return \ref SL_PositionalTrackingParameters containing the parameters used for positional tracking initialization.
    */
    INTERFACE_API struct SL_PositionalTrackingParameters* sl_get_positional_tracking_parameters(int camera_id);

    /**
    \brief Close an opened camera and disable the textures.
    \param camera_id : Id of the camera instance.
     */
    INTERFACE_API void sl_close_camera(int camera_id);

    /**
    \brief Defines a region of interest to focus on for all the SDK, discarding other parts.
    \param camera_id : Id of the camera instance.
    \param roi_mask : The matrix defining the requested region of interest, pixels lower than 127 will be discarded from all modules: depth, positional tracking, etc. If empty, set all pixels as valid. The mask can be either at lower or higher resolution than the current images.
    \param module: Apply the ROI to a list of SDK module, all by default.
    \return An \ref SL_ERROR_CODE if something went wrong.
    \note The function support \ref SL_MAT_TYPE "SL_MAT_TYPE_U8_C1" / \ref SL_MAT_TYPE "SL_MAT_TYPE_UU8_C3" / \ref SL_MAT_TYPE "SL_MAT_TYPE_UU8_C4" images type.
     */
    INTERFACE_API int sl_set_region_of_interest(int camera_id, void* roi_mask, bool module[SL_MODULE_LAST]);

    /**
    \brief Get the previously set or computed region of interest.
    \param camera_id : Id of the camera instance.
    \param roi_mask: The \ref Mat returned.
    \param image_size: The optional size of the returned mask.
    \param module: Specify which module to get the ROI.
    \return An \ref SL_ERROR_CODE if something went wrong.
     */
    INTERFACE_API int sl_get_region_of_interest(int camera_id, void* roi_mask, int width, int height, enum SL_MODULE module);
    /**
    \brief Start the auto detection of a region of interest to focus on for all the SDK, discarding other parts.
    This detection is based on the general motion of the camera combined with the motion in the scene.
    The camera must move for this process, an internal motion detector is used, based on the Positional Tracking module.
    It requires a few hundreds frames of motion to compute the mask.

    \param roi_param: The \ref SL_RegionOfInterestParameters defining parameters for the detection

    \note This module is expecting a static portion, typically a fairly close vehicle hood at the bottom of the image.
     This module may not work correctly or detect incorrect background area, especially with slow motion, if there's no static element.
     This module work asynchronously, the status can be obtained using \ref sl_get_region_of_interest_auto_detection_status(), the result is either auto applied,
     or can be retrieve using \ref sl_get_region_of_interest function.

    \return An \ref SL_ERROR_CODE if something went wrong.
     */
    INTERFACE_API int sl_start_region_of_interest_auto_detection(int camera_id, struct SL_RegionOfInterestParameters* roi_param);
    /**
    \brief Return the status of the automatic Region of Interest Detection
     The automatic Region of Interest Detection is enabled by using \ref sl_start_region_of_interest_auto_detection
     \param camera_id : Id of the camera instance.
    \return \ref SL_REGION_OF_INTEREST_AUTO_DETECTION_STATE the status
     */
    INTERFACE_API enum SL_REGION_OF_INTEREST_AUTO_DETECTION_STATE sl_get_region_of_interest_auto_detection_status(int camera_id);
    /**
    \brief Grabs the latest images from the camera.
    \param camera_id : Id of the camera instance.
    \param runtime : A structure containing all the runtime parameters. Default: a preset of SL_RuntimeParameters.
    \return An error code giving information about the internal process. \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if the method succeeded.
     */
    INTERFACE_API int sl_grab(int camera_id, struct SL_RuntimeParameters* runtime);

    /**
    \brief Read the latest images and IMU from the camera and rectify the images.

    This method is meant to be called frequently in the main loop of your application.

    \note If no new frames is available until timeout is reached, read() will return \ref ERROR_CODE "SL_ERROR_CODE_CAMERA_NOT_DETECTED" since the camera has probably been disconnected.
    \note Returned errors can be displayed using toString().

    \return \ref ERROR_CODE "SL_ERROR_CODE_SUCCESS" means that no problem was encountered.
     */
    INTERFACE_API int sl_read(int camera_id);

    /**
    \brief Lists all the connected devices with their associated information.

    This method lists all the cameras available and provides their serial number, models and other information.
    \param device_list [Out] : The devices properties for each connected camera.
    \param nb_devices [Out] : The number of cameras connected.
     */
    INTERFACE_API void sl_get_device_list(struct SL_DeviceProperties device_list[MAX_CAMERA_PLUGIN], int* nb_devices);

    /**
    \brief List all the streaming devices with their associated information.
    \param device_list [Out] : The devices properties for each connected camera.
    \param nb_devices  [Out]: The number of cameras connected.
    \return The streaming properties for each connected camera.
     */
    INTERFACE_API void sl_get_streaming_device_list(struct SL_StreamingProperties streaming_device_list[MAX_CAMERA_PLUGIN], int* nb_devices);

    /**
    \brief Performs a hardware reset of the ZED 2 and the ZED 2i.
    \param sn : Serial number of the camera to reset, or 0 to reset the first camera detected.
    \param fullReboot : Perform a full reboot (sensors and video modules) if true, otherwise only the video module will be rebooted.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if everything went fine.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_CAMERA_NOT_DETECTED" if no camera was detected.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" otherwise.

    \note This method only works for ZED 2, ZED 2i, and newer camera models.
     */
    INTERFACE_API int sl_reboot(int sn, bool full_reboot);

    /**
    \brief Creates a file for recording the ZED's output into a .SVO or .AVI video.

    An SVO is Stereolabs' own format designed for the ZED. It holds the video feed with timestamps as well as info about the camera used to record it.
    \param camera_id : Id of the camera instance.
    \param filename : Filename of the SVO file.
    \param compression_mode : Compression mode. It can be one for the \ref SL_SVO_COMPRESSION_MODE enum.
    \param bitrate : overrides default bitrate of the SVO file, in KBits/s. Only works if \ref SL_SVO_COMPRESSION_MODE is H264 or H265.
    \param target_fps : Defines the target framerate for the recording module.
    \param transcode : In case of streaming input, if set to false, it will avoid decoding/re-encoding and convert directly streaming input to a SVO file.
               This saves a encoding session and can be especially useful on NVIDIA Geforce cards where the number of encoding session is limited.
    \return An \ref SL_ERROR_CODE that defines if SVO file was successfully created and can be filled with images.
     */
    INTERFACE_API int sl_enable_recording(int camera_id, const char* filename, enum SL_SVO_COMPRESSION_MODE compression_mode, unsigned int bitrate, int target_fps, bool transcode);

    /**
    \brief Get the recording information.
    \return The recording state structure. For more details, see \ref SL_RecordingStatus.
     */
    INTERFACE_API struct SL_RecordingStatus* sl_get_recording_status(int camera_id);
    /**
    \brief Disables the recording initiated by \ref sl_enable_recording() and closes the generated file.
    \param camera_id : Id of the camera instance.
     */
    INTERFACE_API void sl_disable_recording(int camera_id);

    /**
    \brief Returns the \ref SL_RecordingParameters used.

    It corresponds to the structure given as argument to the \ref sl_enable_recording() function.
    \param camera_id : Id of the camera instance.
    \return \ref SL_RecordingParameters containing the parameters used for recording initialization.
     */
    INTERFACE_API struct SL_RecordingParameters* sl_get_recording_parameters(int camera_id);

    /**
    \brief Pauses or resumes the recording.
    \param camera_id : Id of the camera instance.
    \param status : If true, the recording is paused. If false, the recording is resumed.
     */
    INTERFACE_API void sl_pause_recording(int camera_id, bool status);

    /**
    \brief Ingests SL_SVOData in a SVO file.
    \param camera_id : Id of the camera instance.
    \param data : Data to ingest in the SVO file.
    \return sl_ERROR_CODE_SUCCESS in case of success, sl_ERROR_CODE_FAILURE otherwise.

    \note The method works only if the camera is recording.
     */
    INTERFACE_API enum SL_ERROR_CODE sl_ingest_data_into_svo(int camera_id, struct SL_SVOData* data);

    /**
    \brief Gets the size of data available for a given key.
    Must be called before sl_retrieve_svo_data to initialize the array at the correct size.
    \param camera_id : Id of the camera instance.
    \param key : The key of the SVOData that is going to be retrieved.
    \param ts_begin : The beginning of the range.
    \param ts_end : The end of the range.
    \return The number of data available for this key.

    \note The method will always return -1 if not in SVO mode.
     */
    INTERFACE_API int sl_get_svo_data_size(int camera_id, char key[128], unsigned long long ts_begin, unsigned long long ts_end);

    /**
    \brief Retrieves SVOData from an SVO file.
    The user is reponsible for correctly allocating the size of the data array using sl_get_svo_data_size.
    \param camera_id : Id of the camera instance.
    \param nb_data : Size of the array of data.
    \param data : The map to be filled with SVOData objects, with timestamps as keys.
    \param ts_begin : The beginning of the range.
    \param ts_end : The end of the range.
    \return sl_ERROR_CODE_SUCCESS in case of success, sl_ERROR_CODE_FAILURE otherwise.
    \note The method will return sl_ERROR_CODE_FAILURE if not in SVO mode.
    \warning You need to call sl_get_svo_data_size for the key before calling this method to correctly retrieve the data.
     */
    INTERFACE_API enum SL_ERROR_CODE sl_retrieve_svo_data(int camera_id, char* key, int nb_data, struct SL_SVOData** data, unsigned long long ts_begin, unsigned long long ts_end);

    /**
    \brief Gets the number of external channels that can be retrieved from the SVO file.
    \param camera_id : Id of the camera instance.
    \return the number of keys available.
    \note The method will return 0 if not in SVO mode.
     */
    INTERFACE_API int sl_get_svo_data_keys_size(int camera_id);

    /**
    \brief Gets the external channels that can be retrieved from the SVO file.
    The user is reponsible for correctly allocating the size of the keys array using sl_get_svo_data_keys_size.
    \param camera_id : Id of the camera instance.
    \param nb_keys : number of keys.
    \param [Out] keys : List of available keys.

    \note The method will not fill the keys array if not in SVO mode.
     */
    INTERFACE_API void sl_get_svo_data_keys(int camera_id, int nb_keys, char** keys);

    /**
    \brief Initializes and starts the positional tracking processes.

    This function allows you to enable the position estimation of the SDK.
    It only has to be called once in the camera's lifetime.

    \param camera_id : Id of the camera instance.
    \param tracking_param : A structure containing all the specific parameters for the positional tracking. Default: a preset of \ref SL_PositionalTrackingParameters.
    \param area_file_path : .area localization file that describes the surroundings, saved from a previous tracking session.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if everything went fine, \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" otherwise.
     */
    INTERFACE_API int sl_enable_positional_tracking(int camera_id, struct SL_PositionalTrackingParameters* tracking_param, const char* area_file_path);

    // /**
    // \brief Initializes and starts the positional tracking processes.
    // This function allows you to enable the position estimation of the SDK. It only has to be called once in the camera's lifetime.
    //
    // \param camera_id : id of the camera instance.
    // \param initial_world_rotation : rotation of the camera in the world frame when the camera is started. By default, it should be identity.
    // \param initial_world_position : position of the camera in the world frame when the camera is started. By default, it should be identity.
    // \param enable_area_memory : this mode enables the camera to remember its surroundings. This helps correct positional tracking drift, and can be helpful for positioning different cameras relative to one other in space.
    // \param enable_pose_smoothing : this mode enables smooth pose correction for small drift correction.
    // \param set_floor_as_origin : this mode initializes the tracking to be aligned with the floor plane to better position the camera in space.
    // \param set_as_static : this mode defines the camera as static. If true, it will not move in the environment. This allows you to set its position using initial_world_transform.
    // \param enable_imu_fusion : this mode enables or disables IMU fusion. When set to false, only the optical odometry will be used.
    // \param area_file_path : area localization file that describes the surroundings, saved from a previous tracking session.
    // \return \ref SL_ERROR_CODE::SUCCESS if everything went fine, ERROR_CODE::FAILURE otherwise.
    //  */
    //  /**INTERFACE_API int enable_positional_tracking(int camera_id, struct SL_Quaternion *initial_world_rotation, struct SL_Vector3 *initial_world_position, bool enable_area_memory, bool enable_pose_smoothing, bool set_floor_as_origin, bool set_as_static,
    //          bool enable_imu_fusion, const char* area_file_path);*/

    /**
    \brief Disables the positional tracking.
    \param camera_id : Id of the camera instance.
    \param area_file_path : If set, saves the spatial memory into an '.area' file.
    */
    INTERFACE_API void sl_disable_positional_tracking(int camera_id, const char* area_file_path);

    /**
    \brief Saves the current area learning file. The file will contain spatial memory data generated by the tracking.
    \param camera_id : Id of the camera instance.
    \param area_file_path : Save the spatial memory database in an '.area' file.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" if the <b>area_file_path</b> file wasn't found, \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" otherwise.
     */
    INTERFACE_API int sl_save_area_map(int camera_id, const char* area_file_path);

    /**
    \brief Returns the state of the spatial memory export process.

    As \ref sl_save_area_map only starts the exportation, this method allows you to know when the exportation finished or if it failed.
    \param camera_id : Id of the camera instance.
    \return The current \ref SL_AREA_EXPORTING_STATE "state" of the spatial memory export process
     */
    INTERFACE_API int sl_get_area_export_state(int camera_id);

    /**
    \brief Sets the playback cursor to the desired frame number in the SVO file.

    This function allows you to move around within a played-back SVO file. After calling, the next call to sl_grab() will read the provided position.

    \param camera_id : Id of the camera instance.
    \param position : The position of the desired frame to be decoded.
    \return \ref SL_ERROR_CODE to indicate if the method was successful.
     */
    INTERFACE_API enum SL_ERROR_CODE sl_set_svo_position(int camera_id, int position);

    /**
    \brief Pauses or resumes SVO reading
    \param camera_id : Id of the camera instance.
    \param status : If true, the reading is paused. If false, the reading is resumed.
    \note This is only relevant for SVO \ref InitParameters::svo_real_time_mode
     */
    INTERFACE_API void sl_pause_svo_reading(int camera_id, bool status);

    /**
    \brief Returns the camera FPS.
    \param camera_id : Id of the camera instance.
    \return The current frame rate.
     */
    INTERFACE_API float sl_get_camera_fps(int camera_id);

    /**
    \brief Returns the current FPS.
    \param camera_id : Id of the camera instance.
    \return The current frame rate.
        */
    INTERFACE_API float sl_get_current_fps(int camera_id);

    /**
    \brief Returns the width of the current image.
    \param camera_id : Id of the camera instance.
    \return Width of the image.
     */
    int INTERFACE_API sl_get_width(int camera_id);

    /**
    \brief Returns the height of the current image.
    \param camera_id : Id of the camera instance.
    \return Height of the image.
     */
    int INTERFACE_API sl_get_height(int camera_id);

    /**
    \brief Gets the current confidence threshold value for the disparity map (and by extension the depth map).
    \param camera_id : id of the camera instance.
    \return The confidence threshold. -1 if failure.
     */
    INTERFACE_API int sl_get_confidence_threshold(int camera_id);

    /**
    \brief Returns the SL_CameraInformation associated the camera.

    To ensure accurate calibration, it is possible to specify a custom resolution as a parameter when obtaining scaled information, as calibration parameters are resolution-dependent.
    \param camera_id : Id of the camera instance.
    \param res_width : You can specify a size different from default image size to get the scaled camera information.
    \param res_height : You can specify a size different from default image size to get the scaled camera information.
    \return SL_CameraInformation containing the calibration parameters of the ZED, as well as serial number and firmware version.
     */
    INTERFACE_API struct SL_CameraInformation* sl_get_camera_information(int camera_id, int res_width, int res_height);

    /**
    \brief Performs a new self calibration process.

    In some cases, due to temperature changes or strong vibrations, the stereo calibration becomes less accurate. Use this function to update the self-calibration data and get more reliable depth values.
    \param camera_id : Id of the camera instance.
     */
    INTERFACE_API void sl_update_self_calibration(int camera_id);

    /**
    \brief Gets the Calibration Parameters.
    \param camera_id : id of the camera instance.
    \param r_params : if true, returns Intrinsic and Extrinsic stereo parameters for original images (unrectified/distorded), else returns parameters for rectified/undistorded images.
    \return Structure containing Intrinsic and Extrinsic stereo parameters
     */
    INTERFACE_API struct SL_CalibrationParameters* sl_get_calibration_parameters(int camera_id, bool raw_params);

    /**
    \brief Gets the Sensors configuration.
    \param camera_id : id of the camera instance.
    \return Structure containing information about all the sensors available in the current device.
     */
    INTERFACE_API struct SL_SensorsConfiguration* sl_get_sensors_configuration(int camera_id);

    /**
    \brief Gets the IMU to the left camera transform matrix.
    \note This function is only effective if the camera has a model other than a \ref SL_MODEL "SL_MODEL_ZED", which does not contains internal sensors.
    \param camera_id : Id of the camera instance.
    \param translation : Translation between IMU frame and camera frame.
    \param rotation : Rotation between IMU frame and camera frame.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if sensors data have been extracted.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SENSORS_NOT_AVAILABLE" if the camera model is a \ref SL_MODEL "SL_MODEL_ZED".
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_MOTION_SENSORS_REQUIRED" if the camera model is correct but the sensors module is not opened.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_INVALID_FUNCTION_PARAMETERS" if the <b>reference_time</b> is not valid.
     */
    INTERFACE_API void sl_get_camera_imu_transform(int camera_id, struct SL_Vector3* translation, struct SL_Quaternion* rotation);

    /**
    \brief Gets the input type (see \ref SL_INPUT_TYPE).
    \param camera_id : id of the camera instance.
    \return The input type.
     */
    INTERFACE_API int sl_get_input_type(int camera_id);

    /**
    \brief Gets the ZED Serial Number.
    \param camera_id : id of the camera instance.
    \return The serial number of the camera.
     */
    INTERFACE_API int sl_get_zed_serial(int camera_id);

    /**
    \brief Gets the ZED camera current firmware version.
    \param camera_id : Id of the camera instance.
    \return The firmware of the camera.
     */
    INTERFACE_API int sl_get_camera_firmware(int camera_id);

    /**
    \brief Gets the sensor module current firmware version.
    \param camera_id : Id of the camera instance.
    \return The firmware version of the sensor module, 0 if no sensors are available.
     */
    INTERFACE_API int sl_get_sensors_firmware(int camera_id);

    /**
    \brief Gets the ZED Camera model (see \ref SL_MODEL).
    \param camera_id : id of the camera instance.
    \return The ZED Camera model.
     */
    INTERFACE_API int sl_get_camera_model(int camera_id);

    /**
    \brief Get the timestamp at the time the frame has been extracted from USB stream.

    \note It should be called after a sl_grab().
    \param camera_id : Id of the camera instance.
    \return The camera timestamp.
     */
    INTERFACE_API unsigned long long sl_get_image_timestamp(int camera_id);

    /**
    \brief Get the current timestamp at the time the function is called.

    Can be compared to the camera timestamp for synchronization.
    \param camera_id : Id of the camera instance.
    \return The current timestamp.
     */
    INTERFACE_API unsigned long long sl_get_current_timestamp(int camera_id);

    /**
    \brief Returns the number of frames in the SVO file.
    \param camera_id : Id of the camera instance.
    \return The total number of frames in the SVO file. -1 if the SDK is not reading a SVO.
     */
    INTERFACE_API int sl_get_svo_number_of_frames(int camera_id);

    /**
    \brief Test if the video setting is supported by the camera
    \param camera_id : Id of the camera instance.
    \param setting : The video setting to test
    \return true if the \ref SL_VIDEO_SETTINGS is supported by the camera, false otherwise
     */
    INTERFACE_API bool sl_is_camera_setting_supported(int camera_id, enum SL_VIDEO_SETTINGS setting);

    /**
    \brief Sets the value of the requested \ref SL_VIDEO_SETTINGS "camera setting" (gain, brightness, hue, exposure, etc.).
    \param camera_id : Id of the camera instance.
    \param mode : The setting to be set.
    \param value : The value to set.
    \return \ref SL_ERROR_CODE to indicate if the function was successful.
    \note The function works only if the camera is open in LIVE or STREAM mode.
     */
    INTERFACE_API enum SL_ERROR_CODE sl_set_camera_settings(int camera_id, enum SL_VIDEO_SETTINGS mode, int value);

    /**
    \brief Sets the range values of the requested \ref SL_VIDEO_SETTINGS "camera setting" (gain, brightness, hue, exposure, etc.).
    Works for the following settings:
    \ref SL_VIDEO_SETTINGS SL_VIDEO_SETTINGS_AUTO_EXPOSURE_TIME_RANGE
    \ref SL_VIDEO_SETTINGS SL_VIDEO_SETTINGS_AUTO_ANALOG_GAIN_RANGE
    \ref SL_VIDEO_SETTINGS SL_VIDEO_SETTINGS_AUTO_DIGITAL_GAIN_RANGE

    \param camera_id : Id of the camera instance.
    \param mode : The setting to be set.
    \param min : The min value to set.
    \param max : The max value to set.
    \return \ref SL_ERROR_CODE to indicate if the function was successful.
    \note The function works only if the camera is open in LIVE or STREAM mode.
     */
    INTERFACE_API enum SL_ERROR_CODE sl_set_camera_settings_min_max(int camera_id, enum SL_VIDEO_SETTINGS mode, int min, int max);

    /**
    \brief Sets the region of interest for automatic exposure/gain computation
    \param camera_id : Id of the camera instance.
    \param side : \ref SL_SIDE on which to be applied for AEC/AGC computation.
    \param roi : \ref SL_Rect that defines the target to be applied for AEC/AGC computation. Must be given according to camera resolution.
    \param reset : Cancel the manual ROI and reset it to the full image.
    \return \ref SL_ERROR_CODE to indicate if the function was successful.
    \note The function works only if the camera is open in LIVE or STREAM mode.
     */
    INTERFACE_API enum SL_ERROR_CODE sl_set_roi_for_aec_agc(int camera_id, enum SL_SIDE side, struct SL_Rect* roi, bool reset);

    /**
    \brief Returns the current value of the requested \ref SL_VIDEO_SETTINGS "camera setting" (gain, brightness, hue, exposure, etc.).
    \param camera_id : Id of the camera instance.
    \param mode : Setting to be retrieved (see \ref SL_VIDEO_SETTINGS).
    \param value : The requested setting value.
    \return \ref SL_ERROR_CODE to indicate if the function was successful. If successful, setting will be filled with the corresponding value.
    \note The function works only if the camera is open in LIVE or STREAM mode.
    \note Settings are not exported in the SVO file format.
     */
    INTERFACE_API enum SL_ERROR_CODE sl_get_camera_settings(int c_id, enum SL_VIDEO_SETTINGS mode, int* value);

    /**
    \brief Returns the current range of the requested \ref SL_VIDEO_SETTINGS "camera setting" (setting with range value).
    \param camera_id : Id of the camera instance.
    \param mode : Setting to be retrieved (see \ref SL_VIDEO_SETTINGS).
    \param value : The requested setting value.
    \return \ref SL_ERROR_CODE to indicate if the function was successful. If successful, setting will be filled with the corresponding value.
    \note The function works only if the camera is open in LIVE or STREAM mode.
    \note Settings are not exported in the SVO file format.
     */
    INTERFACE_API enum SL_ERROR_CODE sl_get_camera_settings_min_max(int c_id, enum SL_VIDEO_SETTINGS mode, int* minvalue, int* maxvalue);

    /**
    \brief Gets the region of interest for automatic exposure/gain computation.
    \param camera_id : Id of the camera instance.
    \param side : \ref SL_SIDE on which to get the ROI from.
    \param roi [Out] : Region of interest.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if ROI has been applied. Other \ref SL_ERROR_CODE otherwise.
    \note The function works only if the camera is open in LIVE or STREAM mode.
    \note Settings are not exported in the SVO file format.
     */
    INTERFACE_API enum SL_ERROR_CODE sl_get_roi_for_aec_agc(int id, enum SL_SIDE side, struct SL_Rect* roi);

    /**
    \brief Gets the depth min value from InitParameters (see \ref SL_InitParameters::depth_minimum_distance).
    \param camera_id : id of the camera instance.
    \return The min depth value available. -1 if failure.
     */
    INTERFACE_API float sl_get_depth_min_range_value(int camera_id);

    /**
    \brief Gets the depth max value from InitParameters (see \ref SL_InitParameters::depth_maximum_distance).
    \param camera_id : id of the camera instance.
    \return The max depth value available. -1 if failure.
     */
    INTERFACE_API float sl_get_depth_max_range_value(int camera_id);

    /**
    \brief Gets the current range of perceived depth.
    \param camera_id : Id of the camera instance.
    \param min : [Out] Minimum depth detected (in selected \ref SL_UNIT)
    \param max : [Out] Maximum depth detected (in selected \ref SL_UNIT).
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if values have been extracted. Other \ref SL_ERROR_CODE otherwise.
     */
    INTERFACE_API int sl_get_current_min_max_depth(int camera_id, float* min, float* max);

    /**
    \brief Gets the number of zed connected.
    \return The number of Zed cameras connected.
     */
    INTERFACE_API int sl_get_number_zed_connected();

    /**
    \brief Returns the version of the currently installed ZED SDK.
    \return The ZED SDK version installed.
     */
    INTERFACE_API char* sl_get_sdk_version();

    /**
    \brief Change the coordinate system of a transform matrix.
    \param rotation [In, Out] : rotation to transform.
    \param translation [In, Out] : translation to transform.
    \param coord_system_src : the current coordinate system of the translation/rotation.
    \param coord_system_dest: the destination coordinate system for the translation/rotation.
     */
    INTERFACE_API int sl_convert_coordinate_system(struct SL_Quaternion* rotation, struct SL_Vector3* translation, enum SL_COORDINATE_SYSTEM coord_system_src, enum SL_COORDINATE_SYSTEM coord_system_dest);

    // /**
    // \brief Returns the version of the currently installed ZED SDK.
    // \param major : major int of the version filled
    // \param minor : minor int of the version filled
    // \param patch : patch int of the version filled
    //  */
    // INTERFACE_API void sl_get_sdk_version(int *major, int *minor, int *patch);

     /**
     \brief Returns the current playback position in the SVO file.
     \param camera_id : Id of the camera instance.
     \return The current frame position in the SVO file. Returns -1 if the SDK is not reading an SVO.
      */
    INTERFACE_API int sl_get_svo_position(int camera_id);

    /**
    * \brief Retrieves the frame index within the SVO file corresponding to the provided timestamp.
    *  \param camera_id : Id of the camera instance.
    * \param timestamp The target timestamp for which the frame index is to be determined.
    * \return The frame index within the SVO file that aligns with the given timestamp.
    *         Returns -1 if the timestamp falls outside the bounds of the SVO file.
    */
    INTERFACE_API int sl_get_svo_position_at_timestamp(int camera_id, unsigned long long timestamp);

    /**
    \brief Gets the number of frames dropped since sl_grab() was called for the first time. Based on camera timestamps and an FPS comparison.
    \param camera_id : Id of the camera instance.
    \return The number of frames dropped since the first sl_grab() call.
     */
    INTERFACE_API unsigned int sl_get_frame_dropped_count(int camera_id);


    /**
	* \brief Gets the current status of the camera.
	* \param camera_id : Id of the camera instance.
	* \return HealthStatus Structure containing the self diagnostic results of the image/depth/sensors
    */
	INTERFACE_API struct SL_HealthStatus* sl_get_health_status(int camera_id);

    INTERFACE_API struct SL_Resolution* sl_get_retrieve_image_resolution(int camera_id, struct SL_Resolution* res);

    INTERFACE_API struct SL_Resolution* sl_get_retrieve_measure_resolution(int camera_id, struct SL_Resolution* res);
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////// Motion tracking ///////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
    \brief Tells if the tracking module is enabled.
    \param camera_id : Id of the camera instance.
    */
    INTERFACE_API bool sl_is_positional_tracking_enabled(int camera_id);

    /**
    \brief Gets the current position of the camera and state of the tracking, with an optional offset to the tracking frame.
    \param camera_id : Id of the camera instance.
    \param rotation : Quaternion filled with the current rotation of the camera depending on its reference frame.
    \param position : Vector filled with the current position of the camera depending on its reference frame.
    \param target_quaternion : Rotational offset applied to the tracking frame.
    \param target_translation : Positional offset applied to the tracking frame.
    \param reference_frame : Reference frame for setting the rotation/position.
    \return The current state of the tracking process (see \ref SL_POSITIONAL_TRACKING_STATE).
     */
    INTERFACE_API int sl_get_position_at_target_frame(int camera_id, struct SL_Quaternion* rotation, struct SL_Vector3* position, struct SL_Quaternion* target_quaternion, struct SL_Vector3* target_translation, enum SL_REFERENCE_FRAME reference_frame);
    /**
    \brief Gets the current position of the camera and state of the tracking, filling a \ref SL_PoseData struck useful for AR pass-though.
    \param camera_id : Id of the camera instance.
    \param poseData : Current Pose.
    \param reference_frame : Reference frame sor setting the rotation/position.
    \return The current state of the tracking process (see \ref SL_POSITIONAL_TRACKING_STATE).
     */
    INTERFACE_API int sl_get_position_data(int camera_id, struct SL_PoseData* poseData, enum SL_REFERENCE_FRAME reference_frame);
    /**
    \brief Retrieves the estimated position and orientation of the camera in the specified \ref REFERENCE_FRAME "reference frame".
    \param camera_id : Id of the camera instance.
    \param rotation : Quaternion filled with the current rotation of the camera depending on its reference frame.
    \param position : Vector filled with the current position of the camera depending on its reference frame.
    \param reference_frame : Reference frame for setting the rotation/position.
    \return The current state of the tracking process (see \ref SL_POSITIONAL_TRACKING_STATE).
     */
    INTERFACE_API int sl_get_position(int camera_id, struct SL_Quaternion* rotation, struct SL_Vector3* position, enum SL_REFERENCE_FRAME reference_frame);
    /**
    \brief Gets the position of the camera and the current state of the ZED Tracking as a float array (4x4).
    \param camera_id : id of the camera instance.
    \param pose : pose of the camera as a float array (float[16]).
    \param reference_frame : the reference from which you want the pose to be expressed (see SL_REFERENCE_FRAME::WORLD).
    \return The Positional tracking state.
     */
    INTERFACE_API int sl_get_position_array(int camera_id, float* pose, enum SL_REFERENCE_FRAME reference_frame);

    /**
    \brief Get the current positional tracking landmarks.
    \Note the landmarks array has to be freed by the user.
    \param camera_id : id of the camera instance.
    \param landmarks Array of presents landmarks.
	\param count number of landmarks (size of the landmarks array).
    \return ERROR_CODE that indicate if the function succeed or not.
     */
	INTERFACE_API int sl_get_positional_tracking_landmarks(int camera_id, struct SL_Landmark** landmarks, uint32_t* count);

    /**
    \brief Get the current positional tracking 2d landmarks.
    \Note the landmarks array has to be freed by the user.
    \param camera_id : id of the camera instance.
    \param landmarks 2d Array of landmarks.
    \param count number of landmarks (size of the landmarks array).
    \return ERROR_CODE that indicate if the function succeed or not.
     */
    INTERFACE_API int sl_get_positional_tracking_landmarks_2d(int camera_id, struct SL_Landmark2D** landmarks2d, uint32_t* count);
    /**
     \brief Return the current status of positional tracking module.
     \return SL_POSITIONAL_TRACKING_STATUS current status of positional tracking module.
     */
    INTERFACE_API struct SL_PositionalTrackingStatus* sl_get_positional_tracking_status(int camera_id);

    /**
    \brief Resets the tracking, and re-initializes the position with the given pose.
    \param camera_id : Id of the camera instance.
    \param rotation : Rotation of the camera in the world frame when the function is called.
    \param translation : Position of the camera in the world frame when the function is called.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if the tracking has been reset, \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" otherwise.
     */
    INTERFACE_API int sl_reset_positional_tracking(int camera_id, struct SL_Quaternion rotation, struct SL_Vector3 translation);
    /**
    \brief Resets the tracking with an offset.
    \param camera_id : Id of the camera instance.
    \param rotation : Rotation of the camera in the world frame when the function is called.
    \param translation : Position of the camera in the world frame when the function is called.
    \param target_quaternion : Rotation offset to apply.
    \param target_translation : Translation offset to apply.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if the tracking has been reset, \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" otherwise.
     */
    INTERFACE_API int sl_reset_positional_tracking_with_offset(int camera_id, struct SL_Quaternion rotation, struct SL_Vector3 translation, struct SL_Quaternion target_quaternion, struct SL_Vector3 target_translation);
    /**
    \brief Set an optional IMU orientation hint that will be used to assist the tracking during the next \ref sl_grab().

    Prior must come from a external IMU, such as the HMD orientation and should be in a time frame that's as close as possible to the camera.
    \note This function is only effective if the camera has a model other than a \ref SL_MODEL "SL_MODEL_ZED", which does not contains internal sensors.

    \param rotation : Prior rotation.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if sensors data have been extracted.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SENSORS_NOT_AVAILABLE" if the camera model is a \ref SL_MODEL "SL_MODEL_ZED".
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_MOTION_SENSORS_REQUIRED" if the camera model is correct but the sensors module is not opened.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_INVALID_FUNCTION_PARAMETERS" if the <b>reference_time</b> is not valid.
     */
    INTERFACE_API int sl_set_imu_prior_orientation(int camera_id, struct SL_Quaternion rotation);
    /**
    \brief Gets the rotation given by the IMU.
    \note This function is only effective if the camera has a model other than a \ref SL_MODEL "SL_MODEL_ZED", which does not contains internal sensors.
    \param camera_id : id of the camera instance.
    \param quat : [Out] Rotation from the IMU.
    \param time_reference : time reference.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if sensors data have been extracted.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SENSORS_NOT_AVAILABLE" if the camera model is a \ref SL_MODEL "SL_MODEL_ZED".
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_MOTION_SENSORS_REQUIRED" if the camera model is correct but the sensors module is not opened.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_INVALID_FUNCTION_PARAMETERS" if the <b>reference_time</b> is not valid.
     */
    INTERFACE_API int sl_get_imu_orientation(int camera_id, struct SL_Quaternion* quat, enum SL_TIME_REFERENCE time_reference);
    /**
    \brief Gets \ref SL_SensorsData (IMU, magnetometer, barometer) at a specific time reference.
    \note This function is only effective if the camera has a model other than a \ref SL_MODEL "SL_MODEL_ZED", which does not contains internal sensors.
    \param camera_id : Id of the camera instance.
    \param data : The \ref SL_SensorsData variable to store the data.
    \param time_reference : Defines the reference from which you want the data to be expressed.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if sensors data have been extracted.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SENSORS_NOT_AVAILABLE" if the camera model is a \ref SL_MODEL "SL_MODEL_ZED".
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_MOTION_SENSORS_REQUIRED" if the camera model is correct but the sensors module is not opened.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_INVALID_FUNCTION_PARAMETERS" if the <b>reference_time</b> is not valid.
     */
    INTERFACE_API int sl_get_sensors_data(int camera_id, struct SL_SensorsData* data, enum SL_TIME_REFERENCE time_reference);

    /**
	\brief Retrieves the size of the imu batch array. Needs to be called before sl_get_sensors_data_batch().
	\param [out] count : The number of sensors data available in the batch.
    \param camera_id : Id of the camera instance.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if sensors data have been extracted.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SENSORS_NOT_AVAILABLE" if the camera model is a \ref SL_MODEL "SL_MODEL_ZED".
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_MOTION_SENSORS_REQUIRED" if the camera model is correct but the sensors module is not opened.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_INVALID_FUNCTION_PARAMETERS" if the <b>reference_time</b> is not valid.
    */
    INTERFACE_API int sl_get_sensors_data_batch_count(int camera_id, int* count);
    /**
    \brief Retrieves all SL_SensorsData associated to most recent grabbed frame in the specified \ref COORDINATE_SYSTEM of InitParameters.
	\note sl_get_sensors_data_batch needs to be called before this function to retrieve the size of the imu batch array.
    \param [out] data : The SensorsData array to store the data.
    \param camera_id : Id of the camera instance.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if sensors data have been extracted.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SENSORS_NOT_AVAILABLE" if the camera model is a \ref SL_MODEL "SL_MODEL_ZED".
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_MOTION_SENSORS_REQUIRED" if the camera model is correct but the sensors module is not opened.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_INVALID_FUNCTION_PARAMETERS" if the <b>reference_time</b> is not valid.
    */
    INTERFACE_API int sl_get_sensors_data_batch(int camera_id, struct SL_SensorsData** data);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////// Spatial Mapping ///////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
    \brief Consolidates the chucks from a scan. This is used to turn a lots of small meshes (which are efficient for the scanning process)
    into several large meshes (which are more convenient to work with).
    \param camera_id : id of the camera instance.
    \param nb_faces : define the new number of faces per chunk (useful for Unity that doesn't handle chunks over 65K vertices).
    \param nb_vertices : Array of the number of vertices in each sub-mesh.
    \param nb_triangles : Array of the number of triangles in each sub-mesh.
    \param nb_updated_submeshes : Number of sub-meshes.
    \param updates_indices : List of all sub-meshes updated since the last update.
    \param nb_vertices_tot : Total number of updated vertices in all sub-meshes.
    \param nb_triangles_tot : Total number of updated triangles in all sub-meshes.
    \param max_sub_mesh : Maximum number of sub-meshes.
     */
    INTERFACE_API void sl_spatial_mapping_merge_chunks(int camera_id, int nb_faces, int* nb_vertices, int* nb_triangles, int* nb_updated_submeshes, int* updated_indices, int* nb_vertices_tot, int* nb_triangles_tot, const int max_submesh);

    /**
    \brief Initializes and starts the spatial mapping processes.
    \param camera_id : Id of the camera instance.
    \param type : Spatial mapping type (see \ref SL_SPATIAL_MAP_TYPE).
    \param resolution_meter : Spatial mapping resolution in meters.
    \param max_range_meter : Maximum scanning range in meters.
    \param save_texture : True to scan surface textures in addition to geometry.
    \param max_memory_usage : The maximum CPU memory (in megabytes) allocated for the meshing process.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if everything went fine, \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" otherwise.
     */
    INTERFACE_API int sl_enable_spatial_mapping(int camera_id, struct SL_SpatialMappingParameters* mapping_param);
    /**
    \brief Disables the spatial mapping process.
    \param camera_id : Id of the camera instance.
     */
    INTERFACE_API void sl_disable_spatial_mapping(int camera_id);

    /**
    \brief Returns the SL_SpatialMappingParameters used.

    It corresponds to the structure given as argument to the \ref sl_enable_spatial_mapping() method.
    \param camera_id : Id of the camera instance.
    \return \ref SL_SpatialMappingParameters containing the parameters used for spatial mapping initialization.
     */
    INTERFACE_API struct SL_SpatialMappingParameters* sl_get_spatial_mapping_parameters(int camera_id);
    /**
     Pauses or resumes the spatial mapping processes.
     \param camera_id : Id of the camera instance.
     \param status : If true, the integration is paused. If false, the spatial mapping is resumed.
     */
    INTERFACE_API void sl_pause_spatial_mapping(int camera_id, bool status);
    /**
    \brief Starts the spatial map generation process in a non-blocking thread from the spatial mapping process.
    \param camera_id : Id of the camera instance.
     */
    INTERFACE_API void sl_request_mesh_async(int camera_id);
    /**
    \brief Returns the spatial map generation status.

    Useful for knowing when to update and retrieve the mesh or the point cloud.
    \param camera_id : Id of the camera instance.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if the mesh or the point cloud is ready and not yet retrieved, otherwise \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE".
     */
    INTERFACE_API int sl_get_mesh_request_status_async(int camera_id);
    /**
    \brief Returns the current spatial mapping state.

    As the spatial mapping runs asynchronously, this function allows you to get reported errors or status info.
    \param camera_id : Id of the camera instance.
    \return The current \ref SL_SPATIAL_MAPPING_STATE "state" of the spatial mapping process.
     */
    INTERFACE_API enum SL_SPATIAL_MAPPING_STATE sl_get_spatial_mapping_state(int camera_id);
    /**
    \brief Updates the internal version of the mesh and returns the sizes of the meshes.
    \param camera_id : Id of the camera instance.
    \param nb_vertices : Array of the number of vertices in each sub-mesh.
    \param nb_triangles : Array of the number of triangles in each sub-mesh.
    \param nb_submeshes : Number of sub-meshes.
    \param updates_indices : List of all sub-meshes updated since the last update.
    \param num_vertices_tot : Total number of updated vertices in all sub-meshes.
    \param num_triangles_tot : Total number of updated triangles in all sub-meshes.
    \param max_sub_mesh : Maximum number of sub-meshes.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if the mesh is updated.
     */
    INTERFACE_API int sl_update_mesh(int camera_id, int* nb_vertices_per_submesh, int* nb_triangles_per_submesh, int* nb_submeshes, int* updated_indices, int* nb_vertices_tot, int* nb_triangles_tot, const int max_submesh);
    /**
    \brief Retrieves all chunks of the current mesh.
    \note Call \ref sl_update_mesh before calling this.

    Vertex and triangles arrays must be at least of the sizes returned by \ref sl_update_mesh (nb_vertices and nb_triangles).
    \param camera_id : Id of the camera instance.
    \param vertices : Vertices of the mesh
    \param triangles : Triangles of the mesh.
    \param colors : BGR colors of each vertex.
    \param max_submeshes : Maximum number of sub-mesh that can be handled.
    \param uvs : UVs of the texture.
    \param texture_ptr : Texture of the mesh (if enabled).
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if the mesh is retrieved.
     */
    INTERFACE_API int sl_retrieve_mesh(int camera_id, float* vertices, int* triangles, unsigned char* colors, float* uvs, unsigned char* texture_ptr, const int max_submeshes);
    /**
    \brief Updates the Internal version of the mesh and returns the sizes of the meshes.
    \param camera_id : Id of the camera instance.
    \param nb_vertices : Array of the number of vertices in each sub-mesh.
    \param nb_triangles : Array of the number of triangles in each sub-mesh.
    \param nb_submeshes : Number of sub-meshes.
    \param updates_indices : List of all sub-meshes updated since the last update.
    \param num_vertices_tot : Total number of updated vertices in all sub-meshes.
    \param num_triangles_tot : Total number of updated triangles in all sub-meshes.
    \param max_sub_mesh : Maximum number of sub-meshes.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if the chunks are updated.
     */
    INTERFACE_API int sl_update_chunks(int camera_id, int* nb_vertices_per_submesh, int* nb_triangles_per_submesh, int* nb_submeshes, int* updated_indices, int* nb_vertices_tot, int* nb_triangles_tot, const int max_submesh);
    /**
    \brief Retrieves all chunks of the full mesh.
    \note Call \ref sl_update_mesh before calling this.

    Vertex and triangles arrays must be at least of the sizes returned by \ref sl_update_mesh (nbVertices and nbTriangles).
    \param camera_id : Id of the camera instance.
    \param max_submesh : Maximum number of sub-mesh that can be handled.
    \param vertices : Vertices of the chunk.
    \param triangles : Triangles of the chunk.
    \param colors : BGR colors of the chunk.
    \param uvs : UVs of the texture.
    \param texture_ptr : Texture of the mesh (if enabled).
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if the chunk is retrieved.
     */
    INTERFACE_API int sl_retrieve_chunks(int camera_id, float* vertices, int* triangles, unsigned char* colors, float* uvs, unsigned char* texture_ptr, const int max_submesh);

    /**
    \brief Updates the fused point cloud (if spatial map type was \ref SL_SPATIAL_MAP_TYPE "SL_SPATIAL_MAP_TYPE_FUSED_POINT_CLOUD").
    \param camera_id : Id of the camera instance.
    \param num_vertices_tot : The total number of vertices.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if the fused point cloud is updated.
     */
    INTERFACE_API int sl_update_fused_point_cloud(int camera_id, int* nb_vertices_tot);
    /**
    \brief Retrieves all points of the fused point cloud.
    \note Call \ref sl_update_fused_point_cloud before calling this.
    \param camera_id : Id of the camera instance.
    \param vertices : Points of the fused point cloud.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if the fused point cloud is retrieved.
     */
    INTERFACE_API int sl_retrieve_fused_point_cloud(int camera_id, float* vertices);
    /**
    \brief Extracts the current spatial map from the spatial mapping process.
    \param camera_id : Id of the camera instance.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if the mesh is filled and available, otherwise \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE".
     */
    INTERFACE_API int sl_extract_whole_spatial_map(int camera_id);
    /**
    \brief Saves the scanned mesh in a specific file format.
    \param camera_id : Id of the camera instance.
    \param filename : Path and filename of the mesh.
    \param format : File format (extension). Can be .obj, .ply or .bin.
    \return Has the mesh been save successfully.
     */
    INTERFACE_API bool sl_save_mesh(int camera_id, const char* filename, enum SL_MESH_FILE_FORMAT format);
    /**
    \brief Saves the scanned point cloud in a specific file format.
    \param camera_id : Id of the camera instance.
    \param filename : Path and filename of the point cloud.
    \param format : File format (extension). Can be .obj, .ply or .bin.
    \return Has the point cloud been save successfully.
     */
    INTERFACE_API bool sl_save_point_cloud(int c_id, const char* filename, enum SL_MESH_FILE_FORMAT format);
    /**
    \brief Loads a saved mesh file.
    \param camera_id : Id of the camera instance.
    \param filename : Path and filename of the mesh. Should include the extension (.obj, .ply or .bin).
    \param nb_ertices : Array of the number of vertices in each sub-mesh.
    \param nb_triangles : Array of the number of triangles in each sub-mesh.
    \param nb_sub_meshes : Number of sub-meshes.
    \param updated_indices : List of all sub-meshes updated since the last update.
    \param nb_vertices_tot :  Total number of updated vertices in all sub-meshes.
    \param nb_triangles_tot : Array of the number of triangles in each sub-mesh.
    \param max_submesh : Maximum number of sub-meshes that can be handled.
    \param texture_size : Array containing the sizes of all the textures (width, height) if applicable.
    \return Has the mesh been loaded successfully.
     */
    INTERFACE_API bool sl_load_mesh(int camera_id, const char* filename, int* nb_vertices_per_submesh, int* nb_triangles_per_submesh, int* num_submeshes, int* updated_indices, int* nb_vertices_tot, int* nb_triangles_tot, int* textures_size, const int max_submesh);
    /**
    \brief Applies the scanned texture onto the internal scanned mesh.
    \param camera_id : Id of the camera instance.
    \param nb_vertices : Array of the number of vertices in each sub-mesh.
    \param nb_triangles : Array of the number of triangles in each sub-mesh.
    \param nb_sub_meshes : Number of sub-meshes.
    \param updated_indices : List of all sub-meshes updated since the last update.
    \param nb_vertices_tot : Total number of updated vertices in all sub-meshes.
    \param nb_triangles_tot : Array of the number of triangles in each sub-mesh.
    \param max_submesh : Maximum number of sub-meshes that can be handled.
    \param texture_size : Array containing the sizes of all the textures (width, height) if applicable.
    \return Has the texture been applied successfully.
     */
    INTERFACE_API bool sl_apply_texture(int camera_id, int* nb_vertices_per_submesh, int* nb_triangles_per_submesh, int* nb_updated_submeshes, int* updated_indices, int* nb_vertices_tot, int* nb_triangles_tot, int* textures_size, const int max_submesh);
    /**
    \brief Filters a mesh to removes triangles while still preserving its overall shaper (though less accurate).
    \param camera_id : Id of the camera instance.
    \param filter_params : Filter level. Higher settings remove more triangles (\ref SL_MESH_FILTER).
    \param nb_vertices : Array of the number of vertices in each sub-mesh.
    \param nb_triangles : Array of the number of triangles in each sub-mesh.
    \param nb_sub_meshes : Number of sub-meshes.
    \param updated_indices : List of all sub-meshes updated since the last update.
    \param nb_vertices_tot : Total number of updated vertices in all sub-meshes.
    \param nb_triangles_tot : Array of the number of triangles in each sub-mesh.
    \param max_submesh : Maximum number of sub-meshes that can be handled.
    \return Has the mesh been filtered successfully.
     */
    INTERFACE_API bool sl_filter_mesh(int camera_id, enum SL_MESH_FILTER filter_params, int* nb_vertices_per_submesh, int* nb_triangles_per_submesh, int* nb_updated_submeshes, int* updated_indices, int* nb_vertices_tot, int* nb_triangles_tot, const int max_submesh);
    /**
    \brief Gets a vector pointing toward the direction of gravity. This is estimated from a 3D scan of the environment,
    and such, a scan must be started and finished for this value to be calculated.
    \n If using a model other than \ref SL_MODEL "SL_MODEL_ZED", this isn't required thanks to its IMU.
    \param camera_id : Id of the camera instance.
    \param gravity : [Out] The vector of gravity.
     */
    INTERFACE_API void sl_spatial_mapping_get_gravity_estimation(int camera_id, struct SL_Vector3* gravity);


    //////////////////

     /**
    \brief Updates the internal version of the whole mesh and returns the size of its data.
    \param camera_id : id of the camera instance.
    \param nb_vertices : Total number of updated vertices in all sub-meshes.
    \param nb_triangles : Total number of updated triangles in all sub-meshes.
    \return SUCCESS if the chunks are updated.
     */
    INTERFACE_API int sl_update_whole_mesh(int camera_id, int* nb_vertices, int* nb_triangles);
    /**
    \brief Retrieves the full mesh. Call update_mesh before calling this.
    Vertex and triangles arrays must be at least of the sizes returned by update_mesh (nb_vertices and nbTriangles).
    \param camera_id : id of the camera instance.
    \param vertices : Vertices of the mesh
    \param triangles : Triangles of the mesh.
    \param colors : (b,g,r) colors of the mesh.
    \param uvs : uvs of the texture.
    \param texture_ptr : Texture of the mesh (if enabled).
    \return SUCCESS if the chunk is retrieved.
     */
    INTERFACE_API int sl_retrieve_whole_mesh(int camera_id, float* vertices, int* triangles, unsigned char* colors,  float* uvs, unsigned char* texture_ptr);
    /**
    \brief Loads a saved mesh file.
    \param camera_id : id of the camera instance.
    \param filename : Path and filename of the mesh. Should include the extension (.obj, .ply or .bin).
    \param nb_vertices :  Total number of updated vertices in all sub-meshes.
    \param nb_triangles : Array of the number of triangles in each sub-mesh.
    \param max_submesh : Maximum number of sub-meshes that can be handled.
    \param texture_size : Array containing the sizes of all the textures (width ,height) if applicable.
    \return Has the mesh been loaded successfully.
     */
    INTERFACE_API bool sl_load_whole_mesh(int camera_id, const char* filename, int* nb_vertices, int* nb_triangles, int* texture_size);
    /**
    \brief Applies the scanned texture onto the internal scanned mesh.
    \param camera_id : id of the camera instance.
    \param nb_vertices :  Total number of updated vertices in all sub-meshes.
    \param nb_triangles : Array of the number of triangles in each sub-mesh.
    \param texture_size : Array containing the sizes of all the textures (width ,height) if applicable.
    \return Has the texture been applied successfully.
     */
    INTERFACE_API bool sl_apply_whole_texture(int camera_id, int* nb_vertices, int* nb_triangles, int* texture_size);
    /**
    \brief Filters a mesh to removes triangles while still preserving its overall shaper (though less accurate).
    \param camera_id : id of the camera instance.
    \param nb_vertices :  Total number of updated vertices in all sub-meshes.
    \param nb_triangles : Array of the number of triangles in each sub-mesh.
    \return Has the mesh been filtered successfully.
     */
    INTERFACE_API bool sl_filter_whole_mesh(int camera_id, enum SL_MESH_FILTER filter_params, int* nb_vertices, int* nb_triangles);


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////// Plane Detection ///////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
    \brief Detect the floor plane of the scene.
    \param camera_id : Id of the camera instance.
    \param reset_quaternion : The rotation to align the axis with the gravity.
    \param reset_translation : The translation to align the tracking with the floor plane. The initial position will then be at ground height.
    \param prior_rotation : Prior rotation.
    \param prior_translation : Prior translation.
    \return The detected floor plane if the function succeeded.
     */
    INTERFACE_API struct SL_PlaneData* sl_find_floor_plane(int camera_id, struct SL_Quaternion* reset_quaternion, struct SL_Vector3* reset_translation, struct SL_Quaternion prior_rotation, struct SL_Vector3 prior_translation);
    /**
    \brief Checks the plane at the given left image coordinates.
    \param camera_id : Id of the camera instance.
    \param pixel : The image coordinate. The coordinate must be taken from the full-size image.
    \param params : A structure containing all the specific parameters for the plane detection. Default: a preset of \ref SL_PlaneDetectionParameters.
    \param thres : Check if area is enough for Unity. If true, removes smaller planes.
    \return Data of the detected plane.
     */
    INTERFACE_API struct SL_PlaneData* sl_find_plane_at_hit(int camera_id, struct SL_Vector2 pixel, struct SL_PlaneDetectionParameters* params, bool thres);
    /**
    \brief Using data from a detected floor plane, updates supplied vertex and triangles arrays with data needed to make a mesh that represents it.
    \param camera_id : id of the camera instance.
    \param vertices : Array to be filled with mesh vertices.
    \param triangles : Array to be filled with mesh triangles, stored as indexes of each triangle's points.
    \param nb_vertices_tot : Total number of vertices in the mesh.
    \param nb_triangles_tot : Total triangles indexes (3x number of triangles).
    \return SUCCESS if the mesh conversion was successful.
     */
    INTERFACE_API int sl_convert_floorplane_to_mesh(int camera_id, float* vertices, int* triangles, int* nb_vertices_tot, int* nb_triangles_tot);
    /**
    \brief Using data from a detected hit plane, updates supplied vertex and triangles arrays with data needed to make a mesh that represents it.
    \param camera_id : id of the camera instance.
    \param vertices : Array to be filled with mesh vertices.
    \param triangles : Array to be filled with mesh triangles, stored as indexes of each triangle's points.
    \param nb_vertices_tot : Total number of vertices in the mesh.
    \param nb_triangles_tot : Total triangles indexes (3x number of triangles).
    \return SUCCESS if the mesh conversion was successful.
     */
    INTERFACE_API int sl_convert_hitplane_to_mesh(int camera_id, float* vertices, int* triangles, int* nb_vertices_tot, int* nb_triangles_tot);


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////// Retrieve /////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
    \brief Retrieves a measure texture from the ZED SDK.

    Use this to get an individual texture from the last grabbed frame with measurements in every pixel - such as depth map, confidence map etc.
    \n Measure textures are not human-viewable but don't lose accuracy, unlike image textures.
    \param camera_id : Id of the camera instance.
    \param measure_ptr : Pointer to the measure texture.
    \param type : Measure type (depth, confidence, xyz, etc). See \ref SL_MEASURE.
    \param mem : Whether the measure should be on CPU or GPU memory. See \ref SL_MEM.
    \param width : Width of the texture in pixel. If set to 0, the camera resolution or SL_InitParameters.maximum_working_resolution will be taken, whichever is the smallest.
    \param height : Height of the texture in pixel. If set to 0, the camera resolution or SL_InitParameters.maximum_working_resolution will be taken, whichever is the smallest.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if the retrieve succeeded.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_INVALID_FUNCTION_PARAMETERS" if the view mode requires a module not enabled (VIEW::DEPTH with DEPTH_MODE::NONE for example).
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_INVALID_RESOLUTION" if the resolution is higher than one provided by getCameraInformation().camera_configuration.resolution.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" if another error occurred.
     */
    INTERFACE_API int sl_retrieve_measure(int camera_id, void* measure_ptr, enum SL_MEASURE type, enum SL_MEM mem, int width, int height, void* custream);
    /**
    \brief Retrieves an image texture from the ZED SDK in a human-viewable format.

    Image textures work for when you want the result to be visible, such as the direct RGB image from the camera, or a greyscale image of the depth.
    However it will lose accuracy if used to show measurements like depth or confidence, unlike measure textures.
    \param camera_id : Id of the camera instance.
    \param image_ptr : Pointer to the image texture.
    \param type : Image type (left RGB, right depth map, etc). See \ref SL_VIEW.
    \param  mem : Whether the image should be on CPU or GPU memory (\ref SL_MEM).
    \param width : Width of the texture in pixel. If set to 0, the camera resolution or SL_InitParameters.maximum_working_resolution will be taken, whichever is the smallest.
    \param height : Height of the texture in pixel. If set to 0, the camera resolution or SL_InitParameters.maximum_working_resolution will be taken, whichever is the smallest.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if the retrieve succeeded.
     */
    INTERFACE_API int sl_retrieve_image(int camera_id, void* image_ptr, enum SL_VIEW type, enum SL_MEM mem, int width, int height, void* custream);

    /**
    \brief Convert Image format from Unsigned char to Signed char, designed for Unreal Engine pipeline, works on GPU memory.
    \param image_in : input image to convert
    \param image_signed : output image to converted
    \param stream : a cuda stream to put the compute to (def. 0)
    \note If the Output Mat does not satisfies the requirements, it is freed and re-allocated.
    */
    INTERFACE_API int sl_convert_image(void* image_in_ptr, void* image_signed_ptr, void* stream);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////// Streaming Sender //////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
    \brief Creates a streaming pipeline.
    \param camera_id : Id of the camera instance.
    \param codec : defines the codec used for streaming (see \ref SL_STREAMING_CODEC).
    \param bitrate : defines the streaming bitrate in Kbits/s.
    \param port : Defines the port used for streaming.
    \param gop_size : Defines the gop size in number of frames.
    \param adaptative_bitrate : Enable/disable adaptive bitrate.
    \param chunk_size : Defines a single chunk size.
    \param target_framrate : Defines the target framerate for the streaming output.
    \return An \ref SL_ERROR_CODE that defines if the stream was started.
    \note Available range for <b>bitrate</b>: [1000 - 60000]
    | STREAMING_CODEC  | Resolution   | FPS   | bitrate (kbps) |
    |------------------|--------------|-------|----------------|
    | H264             |  HD2K        |   15  |     8500       |
    | H264             |  HD1080      |   30  |    12500       |
    | H264             |  HD720       |   60  |     7000       |
    | H265             |  HD2K        |   15  |     7000       |
    | H265             |  HD1080      |   30  |    11000       |
    | H265             |  HD720       |   60  |     6000       |
     */
    INTERFACE_API int sl_enable_streaming(int camera_id, enum SL_STREAMING_CODEC codec, unsigned int bitrate, unsigned short port, int gop_size, int adaptative_bitrate, int chunk_size, int target_framerate);

    /**
    \brief Disables the streaming initiated by \ref sl_enable_streaming().
    \param camera_id : Id of the camera instance.
     */
    INTERFACE_API void sl_disable_streaming(int camera_id);
    /**
    \brief Tells if the streaming is running.
    \param camera_id : Id of the camera instance.
    \return Is the streaming running successfully.
     */
    INTERFACE_API int sl_is_streaming_enabled(int camera_id);

    /**
    \brief Returns the \ref SL_StreamingParameters used.

    It corresponds to the structure given as argument to the \ref sl_enable_streaming() function.
    \return \ref SL_StreamingParameters containing the parameters used for streaming initialization.
     */
    INTERFACE_API struct SL_StreamingParameters* sl_get_streaming_parameters(int camera_id);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////// Save to File Utils ////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
    \brief Writes the image into a file defined by its extension.
    \param camera_id : id of the camera instance.
    \param view : View of the image (SL_VIEW).
    \param file_name : file path including the name and extension.
    \return SUCCESS if the image was successfuly saved.
     */
    INTERFACE_API int sl_save_current_image(int camera_id, enum SL_VIEW view, const char* file_name);
    /**
    \brief Writes the depth map into a file defined by its extension.
    \param camera_id : id of the camera instance.
    \param side : side of the image (0 = left, 1 = right).
    \param file_name : file path including the name and extension.
    \return SUCCESS if the depth was successfuly saved.
     */
    INTERFACE_API int sl_save_current_depth(int camera_id, enum SL_SIDE side, const char* file_name);
    /**
    \brief Writes the point cloud into a file defined by its extension.
    \param camera_id : id of the camera instance.
    \param side : side of the pc (0 = left, 1 = right).
    \param file_name : file path including the name and extension.
    \return SUCCESS if the point cloud was successfuly saved.
     */
    INTERFACE_API int sl_save_current_point_cloud(int camera_id, enum SL_SIDE side, const char* file_name);



#if WITH_OBJECT_DETECTION

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////// Object Detection //////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

     /**
        \brief Check if a corresponding optimized engine is found for the requested Model based on your rig configuration.
        \param model : AI model to check.
        \param gpu_id : ID of the gpu.
        \return The status of the given model for the specified GPU.
    */
    INTERFACE_API struct SL_AI_Model_status* sl_check_AI_model_status(enum SL_AI_MODELS model, int gpu_id);

    /**
    \brief Optimize the requested model, possible download if the model is not present on the host.
    \param model : AI model to optimize.
    \param gpu_id : ID of the gpu to optimize on.
    \return SUCCESS if the model is well optimized.
    */
    INTERFACE_API int sl_optimize_AI_model(enum SL_AI_MODELS model, int gpu_id);

    /**
    \brief Initializes and starts object detection module.

    The object detection module currently support multiple StereoLabs' model for different purposes: "MULTI_CLASS", "PERSON_HEAD"
    \n The full list of model is available through \ref SL_OBJECT_DETECTION_MODEL and the full list of detectable objects is available through \ref SL_OBJECT_CLASS and \ref SL_OBJECT_SUBCLASS.

    \note - <b>This Deep Learning detection module is not available for \ref MODEL "MODEL::ZED" cameras (ZED first generation).</b>.
    \note - This feature uses AI to locate objects and requires a powerful GPU. A GPU with at least 3GB of memory is recommended.

    Detected objects can be retrieved using the \ref sl_retrieve_objects() function.
    \param camera_id : Id of the camera instance.
    \param object_detection_parameters : A structure containing all the specific parameters for the object detection. Default: a preset of \ref SL_ObjectDetectionParameters.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if everything went fine.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_CORRUPTED_SDK_INSTALLATION" if the AI model is missing or corrupted. In this case, the SDK needs to be reinstalled.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_MODULE_NOT_COMPATIBLE_WITH_CAMERA" if the camera used does not have a IMU (\ref SL_MODEL "SL_MODEL_ZED").
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_MOTION_SENSORS_REQUIRED" if the camera model is correct (not \ref SL_MODEL "SL_MODEL_ZED") but the IMU is missing. It probably happens because \ref SL_InitParameters.sensors_required was set to false and that IMU has not been found.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_INVALID_FUNCTION_CALL" if one of the <b>object_detection_parameters</b> parameter is not compatible with other modules parameters (for example, <b>depth_mode</b> has been set to \ref SL_DEPTH_MODE "SL_DEPTH_MODE_NONE").
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" otherwise.

    \note The IMU gives the gravity vector that helps in the 3D box localization.
     */
    INTERFACE_API int sl_enable_object_detection(int camera_id, struct SL_ObjectDetectionParameters* object_detection_parameters);

    /**
    \brief Returns the \ref SL_ObjectDetectionParameters used.

    It corresponds to the structure given as argument to the \ref sl_enable_object_detection() function.
    \param camera_id : Id of the camera instance.
    \return \ref SL_ObjectDetectionParameters containing the parameters used for object detection initialization.
     */
    INTERFACE_API struct SL_ObjectDetectionParameters* sl_get_object_detection_parameters(int camera_id);


    /**
    \brief Disables the Object Detection process.

    The object detection module immediately stops and frees its memory allocations.
    \param camera_id : Id of the camera instance.
    \param instance_id : Id of the object detection instance. Used when multiple instances of the object detection module are enabled at the same time.
    \param force_disable_all_instances : Should disable all instances of the object detection module or just <b>instance_id</b>.
     */
    INTERFACE_API void sl_disable_object_detection(int camera_id, unsigned int instance_id, bool force_disable_all_instances);


    /**
    \brief Set the Object detection module instance runtime parameters
     By default the object detection module will use the parameters set in the ObjectDetectionRuntimeParameters constructor.
     This can be changed at any time, however since the processing is done in parallel, the parameters will be used for the next inference.
     This function can be called only on parameters change, the previous values will be applied during inference.
    \param camera_id : Id of the camera instance.
    \param params : ObjectDetectionRuntimeParameters parameters
    \param instance_id : Object detection instance id, by default the first instance
    */
    INTERFACE_API int sl_set_object_detection_runtime_parameters(int camera_id, struct SL_ObjectDetectionRuntimeParameters object_detection_parameters, unsigned int instance_id);

    /**
    \brief Initializes and starts the Deep Learning Body Tracking module.

    The body tracking module currently supports multiple classes of human skeleton detection with the \ref SL_BODY_TRACKING_MODEL "SL_BODY_TRACKING_MODEL_HUMAN_BODY_FAST",
    \ref SL_BODY_TRACKING_MODEL "SL_BODY_TRACKING_MODEL_HUMAN_BODY_MEDIUM" or \ref SL_BODY_TRACKING_MODEL "SL_BODY_TRACKING_MODEL_HUMAN_BODY_ACCURATE".
    \n This model only detects humans but provides a full skeleton map for each person.

    \note - <b>This Deep Learning detection module is not available for \ref MODEL "MODEL::ZED" cameras (ZED first generation).</b>.
    \note - This feature uses AI to locate objects and requires a powerful GPU. A GPU with at least 3GB of memory is recommended.

    Detected objects can be retrieved using the \ref sl_retrieve_bodies() function.
    \param camera_id : Id of the camera instance.
    \param body_tracking_parameters : A structure containing all the specific parameters for the body tracking. Default: a preset of \ref SL_BodyTrackingParameters.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if everything went fine.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_CORRUPTED_SDK_INSTALLATION" if the AI model is missing or corrupted. In this case, the SDK needs to be reinstalled.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_MODULE_NOT_COMPATIBLE_WITH_CAMERA" if the camera used does not have a IMU (\ref SL_MODEL "SL_MODEL_ZED").
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_MOTION_SENSORS_REQUIRED" if the camera model is correct (not \ref SL_MODEL "SL_MODEL_ZED") but the IMU is missing. It probably happens because \ref SL_InitParameters.sensors_required was set to false and that IMU has not been found.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_INVALID_FUNCTION_CALL" if one of the <b>object_detection_parameters</b> parameter is not compatible with other modules parameters (for example, <b>depth_mode</b> has been set to \ref SL_DEPTH_MODE "SL_DEPTH_MODE_NONE").
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" otherwise.

    \note The IMU gives the gravity vector that helps in the 3D box localization.
     */
    INTERFACE_API int sl_enable_body_tracking(int camera_id, struct SL_BodyTrackingParameters* body_tracking_parameters);

    /**
    \brief Returns the \ref SL_BodyTrackingParameters used.

    It corresponds to the structure given as argument to the \ref sl_enable_body_tracking() function.
    \param camera_id : Id of the camera instance.
    \return \ref SL_BodyTrackingParameters containing the parameters used for body tracking initialization.
     */
    INTERFACE_API struct SL_BodyTrackingParameters* sl_get_body_tracking_parameters(int camera_id);

    /**
    \brief Disables the body tracking process.

    The body tracking module immediately stops and frees its memory allocations.
    \param camera_id : Id of the camera instance.
    \param instance_id : Id of the body tracking instance. Used when multiple instances of the body tracking module are enabled at the same time.
    \param force_disable_all_instances : Should disable all instances of the tracking module module or just <b>instance_id</b>.
     */
    INTERFACE_API void sl_disable_body_tracking(int camera_id, unsigned int instance_id, bool force_disable_all_instances);

    /**
    \brief Set the Body tracking module instance runtime parameters
     By default the Body tracking module will use the parameters set in the BodyTrackingRuntimeParameters constructor.
     This can be changed at any time, however since the processing is done in parallel, the parameters will be used for the next inference.
     This function can be called only on parameters change, the previous values will be applied during inference.
    \param params : BodyTrackingRuntimeParameters parameters
    \param instance_id : Body tracking instance id, by default the first instance
    */
    INTERFACE_API int sl_set_body_tracking_runtime_parameters(int camera_id, struct SL_BodyTrackingRuntimeParameters body_tracking_parameters, unsigned int instance_id);

    /**
    \brief Generate a UUID like unique id to help identify and track AI detections.
    \param uuid : Unique ID generated.
    \return Size of the unique ID generated.
     */
    INTERFACE_API int sl_generate_unique_id(char* uuid);

    INTERFACE_API int sl_generate_unique_id_str(char* uuid);

    /**
    \brief Feed the 3D Object tracking function with your own 2D bounding boxes from your own detection algorithm.
    \param camera_id : Id of the camera instance.
    \param nb_objects : Number of custom objects (size of the object_in array).
    \param objects_in : 2D detections from custom detection algorithm.
    \param instance_id : Id of the Object detection instance. Used when multiple instances of the BT module are enabled at the same time.
    \note The detection should be done on the current grabbed left image as the internal process will use all current available data to extract 3D informations and perform object tracking.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if everything went fine, \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" otherwise.
   */
    INTERFACE_API int sl_ingest_custom_box_objects(int camera_id, int nb_objects, struct SL_CustomBoxObjectData* objects_in, unsigned int instance_id);

    /**
    \brief Feed the 3D Object tracking function with your own 2D bounding boxes with masks from your own detection algorithm.
    \param camera_id : Id of the camera instance.
    \param nb_objects : Number of custom objects (size of the object_in array).
    \param objects_in : 2D detections from custom detection algorithm.
    \param instance_id : Id of the Object detection instance. Used when multiple instances of the BT module are enabled at the same time.
    \note The detection should be done on the current grabbed left image as the internal process will use all current available data to extract 3D informations and perform object tracking.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if everything went fine, \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" otherwise.
   */
    INTERFACE_API int sl_ingest_custom_mask_objects(int camera_id, int nb_objects, struct SL_CustomMaskObjectData* objects_in, unsigned int instance_id);

    /**
    \brief Retrieve objects detected by the object detection module.
    \param camera_id : Id of the camera instance.
    \param objects : The detected objects will be saved into this object. If the object already contains data from a previous detection, it will be updated, keeping a unique ID for the same person.
    \param object_detection_runtime_parameters : Object detection runtime settings, can be changed at each detection. In async mode, the parameters update is applied on the next iteration.
    \param instance_id : Id of the object detection instance. Used when multiple instances of the object detection module are enabled at the same time.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if everything went fine, \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" otherwise.
     */
    INTERFACE_API int sl_retrieve_objects(int camera_id, struct SL_ObjectDetectionRuntimeParameters* object_detection_runtime_parameters, struct SL_Objects* objects, unsigned int instance_id);

    /**
    \brief Retrieve objects detected by the custom object detection module.
    \param camera_id : Id of the camera instance.
    \param object_detection_runtime_parameters : Custom object detection runtime settings, can be changed at each detection. In async mode, the parameters update is applied on the next iteration.
    \param objects : The detected objects will be saved into this object. If the object already contains data from a previous detection, it will be updated, keeping a unique ID for the same person.
    \param instance_id : Id of the object detection instance. Used when multiple instances of the object detection module are enabled at the same time.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if everything went fine, \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" otherwise.
     */
    INTERFACE_API int sl_retrieve_custom_objects(int camera_id, struct SL_CustomObjectDetectionRuntimeParameters* object_detection_runtime_parameters, struct SL_Objects* objects, unsigned int instance_id);

    /**
    \brief Set the Object detection module instance runtime parameters when using the Custom model (OBJECT_DETECTION_MODEL::CUSTOM_BOX_OBJECTS and CUSTOM_BOX_OBJECTS::CUSTOM_YOLOLIKE_BOX_OBJECTS)
     By default the object detection module will use the parameters set in the CustomObjectDetectionRuntimeParameters constructor.
     This can be changed at any time, however since the processing is done in parallel, the parameters will be used for the next inference.
     This function can be called only on parameters change, the previous values will be applied during inference.
    \param camera_id : Id of the camera instance.
    \param params : CustomObjectDetectionRuntimeParameters parameters
    \param instance_id : Object detection instance id, by default the first instance
    */
    INTERFACE_API int sl_set_custom_object_detection_runtime_parameters(int camera_id, struct SL_CustomObjectDetectionRuntimeParameters custom_object_detection_parameters, unsigned int instance_id);

    /**
    \brief Retrieve bodies detected by the body tracking module.
    \param camera_id : id of the camera instance.
    \param body_tracking_runtime_parameters : Body Tracking runtime settings, can be changed at each detection. In async mode, the parameters update is applied on the next iteration.
    \param bodies : The detected bodies will be saved into this object. If the object already contains data from a previous detection, it will be updated, keeping a unique ID for the same person.
    \param instance_id : Id of the object detection instance. Used when multiple instances of the object detection module are enabled at the same time.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if everything went fine, \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" otherwise.
     */
    INTERFACE_API int sl_retrieve_bodies(int camera_id, struct SL_BodyTrackingRuntimeParameters* body_tracking_runtime_parameters, struct SL_Bodies* bodies, unsigned int instance_id);

    /**
    \brief Updates the internal batch of detected objects.
    \param camera_id : Id of the camera instance.
    \param nb_batches : [Out] Number of batches.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if everything went fine, \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" otherwise.
     */
    INTERFACE_API int sl_update_objects_batch(int camera_id, int* nb_batches);

    /**
    \brief Gets a batch of detected objects.
    \note Need to be called after \ref sl_update_objects_batch().
    \param camera_id : Id of the camera instance.
    \param objs_batch : Structure containing  all the specific data of the object batch module
    \param index : Desired index in the trajectories vector.
    \return \ref SL_ERROR_CODE "SL_ERROR_CODE_SUCCESS" if everything went fine, \ref SL_ERROR_CODE "SL_ERROR_CODE_FAILURE" otherwise.
     */
    INTERFACE_API int sl_get_objects_batch(int camera_id, int index, struct SL_ObjectsBatch* objs_batch);

    INTERFACE_API int sl_get_objects_batch_csharp(int camera_id, int index, int* nb_data, int* id, int* label, int* sublabel, int* tracking_state,
        struct SL_Vector3 positions[MAX_TRAJECTORY_SIZE], float position_covariances[MAX_TRAJECTORY_SIZE][6], struct SL_Vector3 velocities[MAX_TRAJECTORY_SIZE], unsigned long long timestamps[MAX_TRAJECTORY_SIZE],
        struct SL_Vector2 bounding_boxes_2d[MAX_TRAJECTORY_SIZE][4], struct SL_Vector3 bounding_boxes[MAX_TRAJECTORY_SIZE][8], float confidences[MAX_TRAJECTORY_SIZE], int action_states[MAX_TRAJECTORY_SIZE],
        struct SL_Vector2 head_bounding_boxes_2d[MAX_TRAJECTORY_SIZE][4], struct SL_Vector3 head_bounding_boxes[MAX_TRAJECTORY_SIZE][8],
        struct SL_Vector3 head_positions[MAX_TRAJECTORY_SIZE]);

#if 0
    /**
    \brief Updates the internal batch of detected bodies.
    \param camera_id : id of the camera instance.
    \param [Out] : number of batches.
    \return \ref SUCCESS if everything went fine, \ref ERROR_CODE::FAILURE otherwise
     */
    INTERFACE_API int sl_update_bodies_batch(int camera_id, int* nb_batches);



    /**
    \brief Gets a batch of detected objects. Need to be called after update_objects_batch().
    \param camera_id : id of the camera instance.
    \param bodies_batch : Structure containing  all the specific data of the bodies batch module
    \return \ref SUCCESS if everything went fine, \ref ERROR_CODE::FAILURE otherwise
     */
    INTERFACE_API int sl_get_bodies_batch(int camera_id, int index, struct SL_BodiesBatch* bodies_batch);


#endif
#endif
#ifdef __cplusplus
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////// MULTI CAMERA API //////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
extern "C" {
#endif

    /** \brief FusionHandler initialisation. Initializes memory/generic datas
    * \param [in] params : structure containing all init parameters for the fusion API
    * \return SL_FUSION_ERROR_CODE
    */
    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_init(struct SL_InitFusionParameters* params);

    /** \brief process the fusion.
    * \return SL_FUSION_ERROR_CODE
    */
    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_process();

    /*
    * \brief adds a camera to the multi camera handler
    * \param [in] uuid : unique ID that is associated with the camera for easy access.
    * \param [in] params : communications parameters
    * \param [in] pose_translation : position of the camera
    * \param [in] pose_rotation : orientation of the camera
    * \return SL_FUSION_ERROR_CODE
    * */
    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_subscribe(struct SL_CameraIdentifier* uuid, struct SL_CommunicationParameters* params, struct SL_Vector3* pose_translation, struct SL_Quaternion* pose_rotation);

    /**
     * \brief Returns the current sl_VIEW_LEFT of the specified camera, the data is synchronized.
     * \param mat: the CPU BGRA image of the requested camera.
     * \param uuid: the requested camera identifier.
     * \param width: the requested width of the output image, can be lower or equal (default) to the original image width.
     * \param height: the requested height of the output image, can be lower or equal (default) to the original image height.
     * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
     * \note Only the Left BGRA image is available.
     */
    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_retrieve_image(void* mat, struct SL_CameraIdentifier* uuid, int width, int height);

    /**
     * \brief Returns the current measure of the specified camera, the data is synchronized.
     * \param mat: the CPU data of the requested camera.
     * \param uuid: the requested camera identifier.
     * \param measure: the requested measure type, by default DEPTH (F32_C1)
     * \param width: the requested width of the output image, can be lower or equal (default) to the original image width.
     * \param height: the requested height of the output image, can be lower or equal (default) to the original image height.
     *\param reference_frame: the requested reference frame, by default BASELINK. it is only available for fused point clouds
     * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
     * \note Only MEASURE: DEPTH, XYZ, XYZRGBA, XYZBGRA, XYZARGB, XYZABGR, DEPTH_U16_MM are available.
     */
    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_retrieve_measure(void* mat, struct SL_CameraIdentifier* uuid, enum SL_MEASURE measure, int width, int height, enum SL_FUSION_REFERENCE_FRAME reference_frame);

    /**
     * \brief Remove the specified camera from data provider.
     * \param uuid: The requested camera identifier.
     * \return \ref SL_FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an SL_FUSION_ERROR_CODE.
     */
    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_unsubscribe(struct SL_CameraIdentifier* uuid);

    /*
    * \brief update the pose of the camera in the fusion coordinate space
    * \param [in] uuid : unique ID that is associated with the camera for easy access.
    * \param [in] pose_translation : new position of the camera
    * \param [in] pose_rotation : new orientation of the camera
    * \return SL_FUSION_ERROR_CODE
    * */
    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_update_pose(struct SL_CameraIdentifier* uuid, struct SL_Vector3* pose_translation, struct SL_Quaternion* pose_rotation);

    /**
     * \brief Get the specified camera position inside fusion WORLD.
     * \param uuid: The requested camera identifier.
     * \param pose: The World position of the camera, regarding the other camera of the setup.
     * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
     */
    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_get_pose(struct SL_CameraIdentifier* uuid, struct SL_Vector3* pose_translation, struct SL_Quaternion* pose_rotation);

    /*
    * \brief Returns the state of a connected data sender.
    * \param [in] uuid : Identifier of the camera.
    * \return SL_SENDER_ERROR_CODE : State of the sender
    * */
    INTERFACE_API enum SL_SENDER_ERROR_CODE sl_fusion_get_sender_state(struct SL_CameraIdentifier* uuid);

    /**
    \brief Read a Configuration JSON file to configure a fusion process.
    \param json_config_filename : The name of the JSON file containing the configuration
    \param coord_sys : The COORDINATE_SYSTEM in which you want the World Pose to be in.
    \param unit : The UNIT in which you want the World Pose to be in.

    \return a vector of \ref SL_FusionConfiguration for all the camera present in the file.
    \note empty if no data were found for the requested camera.
     */
    INTERFACE_API void sl_fusion_read_configuration_file(const char* json_config_filename, enum SL_COORDINATE_SYSTEM coord_system, enum SL_UNIT unit, struct SL_FusionConfiguration configs[MAX_FUSED_CAMERAS], int* nb_cameras);


    /**
    \brief Read a Configuration JSON string to configure a fusion process.
    \param fusion_configuration : The string containing the configuration (it will be parsed like a json).
    \param coord_sys : The COORDINATE_SYSTEM in which you want the World Pose to be in.
    \param unit : The UNIT in which you want the World Pose to be in.

    \return A vector of \ref FusionConfiguration for all the camera present in the file.
    \note Empty if no data were found for the requested camera.
     */
    INTERFACE_API void sl_fusion_read_configuration(const char* fusion_configuration, enum SL_COORDINATE_SYSTEM coord_system, enum SL_UNIT unit, struct SL_FusionConfiguration configs[MAX_FUSED_CAMERAS], int* nb_cameras);

    /////////////////////////////////////////////////////////////////////
    ///////////////////// Object Detection Fusion ///////////////////////
    /////////////////////////////////////////////////////////////////////

    /** \brief enables Object detection fusion module
    * \param [in] parameters defined by \ref sl::ObjectDetectionFusionParameters
    * \return SL_FUSION_ERROR_CODE
    */
    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_enable_body_tracking(struct SL_BodyTrackingFusionParameters* params);

	/**
	\brief Disable the object detection module.
	 */
	INTERFACE_API void sl_fusion_disable_body_tracking();

    /**
    * \brief retrieves a list of bodies (in SL_Bodies class type) seen by all cameras and merged as if it was seen by a single super-camera.
    *\note Internal calls retrieveBodies() for all listed cameras, then merged into a single SL_Bodies
    * \param [out] bodies: list of bodies seen by all available cameras
    * \param reference_frame: The reference frame in which the objects will be expressed.
    * \note Only the 3d informations is available in the returned object.
    * \return SL_FUSION_ERROR_CODE
    */
    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_retrieve_bodies(struct SL_Bodies* bodies, struct SL_BodyTrackingFusionRuntimeParameters* rt, struct SL_CameraIdentifier uuid, enum SL_FUSION_REFERENCE_FRAME reference_frame);

    /**
     * \brief get the stats of a given camera in the Fusion API side
     * It can be the received FPS, drop frame, latency, etc
     * \param metrics : structure containing all the metrics available
     * \return SL_FUSION_ERROR_CODE
     */
    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_get_process_metrics(struct SL_FusionMetrics* metrics);


    /////////////////////////////////////////////////////////////////////
    ///////////////////// Positional tracking  //////////////////////////
    /////////////////////////////////////////////////////////////////////

    /**
    * \brief enable positional tracking fusion.
    * \note note that for the alpha version of the API, the positional tracking fusion doesn't support the area memory feature
    *
    * \return SL_FUSION_ERROR_CODE
    */
    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_enable_positional_tracking(struct SL_PositionalTrackingFusionParameters* params);

    /**
     * \brief Get the Fused Position of the camera system
     *
     * \param camera_pose will contain the camera pose in world position (world position is given by the calibration of the cameras system)
     * \param reference_frame defines the reference from which you want the pose to be expressed. Default : \ref REFERENCE_FRAME "REFERENCE_FRAME::WORLD".
     * \param uuid Camera identifier
     * \return POSITIONAL_TRACKING_STATE is the current state of the tracking process
     */
    INTERFACE_API enum SL_POSITIONAL_TRACKING_STATE sl_fusion_get_position(struct SL_PoseData* pose, enum SL_REFERENCE_FRAME reference_frame,
                                                           struct SL_CameraIdentifier* uuid, enum SL_POSITION_TYPE retrieve_type);


    /**
     * @brief Get the current status of fused position.
     * \return SL_FusedPositionalTrackingStatus is the current status of the tracking process.
     */
    INTERFACE_API struct SL_FusedPositionalTrackingStatus* sl_fusion_get_fused_positional_tracking_status();

    /**
     * \brief disable the positional tracking
     */
    INTERFACE_API void sl_fusion_disable_positional_tracking();

    /////////////////////////////////////////////////////////////////////
    /////////////////////////// GNSS Fusion /////////////////////////////
    /////////////////////////////////////////////////////////////////////

    /**
     * @brief Convert a ENU position expressed inside the Fusion coordinate system to a global world coordinate.
     * \param in Input enu coordinate.
     * \param out Converted position in lat/lng coordinate system.
     * \return ERROR_CODE FAILURE if the conversion failed, SUCCESS otherwise.
     */
    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_enu_to_geo(struct SL_ENU* in, struct SL_LatLng* out);

    /**
     * @brief Convert a global world coordinate to a ENU position expressed inside the Fusion coordinate system.
     * \param in Input lat/lng coordinate.
     * \param out Converted position in ENU coordinate system.
     * \return ERROR_CODE FAILURE if the conversion failed, SUCCESS otherwise.
     */
    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_geo_to_enu(struct SL_LatLng* in, struct SL_ENU* out);

    /**
     * \brief Add GNSS that will be used by fusion for computing fused pose.
     * \param out [in]: the current GNSS data
     * \param radian [in] : true if the gnssdata is set in radian
     */
    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_ingest_gnss_data(struct SL_GNSSData* gnss_data, bool radian);

    /**
     * \brief returns the current GNSS data
     * \param out [out]: the current GNSS data
     * \param radian [in] : true if the gnss data is set in radian
     * \return POSITIONAL_TRACKING_STATE is the current state of the tracking process
     */
    INTERFACE_API enum SL_POSITIONAL_TRACKING_STATE sl_fusion_get_current_gnss_data(struct SL_GNSSData* data, bool radian);

    /**
     * \brief returns the current GeoPose
     * \param pose [out]: the current GeoPose
     * \param radian [in] : true if the geopose is set in radian.
     * \return SL_GNSS_FUSION_STATUS is the current state of the tracking process
     */
    INTERFACE_API enum SL_GNSS_FUSION_STATUS sl_fusion_get_geo_pose(struct SL_GeoPose* pose, bool radian);

    /**
     * \brief Convert latitude / longitude into position in sl::Fusion coordinate system.
     * \param in: the current GeoPose
     * \param out [out]: the current Pose
     * \param radian [in] : true if the geopose is set in radian.
     * \return SL_GNSS_FUSION_STATUS is the current state of the tracking process
     */
    INTERFACE_API enum SL_GNSS_FUSION_STATUS sl_fusion_geo_to_camera(struct SL_LatLng* in, struct SL_PoseData* out, bool radian);

    /**
     * \brief returns the current GeoPose
     * \param pose [out]: the current GeoPose
     * \param radian [in] : true if the geopose is set in radian.
     * \return SL_GNSS_FUSION_STATUS is the current state of the tracking process
     */
    INTERFACE_API enum SL_GNSS_FUSION_STATUS sl_fusion_camera_to_geo(struct SL_PoseData* in, struct SL_GeoPose* out, bool radian);

    /**
     * @brief returns the current timestamp
     * \return the current timestamp in nanoseconds.
     */
    INTERFACE_API unsigned long long sl_fusion_get_current_timestamp();

    /**
     * \brief Get the current calibration uncertainty defined during calibration process
     *
     * @param yaw_std [out] yaw uncertainty
     * @param x_std [out] position uncertainty
     * \return SL_GNSS_FUSION_STATUS representing current initialisation status
     */
    INTERFACE_API enum SL_GNSS_FUSION_STATUS sl_fusion_get_current_gnss_calibration_std(float* yaw_std, struct SL_Vector3* position_std);

    /**
     * \brief Get the calibration found between VIO and GNSS
     * \return sl::Transform transform containing calibration found between VIO and GNSS
     */
    INTERFACE_API void sl_fusion_get_geo_tracking_calibration(struct SL_Vector3* translation, struct SL_Quaternion* rotation);

	/**
	\brief Close Multi Camera instance.
	 */
	INTERFACE_API void sl_fusion_close();

#ifdef __cplusplus
}
#endif



#ifdef __cplusplus
extern "C" {
#endif

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   ////////////////////////////////////////////////////////////////// Mat ///////////////////////////////////////////////////////////////////////////////////
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
    \brief Creates a matrix with the given resolution.
    \param width : Width of the new matrix.
    \param height : Height of the new matrix.
    \param type : Data type and number of channels the matrix will hold (see \ref SL_MAT_TYPE).
    \param mem : Whether the matrix should be stored on CPU or GPU memory (\ref SL_MEM).
    \return Pointer of the matrix.
    */
    INTERFACE_API void* sl_mat_create_new(int width, int height, enum SL_MAT_TYPE type, enum SL_MEM mem);
    /**
    \brief Creates an empty matrix.
    \return Pointer of the empty matrix.
     */
    INTERFACE_API void* sl_mat_create_new_empty();
    /**
    \brief Returns whether the matrix is initialized or not.
    \param ptr : Pointer of the matrix.
    \return True if the matrix has been initialized.
     */
    INTERFACE_API bool sl_mat_is_init(void* ptr);
    /**
    \brief Frees the memory of the matrix.
    \param ptr : Pointer of the matrix to free.
    \param mem : Specifies which memory you wish to free. Default: \ref SL_MEM_CPU
     */
    INTERFACE_API void sl_mat_free(void* ptr, enum SL_MEM mem);
    /**
    \brief Returns the information about the matrix into a string.
    \param ptr : Pointer of the matrix to get information from.
    \param buffer : Buffer providing matrix information.
     */
    INTERFACE_API void sl_mat_get_infos(void* ptr, char* buffer);

    // GET values
    /**
    \brief Returns the value of a specific point in the matrix of type \ref SL_MAT_TYPE_U8_C1.
    \param ptr : Pointer of the matrix.
    \param col : Column of the point to get the value from.
    \param row : Row of the point to get the value from.
    \param value[Out] : Variable to store the value to get.
    \param mem : Which memory should be read.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_get_value_uchar(void* ptr, int col, int row, unsigned char* value, enum SL_MEM mem);
    /**
    \brief Returns the value of a specific point in the matrix of type \ref SL_MAT_TYPE_U8_C2.
    \param ptr : Pointer of the matrix.
    \param col : Column of the point to get the value from.
    \param row : Row of the point to get the value from.
    \param value[Out] : Variable to store the value to get.
    \param mem : Which memory should be read.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_get_value_uchar2(void* ptr, int col, int row, struct SL_Uchar2* value, enum SL_MEM mem);
    /**
    \brief Returns the value of a specific point in the matrix of type \ref SL_MAT_TYPE_U8_C3.
    \param ptr : Pointer of the matrix.
    \param col : Column of the point to get the value from.
    \param row : Row of the point to get the value from.
    \param value[Out] : Variable to store the value to get.
    \param mem : Which memory should be read.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_get_value_uchar3(void* ptr, int col, int row, struct SL_Uchar3* value, enum SL_MEM mem);
    /**
    \brief Returns the value of a specific point in the matrix of type \ref SL_MAT_TYPE_U8_C4.
    \param ptr : Pointer of the matrix.
    \param col : Column of the point to get the value from.
    \param row : Row of the point to get the value from.
    \param value[Out] : Variable to store the value to get.
    \param mem : Which memory should be read.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_get_value_uchar4(void* ptr, int col, int row, struct SL_Uchar4* value, enum SL_MEM mem);

    /**
    \brief Returns the value of a specific point in the matrix of type \ref SL_MAT_TYPE_F32_C1.
    \param ptr : Pointer of the matrix.
    \param col : Column of the point to get the value from.
    \param row : Row of the point to get the value from.
    \param value[Out] : Variable to store the value to get.
    \param mem : Which memory should be read.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_get_value_float(void* ptr, int col, int row, float* value, enum SL_MEM mem);
    /**
    \brief Returns the value of a specific point in the matrix of type \ref SL_MAT_TYPE_F32_C2.
    \param ptr : Pointer of the matrix.
    \param col : Column of the point to get the value from.
    \param row : Row of the point to get the value from.
    \param value[Out] : Variable to store the value to get.
    \param mem : Which memory should be read.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_get_value_float2(void* ptr, int col, int row, struct SL_Vector2* value, enum SL_MEM mem);
    /**
    \brief Returns the value of a specific point in the matrix of type \ref SL_MAT_TYPE_F32_C3.
    \param ptr : Pointer of the matrix.
    \param col : Column of the point to get the value from.
    \param row : Row of the point to get the value from.
    \param value[Out] : Variable to store the value to get.
    \param mem : Which memory should be read.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_get_value_float3(void* ptr, int col, int row, struct SL_Vector3* value, enum SL_MEM mem);
    /**
    \brief Returns the value of a specific point in the matrix of type \ref SL_MAT_TYPE_F32_C4.
    \param ptr : Pointer of the matrix.
    \param col : Column of the point to get the value from.
    \param row : Row of the point to get the value from.
    \param value[Out] : Variable to store the value to get.
    \param mem : Which memory should be read.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_get_value_float4(void* ptr, int col, int row, struct SL_Vector4* value, enum SL_MEM mem);

    // SET VALUE
    /**
    \brief Sets a value to a specific point in the matrix of type \ref SL_MAT_TYPE_U8_C1.
    \param ptr : Pointer of the matrix.
    \param col : Column of the point to set the value.
    \param row : Row of the point to set the value.
    \param value : Value to be set.
    \param mem : Which memory will be updated.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_set_value_uchar(void* ptr, int col, int row, unsigned char value, enum SL_MEM mem);
    /**
    \brief Sets a value to a specific point in the matrix of type \ref SL_MAT_TYPE_U8_C2.
    \param ptr : Pointer of the matrix.
    \param col : Column of the point to set the value.
    \param row : Row of the point to set the value.
    \param value : Value to be set.
    \param mem : Which memory will be updated.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_set_value_uchar2(void* ptr, int col, int row, struct SL_Uchar2 value, enum SL_MEM mem);
    /**
    \brief Sets a value to a specific point in the matrix of type \ref SL_MAT_TYPE_U8_C3.
    \param ptr : Pointer of the matrix.
    \param col : Column of the point to set the value.
    \param row : Row of the point to set the value.
    \param value : Value to be set.
    \param mem : Which memory will be updated.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_set_value_uchar3(void* ptr, int col, int row, struct SL_Uchar3 value, enum SL_MEM mem);
    /**
    \brief Sets a value to a specific point in the matrix of type \ref SL_MAT_TYPE_U8_C4.
    \param ptr : Pointer of the matrix.
    \param col : Column of the point to set the value.
    \param row : Row of the point to set the value.
    \param value : Value to be set.
    \param mem : Which memory will be updated.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_set_value_uchar4(void* ptr, int col, int row, struct SL_Uchar4 value, enum SL_MEM mem);
    /**
    \brief Sets a value to a specific point in the matrix of type \ref SL_MAT_TYPE_F32_C1.
    \param ptr : Pointer of the matrix.
    \param col : Column of the point to set the value.
    \param row : Row of the point to set the value.
    \param value : Value to be set.
    \param mem : Which memory will be updated.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_set_value_float(void* ptr, int col, int row, float value, enum SL_MEM mem);
    /**
    \brief Sets a value to a specific point in the matrix of type \ref SL_MAT_TYPE_F32_C2.
    \param ptr : Pointer of the matrix.
    \param col : Column of the point to set the value.
    \param row : Row of the point to set the value.
    \param value : Value to be set.
    \param mem : Which memory will be updated.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_set_value_float2(void* ptr, int col, int row, struct SL_Vector2 value, enum SL_MEM mem);
    /**
    \brief Sets a value to a specific point in the matrix of type \ref SL_MAT_TYPE_F32_C3.
    \param ptr : Pointer of the matrix.
    \param col : Column of the point to set the value.
    \param row : Row of the point to set the value.
    \param value : Value to be set.
    \param mem : Which memory will be updated.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_set_value_float3(void* ptr, int col, int row, struct SL_Vector3 value, enum SL_MEM mem);
    /**
    \brief Sets a value to a specific point in the matrix of type \ref SL_MAT_TYPE_F32_C4.
    \param ptr : Pointer of the matrix.
    \param col : Column of the point to set the value.
    \param row : Row of the point to set the value.
    \param value : Value to be set.
    \param mem : Which memory will be updated.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_set_value_float4(void* ptr, int col, int row, struct SL_Vector4 value, enum SL_MEM mem);

    //SET TO
    /**
    \brief Fills the matrix of type \ref SL_MAT_TYPE_U8_C1 with the given value.
    \param ptr : Pointer of the matrix.
    \param value : Value with which to fill the matrix.
    \param mem : Which buffer to fill.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_set_to_uchar(void* ptr, unsigned char value, enum SL_MEM mem);
    /**
    \brief Fills the matrix of type \ref SL_MAT_TYPE_U8_C2 with the given value.
    \param ptr : Pointer of the matrix.
    \param value : Value with which to fill the matrix.
    \param mem : Which buffer to fill.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_set_to_uchar2(void* ptr, struct SL_Uchar2 value, enum SL_MEM mem);
    /**
    \brief Fills the matrix of type \ref SL_MAT_TYPE_U8_C3 with the given value.
    \param ptr : Pointer of the matrix.
    \param value : Value with which to fill the matrix.
    \param mem : Which buffer to fill.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_set_to_uchar3(void* ptr, struct SL_Uchar3 value, enum SL_MEM mem);
    /**
    \brief Fills the matrix of type \ref SL_MAT_TYPE_U8_C4 with the given value.
    \param ptr : Pointer of the matrix.
    \param value : Value with which to fill the matrix.
    \param mem : Which buffer to fill.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_set_to_uchar4(void* ptr, struct SL_Uchar4 value, enum SL_MEM mem);
    /**
    \brief Fills the matrix of type \ref SL_MAT_TYPE_F32_C1 with the given value.
    \param ptr : Pointer of the matrix.
    \param value : Value with which to fill the matrix.
    \param mem : Which buffer to fill.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_set_to_float(void* ptr, float value, enum SL_MEM mem);
    /**
    \brief Fills the matrix of type \ref SL_MAT_TYPE_F32_C2 with the given value.
    \param ptr : Pointer of the matrix.
    \param value : Value with which to fill the matrix.
    \param mem : Which buffer to fill.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_set_to_float2(void* ptr, struct SL_Vector2 value, enum SL_MEM mem);
    /**
    \brief Fills the matrix of type \ref SL_MAT_TYPE_F32_C3 with the given value.
    \param ptr : Pointer of the matrix.
    \param value : Value with which to fill the matrix.
    \param mem : Which buffer to fill.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_set_to_float3(void* ptr, struct SL_Vector3 value, enum SL_MEM mem);
    /**
    \brief Fills the matrix of type \ref SL_MAT_TYPE_F32_C4 with the given value.
    \param ptr : Pointer of the matrix.
    \param value : Value with which to fill the matrix.
    \param mem : Which buffer to fill.
    \return \ref SL_ERROR_CODE (as an integer) indicating if the get was successful, or why it wasn't.
     */
    INTERFACE_API int sl_mat_set_to_float4(void* ptr, struct SL_Vector4 value, enum SL_MEM mem);

    /**
    \brief Copies data from the GPU to the CPU, if possible.
    \param ptr : Pointer of the matrix.
    \return \ref SL_ERROR_CODE_SUCCESS if everything went well, \ref SL_ERROR_CODE_FAILURE otherwise.
     */
    INTERFACE_API int sl_mat_update_cpu_from_gpu(void* ptr);
    /**
    \brief Copies data from the CPU to the GPU, if possible.
    \param ptr : Pointer of the matrix.
    \return \ref SL_ERROR_CODE_SUCCESS if everything went well, \ref SL_ERROR_CODE_FAILURE otherwise.
     */
    INTERFACE_API int sl_mat_update_gpu_from_cpu(void* ptr);
    /**
    \brief Copies data from this matrix to another matrix (deep copy).
    \param ptr : Pointer of the source matrix.
    \param ptr_dest : Pointer of the destination matrix.
    \param cpy_type : Specifies the memory that will be used for the copy (see \ref SL_COPY_TYPE).
    \return \ref SL_ERROR_CODE_SUCCESS if everything went well, \ref SL_ERROR_CODE_FAILURE otherwise.
     */
    INTERFACE_API int sl_mat_copy_to(void* ptr, void* ptr_dest, enum SL_COPY_TYPE cpy_type);

    /**
    \brief Reads an image from a file.

    Supports .png and .jpeg. Only works if matrix has access to \ref SL_MEM_CPU.
    \param ptr : Pointer of the matrix.
    \param file_path : Path of the file to read from (including the name and extension).
    \return \ref SL_ERROR_CODE_SUCCESS if everything went well, \ref SL_ERROR_CODE_FAILURE otherwise.
     */
    INTERFACE_API int sl_mat_read(void* ptr, const char* file_path);
    /**
    \brief Writes the Mat into a file as an image. Only works if Mat has access to MEM_CPU.
    \param ptr : Pointer of the matrix.
    \param file_path : Path of the file to write (including the name and extension).
    \return \ref SL_ERROR_CODE_SUCCESS if everything went well, \ref SL_ERROR_CODE_FAILURE otherwise.
     */
    INTERFACE_API int sl_mat_write(void* ptr, const char* file_path);
    /**
    \brief Returns the width of the matrix.
    \param ptr : Pointer of the matrix.
    \return Width of the matrix in pixels.
     */
    INTERFACE_API int sl_mat_get_width(void* ptr);
    /**
    \brief Returns the height of the matrix.
    \param ptr : Pointer of the matrix.
    \return Height of the matrix in pixels.
     */
    INTERFACE_API int sl_mat_get_height(void* ptr);
    /**
    \brief Returns the number of values stored in one pixel.
    \param ptr : Pointer of the matrix.
    \return Number of values in a pixel.
     */
    INTERFACE_API int sl_mat_get_channels(void* ptr);
    /**
    \brief Gets the type of memory (CPU and/or GPU).
    \param ptr : Pointer of the matrix.
    \return Type of allocated memory (\ref SL_MEM).
     */
    INTERFACE_API int sl_mat_get_memory_type(void* ptr);

    /**
    \brief Returns the format of the matrix.
    \param ptr : Pointer of the matrix.
    \return The memory type (\ref SL_MAT_TYPE).
     */
    INTERFACE_API int sl_mat_get_data_type(void* ptr);
    /**
    \brief Returns the size of one pixel in bytes.
    \param ptr : Pointer of the matrix.
    \return Size of a pixel in bytes.
     */
    INTERFACE_API int sl_mat_get_pixel_bytes(void* ptr);
    /**
    \brief Returns the memory step in number of elements (size in one pixel row).
    \param ptr : Pointer of the matrix.
    \param mem : Specifies whether you want \ref SL_MEM_CPU or \ref SL_MEM_GPU step.
    \return The step in number of elements.
     */
    INTERFACE_API int sl_mat_get_step(void* ptr, enum SL_MEM mem);
    /**
    \brief Returns the memory step in bytes (size of one pixel row).
    \param ptr : Pointer of the matrix.
    \param mem : Specifies whether you want \ref SL_MEM_CPU or \ref SL_MEM_GPU step.
    \return The step in bytes of the specified memory.
     */
    INTERFACE_API int sl_mat_get_step_bytes(void* ptr, enum SL_MEM mem);
    /**
    \brief Returns the size of a row in bytes.
    \param ptr : Pointer of the matrix.
    \return Size of a row in bytes.
     */
    INTERFACE_API int sl_mat_get_width_bytes(void* ptr);
    /**
    \brief Returns whether the matrix is the owner of the memory it accesses.
    \return true if the matrix is owning its memory, else false.
     */
    INTERFACE_API bool sl_mat_is_memory_owner(void* ptr);
    /**
    \brief Returns the resolution (width and height) of the matrix.
    \param ptr : Pointer of the matrix.
    \return \ref SL_Resolution of the matrix in pixels.
     */
    INTERFACE_API struct SL_Resolution sl_mat_get_resolution(void* ptr);
    /**
    \brief Allocates memory for the matrix.
    \param ptr : Pointer of the matrix.
    \param width : Width of the matrix in pixels.
    \param height : Height of the matrix in pixels.
    \param type : Type of the matrix (\ref SL_MAT_TYPE_F32_C1, \ref SL_MAT_TYPE_U8_C4, etc.).
    \param mem : Where the buffer will be stored.
     */
    INTERFACE_API void sl_mat_alloc(void* ptr, int width, int height, enum SL_MAT_TYPE type, enum SL_MEM mem);
    /**
    \brief Copies data from another matrix into this one (deep copy).
    \param ptr : Pointer of the matrix.
    \param ptr_source : Source matrix from which to copy.
    \param copy_type : Specifies the memory that will be used for the copy (see \ref SL_COPY_TYPE).
    \param mem : Where the buffer will be stored.
    \return \ref SL_ERROR_CODE_SUCCESS if everything went well, \ref SL_ERROR_CODE_FAILURE otherwise.
     */
    INTERFACE_API int sl_mat_set_from(void* ptr, void* ptr_source, enum SL_COPY_TYPE copy_type);
    /**
    \brief Gets a pointer to the matrix.
    \param ptr : Pointer of the matrix.
    \param mem : Whether the matrix should exist on CPU or GPU memory (\ref SL_MEM).
    \return A pointer to the matrix.
     */
    INTERFACE_API int* sl_mat_get_ptr(void* ptr, enum SL_MEM mem);
    /**
    \brief Duplicates a matrix by copying all its data into a new one (deep copy).
    \param ptr : Pointer of the matrix.
    \param ptr_source : Source matrix from which to copy.
     */
    INTERFACE_API int sl_mat_clone(void* ptr, void* ptr_source);

    /**
    \brief Swaps the content of the provided matrix (only swaps the pointers, no data copy).
    \param ptr1 : Pointer of the first matrix to swap.
    \param ptr2 : Pointer of the second matrix to swap.
    */
    INTERFACE_API void sl_mat_swap(void* ptr_1, void* ptr_2);

    /**
    \brief Convert the color channels of the Mat (RGB<->BGR or RGBA<->BGRA)
     * This methods works only on 8U_C4 or 8U_C3
     */
    INTERFACE_API int sl_mat_convert_color(void* ptr, enum SL_MEM memory, bool swap_RB_channels, void* stream);

    /**
    \brief Convert the color channels of the Mat into another Mat
     * This methods works only on 8U_C4 if remove_alpha_channels is enabled, or 8U_C4 and 8U_C3 if swap_RB_channels is enabled
     * The inplace method sl::Mat::convertColor can be used for only swapping the Red and Blue channel efficiently
     */
    INTERFACE_API int sl_convert_color(void* mat1, void* mat2, bool swap_RB_channels, bool remove_alpha_channels, enum SL_MEM memory, void* stream);

    /**
    \brief Convert an image into a GPU Tensor in planar channel configuration (NCHW), ready to use for deep learning model
    \param image_in : input image to convert
    \param tensor_out : output GPU tensor
    \param resolution_out : resolution of the output image, generally square, although not mandatory
    \param scalefactor : Scale factor applied to each pixel value, typically to convert the char value into [0-1] float
    \param mean : mean, statistic to normalized the pixel values, applied AFTER the scale. For instance for imagenet statistics the mean would be sl::float3(0.485, 0.456, 0.406)
    \param stddev : standard deviation, statistic to normalized the pixel values, applied AFTER the scale. For instance for imagenet statistics the standard deviation would be sl::float3(0.229, 0.224, 0.225)
    \param keep_aspect_ratio : indicates if the original width and height ratio should be kept using padding (sometimes refer to as letterboxing) or if the image should be stretched
    \param swap_RB_channels : indicates if the Red and Blue channels should be swapped (RGB<->BGR or RGBA<->BGRA)
    \param stream : a cuda stream to put the compute
     */
    INTERFACE_API int sl_blob_from_image(void* image_in, void* tensor_out, struct SL_Resolution resolution_out,
        float scalefactor, struct SL_Vector3 mean, struct SL_Vector3 stddev, bool keep_aspect_ratio, bool swap_RB_channels,
        void* stream);

    /**
    \brief Convert a list of images into a GPU Tensor in planar channel configuration (NCHW), ready to use for deep learning model
    \param image_in : input images to convert
    \param nb_images : number of images to convert
    \param tensor_out : output GPU tensor batched
    \param resolution_out : resolution of the output image, generally square, although not mandatory
    \param scalefactor : Scale factor applied to each pixel value, typically to convert the char value into [0-1] float
    \param mean : mean, statistic to normalized the pixel values, applied AFTER the scale. For instance for imagenet statistics the mean would be sl::float3(0.485, 0.456, 0.406)
    \param stddev : standard deviation, statistic to normalized the pixel values, applied AFTER the scale. For instance for imagenet statistics the standard deviation would be sl::float3(0.229, 0.224, 0.225)
    \param keep_aspect_ratio : indicates if the original width and height ratio should be kept using padding (sometimes refer to as letterboxing) or if the image should be stretched
    \param swap_RB_channels : indicates if the Red and Blue channels should be swapped (RGB<->BGR or RGBA<->BGRA)
    \param stream : a cuda stream to put the compute
     */

	INTERFACE_API int sl_blob_from_images(void** image_in, int nb_images, void* tensor_out, struct SL_Resolution resolution_out,
		float scalefactor, struct SL_Vector3 mean, struct SL_Vector3 stddev, bool keep_aspect_ratio, bool swap_RB_channels,
        void* stream);

    /**
    \brief Check if the camera is a ZED One (Monocular) or ZED (Stereo)
    \param m : Camera model
    */
    INTERFACE_API bool sl_is_camera_one(enum SL_MODEL m);

    /**
    \brief Check if a resolution is available for a given camera model
    \param r : Resolution to check
    \param m : Camera model
    */
    INTERFACE_API bool sl_is_resolution_available(enum SL_RESOLUTION r, enum SL_MODEL m);

    /**
    \brief Check if a frame rate is available for a given resolution and camera model
    \param fps : Frame rate to check
    \param r : Resolution to check
    \param m : Camera model
    */
    INTERFACE_API bool sl_is_FPS_available(int fps, enum SL_RESOLUTION r, enum SL_MODEL m);

    /**
    \brief Check if a resolution for a given camera model is available for HDR
    \param r : Resolution to check
    \param m : Camera model
    */
    INTERFACE_API bool sl_is_HDR_available(enum SL_RESOLUTION r, enum SL_MODEL m);


#ifdef __cplusplus
}
#endif

#endif
