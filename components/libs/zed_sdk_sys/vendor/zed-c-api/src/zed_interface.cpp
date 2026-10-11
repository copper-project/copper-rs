#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <stddef.h>

#include <sl/Camera.hpp>

#include "ZEDController.hpp"
#include "ZEDFusionController.hpp"
#include "sl/c_api/zed_interface.h"

#define FUNC_MAT_ARGS(name, args) mat_##s(int* ptr, ##args);
#define FUNC_MAT(name) mat_##s(int* ptr);
#define MAT ((sl::Mat*)ptr)

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <wchar.h>
#include <string>
#include <vector>


#include <cstring>
#include <assert.h>
#include <fcntl.h>              /* low-level i/o */
#include <errno.h>
#include <string>
#include <iostream>
#include <math.h>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <ctime>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <mutex>
#include <atomic>
#include <queue>
#include <time.h>
#include <thread>

#ifdef _WIN32
#include <tchar.h>
#include <SetupAPI.h>
#pragma comment(lib,"Strmiids.lib")
#pragma comment(lib,"SetupAPI.lib")

//this is for TryEnterCriticalSection
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x400
#endif
#include <windows.h>
#endif

#include "sl/c_api/types_c.h"

#include <deque>
#include <numeric>      // std::accumulate


/*************************************
 *    Interface functions
 **************************************/
#ifdef __cplusplus
extern "C" {
#endif

    /*
    Utils functions
     */

    INTERFACE_API void sl_free(void* ptr)
    {
        free(ptr);
    }

    INTERFACE_API void sl_unload_all_instances() {
        for (int i = 0; i < MAX_CAMERA_PLUGIN; i++)
            ZEDController::destroyInstance(i);
    }

    INTERFACE_API void sl_unload_instance(int id) {
        ZEDController::destroyInstance(id);
    }

    /*
    Create a new Camera in live
     */
    INTERFACE_API bool sl_create_camera(int id) {
        if (ZEDController::isNotCreated(id)) {
            ZEDController::get(id)->createCamera(false);
            return true;
        }
        else
            return false; //already created
    }

    INTERFACE_API void sl_close_camera(int id) {
        ZEDController::get(id)->destroy();
    }

    INTERFACE_API int sl_open_camera(int id, SL_InitParameters* init_parameters, const unsigned int serial_number, const char* path_svo, const char* ip, int stream_port, int gmsl_port, const char* output_file, const char* opt_settings_path, const char* opencv_calib_path) {
        int err = (int)sl::ERROR_CODE::CAMERA_NOT_DETECTED;
        if (init_parameters->input_type == (SL_INPUT_TYPE)sl::INPUT_TYPE::SVO)
        {
            err = ZEDController::get(id)->initFromSVO(init_parameters, path_svo, output_file, opt_settings_path, opencv_calib_path);
        }
        else if (init_parameters->input_type == (SL_INPUT_TYPE)sl::INPUT_TYPE::STREAM)
        {
            err = ZEDController::get(id)->initFromStream(init_parameters, ip, stream_port, output_file, opt_settings_path, opencv_calib_path);
        }
        else if (init_parameters->input_type == (SL_INPUT_TYPE)sl::INPUT_TYPE::GMSL)
        {
            err = ZEDController::get(id)->initFromGMSL(init_parameters, serial_number, gmsl_port, output_file, opt_settings_path, opencv_calib_path);
        }
        else // USB
        {
            err = ZEDController::get(id)->initFromLive(init_parameters, serial_number, output_file, opt_settings_path, opencv_calib_path);
        }

        return err;
    }

    INTERFACE_API int sl_open_camera_from_camera_id(int camera_id, SL_InitParameters* init_parameters, const char* output_file, const char* opt_settings_path, const char* opencv_calib_path)
    {
        return ZEDController::get(camera_id)->initFromLive(init_parameters, 0, output_file, opt_settings_path, opencv_calib_path);
    }

    INTERFACE_API int sl_open_camera_from_serial_number(int camera_id, SL_InitParameters* init_parameters, const unsigned int serial_number, const char* output_file, const char* opt_settings_path, const char* opencv_calib_path)
    {
		return ZEDController::get(camera_id)->initFromLive(init_parameters, serial_number, output_file, opt_settings_path, opencv_calib_path);
    }

    INTERFACE_API int sl_open_camera_from_svo_file(int camera_id, SL_InitParameters* init_parameters, const char* path_svo, const char* output_file, const char* opt_settings_path, const char* opencv_calib_path)
    {
        return ZEDController::get(camera_id)->initFromSVO(init_parameters, path_svo, output_file, opt_settings_path, opencv_calib_path);
    }

    INTERFACE_API int sl_open_camera_from_stream(int camera_id, SL_InitParameters * init_parameters, const char* ip, int stream_port, const char* output_file, const char* opt_settings_path, const char* opencv_calib_path)
    {
		return ZEDController::get(camera_id)->initFromStream(init_parameters, ip, stream_port, output_file, opt_settings_path, opencv_calib_path);
    }

    INTERFACE_API bool sl_is_opened(int c_id)
    {

        return  ZEDController::get(c_id)->zed.isOpened();
    }

    INTERFACE_API enum SL_ERROR_CODE sl_start_publishing(int c_id, struct SL_CommunicationParameters* params)
    {
        if (!ZEDController::get(c_id)->isNull())
        {
            sl::CommunicationParameters comm_params;
            if (params->communication_type == SL_COMM_TYPE_INTRA_PROCESS)
            {
                comm_params.setForSharedMemory();
            }
            else // NETWORK
            {
                comm_params.setForLocalNetwork(std::string(params->ip_add), params->ip_port);
            }

            return (SL_ERROR_CODE)ZEDController::get(c_id)->zed.startPublishing(comm_params);
        }

        return SL_ERROR_CODE_FAILURE;
    }

    INTERFACE_API enum SL_ERROR_CODE sl_stop_publishing(int c_id)
    {
        if (!ZEDController::get(c_id)->isNull())
        {
            return (SL_ERROR_CODE)ZEDController::get(c_id)->zed.startPublishing();
        }

        return SL_ERROR_CODE_FAILURE;
    }

    INTERFACE_API int sl_set_region_of_interest(int c_id, void* ptr, bool module[SL_MODULE_LAST]) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->setRegionOfInterest(*MAT, module);
        }
        else
            return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API int sl_get_region_of_interest(int c_id, void* ptr, int width, int height, enum SL_MODULE module)
    {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->getRegionOfInterest(*MAT, sl::Resolution(width, height), module);
        }
        else
            return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API int sl_start_region_of_interest_auto_detection(int c_id, struct SL_RegionOfInterestParameters* roi_param)
    {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->startRegionOfInterestAutoDetection(roi_param);
        }
        else
            return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API enum SL_REGION_OF_INTEREST_AUTO_DETECTION_STATE sl_get_region_of_interest_auto_detection_status(int c_id)
    {
        if (!ZEDController::get(c_id)->isNull()) {
            return (enum SL_REGION_OF_INTEREST_AUTO_DETECTION_STATE)ZEDController::get(c_id)->zed.getRegionOfInterestAutoDetectionStatus();
        }
        else
            return SL_REGION_OF_INTEREST_AUTO_DETECTION_STATE_NOT_ENABLED;
    }

    INTERFACE_API CUcontext sl_get_cuda_context(int c_id)
    {
        return  ZEDController::get(c_id)->zed.getCUDAContext();
    }

    INTERFACE_API SL_InitParameters* sl_get_init_parameters(int c_id) {

        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->getInitParameters();
        }
        else
            return nullptr;
    }

    INTERFACE_API SL_RuntimeParameters* sl_get_runtime_parameters(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->getRuntimeParameters();
        }
        else
            return nullptr;
    }

    INTERFACE_API SL_PositionalTrackingParameters* sl_get_positional_tracking_parameters(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->getPositionalTrackingParameters();
        }
        else
            return nullptr;
    }

    INTERFACE_API int sl_get_number_zed_connected() {
        int nC = sl::Camera::getDeviceList().size();
        return nC;
    }

    INTERFACE_API char* sl_get_sdk_version() {
        std::string s = std::string(sl::Camera::getSDKVersion().c_str());
        return strdup(s.c_str());  // allocates and copies
    }

    INTERFACE_API int sl_convert_coordinate_system(struct SL_Quaternion* rotation, struct SL_Vector3* translation, enum SL_COORDINATE_SYSTEM coord_system_src, enum SL_COORDINATE_SYSTEM coord_system_dest) {

        sl::Translation sl_trans(translation->x, translation->y, translation->z);
        sl::Orientation sl_rotation(sl::float4(rotation->x, rotation->y, rotation->z, rotation->w));

        sl::Transform motionMat(sl_rotation, sl_trans);

        sl::ERROR_CODE err = sl::convertCoordinateSystem(motionMat, (sl::COORDINATE_SYSTEM)coord_system_src, (sl::COORDINATE_SYSTEM)coord_system_dest);
        sl_rotation = motionMat.getOrientation();
        sl_trans = motionMat.getTranslation();
        rotation->x = sl_rotation.x;
        rotation->y = sl_rotation.y;
        rotation->z = sl_rotation.z;
        rotation->w = sl_rotation.w;

        translation->x = sl_trans.x;
        translation->y = sl_trans.y;
        translation->z = sl_trans.z;

        return (int)err;
    }

    /*INTERFACE_API void sl_get_sdk_version(int *major, int *minor, int *patch) {

        sl::Camera::getSDKVersion(*major, *minor, *patch);
    }*/

    INTERFACE_API int sl_get_input_type(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->getInputType();
        }
        return -1;
    }

    INTERFACE_API int sl_get_zed_serial(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->getSLCameraInformation()->serial_number;
        }
        return -1;
    }

    INTERFACE_API int sl_get_camera_firmware(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->getSLCameraInformation()->camera_configuration.firmware_version;
        }

        return -1;
    }

    INTERFACE_API int sl_get_sensors_firmware(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->getSLCameraInformation()->sensors_configuration.firmware_version;
        }

        return -1;
    }

    INTERFACE_API int sl_get_camera_model(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->getCameraModel();
        }

        return (int)sl::MODEL::LAST;
    }

    INTERFACE_API int sl_grab(int c_id, SL_RuntimeParameters* runtime) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->grab(runtime);
        else
            return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API int sl_read(int c_id) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->zed.read();
        else
            return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API void sl_get_device_list(struct SL_DeviceProperties device_list[MAX_CAMERA_PLUGIN], int* nbDevices) {

        std::vector<sl::DeviceProperties> devices = sl::Camera::getDeviceList();
        *nbDevices = devices.size();
        for (int i = 0; i < devices.size(); i++) {
            if (i < MAX_CAMERA_PLUGIN) {
                SL_DeviceProperties device;
                memset(&device, 0, sizeof(SL_DeviceProperties));
                device.camera_model = (SL_MODEL)devices[i].camera_model;
                device.camera_state = (SL_CAMERA_STATE)devices[i].camera_state;
                device.id = devices[i].id;
                device.sn = devices[i].serial_number;
                device.input_type = (SL_INPUT_TYPE)devices[i].input_type;
                device.i2c_port = devices[i].i2c_port;
                device.sensor_address_left = devices[i].sensor_address_left;
                device.sensor_address_right = devices[i].sensor_address_right;
                device.gmsl_port = devices[i].gmsl_port;
                strncpy(device.path, devices[i].path.c_str(), std::min((int)devices[i].path.size(), 512));
                strncpy(device.video_device, devices[i].video_device, std::min((int)devices[i].video_device.size(), 512));
                strncpy(device.camera_badge, devices[i].camera_badge.c_str(), std::min((int)devices[i].camera_badge.size(), 128));
                strncpy(device.camera_sensor_model, devices[i].camera_sensor_model.c_str(), std::min((int)devices[i].camera_sensor_model.size(), 128));
                strncpy(device.camera_name, devices[i].camera_name.c_str(), std::min((int)devices[i].camera_name.size(), 128));

                memcpy(&device.identifier[0], &devices[i].identifier, sizeof(unsigned char) * 3);

                device_list[i] = device;
            }
        }
    }

    INTERFACE_API void sl_get_streaming_device_list(struct SL_StreamingProperties streaming_device_list[MAX_CAMERA_PLUGIN], int* nbDevices) {

        std::vector<sl::StreamingProperties> devices = sl::Camera::getStreamingDeviceList();
        *nbDevices = devices.size();
        for (int i = 0; i < devices.size(); i++) {
            if (i < MAX_CAMERA_PLUGIN) {
                SL_StreamingProperties device;

                device.codec = (SL_STREAMING_CODEC)devices[i].codec;
                device.current_bitrate = devices[i].current_bitrate;
                device.port = devices[i].port;
                device.serial_number = devices[i].serial_number;
                device.camera_model = (SL_MODEL)devices[i].camera_model;

                if (devices[i].ip.size() < 16) {
                    memcpy(&device.ip[0], devices[i].ip.c_str(), devices[i].ip.size() * sizeof(unsigned char));
                }
                streaming_device_list[i] = device;
            }
        }
    }

    INTERFACE_API int sl_reboot(int sn, bool full_reboot) {
        return (int)sl::Camera::reboot(sn, full_reboot);
    }


    //////// Recording //////////////////

    INTERFACE_API int sl_enable_recording(int c_id, const char* filename, enum SL_SVO_COMPRESSION_MODE compression_mode, unsigned int bitrate, int target_fps, bool transcode) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->enableRecording(filename, (sl::SVO_COMPRESSION_MODE)compression_mode, bitrate, target_fps, transcode);
        }
        return (int)sl::ERROR_CODE::CAMERA_NOT_DETECTED;
    }

    INTERFACE_API struct SL_RecordingStatus* sl_get_recording_status(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->getRecordingStatus();
        }
        else
            return nullptr;
    }

    INTERFACE_API void sl_disable_recording(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            ZEDController::get(c_id)->disableRecording();
        }
    }

    INTERFACE_API struct SL_RecordingParameters* sl_get_recording_parameters(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->getRecordingParameters();
        }
        else
            return nullptr;
    }

    INTERFACE_API void sl_pause_recording(int c_id, bool status) {
        if (!ZEDController::get(c_id)->isNull()) {
            ZEDController::get(c_id)->zed.pauseRecording(status);
        }
    }

    INTERFACE_API enum SL_ERROR_CODE sl_ingest_data_into_svo(int c_id, struct SL_SVOData* data)
    {
        if (!ZEDController::get(c_id)->isNull()) {
            return (SL_ERROR_CODE)ZEDController::get(c_id)->ingestDataIntoSVO(data);
        }
        else
            return SL_ERROR_CODE_CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API int sl_get_svo_data_size(int c_id, char key[128], unsigned long long ts_begin, unsigned long long ts_end)
    {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->getSVODataSize(key, ts_begin, ts_end);
        }
        else
            return -1;
    }

    INTERFACE_API enum SL_ERROR_CODE sl_retrieve_svo_data(int c_id, char* key, int nb_data, struct SL_SVOData** data, unsigned long long ts_begin, unsigned long long ts_end)
    {
        if (!ZEDController::get(c_id)->isNull()) {
            return (SL_ERROR_CODE)ZEDController::get(c_id)->retrieveSVOData(key, nb_data, data, ts_begin, ts_end);
        }
        else
            return SL_ERROR_CODE_CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API int sl_get_svo_data_keys_size(int c_id)
    {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->getSVODataKeysSize();
        }
        else
            return -1;
    }

    INTERFACE_API void sl_get_svo_data_keys(int c_id, int nb_keys, char** keys)
    {
        if (!ZEDController::get(c_id)->isNull()) {
            ZEDController::get(c_id)->getSVODataKeys(nb_keys, keys);
        }
    }

    //////// Camera //////////////////

    INTERFACE_API enum SL_ERROR_CODE sl_set_svo_position(int c_id, int position) {
        if (!ZEDController::get(c_id)->isNull()) {
            ZEDController::get(c_id)->lock();
            SL_ERROR_CODE err = (SL_ERROR_CODE)ZEDController::get(c_id)->zed.setSVOPosition(position);
            ZEDController::get(c_id)->unlock();
            return err;
        }
            return (SL_ERROR_CODE)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API int sl_get_svo_position(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->zed.getSVOPosition();
        }
        return -1;
    }

    INTERFACE_API int sl_get_svo_position_at_timestamp(int c_id, unsigned long long timestamp)
    {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->zed.getSVOPositionAtTimestamp(sl::Timestamp(timestamp));
        }
        return -1;
    }

    INTERFACE_API int sl_get_svo_number_of_frames(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->zed.getSVONumberOfFrames();
        }
        return -1;
    }

    INTERFACE_API void sl_pause_svo_reading(int c_id, bool status)
    {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->zed.pauseSVOReading(status);
        }
    }

    INTERFACE_API float sl_get_camera_fps(int c_id) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->zed.getCameraInformation().camera_configuration.fps;
        else
            return -1;
    }

    INTERFACE_API float sl_get_current_fps(int c_id) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->zed.getCurrentFPS();
        else
            return -1;
    }

    int INTERFACE_API sl_get_width(int c_id) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->getWidth();
        else
            return -1;

    }

    int INTERFACE_API sl_get_height(int c_id) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->getHeight();
        else
            return -1;
    }

    INTERFACE_API unsigned int sl_get_frame_dropped_count(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->zed.getFrameDroppedCount();
        }
        return 0;
    }

    INTERFACE_API void sl_update_self_calibration(int c_id) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->zed.updateSelfCalibration();
    }

    INTERFACE_API struct SL_CameraInformation* sl_get_camera_information(int c_id, int width, int height) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->getCameraInformation(width, height);
        else
            return nullptr;
    }

    INTERFACE_API struct SL_CalibrationParameters* sl_get_calibration_parameters(int c_id, bool raw) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->getCalibrationParameters(raw);
        else
            return nullptr;
    }

    INTERFACE_API struct SL_SensorsConfiguration* sl_get_sensors_configuration(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->getSensorsConfiguration();
        }
        else
            return nullptr;
    }

    INTERFACE_API void sl_get_camera_imu_transform(int c_id, struct SL_Vector3* translation, struct SL_Quaternion* rotation) {

        if (!ZEDController::get(c_id)->isNull()) {
            sl::Transform t = ZEDController::get(c_id)->getSLCameraInformation()->sensors_configuration.camera_imu_transform;
            sl::Translation trans = t.getTranslation();
            translation->x = trans.x;
            translation->y = trans.y;
            translation->z = trans.z;

            sl::Orientation orien = t.getOrientation();
            rotation->x = orien.x;
            rotation->y = orien.y;
            rotation->z = orien.z;
            rotation->w = orien.w;
        }

    }

    INTERFACE_API unsigned long long sl_get_image_timestamp(int c_id) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->zed.getTimestamp(sl::TIME_REFERENCE::IMAGE);
        else
            return 0ULL;
    }

    INTERFACE_API unsigned long long sl_get_current_timestamp(int c_id) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->zed.getTimestamp(sl::TIME_REFERENCE::CURRENT);
        else
            return 0ULL;
    }

    INTERFACE_API bool sl_is_camera_setting_supported(int c_id, enum SL_VIDEO_SETTINGS setting)
    {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->zed.isCameraSettingSupported((sl::VIDEO_SETTINGS)setting);
        else return false;
    }

    INTERFACE_API SL_ERROR_CODE sl_set_camera_settings(int c_id, enum SL_VIDEO_SETTINGS mode, int value) {
        if (!ZEDController::get(c_id)->isNull())
            return (SL_ERROR_CODE)ZEDController::get(c_id)->zed.setCameraSettings((sl::VIDEO_SETTINGS)mode, value);
        else return SL_ERROR_CODE_FAILURE;
    }

    INTERFACE_API SL_ERROR_CODE sl_set_camera_settings_min_max(int c_id, enum SL_VIDEO_SETTINGS mode, int min, int max) {
        if (!ZEDController::get(c_id)->isNull())
            return (SL_ERROR_CODE)ZEDController::get(c_id)->zed.setCameraSettings((sl::VIDEO_SETTINGS)mode, min, max);
        else return SL_ERROR_CODE_FAILURE;
    }

    INTERFACE_API SL_ERROR_CODE sl_set_roi_for_aec_agc(int c_id, enum SL_SIDE side, struct SL_Rect* roi, bool reset) {
        if (!ZEDController::get(c_id)->isNull()) {
            sl::Rect rect = sl::Rect(roi->x, roi->y, roi->width, roi->height);
            return (SL_ERROR_CODE)ZEDController::get(c_id)->zed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC_ROI, rect, (sl::SIDE)side, reset);
        }
        else
            return (SL_ERROR_CODE)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API SL_ERROR_CODE sl_get_camera_settings(int c_id, enum SL_VIDEO_SETTINGS mode, int* value) {
        if (!ZEDController::get(c_id)->isNull())
        {
            return (SL_ERROR_CODE)ZEDController::get(c_id)->zed.getCameraSettings((sl::VIDEO_SETTINGS)mode, *value);
        }
        else
            return (SL_ERROR_CODE)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API SL_ERROR_CODE sl_get_camera_settings_min_max(int c_id, enum SL_VIDEO_SETTINGS mode, int* minvalue, int* maxvalue) {
        if (!ZEDController::get(c_id)->isNull())
        {
            return (SL_ERROR_CODE)ZEDController::get(c_id)->zed.getCameraSettings((sl::VIDEO_SETTINGS)mode, *minvalue, *maxvalue);
        }
        else
            return (SL_ERROR_CODE)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API SL_ERROR_CODE sl_get_roi_for_aec_agc(int c_id, enum SL_SIDE side, struct SL_Rect* roi) {
        if (!ZEDController::get(c_id)->isNull()) {
            sl::Rect rect;
            SL_ERROR_CODE err = (SL_ERROR_CODE)ZEDController::get(c_id)->zed.getCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC_ROI, rect, (sl::SIDE)side);
            roi->x = rect.x;
            roi->y = rect.y;
            roi->width = rect.width;
            roi->height = rect.height;

            return err;
        }
        else
            return (SL_ERROR_CODE)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API struct SL_HealthStatus* sl_get_health_status(int camera_id)
    {
        if (!ZEDController::get(camera_id)->isNull()) {

            return ZEDController::get(camera_id)->getHealthStatus();
        }
        else
        {
            return nullptr;
        }
    }

    INTERFACE_API struct SL_Resolution* sl_get_retrieve_image_resolution(int camera_id, struct SL_Resolution* res)
    {
		if (!ZEDController::get(camera_id)->isNull())
			return ZEDController::get(camera_id)->getRetrieveImageResolution(res);
		else
			return nullptr;
    }

    INTERFACE_API struct SL_Resolution* sl_get_retrieve_measure_resolution(int camera_id, struct SL_Resolution* res)
    {
		if (!ZEDController::get(camera_id)->isNull())
			return ZEDController::get(camera_id)->getRetrieveMeasureResolution(res);
		else
			return nullptr;
	}


    ///////////////////////// DEPTH ////////////////////////////////

    INTERFACE_API float sl_get_depth_min_range_value(int c_id) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->zed.getInitParameters().depth_minimum_distance;
        else
            return -1;
    }

    INTERFACE_API float sl_get_depth_max_range_value(int c_id) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->zed.getInitParameters().depth_maximum_distance;
        else
            return -1;
    }

    INTERFACE_API int sl_get_current_min_max_depth(int c_id, float* min, float* max)
    {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->zed.getCurrentMinMaxDepth(*min, *max);
        else
            return 1; //SL_ERROR_CODE_FAILURE
    }

    INTERFACE_API int sl_get_confidence_threshold(int c_id) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->zed.getRuntimeParameters().confidence_threshold;
        else
            return -1;
    }


    ///////////////////// MOTION TRACKING //////////////////////////////

    /*INTERFACE_API int enable_positional_tracking(int c_id, SL_Quaternion *initial_world_rotation, SL_Vector3 *initial_world_position, bool enable_area_memory, bool enable_pose_smoothing, bool set_floor_as_origin, bool set_as_static,
                bool enable_imu_fusion, const char* area_file_path)
        {
                if (!ZEDController::get(c_id)->isNull())
                        return (int)ZEDController::get(c_id)->enableTracking(initial_world_rotation, initial_world_position, enable_area_memory, enable_pose_smoothing, set_floor_as_origin, set_as_static, enable_imu_fusion, area_file_path);
                else
                        return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }*/

    INTERFACE_API bool sl_is_positional_tracking_enabled(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->zed.isPositionalTrackingEnabled();
        }
        else
            return false;
    }

    INTERFACE_API int sl_enable_positional_tracking(int c_id, SL_PositionalTrackingParameters* tracking_param, const char* area_path) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->enableTracking(tracking_param, area_path);
        }
        else
            return (int)SL_ERROR_CODE_CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API int sl_get_area_export_state(int c_id) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->zed.getAreaExportState();
        else
            return (int)SL_AREA_EXPORTING_STATE::SL_AREA_EXPORTING_STATE_FILE_ERROR;
    }

    INTERFACE_API void sl_disable_positional_tracking(int c_id, const char* path) {
        if (!ZEDController::get(c_id)->isNull())
            ZEDController::get(c_id)->disableTracking(path);
    }

    INTERFACE_API int sl_save_area_map(int c_id, const char* path) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->zed.saveAreaMap(path);
        else
            return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API int sl_get_position(int c_id, SL_Quaternion* quat, SL_Vector3* vec, enum SL_REFERENCE_FRAME reference_frame) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->getPosition(quat, vec, (sl::REFERENCE_FRAME)reference_frame);
        else
            return (int)sl::POSITIONAL_TRACKING_STATE::OFF;
    }

    INTERFACE_API int sl_get_position_data(int c_id, SL_PoseData* poseData, enum SL_REFERENCE_FRAME reference_frame) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->getPosition(poseData, reference_frame);
        else
            return (int)sl::POSITIONAL_TRACKING_STATE::OFF;
    }

    INTERFACE_API int sl_get_position_array(int c_id, float* pose, enum SL_REFERENCE_FRAME mat_type) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->getPoseArray(pose, (int)mat_type);
        else
            return (int)sl::POSITIONAL_TRACKING_STATE::OFF;
    }

    INTERFACE_API int sl_get_positional_tracking_landmarks(int camera_id, struct SL_Landmark** landmarks, uint32_t* count)
    {
		if (!ZEDController::get(camera_id)->isNull())
			return (int)ZEDController::get(camera_id)->getPositionalTrackingLandmarks(landmarks, count);
		else
			return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API int sl_get_positional_tracking_landmarks_2d(int camera_id, struct SL_Landmark2D** landmarks, uint32_t* count)
    {
        if (!ZEDController::get(camera_id)->isNull())
            return (int)ZEDController::get(camera_id)->getPositionalTrackingLandmarks2d(landmarks, count);
        else
            return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API struct SL_PositionalTrackingStatus* sl_get_positional_tracking_status(int c_id)
    {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->getPositionalTrackingStatus();
        else
            return nullptr;
    }

    INTERFACE_API int sl_get_position_at_target_frame(int c_id, SL_Quaternion* quat, SL_Vector3* vec, SL_Quaternion* targetQuaternion, SL_Vector3* targetTranslation, enum SL_REFERENCE_FRAME reference_frame) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->getPosition(quat, vec, targetTranslation, targetQuaternion, reference_frame);
        else
            return (int)sl::POSITIONAL_TRACKING_STATE::OFF;
    }

    INTERFACE_API int sl_get_imu_orientation(int c_id, SL_Quaternion* quat, enum SL_TIME_REFERENCE time_reference) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->getIMUOrientation(quat, (int)time_reference);
        else
            return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API int sl_get_sensors_data(int c_id, SL_SensorsData* data, enum SL_TIME_REFERENCE time_reference) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->getSensorsData(data, (int)time_reference);
        else
            return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API int sl_get_sensors_data_batch_count(int camera_id, int* count)
	{
		if (!ZEDController::get(camera_id)->isNull())
			return (int)ZEDController::get(camera_id)->getSensorsDataBatchCount(count);
		else
			return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
	}

    INTERFACE_API int sl_get_sensors_data_batch(int camera_id, SL_SensorsData** data)
    {
		if (!ZEDController::get(camera_id)->isNull())
			return (int)ZEDController::get(camera_id)->getSensorsDataBatch(data);
		else
			return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API int sl_reset_positional_tracking(int c_id, SL_Quaternion rotation, SL_Vector3 translation) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->resetTracking(rotation, translation);
        else
            return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API int sl_reset_positional_tracking_with_offset(int c_id, SL_Quaternion rotation, SL_Vector3 translation, SL_Quaternion targetQuaternion, SL_Vector3 targetTranslation) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->resetTrackingWithOffset(rotation, translation, targetQuaternion, targetTranslation);
        else
            return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API int sl_set_imu_prior_orientation(int c_id, SL_Quaternion rotation) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->setIMUPriorOrientation(rotation);

        return -1;
    }

    /*********************************************** Spatial Mapping  functions ***********************************/

    INTERFACE_API int sl_enable_spatial_mapping(int c_id, struct SL_SpatialMappingParameters* mapping_param) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->enableSpatialMapping(*mapping_param);
        else
            return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API void sl_disable_spatial_mapping(int c_id) {
        if (!ZEDController::get(c_id)->isNull())
            ZEDController::get(c_id)->disableSpatialMapping();

    }

    INTERFACE_API SL_SpatialMappingParameters* sl_get_spatial_mapping_parameters(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->getSpatialMappingParameters();
        }
        else
            return nullptr;
    }

    INTERFACE_API void sl_pause_spatial_mapping(int c_id, bool status) {
        if (!ZEDController::get(c_id)->isNull())
            ZEDController::get(c_id)->zed.pauseSpatialMapping(status);

    }

    INTERFACE_API void sl_request_mesh_async(int c_id) {
        if (!ZEDController::get(c_id)->isNull())
            ZEDController::get(c_id)->requestMeshAsync();

    }

    INTERFACE_API int sl_get_mesh_request_status_async(int c_id) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->zed.getSpatialMapRequestStatusAsync();
        else
            return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API void sl_spatial_mapping_merge_chunks(int c_id, int numberFaces, int* numVertices, int* numTriangles, int* numUpdatedSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh) {
        ZEDController::get(c_id)->mergeChunks(numberFaces, numVertices, numTriangles, numUpdatedSubmeshes, updatedIndices, numVerticesTot, numTrianglesTot, maxSubmesh);
    }

    INTERFACE_API void sl_spatial_mapping_get_gravity_estimation(int c_id, SL_Vector3* gravity) {
        if (!ZEDController::get(c_id)->isNull()) {
            sl::float3 tmp = ZEDController::get(c_id)->getGravityEstimation();
            gravity->x = tmp.x;
            gravity->y = tmp.y;
            gravity->z = tmp.z;
        }
    }

    INTERFACE_API enum SL_SPATIAL_MAPPING_STATE sl_get_spatial_mapping_state(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (enum SL_SPATIAL_MAPPING_STATE)ZEDController::get(c_id)->zed.getSpatialMappingState();
        }
        return SL_SPATIAL_MAPPING_STATE_NOT_ENABLED;
    }

    INTERFACE_API int sl_update_mesh(int c_id, int* numVertices, int* numTriangles, int* numSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->updateMesh(numVertices, numTriangles, numSubmeshes, updatedIndices, numVerticesTot, numTrianglesTot, maxSubmesh);
        else
            return -1;
    }

    INTERFACE_API int sl_retrieve_mesh(int c_id, float* vertices, int* triangles, unsigned char* colors, float* uvs, unsigned char* texturePtr, const int maxSubmesh) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->retrieveMesh(vertices, triangles, colors, maxSubmesh, uvs, texturePtr);
        else
            return -1;
    }

    INTERFACE_API int sl_update_chunks(int c_id, int* numVertices, int* numTriangles, int* numSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->updateChunks(numVertices, numTriangles, numSubmeshes, updatedIndices, numVerticesTot, numTrianglesTot, maxSubmesh);
        else
            return -1;
    }

    INTERFACE_API int sl_retrieve_chunks(int c_id, float* vertices, int* triangles, unsigned char* colors, float* uvs, unsigned char* texturePtr, const int maxSubmesh) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->retrieveChunks(maxSubmesh, vertices, triangles, colors, uvs, texturePtr);
        else
            return -1;
    }

    INTERFACE_API int sl_update_fused_point_cloud(int c_id, int* numVerticesTot) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->updateFusedPointCloud(numVerticesTot);
        else
            return -1;
    }

    INTERFACE_API int sl_retrieve_fused_point_cloud(int c_id, float* vertices) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->retrieveFusedPointCloud(vertices);
        else
            return -1;
    }

    INTERFACE_API int sl_extract_whole_spatial_map(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->extractWholeSpatialMap();
        }
        else
            return -1;
    }

    INTERFACE_API bool sl_save_mesh(int c_id, const char* filename, enum SL_MESH_FILE_FORMAT format) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->saveMesh(filename, (sl::MESH_FILE_FORMAT)format);
        else
            return false;
    }

    INTERFACE_API bool sl_save_point_cloud(int c_id, const char* filename, enum SL_MESH_FILE_FORMAT format) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->savePointCloud(filename, (sl::MESH_FILE_FORMAT)format);
        else
            return false;
    }

    INTERFACE_API bool sl_load_mesh(int c_id, const char* filename, int* numVertices, int* numTriangles, int* numSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, int* texturesSize, const int maxSubmesh) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->loadMesh(filename, numVertices, numTriangles, numSubmeshes, updatedIndices, numVerticesTot, numTrianglesTot, maxSubmesh, texturesSize);
        else
            return false;
    }

    INTERFACE_API bool sl_apply_texture(int c_id, int* numVertices, int* numTriangles, int* numUpdatedSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, int* texturesSize, const int maxSubmesh) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->applyTexture(numVertices, numTriangles, numUpdatedSubmeshes, updatedIndices, numVerticesTot, numTrianglesTot, texturesSize, maxSubmesh);
        else
            return false;
    }

    INTERFACE_API bool sl_filter_mesh(int c_id, enum SL_MESH_FILTER filter_params, int* nb_vertices, int* nb_triangles, int* nb_updated_submeshes, int* updated_indices, int* nb_vertices_tot, int* nb_triangles_tot, const int max_submesh) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->filterMesh((sl::MeshFilterParameters::MESH_FILTER)filter_params, nb_vertices, nb_triangles, nb_updated_submeshes, updated_indices, nb_vertices_tot, nb_triangles_tot, max_submesh);
        else
            return false;
    }

    INTERFACE_API int sl_update_whole_mesh(int c_id, int* nb_vertices, int* nb_triangles) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->updateWholeMesh(nb_vertices, nb_triangles);
        else
            return false;
    }

    INTERFACE_API int sl_retrieve_whole_mesh(int c_id, float* vertices, int* triangles, unsigned char* colors, float* uvs, unsigned char* texture_ptr) {
        if (!ZEDController::get(c_id)->isNull())
            return (int)ZEDController::get(c_id)->retrieveWholeMesh(vertices, triangles, colors, uvs, texture_ptr);
        else
            return false;
    }

    INTERFACE_API bool sl_load_whole_mesh(int c_id, const char* filename, int* nb_vertices, int* nb_triangles, int* textures_size) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->loadWholeMesh(filename, nb_vertices, nb_triangles, textures_size);
        else
            return false;
    }

    INTERFACE_API bool sl_apply_whole_texture(int c_id, int* nb_vertices, int* nb_triangles, int* textures_size) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->applyWholeTexture(nb_vertices, nb_triangles, textures_size);
        else
            return false;
    }

    INTERFACE_API bool sl_filter_whole_mesh(int c_id, enum SL_MESH_FILTER filter_params, int* nb_vertices, int* nb_triangles) {
        if (!ZEDController::get(c_id)->isNull())
            return ZEDController::get(c_id)->filterWholeMesh((sl::MeshFilterParameters::MESH_FILTER)filter_params, nb_vertices, nb_triangles);
        else
            return false;
    }




    /*********************************************** Plane Detection functions ***********************************/

    INTERFACE_API SL_PlaneData* sl_find_floor_plane(int c_id, SL_Quaternion* resetQuaternion, SL_Vector3* resetTranslation, SL_Quaternion priorRotation, SL_Vector3 priorTranslation) {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->findFloorPlane(resetQuaternion, resetTranslation, priorRotation, priorTranslation);
        }
        return nullptr;
    }

    INTERFACE_API SL_PlaneData* sl_find_plane_at_hit(int c_id, SL_Vector2 pixel, struct SL_PlaneDetectionParameters* params, bool refine) {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->findPlaneAtHit(pixel, params, refine);
        }
        return nullptr;
    }

    INTERFACE_API int sl_convert_floorplane_to_mesh(int c_id, float* Vertices, int* Triangles, int* numVerticesTot, int* numTrianglesTot) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->convertCurrentFloorPlaneToChunk(Vertices, Triangles, numVerticesTot, numTrianglesTot);
        }
        else
            return (int)sl::ERROR_CODE::FAILURE;
    }

    INTERFACE_API int sl_convert_hitplane_to_mesh(int c_id, float* Vertices, int* Triangles, int* numVerticesTot, int* numTrianglesTot) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->convertCurrentHitPlaneToChunk(Vertices, Triangles, numVerticesTot, numTrianglesTot);
        }
        else
            return (int)sl::ERROR_CODE::FAILURE;
    }

    /************************************************ Streaming Sender ***********************************/
    INTERFACE_API int sl_enable_streaming(int cameraID, enum SL_STREAMING_CODEC codec, unsigned int bitrate, unsigned short port, int gopSize, int adaptativeBitrate, int chunk_size, int target_framerate) {
        if (!ZEDController::get(cameraID)->isNull()) {
            return (int)ZEDController::get(cameraID)->enableStreaming((sl::STREAMING_CODEC)codec, bitrate, port, gopSize, (bool)adaptativeBitrate, chunk_size, target_framerate);
        }
        else
            return (int)sl::ERROR_CODE::FAILURE;
    }

    INTERFACE_API int sl_is_streaming_enabled(int cameraID) {
        if (!ZEDController::get(cameraID)->isNull()) {
            return (int)ZEDController::get(cameraID)->isStreamingEnabled();
        }
        else
            return 0;
    }

    INTERFACE_API void sl_disable_streaming(int cameraID) {
        if (!ZEDController::get(cameraID)->isNull()) {
            ZEDController::get(cameraID)->disableStreaming();
        }
    }

    INTERFACE_API struct SL_StreamingParameters* sl_get_streaming_parameters(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->getStreamingParameters();
        }
        else
            return nullptr;
    }

    /*********************************************** Save to File utils ***********************************/
    INTERFACE_API int sl_save_current_image(int cameraID, enum SL_VIEW view, const char* fileName) {
        if (!ZEDController::get(cameraID)->isNull()) {
            return (int)ZEDController::get(cameraID)->saveCurrentImage((sl::VIEW)view, fileName);
        }
        return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API int sl_save_current_depth(int cameraID, enum SL_SIDE side, const char* fileName) {
        if (!ZEDController::get(cameraID)->isNull()) {
            return (int)ZEDController::get(cameraID)->saveCurrentDepth(side, fileName);
        }
        return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }

    INTERFACE_API int sl_save_current_point_cloud(int cameraID, enum SL_SIDE side, const char* fileName) {
        if (!ZEDController::get(cameraID)->isNull()) {
            return (int)ZEDController::get(cameraID)->saveCurrentPointCloud((int)side, fileName);
        }
        return (int)sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
    }


    /*********************************************** Object Detection ***********************************/
#if WITH_OBJECT_DETECTION


    INTERFACE_API struct SL_AI_Model_status* sl_check_AI_model_status(enum SL_AI_MODELS model, int gpu_id) {
        SL_AI_Model_status* status = new SL_AI_Model_status();
        memset(status, 0, sizeof(SL_AI_Model_status));
        sl::AI_Model_status zed_status = sl::checkAIModelStatus((sl::AI_MODELS)model, gpu_id);

        status->optimized = zed_status.optimized;
        status->downloaded = zed_status.downloaded;
        return status;
    }

    INTERFACE_API int sl_optimize_AI_model(enum SL_AI_MODELS model, int gpu_id) {
        return (int)sl::optimizeAIModel((sl::AI_MODELS)model, gpu_id);
    }

    INTERFACE_API int sl_enable_object_detection(int c_id, SL_ObjectDetectionParameters* params) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->enableObjectDetection(params);
        }
        else
            return (int)sl::ERROR_CODE::FAILURE;
    }

    INTERFACE_API SL_ObjectDetectionParameters* sl_get_object_detection_parameters(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->getObjectDetectionParameters();
        }
        else
            return nullptr;
    }

    INTERFACE_API void sl_disable_object_detection(int c_id, unsigned int instance_id, bool force_disable_all_instances) {
        if (!ZEDController::get(c_id)->isNull()) {
            ZEDController::get(c_id)->disableObjectDetection(instance_id, force_disable_all_instances);
        }
    }

    INTERFACE_API int sl_set_object_detection_runtime_parameters(int c_id, SL_ObjectDetectionRuntimeParameters object_detection_parameters, unsigned int instance_id)
    {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->setObjectDetectionRuntimeParameters(object_detection_parameters, instance_id);
        }
        else
            return (int)sl::ERROR_CODE::FAILURE;
    }

    INTERFACE_API int sl_enable_body_tracking(int c_id, SL_BodyTrackingParameters* params) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->enableBodyTracking(params);
        }
        else
            return (int)sl::ERROR_CODE::FAILURE;
    }

    INTERFACE_API SL_BodyTrackingParameters* sl_get_body_tracking_parameters(int c_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return ZEDController::get(c_id)->getBodyTrackingParameters();
        }
        else
            return nullptr;
    }

    INTERFACE_API void sl_disable_body_tracking(int c_id, unsigned int instance_id, bool force_disable_all_instances) {
        if (!ZEDController::get(c_id)->isNull()) {
            ZEDController::get(c_id)->disableBodyTracking(instance_id, force_disable_all_instances);
        }
    }

    INTERFACE_API int sl_set_body_tracking_runtime_parameters(int c_id, SL_BodyTrackingRuntimeParameters body_tracking_parameters, unsigned int instance_id)
    {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->setBodyTrackingRuntimeParameters(body_tracking_parameters, instance_id);
        }
        else
            return (int)sl::ERROR_CODE::FAILURE;
    }

    INTERFACE_API int sl_generate_unique_id(char* id) {
        sl::String sdk_id = sl::generate_unique_id();
        ///* error */ memcpy(id, sdk_id.c_str(), sdk_id.size() * sizeof(char));
        /* works */ memcpy(id, sdk_id.c_str(), (sdk_id.size()+1) * sizeof(char));
        return sdk_id.size();
    }

    INTERFACE_API int sl_generate_unique_id_str(char* id) {
        sl::String sdk_id_sl = sl::generate_unique_id();
        std::string sdk_id(sdk_id_sl.c_str());
        ///* error */ memcpy(id, sdk_id.c_str(), sdk_id.size() * sizeof(char));
        /* works */ memcpy(id, sdk_id.c_str(), (sdk_id.size()+1) * sizeof(char));
        return sdk_id.size();
    }

    INTERFACE_API int sl_ingest_custom_box_objects(int c_id, int nb_objects, struct SL_CustomBoxObjectData* objects_in, unsigned int instance_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->ingestCustomBoxObjectData(nb_objects, objects_in, instance_id);
        }
        else {
            return (int)sl::ERROR_CODE::FAILURE;
        }
    }

    INTERFACE_API int sl_ingest_custom_mask_objects(int c_id, int nb_objects, struct SL_CustomMaskObjectData* objects_in, unsigned int instance_id) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->ingestCustomMaskObjectData(nb_objects, objects_in, instance_id);
        }
        else {
            return (int)sl::ERROR_CODE::FAILURE;
        }
    }

    INTERFACE_API int sl_retrieve_objects(int camera_id, struct SL_ObjectDetectionRuntimeParameters* object_detection_runtime_parameters, struct SL_Objects* objects, unsigned int instance_id) {
        if (!ZEDController::get(camera_id)->isNull()) {
            if (object_detection_runtime_parameters == nullptr)
				return (int)ZEDController::get(camera_id)->retrieveObjectDetectionData(objects, instance_id);
			else
                return (int)ZEDController::get(camera_id)->retrieveObjectDetectionData(object_detection_runtime_parameters, objects, instance_id);
        }
        else
            return (int)sl::ERROR_CODE::FAILURE;
    }

    INTERFACE_API int sl_retrieve_custom_objects(int camera_id, struct SL_CustomObjectDetectionRuntimeParameters* object_detection_runtime_parameters, struct SL_Objects* objects, unsigned int instance_id) {
        if (!ZEDController::get(camera_id)->isNull()) {
            return (int)ZEDController::get(camera_id)->retrieveCustomObjectDetectionData(object_detection_runtime_parameters, objects, instance_id);
        }
        else
            return (int)sl::ERROR_CODE::FAILURE;
    }

    INTERFACE_API int sl_set_custom_object_detection_runtime_parameters(int c_id, SL_CustomObjectDetectionRuntimeParameters custom_object_detection_parameters, unsigned int instance_id)
    {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->setCustomObjectDetectionRuntimeParameters(custom_object_detection_parameters, instance_id);
        }
        else
            return (int)sl::ERROR_CODE::FAILURE;
    }

    INTERFACE_API int sl_retrieve_bodies(int c_id, SL_BodyTrackingRuntimeParameters* runtimeParams, SL_Bodies* bodies, unsigned int instance_id) {
        if (!ZEDController::get(c_id)->isNull()) {
			if (runtimeParams == nullptr)
				return (int)ZEDController::get(c_id)->retrieveBodyTrackingData(bodies, instance_id);
			else
            return (int)ZEDController::get(c_id)->retrieveBodyTrackingData(runtimeParams, bodies, instance_id);
        }
        else
            return (int)sl::ERROR_CODE::FAILURE;
    }


    INTERFACE_API int sl_update_objects_batch(int c_id, int* nb_batches) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->updateObjectsBatch(nb_batches);
        }
        else
            return (int)sl::ERROR_CODE::FAILURE;
    }

    INTERFACE_API int sl_get_objects_batch_csharp(int c_id, int index, int* nb_data, int* id, int* label, int* sublabel, int* tracking_state,
        struct SL_Vector3 positions[MAX_TRAJECTORY_SIZE], float position_covariances[MAX_TRAJECTORY_SIZE][6], struct SL_Vector3 velocities[MAX_TRAJECTORY_SIZE], unsigned long long timestamps[MAX_TRAJECTORY_SIZE],
        struct SL_Vector2 bounding_boxes_2d[MAX_TRAJECTORY_SIZE][4], struct SL_Vector3 bounding_boxes[MAX_TRAJECTORY_SIZE][8], float confidences[MAX_TRAJECTORY_SIZE], int action_states[MAX_TRAJECTORY_SIZE],
        struct SL_Vector2 head_bounding_boxes_2d[MAX_TRAJECTORY_SIZE][4], struct SL_Vector3 head_bounding_boxes[MAX_TRAJECTORY_SIZE][8],
        struct SL_Vector3 head_positions[MAX_TRAJECTORY_SIZE]) {

        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->getObjectsBatchDataCSharp(index, nb_data, id, label, sublabel, tracking_state,
                positions, position_covariances, velocities, timestamps,
                bounding_boxes_2d, bounding_boxes, confidences, action_states,head_bounding_boxes_2d, head_bounding_boxes, head_positions);
        }
        else
            return (int)sl::ERROR_CODE::FAILURE;
    }

    INTERFACE_API int sl_get_objects_batch(int c_id, int index, struct SL_ObjectsBatch* objs_batch) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->getObjectsBatchData(index, objs_batch);
        }
        else
            return (int)sl::ERROR_CODE::FAILURE;
    }

#if 0

    INTERFACE_API int sl_update_bodies_batch(int c_id, int* nb_batches) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->updateBodiesBatch(nb_batches);
        }
        else
            return (int)sl::ERROR_CODE::FAILURE;
    }

    INTERFACE_API int sl_get_bodies_batch(int c_id, int index, struct SL_BodiesBatch* bodies_batch) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->getBodiesBatchData(index, bodies_batch);
        }
        else
            return (int)sl::ERROR_CODE::FAILURE;
    }
#endif

#endif

	/*************************** MULTI CAM*************************/

    INTERFACE_API SL_FUSION_ERROR_CODE sl_fusion_init(struct SL_InitFusionParameters* params)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->init(params);
        }
        else
        {
            return SL_FUSION_ERROR_CODE_FAILURE;
        }
    }


    INTERFACE_API SL_FUSION_ERROR_CODE sl_fusion_process()
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->process();
        }
        else
        {
            return SL_FUSION_ERROR_CODE_FAILURE;
        }
    }

    INTERFACE_API SL_FUSION_ERROR_CODE sl_fusion_subscribe(struct SL_CameraIdentifier* uuid, struct SL_CommunicationParameters* params, struct SL_Vector3* pose_translation, struct SL_Quaternion* pose_rotation)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->subscribe(uuid, params, pose_translation, pose_rotation);
        }
        else
        {
            return SL_FUSION_ERROR_CODE_FAILURE;
        }
    }

    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_unsubscribe(struct SL_CameraIdentifier* uuid)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->unsubscribe(uuid);
        }
        else
        {
            return SL_FUSION_ERROR_CODE_FAILURE;
        }
    }

    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_retrieve_image(void* ptr, struct SL_CameraIdentifier* uuid, int width, int height)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            sl::CameraIdentifier sl_uuid;
            sl_uuid.sn = uuid->sn;
            return (SL_FUSION_ERROR_CODE)ZEDFusionController::get()->fusion.retrieveImage(*MAT, sl_uuid, sl::Resolution(width, height));
        }
        else
        {
            return SL_FUSION_ERROR_CODE_FAILURE;
        }
    }

    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_retrieve_measure(void* ptr, struct SL_CameraIdentifier* uuid, enum SL_MEASURE measure, int width, int height, enum SL_FUSION_REFERENCE_FRAME reference_frame)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            sl::CameraIdentifier sl_uuid;
            sl_uuid.sn = uuid->sn;
            return (SL_FUSION_ERROR_CODE)ZEDFusionController::get()->fusion.retrieveMeasure(*MAT, sl_uuid, (sl::MEASURE)measure, sl::Resolution(width, height), (sl::FUSION_REFERENCE_FRAME)reference_frame);
        }
        else
        {
            return SL_FUSION_ERROR_CODE_FAILURE;
        }
    }

    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_update_pose(struct SL_CameraIdentifier* uuid, struct SL_Vector3* pose_translation, struct SL_Quaternion* pose_rotation)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->updatePose(uuid, pose_translation, pose_rotation);
        }
        else
        {
            return SL_FUSION_ERROR_CODE_FAILURE;
        }
    }


    INTERFACE_API SL_FUSION_ERROR_CODE sl_fusion_get_pose(SL_CameraIdentifier* uuid, SL_Vector3* pose_translation, SL_Quaternion* pose_rotation)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->getPose(uuid, pose_translation, pose_rotation);
        }
        else
        {
            return SL_FUSION_ERROR_CODE_FAILURE;
        }
    }

    INTERFACE_API enum SL_SENDER_ERROR_CODE sl_fusion_get_sender_state(struct SL_CameraIdentifier* uuid)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            sl::CameraIdentifier sdk_uuid;
            sdk_uuid.sn = uuid->sn;

            return ZEDFusionController::get()->getSenderState(uuid);
        }
        else
        {
            return SL_SENDER_ERROR_CODE_DISCONNECTED;
        }
    }

    INTERFACE_API void sl_fusion_read_configuration_file(const char* json_config_filename, enum SL_COORDINATE_SYSTEM coord_system, enum SL_UNIT unit, struct SL_FusionConfiguration configs[MAX_FUSED_CAMERAS], int* nb_cameras)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            ZEDFusionController::get()->readFusionConfigFile(json_config_filename, coord_system, unit, configs, *nb_cameras);
        }
    }
    INTERFACE_API void sl_fusion_read_configuration(const char* fusion_configuration, enum SL_COORDINATE_SYSTEM coord_system, enum SL_UNIT unit, struct SL_FusionConfiguration configs[MAX_FUSED_CAMERAS], int* nb_cameras)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            ZEDFusionController::get()->readFusionConfig(fusion_configuration, coord_system, unit, configs, *nb_cameras);
        }
    }


	INTERFACE_API SL_FUSION_ERROR_CODE sl_fusion_enable_body_tracking(SL_BodyTrackingFusionParameters* params)
	{
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->enableBodyTracking(params);
        }
        else
        {
            return SL_FUSION_ERROR_CODE_FAILURE;
        }
	}

	INTERFACE_API void sl_fusion_disable_body_tracking() {
		ZEDFusionController::get()->disableBodyTracking();
	}

    INTERFACE_API SL_FUSION_ERROR_CODE sl_fusion_retrieve_bodies(struct SL_Bodies* bodies, struct SL_BodyTrackingFusionRuntimeParameters* rt, struct SL_CameraIdentifier uuid, enum SL_FUSION_REFERENCE_FRAME reference_frame)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->retrieveBodies(bodies, rt, uuid, reference_frame);
        }
        else
        {
            return SL_FUSION_ERROR_CODE_FAILURE;
        }
    }

    INTERFACE_API SL_FUSION_ERROR_CODE sl_fusion_get_process_metrics(struct SL_FusionMetrics* metrics)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->getProcessMetrics(metrics);
        }
        else
        {
            return SL_FUSION_ERROR_CODE_FAILURE;
        }
    }

    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_enable_positional_tracking(struct SL_PositionalTrackingFusionParameters* params)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->enablePositionalTracking(params);
        }
        else
        {
            return SL_FUSION_ERROR_CODE_FAILURE;
        }
    }

    INTERFACE_API enum SL_POSITIONAL_TRACKING_STATE sl_fusion_get_position(SL_PoseData* pose, enum SL_REFERENCE_FRAME reference_frame,
        struct SL_CameraIdentifier* uuid, enum SL_POSITION_TYPE retrieve_type)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->getPosition(pose, reference_frame, uuid, retrieve_type);
        }
        else
        {
            return SL_POSITIONAL_TRACKING_STATE_OFF;
        }
    }

    INTERFACE_API struct SL_FusedPositionalTrackingStatus* sl_fusion_get_fused_positional_tracking_status()
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->getFusedPositionalTrackingStatus();
        }
        else
        {
            return nullptr;
        }
    }


    INTERFACE_API void sl_fusion_disable_positional_tracking()
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->disablePositionalTracking();
        }
    }

    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_geo_to_enu(SL_LatLng* in, SL_ENU* out)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->geoToEnu(in, out);
        }
        else
        {
            return SL_FUSION_ERROR_CODE_FAILURE;
        }
    }

    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_enu_to_geo(SL_ENU* in, SL_LatLng* out)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->enuToGeo(in, out);
        }
        else
        {
            return SL_FUSION_ERROR_CODE_FAILURE;
        }
    }

    INTERFACE_API enum SL_FUSION_ERROR_CODE sl_fusion_ingest_gnss_data(struct SL_GNSSData* gnss_data, bool radian)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->ingestGNSSData(gnss_data, radian);
        }
        else
            return SL_FUSION_ERROR_CODE_FAILURE;
    }

    INTERFACE_API enum SL_POSITIONAL_TRACKING_STATE sl_fusion_get_current_gnss_data(struct SL_GNSSData* data, bool radian)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->getCurrentGNSSData(data, radian);
        }
        else
        {
            return SL_POSITIONAL_TRACKING_STATE_OFF;
        }
    }

    INTERFACE_API enum SL_GNSS_FUSION_STATUS sl_fusion_get_geo_pose(SL_GeoPose* pose, bool radian)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->getGeoPose(pose, radian);
        }
        else
        {
            return SL_GNSS_FUSION_STATUS_OFF;
        }
    }

    INTERFACE_API enum SL_GNSS_FUSION_STATUS sl_fusion_geo_to_camera(struct SL_LatLng* in, struct SL_PoseData* out, bool radian)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->geoToCamera(in, out, radian);
        }
        else
        {
            return SL_GNSS_FUSION_STATUS_OFF;
        }
    }

    INTERFACE_API enum SL_GNSS_FUSION_STATUS sl_fusion_camera_to_geo(struct SL_PoseData* in, struct SL_GeoPose* out, bool radian)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->cameraToGeo(in, out, radian);
        }
        else
        {
            return SL_GNSS_FUSION_STATUS_OFF;
        }
    }

    INTERFACE_API unsigned long long sl_fusion_get_current_timestamp()
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->fusion.getCurrentTimeStamp().getNanoseconds();
        }
        else
        {
            return 0;
        }
    }

    INTERFACE_API enum SL_GNSS_FUSION_STATUS sl_fusion_get_current_gnss_calibration_std(float* yaw_std, struct SL_Vector3* position_std)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->getCurrentGNSSCalibrationSTD(yaw_std, position_std);
        }
        else
        {
            return SL_GNSS_FUSION_STATUS_OFF;
        }
    }


    INTERFACE_API void sl_fusion_get_geo_tracking_calibration(struct SL_Vector3* translation, struct SL_Quaternion* rotation)
    {
        if (!ZEDFusionController::get()->isNotCreated())
        {
            return ZEDFusionController::get()->getGeoTrackingCalibration(translation, rotation);
        }
    }

	INTERFACE_API void sl_fusion_close() {
		ZEDFusionController::get()->close();
	}

    /***************************MAT*************************/
    INTERFACE_API int sl_retrieve_measure(int c_id, void* ptr, enum SL_MEASURE type, enum SL_MEM mem, int width, int height, void* custream) {
        if (!ZEDController::get(c_id)->isNull()) {
            return (int)ZEDController::get(c_id)->zed.retrieveMeasure(*MAT, (sl::MEASURE)type, (sl::MEM)(mem + 1), sl::Resolution(width, height), (cudaStream_t)custream);
        }
        return (int)sl::ERROR_CODE::CAMERA_NOT_DETECTED;
    }

    INTERFACE_API int sl_retrieve_image(int c_id, void* ptr, enum SL_VIEW type, enum SL_MEM mem, int width, int height, void* custream) {
        if (!ZEDController::get(c_id)->isNull()) {
            if (type == SL_VIEW_LEFT_NV12_UNRECTIFIED || type == SL_VIEW_RIGHT_NV12_UNRECTIFIED) {
                return (int)sl::ERROR_CODE::INVALID_FUNCTION_PARAMETERS;
            }
            else {
                return (int)ZEDController::get(c_id)->zed.retrieveImage(*MAT, (sl::VIEW)type, (sl::MEM)(mem + 1), sl::Resolution(width, height), (cudaStream_t)custream);
            }
        }
        return (int)sl::ERROR_CODE::CAMERA_NOT_DETECTED;
    }

    INTERFACE_API int sl_convert_image(void* image_in_ptr, void* image_signed_ptr, void* stream) {
        return (int)sl::convertImage(*(sl::Mat*)image_in_ptr, *(sl::Mat*)image_signed_ptr, (cudaStream_t)stream);
    }

    INTERFACE_API void* sl_mat_create_new(int width, int height, enum SL_MAT_TYPE type, enum SL_MEM mem) {
        return (void*)(new sl::Mat(sl::Resolution(width, height), (sl::MAT_TYPE)type, (sl::MEM)(mem + 1)));
    }

    INTERFACE_API void* sl_mat_create_new_empty() {
        return (void*)(new sl::Mat());
    }

    INTERFACE_API bool sl_mat_is_init(void* ptr) {
        return MAT->isInit();
    }

    INTERFACE_API void sl_mat_free(void* ptr, enum SL_MEM mem) {
        if (ptr != nullptr) {
          MAT->free((sl::MEM)(mem + 1));
          delete MAT;
        }
    }

    INTERFACE_API void sl_mat_get_infos(void* ptr, char* buffer) {
        strcpy(buffer, MAT->getInfos().c_str());
    }
    // GET

    INTERFACE_API int sl_mat_get_value_uchar(void* ptr, int col, int raw, unsigned char* value, enum SL_MEM mem) {
        return (int)(MAT->getValue<unsigned char>(col, raw, value, (sl::MEM)(mem + 1)));
    }

    INTERFACE_API int sl_mat_get_value_uchar2(void* ptr, int col, int raw, SL_Uchar2* value, enum SL_MEM mem) {
        sl::uchar2 u;
        int err = (int)(MAT->getValue<sl::uchar2>(col, raw, &u, (sl::MEM)(mem + 1)));
        value->x = u.x;
        value->y = u.y;

        return err;
    }

    INTERFACE_API int sl_mat_get_value_uchar3(void* ptr, int x, int y, SL_Uchar3* value, enum SL_MEM mem) {
        sl::uchar3 u;
        int err = (int)(MAT->getValue<sl::uchar3>(x, y, &u, (sl::MEM)(mem + 1)));
        value->x = u.x;
        value->y = u.y;
        value->z = u.z;

        return err;
    }

    INTERFACE_API int sl_mat_get_value_uchar4(void* ptr, int x, int y, SL_Uchar4* value, enum SL_MEM mem) {
        sl::uchar4 u;
        int err = (int)(MAT->getValue<sl::uchar4>(x, y, &u, (sl::MEM)(mem + 1)));

        value->x = u.x;
        value->y = u.y;
        value->z = u.z;
        value->w = u.w;

        return err;
    }

    INTERFACE_API int sl_mat_get_value_float(void* ptr, int x, int y, float* value, enum SL_MEM mem) {
        return (int)(MAT->getValue<float>(x, y, value, (sl::MEM)(mem + 1)));
    }

    INTERFACE_API int sl_mat_get_value_float2(void* ptr, int x, int y, SL_Vector2* value, enum SL_MEM mem) {
        sl::float2 f;
        int err = (int)(MAT->getValue<sl::float2>(x, y, &f, (sl::MEM)(mem + 1)));
        value->x = f.x;
        value->y = f.y;

        return err;
    }

    INTERFACE_API int sl_mat_get_value_float3(void* ptr, int x, int y, SL_Vector3* value, enum SL_MEM mem) {
        sl::float3 f;
        int err = (int)(MAT->getValue<sl::float3>(x, y, &f, (sl::MEM)(mem + 1)));
        value->x = f.x;
        value->y = f.y;
        value->z = f.z;

        return err;
    }

    INTERFACE_API int sl_mat_get_value_float4(void* ptr, int x, int y, SL_Vector4* value, enum SL_MEM mem) {
        sl::float4 f;
        int err = (int)(MAT->getValue<sl::float4>(x, y, &f, (sl::MEM)(mem + 1)));
        value->x = f.x;
        value->y = f.y;
        value->z = f.z;
        value->w = f.w;

        return err;
    }

    // SET

    INTERFACE_API int sl_mat_set_value_uchar(void* ptr, int x, int y, unsigned char value, enum SL_MEM mem) {
        return (int)(MAT->setValue<unsigned char>(x, y, value, (sl::MEM)(mem + 1)));
    }

    INTERFACE_API int sl_mat_set_value_uchar2(void* ptr, int x, int y, SL_Uchar2 value, enum SL_MEM mem) {
        sl::uchar2 u = sl::uchar2(value.x, value.y);
        return (int)(MAT->setValue<sl::uchar2>(x, y, u, (sl::MEM)(mem + 1)));
    }

    INTERFACE_API int sl_mat_set_value_uchar3(void* ptr, int x, int y, SL_Uchar3 value, enum SL_MEM mem) {
        sl::uchar3 u = sl::uchar3(value.x, value.y, value.z);
        return (int)(MAT->setValue<sl::uchar3>(x, y, u, (sl::MEM)(mem + 1)));
    }

    INTERFACE_API int sl_mat_set_value_uchar4(void* ptr, int x, int y, SL_Uchar4 value, enum SL_MEM mem) {
        sl::uchar4 f = sl::uchar4(value.x, value.y, value.z, value.w);
        return (int)(MAT->setValue<sl::uchar4>(x, y, f, (sl::MEM)(mem + 1)));
    }

    INTERFACE_API int sl_mat_set_value_float(void* ptr, int x, int y, float value, enum SL_MEM mem) {
        return (int)(MAT->setValue<float>(x, y, value, (sl::MEM)(mem + 1)));
    }

    INTERFACE_API int sl_mat_set_value_float2(void* ptr, int x, int y, SL_Vector2 value, enum SL_MEM mem) {
        sl::float2 f = sl::float2(value.x, value.y);
        return (int)(MAT->setValue<sl::float2>(x, y, f, (sl::MEM)(mem + 1)));
    }

    INTERFACE_API int sl_mat_set_value_float3(void* ptr, int x, int y, SL_Vector3 value, enum SL_MEM mem) {
        sl::float3 f = sl::float3(value.x, value.y, value.z);
        return (int)(MAT->setValue<sl::float3>(x, y, f, (sl::MEM)(mem + 1)));
    }

    INTERFACE_API int sl_mat_set_value_float4(void* ptr, int x, int y, SL_Vector4 value, enum SL_MEM mem) {
        sl::float4 f = sl::float4(value.x, value.y, value.z, value.w);
        return (int)(MAT->setValue<sl::float4>(x, y, f, (sl::MEM)(mem + 1)));
    }
    // SET TO

    INTERFACE_API int sl_mat_set_to_uchar(void* ptr, unsigned char value, enum SL_MEM mem) {
        return (int)(MAT->setTo<unsigned char>(value, (sl::MEM)(mem + 1)));
    }

    INTERFACE_API int sl_mat_set_to_uchar2(void* ptr, SL_Uchar2 value, enum SL_MEM mem) {
        sl::uchar2 f = sl::uchar2(value.x, value.y);
        return (int)(MAT->setTo<sl::uchar2>(f, (sl::MEM)(mem + 1)));

    }

    INTERFACE_API int sl_mat_set_to_uchar3(void* ptr, SL_Uchar3 value, enum SL_MEM mem) {
        sl::uchar3 f = sl::uchar3(value.x, value.y, value.z);
        return (int)(MAT->setTo<sl::uchar3>(f, (sl::MEM)(mem + 1)));

    }

    INTERFACE_API int sl_mat_set_to_uchar4(void* ptr, SL_Uchar4 value, enum SL_MEM mem) {
        sl::uchar4 f = sl::uchar4(value.x, value.y, value.z, value.w);
        return (int)(MAT->setTo<sl::uchar4>(f, (sl::MEM)(mem + 1)));
    }

    INTERFACE_API int sl_mat_set_to_float(void* ptr, float value, enum SL_MEM mem) {
        return (int)(MAT->setTo<float>(value, (sl::MEM)(mem + 1)));

    }

    INTERFACE_API int sl_mat_set_to_float2(void* ptr, SL_Vector2 value, enum SL_MEM mem) {
        sl::float2 f = sl::float2(value.x, value.y);
        return (int)(MAT->setTo<sl::float2>(f, (sl::MEM)(mem + 1)));

    }

    INTERFACE_API int sl_mat_set_to_float3(void* ptr, SL_Vector3 value, enum SL_MEM mem) {
        sl::float3 f = sl::float3(value.x, value.y, value.z);
        return (int)(MAT->setTo<sl::float3>(f, (sl::MEM)(mem + 1)));

    }

    INTERFACE_API int sl_mat_set_to_float4(void* ptr, SL_Vector4 value, enum SL_MEM mem) {
        sl::float4 f = sl::float4(value.x, value.y, value.z, value.w);
        return (int)(MAT->setTo<sl::float4>(f, (sl::MEM)(mem + 1)));
    }

    INTERFACE_API int sl_mat_update_cpu_from_gpu(void* ptr) {
        return (int)MAT->updateCPUfromGPU();
    }

    INTERFACE_API int sl_mat_update_gpu_from_cpu(void* ptr) {
        return (int)MAT->updateGPUfromCPU();
    }

    INTERFACE_API int sl_mat_copy_to(void* ptr, void* ptrDest, enum SL_COPY_TYPE cpyType) {
        return (int)MAT->copyTo(*(sl::Mat*)ptrDest, (sl::COPY_TYPE)cpyType);
    }

    INTERFACE_API int sl_mat_read(void* ptr, const char* filePath) {
        return (int)MAT->read(filePath);
    }

    INTERFACE_API int sl_mat_write(void* ptr, const char* filePath) {
        return (int)(MAT->write(filePath));
    }

    INTERFACE_API int sl_mat_get_width(void* ptr) {
        return MAT->getWidth();
    }

    INTERFACE_API int sl_mat_get_height(void* ptr) {
        return MAT->getHeight();
    }

    INTERFACE_API int sl_mat_get_channels(void* ptr) {
        return MAT->getChannels();
    }

    INTERFACE_API int sl_mat_get_memory_type(void* ptr) {
        return (int)(MAT->getMemoryType()) - 1;
    }

    INTERFACE_API int sl_mat_get_data_type(void* ptr) {
        return (int)(MAT->getDataType());
    }

    INTERFACE_API int sl_mat_get_pixel_bytes(void* ptr) {
        return MAT->getPixelBytes();
    }

    INTERFACE_API int sl_mat_get_step(void* ptr, enum SL_MEM mem) {
        return MAT->getStep((sl::MEM)(mem + 1));
    }

    INTERFACE_API int sl_mat_get_step_bytes(void* ptr, enum SL_MEM mem) {
        return MAT->getStepBytes((sl::MEM)(mem + 1));
    }

    INTERFACE_API int sl_mat_get_width_bytes(void* ptr) {
        return MAT->getWidthBytes();
    }

    INTERFACE_API bool sl_mat_is_memory_owner(void* ptr) {
        return MAT->isMemoryOwner();
    }

    INTERFACE_API SL_Resolution sl_mat_get_resolution(void* ptr) {
        sl::Resolution sl_res = MAT->getResolution();
        SL_Resolution c_res;
        c_res.height = sl_res.height;
        c_res.width = sl_res.width;
        return c_res;
    }

    INTERFACE_API void sl_mat_alloc(void* ptr, int width, int height, enum SL_MAT_TYPE type, enum SL_MEM mem) {
        MAT->alloc(width, height, (sl::MAT_TYPE)type, (sl::MEM)(mem + 1));
    }

    INTERFACE_API int sl_mat_set_from(void* ptr, void* ptrSource, enum SL_COPY_TYPE copyType) {
        return (int)MAT->setFrom(*(sl::Mat*)ptrSource, (sl::COPY_TYPE)copyType);
    }

    INTERFACE_API int* sl_mat_get_ptr(void* ptr, enum SL_MEM mem) {
        return (int*)MAT->getPtr<sl::uchar1>((sl::MEM)(mem + 1));
    }

    INTERFACE_API int sl_mat_clone(void* ptr, void* ptrSource) {
        return (int)MAT->clone(*(sl::Mat*)ptrSource);
    }

    INTERFACE_API void sl_mat_swap(void* ptr_1, void* ptr_2) {
        sl::Mat::swap(*(sl::Mat*)ptr_1, *(sl::Mat*)ptr_2);
    }

    /**
    \brief Convert the color channels of the Mat into another Mat
     * This methods works only on 8U_C4 if remove_alpha_channels is enabled, or 8U_C4 and 8U_C3 if swap_RB_channels is enabled
     * The inplace method sl::Mat::convertColor can be used for only swapping the Red and Blue channel efficiently
     */
    INTERFACE_API int sl_mat_convert_color(void* ptr, enum SL_MEM memory, bool swap_RB_channels, void* stream)
    {
		return (int)MAT->convertColor((sl::MEM)(memory + 1), (cudaStream_t)stream, swap_RB_channels);
    }

    /**
    \brief Convert the color channels of the Mat (RGB<->BGR or RGBA<->BGRA)
     * This methods works only on 8U_C4 or 8U_C3
     */
    INTERFACE_API int sl_convert_color(void* mat1, void* mat2, bool swap_RB_channels, bool remove_alpha_channels, enum SL_MEM memory, void* stream)
    {
		return (int)sl::Mat::convertColor(*(sl::Mat*)mat1, *(sl::Mat*)mat2, swap_RB_channels, remove_alpha_channels, (sl::MEM)(memory + 1), (cudaStream_t)stream);
    }

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
	INTERFACE_API int sl_blob_from_image(void* ptr, void* tensor_out, struct SL_Resolution resolution_out,
        float scalefactor, struct SL_Vector3 mean, struct SL_Vector3 stddev, bool keep_aspect_ratio, bool swap_RB_channels,
        void* stream)
	{
		return (int)sl::blobFromImage(*MAT, *(sl::Mat*)tensor_out, sl::Resolution(resolution_out.width, resolution_out.height),
            scalefactor, sl::float3(mean.x, mean.y, mean.z),
            sl::float3(stddev.x, stddev.y, stddev.z),
            keep_aspect_ratio, swap_RB_channels, (cudaStream_t)stream);
    }

    /**
    \brief Convert an image array into a GPU Tensor in planar channel configuration (NCHW), ready to use for deep learning model
    \param image_in : input images to convert
    \param tensor_out : output GPU tensor
    \param resolution_out : resolution of the output image, generally square, although not mandatory
    \param scalefactor : Scale factor applied to each pixel value, typically to convert the char value into [0-1] float
    \param mean : mean, statistic to normalized the pixel values, applied AFTER the scale. For instance for imagenet statistics the mean would be sl::float3(0.485, 0.456, 0.406)
    \param stddev : standard deviation, statistic to normalized the pixel values, applied AFTER the scale. For instance for imagenet statistics the standard deviation would be sl::float3(0.229, 0.224, 0.225)
    \param keep_aspect_ratio : indicates if the original width and height ratio should be kept using padding (sometimes refer to as letterboxing) or if the image should be stretched
    \param swap_RB_channels : indicates if the Red and Blue channels should be swapped (RGB<->BGR or RGBA<->BGRA)
    \param stream : a cuda stream to put the compute
     */
    INTERFACE_API int sl_blob_from_images(void** array_ptr, int nb_images, void* tensor_out, struct SL_Resolution resolution_out,
        float scalefactor, struct SL_Vector3 mean, struct SL_Vector3 stddev, bool keep_aspect_ratio, bool swap_RB_channels,
        void* stream)
    {
        std::vector<sl::Mat> array(nb_images);
		for (int i = 0; i < nb_images; i++) {
			array[i] = *(sl::Mat*)array_ptr[i];
		}
		int res = (int)sl::blobFromImages(array, *(sl::Mat*)tensor_out, sl::Resolution(resolution_out.width, resolution_out.height),
			scalefactor, sl::float3(mean.x, mean.y, mean.z),
			sl::float3(stddev.x, stddev.y, stddev.z),
			keep_aspect_ratio, swap_RB_channels, (cudaStream_t)stream);

		return res;
    }

    INTERFACE_API bool sl_is_camera_one(enum SL_MODEL m){
        auto model = static_cast<sl::MODEL>(m);
        return sl::isCameraOne(model);
    }

    INTERFACE_API bool sl_is_resolution_available(enum SL_RESOLUTION r, enum SL_MODEL m){
        auto model = static_cast<sl::MODEL>(m);
        auto res = static_cast<sl::RESOLUTION>(r);
        return sl::isAvailable(res, model);
    }

    INTERFACE_API bool sl_is_FPS_available(int fps, enum SL_RESOLUTION r, enum SL_MODEL m){
        auto model = static_cast<sl::MODEL>(m);
        auto res = static_cast<sl::RESOLUTION>(r);
        return sl::isAvailable(fps, res, model);
    }

    INTERFACE_API bool sl_is_HDR_available(enum SL_RESOLUTION r, enum SL_MODEL m){
        auto model = static_cast<sl::MODEL>(m);
        auto res = static_cast<sl::RESOLUTION>(r);
        return sl::supportHDR(res, model);
    }

#ifdef __cplusplus
}
#endif
