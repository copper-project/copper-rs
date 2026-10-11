#ifndef __ZED_MC_CONTROLLER_H__
#define __ZED_MC_CONTROLLER_H__


#include <sl/Camera.hpp>
#include <sl/Fusion.hpp>
#ifdef _WIN32
#include <Windows.h>
#endif
#include <cuda.h>
#include "cuda_runtime.h"
#include "sl/c_api/types_c.h"

static std::mutex mglobalmutex;

class ZEDFusionController {
public:
    ZEDFusionController();
    ~ZEDFusionController();

    static ZEDFusionController* get() {
        if (!instance) // Only allow one instance of class to be generated.
            instance = new ZEDFusionController();
        return instance;
    }

    static void destroyInstance() {
        if (!instance) // Only allow one instance of class to be generated.
            delete instance;
        instance = nullptr;

    }

    static bool isNotCreated() {
        return (instance == nullptr);
    }


	void close();

    /**
     * \brief Runs the main function of the Fusion, this trigger the retrieve and synchronization of all connected senders and updates the enabled modules.
     * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
     */
    enum SL_FUSION_ERROR_CODE process();

    /**
     * \brief Set the specified camera as a data provider.
     * \param uuid: The requested camera identifier.
     * \param params: The communication parameters to connect to the camera.
     * \param pose_translation: The World translation of the camera, regarding the other camera of the setup.
     * \param pose_rotation: The World rotation of the camera, regarding the other camera of the setup.
     * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
     */
    enum SL_FUSION_ERROR_CODE subscribe(struct SL_CameraIdentifier* uuid, struct SL_CommunicationParameters* params, struct SL_Vector3* pose_translation, struct SL_Quaternion* pose_rotation);

    enum SL_FUSION_ERROR_CODE unsubscribe(struct SL_CameraIdentifier* uuid);

    /**
     * \brief gets the specified camera position inside fusion WORLD.
     * \param uuid: The requested camera identifier.
     * \param pose_translation: The World translation of the camera, regarding the other camera of the setup.
     * \param pose_rotation: The World rotation of the camera, regarding the other camera of the setup.
     * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
     */
    enum SL_FUSION_ERROR_CODE updatePose(struct SL_CameraIdentifier* uuid, struct SL_Vector3* pose_translation, struct SL_Quaternion* pose_rotation);

    enum SL_FUSION_ERROR_CODE getPose(struct SL_CameraIdentifier* uuid, struct SL_Vector3* pose_translation, struct SL_Quaternion* pose_rotation);

    /**
     * \brief Returns the state of each connected data senders.
     * \return The individual state of each connected senders.
     */
    enum SL_SENDER_ERROR_CODE getSenderState(struct SL_CameraIdentifier* uuid);

    /**
    \brief Read a Configuration JSON file to configure a fusion process.
    \param json_config_filename : The name of the JSON file containing the configuration.
    \param coord_system : The COORDINATE_SYSTEM in which you want the World Pose to be in.
    \param unit : The UNIT in which you want the World Pose to be in.
    \param configs An array of SL_FusionConfiguration for all the camera present in the file.
    \param nb_cameras the size of the aray of SL_FusionConfiguration
    \note Empty if no data were found for the requested camera.
     */
    void readFusionConfigFile(const char* json_config_filename, enum SL_COORDINATE_SYSTEM coord_system, enum SL_UNIT unit, struct SL_FusionConfiguration configs[MAX_FUSED_CAMERAS],int& nb_cameras);

    /**
    \brief Read a Configuration JSON string to configure a fusion process.
    \param fusion_configuration : The string containing the configuration (it will be parsed like a json).
    \param file_size : size of the configuration file.
    \param coord_sys : The COORDINATE_SYSTEM in which you want the World Pose to be in.
    \param unit : The UNIT in which you want the World Pose to be in.

    \return A vector of \ref FusionConfiguration for all the camera present in the file.
    \note Empty if no data were found for the requested camera.
     */
    void readFusionConfig(const char* json_config_filename, enum SL_COORDINATE_SYSTEM coord_system, enum SL_UNIT unit, struct SL_FusionConfiguration configs[MAX_FUSED_CAMERAS], int& nb_cameras);


    /////////////////////////////////////////////////////////////////////
    ///////////////////// Object Detection Fusion ///////////////////////
    /////////////////////////////////////////////////////////////////////

    ///
    /// \brief Init multicam parameters
    /// \param [in] init parameters
    /// \return SL_FUSION_ERROR_CODE
    ///
    enum SL_FUSION_ERROR_CODE init(struct SL_InitFusionParameters* init_parameters);

    ///
    /// \brief enables body tracking fusion module
    /// \param [in] parameters defined by \ref SL_BodyTrackingFusionParameters
    /// \return SL_FUSION_ERROR_CODE
    ///
    enum SL_FUSION_ERROR_CODE enableBodyTracking(struct SL_BodyTrackingFusionParameters* params);

    /**
     * \brief get the stats of a given camera in the Fusion API side
     * It can be the received FPS, drop frame, latency, etc
     * \param metrics : structure containing all the metrics available
     * \return SL_FUSION_ERROR_CODE
     */
    enum SL_FUSION_ERROR_CODE getProcessMetrics(struct SL_FusionMetrics* metrics);

    ///
    /// \brief disable Object detection fusion module
    /// \return
    ///
	void disableBodyTracking();

    ///
    /// \brief retrieves a list of bodies (in SL_Bodies class type) seen by all cameras and merged as if it was seen by a single super-camera.
    /// \note Internal calls retrieveBodies() for all listed cameras, then merged into a single SL_Bodies
    /// \param [out] bodies: list of bodies seen by all available cameras
    /// \note Only the 3d informations is available in the returned object.
    ///
    enum SL_FUSION_ERROR_CODE retrieveBodies(struct SL_Bodies* bodies, struct SL_BodyTrackingFusionRuntimeParameters* rt, struct SL_CameraIdentifier uuid, enum SL_FUSION_REFERENCE_FRAME reference_frame);

    /////////////////////////////////////////////////////////////////////
    ///////////////////// Positional Tracking Fusion ///////////////////////
    /////////////////////////////////////////////////////////////////////


    /**
    * \brief enable positional tracking fusion.
    * \note note that for the alpha version of the API, the positional tracking fusion doesn't support the area memory feature
    *
    * \return ERROR_CODE
    */
    enum SL_FUSION_ERROR_CODE enablePositionalTracking(struct SL_PositionalTrackingFusionParameters* params);

    /**
     * \brief Get the Fused Position of the camera system
     *
     * \param camera_pose will contain the camera pose in world position (world position is given by the calibration of the cameras system)
     * \param reference_frame defines the reference from which you want the pose to be expressed. Default : \ref REFERENCE_FRAME "REFERENCE_FRAME::WORLD".
     * \param uuid Camera identifier
     * \return POSITIONAL_TRACKING_STATE is the current state of the tracking process
     */
    enum SL_POSITIONAL_TRACKING_STATE getPosition(struct SL_PoseData* pose, enum SL_REFERENCE_FRAME reference_frame, struct SL_CameraIdentifier* uuid, enum SL_POSITION_TYPE retrieve_type);

    /**
     * @brief Get the current status of fused position.
     * \return SL_FusedPositionalTrackingStatus is the current status of the tracking process.
     */
    struct SL_FusedPositionalTrackingStatus* getFusedPositionalTrackingStatus();

    ///
    /// \brief disable Positional Tracking fusion module
    ///
    void disablePositionalTracking();


    enum SL_FUSION_ERROR_CODE ingestGNSSData(struct SL_GNSSData* data, bool radian);

    enum SL_POSITIONAL_TRACKING_STATE getCurrentGNSSData(struct SL_GNSSData* data, bool radian);

    enum SL_GNSS_FUSION_STATUS getGeoPose(struct SL_GeoPose* pose, bool radian);

    enum SL_GNSS_FUSION_STATUS geoToCamera(struct SL_LatLng* in, struct SL_PoseData* out, bool radian);

    enum SL_GNSS_FUSION_STATUS cameraToGeo(struct SL_PoseData* in, struct SL_GeoPose* out, bool radian);

    enum SL_FUSION_ERROR_CODE enuToGeo(struct SL_ENU* in, struct SL_LatLng* out);

	enum SL_FUSION_ERROR_CODE geoToEnu(struct SL_LatLng* in, struct SL_ENU* out);

    enum SL_GNSS_FUSION_STATUS getCurrentGNSSCalibrationSTD(float* yaw_std, struct SL_Vector3* position_std);

    void getGeoTrackingCalibration(struct SL_Vector3* translation, struct SL_Quaternion* rotation);

	void destroy();

private:
	sl::BodyTrackingFusionParameters BT_fusion_init_params;
    static ZEDFusionController* instance;
public :
    sl::Fusion fusion;
};

#endif
