#ifndef __ZED_CONTROLLER_H__
#define __ZED_CONTROLLER_H__

#include <memory>
#include <sl/Camera.hpp>
#ifdef _WIN32
#include <Windows.h>
#endif
#include <cuda.h>
#include "cuda_runtime.h"
#include "sl/c_api/types_c.h"

enum TRACKING_TYPE {
    PATH,
    POSE
};

static std::mutex globalmutex;

inline struct SL_Resolution* convertResolution(const sl::Resolution& res)
{
	struct SL_Resolution* res_ = new SL_Resolution();
	res_->width = res.width;
	res_->height = res.height;
	return res_;
}

inline struct SL_Vector3* convertVector3(const sl::float3& vector)
{
	struct SL_Vector3* vector_ = new SL_Vector3();
	vector_->x = vector.x;
	vector_->y = vector.y;
	vector_->z = vector.z;
	return vector_;
}

inline struct SL_Quaternion* convertQuaternion(const sl::float4& vector)
{
	struct SL_Quaternion* quat_ = new SL_Quaternion();
    quat_->x = vector.x;
    quat_->y = vector.y;
    quat_->z = vector.z;
    quat_->w = vector.w;
	return quat_;
}

class ZEDController {
public:
    ZEDController(int i);
    ~ZEDController();

    static ZEDController* get(int i) {
        if (!instance[i]) // Only allow one instance of class to be generated.
            instance[i] = new ZEDController(i);

        return instance[i];
    }

    static void destroyInstance(int i) {
        if (!instance[i]) // Only allow one instance of class to be generated.
            delete instance[i];
        instance[i] = nullptr;

    }

    static bool isNotCreated(int i) {
        return (instance[i] == nullptr);
    }

    inline bool isNull() {
		return !zed.isOpened();
    }
    void createCamera(bool verbose);

    void destroy();

    int initFromUSB(SL_InitParameters *params, const unsigned int serial_number, const char* outputFile, const char* opt_settings_path, const char* opencv_calib_path);
    int initFromSVO(SL_InitParameters *params, const char* pathSVO, const char* outputFile, const char* opt_settings_path, const char* opencv_calib_path);
    int initFromStream(SL_InitParameters *params, const char* ip, int port, const char* outputFile, const char* opt_settings_path, const char* opencv_calib_path);
    int initFromGMSL(SL_InitParameters* params, const unsigned int serial_number, int gmsl_port, const char* output_file, const char* opt_settings_path, const char* opencv_calib_path);
    int initFromLive(SL_InitParameters* params, const unsigned int serial_number, const char* output_file, const char* opt_settings_path, const char* opencv_calib_path);

    sl::POSITIONAL_TRACKING_STATE getPoseArray(float* pose, int mat_type);

	int getPositionalTrackingLandmarks(SL_Landmark** landmarks, uint32_t* count);
    int getPositionalTrackingLandmarks2d(SL_Landmark2D** landmarks, uint32_t* count);

    struct SL_PositionalTrackingStatus* getPositionalTrackingStatus();

    inline unsigned int getWidth() {
        return width;
    }

    inline unsigned int getHeight() {
        return height;
    }

    sl::CameraInformation* getSLCameraInformation();
    SL_CalibrationParameters* getCalibrationParameters(bool raw);
    SL_SensorsConfiguration* getSensorsConfiguration();
	SL_CameraInformation* getCameraInformation(int width, int height);
    sl::MODEL getCameraModel();
    SL_InitParameters* getInitParameters();
    SL_RuntimeParameters* getRuntimeParameters();
    SL_PositionalTrackingParameters* getPositionalTrackingParameters();
    sl::ERROR_CODE grab(SL_RuntimeParameters *runtimeParameters);
	SL_HealthStatus* getHealthStatus();
    SL_Resolution* getRetrieveImageResolution(SL_Resolution* res);
    SL_Resolution* getRetrieveMeasureResolution(SL_Resolution* res);

    /************Recording******************/
    sl::ERROR_CODE enableRecording(const char* path, sl::SVO_COMPRESSION_MODE compressionMode, unsigned int bitrate, int targetFPS, bool transcode);
    void disableRecording();
	SL_RecordingStatus* getRecordingStatus();
	SL_RecordingParameters* getRecordingParameters();

    sl::ERROR_CODE ingestDataIntoSVO(struct SL_SVOData* data);
    int getSVODataSize(char* key, unsigned long long ts_begin, unsigned long long ts_end);
    sl::ERROR_CODE retrieveSVOData(char* key, int nb_data, struct SL_SVOData** data, unsigned long long ts_begin, unsigned long long ts_end);
    int getSVODataKeysSize();
    void getSVODataKeys(int nb_keys, char** keys);

    /************Tracking*******************/
    sl::ERROR_CODE enableTracking(SL_PositionalTrackingParameters* tracking_params, const char* area_file_path);
    sl::POSITIONAL_TRACKING_STATE getPosition(SL_Quaternion *quat, SL_Vector3 *vec, sl::REFERENCE_FRAME mat_type);
    sl::POSITIONAL_TRACKING_STATE getPosition(SL_Quaternion *quat, SL_Vector3 *vec, SL_Vector3 *offset, SL_Quaternion *offsetRotation, int type);
    sl::POSITIONAL_TRACKING_STATE getPosition(SL_PoseData *pose, int reference_frame);

    sl::ERROR_CODE resetTracking(SL_Quaternion rotation, SL_Vector3 translation);
    sl::ERROR_CODE resetTrackingWithOffset(SL_Quaternion rotation, SL_Vector3 translation, SL_Quaternion offsetRotation, SL_Vector3 offsetTranslation);
    void disableTracking(const char *path);


    sl::ERROR_CODE setIMUPriorOrientation(SL_Quaternion rotation);
    sl::ERROR_CODE getIMUOrientation(SL_Quaternion *quaternion, int time_reference);
    sl::ERROR_CODE getSensorsData(SL_SensorsData *sensorData, int time_reference);

    sl::ERROR_CODE getSensorsDataBatchCount(int* count);
    sl::ERROR_CODE getSensorsDataBatch(struct SL_SensorsData** imu_data);
    sl::ERROR_CODE startRegionOfInterestAutoDetection(SL_RegionOfInterestParameters* roi_params);
    sl::ERROR_CODE setRegionOfInterest(sl::Mat mask, bool module[SL_MODULE_LAST]);
    sl::ERROR_CODE getRegionOfInterest(sl::Mat mask, sl::Resolution mask_size, enum SL_MODULE module);

    /***********Spatial mapping *********/
    sl::ERROR_CODE enableSpatialMapping(struct SL_SpatialMappingParameters mapping_param);
    void requestMeshAsync();
    sl::ERROR_CODE updateMesh(int* numVertices, int* numTriangles, int* numSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh);
    sl::ERROR_CODE extractWholeSpatialMap();
    bool filterMesh(sl::MeshFilterParameters::MESH_FILTER filterParams, int* numVertices, int* numTriangles, int* numUpdatedSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh);
    sl::ERROR_CODE retrieveMesh(float* vertices, int* triangles, unsigned char* colors,  const int maxSubmesh, float* uvs, unsigned char* texturePtr);
    sl::ERROR_CODE updateChunks(int* numVertices, int* numTriangles, int* numSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh);
    sl::ERROR_CODE retrieveChunks(const int maxSubmesh, float* vertices, int* triangles, unsigned char* colors, float* uvs, unsigned char* texturePtr);

    sl::ERROR_CODE updateWholeMesh(int* nb_vertices, int* nb_triangles);
    sl::ERROR_CODE retrieveWholeMesh(float* vertices, int* triangles, unsigned char* colors, float* uvs, unsigned char* texture_ptr);
    bool loadWholeMesh(const char* filename, int* nb_vertices, int* nb_triangles, int* texture_size);
    bool applyWholeTexture(int* nb_vertices, int* nb_triangles, int* texture_size);
    bool filterWholeMesh(sl::MeshFilterParameters::MESH_FILTER filter_param, int* nb_vertices, int* nb_triangles);

    sl::ERROR_CODE updateFusedPointCloud(int* numPointsTot);
    sl::ERROR_CODE retrieveFusedPointCloud(float* vertices);

    void disableSpatialMapping();
	SL_SpatialMappingParameters* getSpatialMappingParameters();
    bool saveMesh(const char* filename, sl::MESH_FILE_FORMAT format);
    bool savePointCloud(const char* filename, sl::MESH_FILE_FORMAT format);
    bool loadMesh(const char* filename, int* numVertices, int* numTriangles, int* numSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh, int* texturesSize = nullptr);
    bool applyTexture(int* numVertices, int* numTriangles, int* numUpdatedSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, int* texturesSize, const int maxSubmesh);
    void mergeChunks(int size, int* numVertices, int* numTriangles, int* numUpdatedSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh);
    sl::float3 getGravityEstimation();

    void lock() {
        sdk_mutex.lock();
    }

    void unlock() {
        sdk_mutex.unlock();
    }

    /*********************************
     ******* Plane Detection *********
     *************************************/
    SL_PlaneData* findFloorPlane(SL_Quaternion *resetQuaternion, SL_Vector3* resetTranslation, SL_Quaternion priorQuaternion, SL_Vector3 priorTranslation);
    SL_PlaneData* findPlaneAtHit(SL_Vector2 pixel, struct SL_PlaneDetectionParameters* params, bool thres);
    sl::ERROR_CODE convertCurrentFloorPlaneToChunk(float* vertices, int* triangles, int* numVerticesTot, int* numTrianglesTot);
    sl::ERROR_CODE convertCurrentHitPlaneToChunk(float* vertices, int* triangles, int* numVerticesTot, int* numTrianglesTot);


    /*********************************
     ******* Streaming module (sdk) *********
     *************************************/
    sl::ERROR_CODE enableStreaming(sl::STREAMING_CODEC codec, unsigned int bitrate, unsigned short port, int gopSize, bool adaptativeBitrate, int chunk_size, int target_framerate);
    bool isStreamingEnabled();
    void disableStreaming();
	SL_StreamingParameters* getStreamingParameters();

    /*****************************
     ****** Save fct utils ********
     ******************************/
    sl::ERROR_CODE saveCurrentImage(sl::VIEW, const char* filename);
    sl::ERROR_CODE saveCurrentDepth(int side, const char* filename);
    sl::ERROR_CODE saveCurrentPointCloud(int side, const char* filename);

    /*********************************
     ******* Object Detection (sdk) *********
     *************************************/
#if WITH_OBJECT_DETECTION
    sl::ERROR_CODE enableObjectDetection(SL_ObjectDetectionParameters* objparams);
    sl::ERROR_CODE enableBodyTracking(SL_BodyTrackingParameters* body_params);
	SL_ObjectDetectionParameters* getObjectDetectionParameters();
    SL_BodyTrackingParameters* getBodyTrackingParameters();
    void disableObjectDetection(unsigned int instance_id, bool force_disable_all_instances);
    void disableBodyTracking(unsigned int instance_id, bool force_disable_all_instances);
    sl::ERROR_CODE setBodyTrackingRuntimeParameters(SL_BodyTrackingRuntimeParameters body_params, unsigned int instance_id);
    sl::ERROR_CODE setObjectDetectionRuntimeParameters(SL_ObjectDetectionRuntimeParameters object_detection_params, unsigned int instance_id);
    sl::ERROR_CODE setCustomObjectDetectionRuntimeParameters(SL_CustomObjectDetectionRuntimeParameters custom_object_detection_params, unsigned int instance_id);
    sl::ERROR_CODE ingestCustomBoxObjectData(int nb_objects, SL_CustomBoxObjectData* objects_in, unsigned int instance_id);
    sl::ERROR_CODE ingestCustomMaskObjectData(int nb_objects, SL_CustomMaskObjectData* objects_in, unsigned int instance_id);
    sl::ERROR_CODE retrieveObjectDetectionData(SL_ObjectDetectionRuntimeParameters* objruntimeparams, SL_Objects* data, unsigned int instance_id);
    sl::ERROR_CODE retrieveObjectDetectionData(SL_Objects* data, unsigned int instance_id);
    sl::ERROR_CODE retrieveCustomObjectDetectionData(SL_CustomObjectDetectionRuntimeParameters* objruntimeparams, SL_Objects* data, unsigned int instance_id);
    sl::ERROR_CODE retrieveBodyTrackingData(SL_BodyTrackingRuntimeParameters* bodyruntimeparams, SL_Bodies* data, unsigned int instance_id);
    sl::ERROR_CODE retrieveBodyTrackingData(SL_Bodies* data, unsigned int instance_id);

    sl::ERROR_CODE updateObjectsBatch(int* nb_batches);
    sl::ERROR_CODE getObjectsBatchData(int index, struct SL_ObjectsBatch *objs);

	sl::ERROR_CODE getObjectsBatchDataCSharp(int index, int* num_data, int* id, int* label, int* sublabel, int* tracking_state,
		SL_Vector3 positions[MAX_TRAJECTORY_SIZE], float position_covariances[MAX_TRAJECTORY_SIZE][6], SL_Vector3 velocities[MAX_TRAJECTORY_SIZE], unsigned long long timestamps[MAX_TRAJECTORY_SIZE],
		SL_Vector2 bounding_boxes_2d[MAX_TRAJECTORY_SIZE][4], SL_Vector3 bounding_boxes[MAX_TRAJECTORY_SIZE][8], float confidences[MAX_TRAJECTORY_SIZE], int action_states[MAX_TRAJECTORY_SIZE],
		SL_Vector2 head_bounding_boxes_2d[MAX_TRAJECTORY_SIZE][4], SL_Vector3 head_bounding_boxes[MAX_TRAJECTORY_SIZE][8], SL_Vector3 head_positions[MAX_TRAJECTORY_SIZE]);
#if 0
    sl::ERROR_CODE updateBodiesBatch(int* nb_batches);
    sl::ERROR_CODE getBodiesBatchData(int index, struct SL_BodiesBatch* bodies);
    sl::ERROR_CODE getBodiesBatchDataCSharp(int index, int* num_data, int* id, int* tracking_state,
        SL_Vector3 positions[MAX_TRAJECTORY_SIZE], float position_covariances[MAX_TRAJECTORY_SIZE][6], SL_Vector3 velocities[MAX_TRAJECTORY_SIZE], unsigned long long timestamps[MAX_TRAJECTORY_SIZE],
        SL_Vector2 bounding_boxes_2d[MAX_TRAJECTORY_SIZE][4], SL_Vector3 bounding_boxes[MAX_TRAJECTORY_SIZE][8], float confidences[MAX_TRAJECTORY_SIZE], int action_states[MAX_TRAJECTORY_SIZE],
        SL_Vector2 keypoints_2d[MAX_TRAJECTORY_SIZE][70], SL_Vector3 keypoints[MAX_TRAJECTORY_SIZE][70], SL_Vector2 head_bounding_boxes_2d[MAX_TRAJECTORY_SIZE][4], SL_Vector3 head_bounding_boxes[MAX_TRAJECTORY_SIZE][8],
        SL_Vector3 head_positions[MAX_TRAJECTORY_SIZE], float keypoints_confidences[MAX_TRAJECTORY_SIZE][70]);
#endif
#endif

    int getInputType();

    /**************************************************/
    sl::Camera zed;

private:

#if WITH_OBJECT_DETECTION
    template <typename SL_ObjectDetectionRuntimeParameters_t>
    sl::ERROR_CODE retrieveObjectsGeneric(SL_ObjectDetectionRuntimeParameters_t* runtimeParams, SL_Objects* data, unsigned int instance_id);
#endif

    int open();

    bool cameraOpened = false;

    sl::CalibrationParameters camParameters;
    sl::CameraInformation camInformations;
    SL_PlaneData currentFloorPlane;
    SL_PlaneData currentPlaneAtHit;
    sl::Plane currentFloorPlaneSDK;
    sl::Plane currentPlaneAtHitSDK;
    bool activTracking;

    std::map<sl::Timestamp, sl::SVOData> currentSVOData;
    bool isSVODataReady = false;

    int input_type;
    unsigned int width;
    unsigned int height;
    sl::RESOLUTION resolution;

    bool stabilizer = false;
    bool verbosity = false;

    sl::Transform previousPath;
    sl::RuntimeParameters runtimeParams;
    CUstream stream;

    /*MESH*/
    bool isMeshUpdated = false;
    bool areTextureReady = false;
    bool isTextured = false;

    bool isSensorsBatchReady = false;
    std::vector<sl::SensorsData> sensorsDataBatch;

    bool saveTexture;
    sl::Mesh mesh;
    sl::FusedPointCloud pointCloudFused;
    sl::ERROR_CODE OpeningErrorCode;
    std::thread threadOpening;
    sl::InitParameters initParams;
    CUcontext context;

    std::mutex sdk_mutex;
    static ZEDController* instance[MAX_CAMERA_PLUGIN];


    int grab_count_t = 0;
    sl::Timestamp initial_Timestamp = 0;

    std::vector<sl::ObjectsBatch> objects_trajectories;

    bool isObjectsTrajectoriesUpdated = false;

    std::vector<sl::Plane> currentPlanes;
    FILE* fskl = nullptr;

    int camera_ID = 0;

    sl::OBJECT_DETECTION_MODEL current_object_detection_model;
    sl::BODY_TRACKING_MODEL current_body_tracking_model;
};

#endif
