#include "ZEDFusionController.hpp"
#include <algorithm>
#include <cmath>

#include <sl/Camera.hpp>

ZEDFusionController* ZEDFusionController::instance = nullptr;

ZEDFusionController::ZEDFusionController() {
}

ZEDFusionController::~ZEDFusionController() {
     destroy();
}

void ZEDFusionController::destroy() {
	fusion.close();
}

void ZEDFusionController::close() {
	fusion.close();
}

SL_FUSION_ERROR_CODE ZEDFusionController::init(struct SL_InitFusionParameters* init_parameters) {
	sl::InitFusionParameters init_params;

	init_params.coordinate_system = (sl::COORDINATE_SYSTEM)init_parameters->coordinate_system;
	init_params.coordinate_units = (sl::UNIT)init_parameters->coordinate_units;
	init_params.output_performance_metrics = init_parameters->output_performance_metrics;
	init_params.verbose = init_parameters->verbose;
	init_params.synchronization_parameters.windows_size = init_parameters->synchronization_parameters.windows_size;
	init_params.synchronization_parameters.data_source_timeout = init_parameters->synchronization_parameters.data_source_timeout;
	init_params.synchronization_parameters.keep_last_data = init_parameters->synchronization_parameters.keep_last_data;
	init_params.synchronization_parameters.maximum_lateness = init_parameters->synchronization_parameters.maximum_lateness;
	init_params.sdk_gpu_id = init_parameters->sdk_gpu_id;
	init_params.sdk_cuda_ctx = init_parameters->sdk_cuda_ctx;
	init_params.maximum_working_resolution = sl::Resolution(init_parameters->maximum_working_resolution.width, init_parameters->maximum_working_resolution.height);
	sl::FUSION_ERROR_CODE err = fusion.init(init_params);

	return (SL_FUSION_ERROR_CODE)err;
}


SL_FUSION_ERROR_CODE ZEDFusionController::process() {

	return (SL_FUSION_ERROR_CODE)fusion.process();
}

SL_FUSION_ERROR_CODE ZEDFusionController::subscribe(struct SL_CameraIdentifier* uuid, struct SL_CommunicationParameters* params, struct SL_Vector3* pose_translation, struct SL_Quaternion* pose_rotation) {

	sl::CameraIdentifier sl_uuid;
	sl_uuid.sn = uuid->sn;

	sl::CommunicationParameters comm_params;
	if (params->communication_type == SL_COMM_TYPE_INTRA_PROCESS)
	{
		comm_params.setForSharedMemory();
	}
	else // NETWORK
	{
		comm_params.setForLocalNetwork(std::string(params->ip_add), params->ip_port);
	}

	sl::Transform pose;
	sl::float3 translation(pose_translation->x, pose_translation->y, pose_translation->z);
	sl::float4 orientation(pose_rotation->x, pose_rotation->y, pose_rotation->z, pose_rotation->w);
	pose.setTranslation(translation);
	pose.setOrientation(orientation);

	return (SL_FUSION_ERROR_CODE)fusion.subscribe(sl_uuid, comm_params, pose);
}

enum SL_FUSION_ERROR_CODE ZEDFusionController::unsubscribe(struct SL_CameraIdentifier* uuid)
{
	sl::CameraIdentifier sl_uuid;
	sl_uuid.sn = uuid->sn;

	return (SL_FUSION_ERROR_CODE)fusion.unsubscribe(sl_uuid);
}

SL_FUSION_ERROR_CODE ZEDFusionController::updatePose(struct SL_CameraIdentifier* uuid, struct SL_Vector3* pose_translation, struct SL_Quaternion* pose_rotation)
{
	sl::CameraIdentifier sl_uuid;
	sl_uuid.sn = uuid->sn;

	sl::Transform pose;
	sl::float3 translation(pose_translation->x, pose_translation->y, pose_translation->z);
	sl::float4 orientation(pose_rotation->x, pose_rotation->y, pose_rotation->z, pose_rotation->w);
	pose.setTranslation(translation);
	pose.setOrientation(orientation);

	return (SL_FUSION_ERROR_CODE)fusion.updatePose(sl_uuid, pose);
}

SL_FUSION_ERROR_CODE ZEDFusionController::getPose(SL_CameraIdentifier* uuid, SL_Vector3* pose_translation, SL_Quaternion* pose_rotation)
{
	sl::FUSION_ERROR_CODE err = sl::FUSION_ERROR_CODE::FAILURE;
	sl::Transform pose;
	err = fusion.getPose(sl::CameraIdentifier(uuid->sn), pose);

	if (err == sl::FUSION_ERROR_CODE::SUCCESS)
	{
		pose_translation->x = pose.getTranslation().x;
		pose_translation->y = pose.getTranslation().y;
		pose_translation->z = pose.getTranslation().z;
		pose_rotation->x = pose.getOrientation().x;
		pose_rotation->y = pose.getOrientation().y;
		pose_rotation->z = pose.getOrientation().z;
		pose_rotation->w = pose.getOrientation().w;
	}
	return (SL_FUSION_ERROR_CODE)err;
}

SL_FUSION_ERROR_CODE ZEDFusionController::enableBodyTracking(struct SL_BodyTrackingFusionParameters* params)
{
	BT_fusion_init_params.enable_tracking = params->enable_tracking;
	BT_fusion_init_params.enable_body_fitting = params->enable_body_fitting;

	return (SL_FUSION_ERROR_CODE)fusion.enableBodyTracking(BT_fusion_init_params);
}

void ZEDFusionController::disableBodyTracking() {
	fusion.disableBodyTracking();
}

SL_FUSION_ERROR_CODE ZEDFusionController::retrieveBodies(struct SL_Bodies* data, struct SL_BodyTrackingFusionRuntimeParameters* rt, struct SL_CameraIdentifier uuid, enum SL_FUSION_REFERENCE_FRAME reference_frame) {
	memset(data, 0, sizeof(SL_Bodies));

	sl::BodyTrackingFusionRuntimeParameters bt_rt;
	bt_rt.skeleton_minimum_allowed_keypoints = rt->skeleton_minimum_allowed_keypoints;
	bt_rt.skeleton_minimum_allowed_camera = rt->skeleton_minimum_allowed_camera;
	bt_rt.skeleton_smoothing = rt->skeleton_smoothing;

	sl::CameraIdentifier sdk_uuid;
	sdk_uuid.sn = uuid.sn;

	sl::Bodies bodies;
	sl::FUSION_ERROR_CODE v = fusion.retrieveBodies(bodies, bt_rt, sdk_uuid, (sl::FUSION_REFERENCE_FRAME)reference_frame);
	if (v == sl::FUSION_ERROR_CODE::SUCCESS) {
		data->is_new = (int)bodies.is_new;

		data->is_tracked = bodies.is_tracked;
		int size_objects = bodies.body_list.size();
		data->timestamp = bodies.timestamp;
		data->nb_bodies = size_objects;
		data->body_format = (SL_BODY_FORMAT)bodies.body_format;
		data->inference_precision_mode = (SL_INFERENCE_PRECISION)bodies.inference_precision_mode;

		int count = 0;

		for (auto& p : bodies.body_list) {
			if (count < MAX_NUMBER_OBJECT) {
				data->body_list[count].tracking_state = (SL_OBJECT_TRACKING_STATE)p.tracking_state;
				data->body_list[count].action_state = (SL_OBJECT_ACTION_STATE)p.action_state;
				data->body_list[count].id = p.id;
				data->body_list[count].confidence = p.confidence;

				memcpy(data->body_list[count].unique_object_id, p.unique_object_id, 37 * sizeof(char));

				for (int k = 0; k < 6; k++)
					data->body_list[count].position_covariance[k] = p.position_covariance[k];

				data->body_list[count].mask = (int*)(new sl::Mat(p.mask));

				for (int l = 0; l < p.bounding_box_2d.size(); l++) {
					data->body_list[count].bounding_box_2d[l].x = (float)p.bounding_box_2d.at(l).x;
					data->body_list[count].bounding_box_2d[l].y = (float)p.bounding_box_2d.at(l).y;
				}

				for (int l = 0; l < p.head_bounding_box.size(); l++) {
					data->body_list[count].head_bounding_box[l].x = (float)p.head_bounding_box.at(l).x;
					data->body_list[count].head_bounding_box[l].y = (float)p.head_bounding_box.at(l).y;
				}

				for (int l = 0; l < p.head_bounding_box_2d.size(); l++) {
					data->body_list[count].head_bounding_box_2d[l].x = (float)p.head_bounding_box_2d.at(l).x;
					data->body_list[count].head_bounding_box_2d[l].y = (float)p.head_bounding_box_2d.at(l).y;
				}

				// World data
				data->body_list[count].position.x = p.position.x;
				data->body_list[count].position.y = p.position.y;
				data->body_list[count].position.z = p.position.z;

				data->body_list[count].velocity.x = p.velocity.x;
				data->body_list[count].velocity.y = p.velocity.y;
				data->body_list[count].velocity.z = p.velocity.z;

				data->body_list[count].dimensions.x = p.dimensions.x;
				data->body_list[count].dimensions.y = p.dimensions.y;
				data->body_list[count].dimensions.z = p.dimensions.z;

				data->body_list[count].head_position.x = p.head_position.x;
				data->body_list[count].head_position.y = p.head_position.y;
				data->body_list[count].head_position.z = p.head_position.z;

				// 3D Bounding box in world frame
				for (int m = 0; m < 8; m++) {
					if (m < p.bounding_box.size()) {
						data->body_list[count].bounding_box[m].x = p.bounding_box.at(m).x;
						data->body_list[count].bounding_box[m].y = p.bounding_box.at(m).y;
						data->body_list[count].bounding_box[m].z = p.bounding_box.at(m).z;
					}
				}

				for (int i = 0; i < (int)p.keypoint.size(); i++) {
					data->body_list[count].keypoint_2d[i].x = p.keypoint_2d.at(i).x;
					data->body_list[count].keypoint_2d[i].y = p.keypoint_2d.at(i).y;

					data->body_list[count].keypoint[i].x = p.keypoint.at(i).x;
					data->body_list[count].keypoint[i].y = p.keypoint.at(i).y;
					data->body_list[count].keypoint[i].z = p.keypoint.at(i).z;
					data->body_list[count].keypoint_confidence[i] = p.keypoint_confidence.at(i);

					for (int k = 0; k < 6; k++)
					{
						data->body_list[count].keypoint_covariances[i][k] = p.keypoint_covariances[i][k];
					}
				}

				data->body_list[count].global_root_orientation.x = p.global_root_orientation.x;
				data->body_list[count].global_root_orientation.y = p.global_root_orientation.y;
				data->body_list[count].global_root_orientation.z = p.global_root_orientation.z;
				data->body_list[count].global_root_orientation.w = p.global_root_orientation.w;

				for (int i = 0; i < p.local_orientation_per_joint.size(); i++) { // 38 or 70

					data->body_list[count].local_orientation_per_joint[i].x = p.local_orientation_per_joint[i].x;
					data->body_list[count].local_orientation_per_joint[i].y = p.local_orientation_per_joint[i].y;
					data->body_list[count].local_orientation_per_joint[i].z = p.local_orientation_per_joint[i].z;
					data->body_list[count].local_orientation_per_joint[i].w = p.local_orientation_per_joint[i].w;

					data->body_list[count].local_position_per_joint[i].x = p.local_position_per_joint[i].x;
					data->body_list[count].local_position_per_joint[i].y = p.local_position_per_joint[i].y;
					data->body_list[count].local_position_per_joint[i].z = p.local_position_per_joint[i].z;
				}
				count++;

			}
		}
	}

	return (SL_FUSION_ERROR_CODE)v;
}


SL_FUSION_ERROR_CODE ZEDFusionController::getProcessMetrics(struct SL_FusionMetrics* metrics)
{
	memset(metrics, 0, sizeof(SL_FusionMetrics));
	sl::FusionMetrics sdk_metrics;

	sl::FUSION_ERROR_CODE err = fusion.getProcessMetrics(sdk_metrics);

	if (err == sl::FUSION_ERROR_CODE::SUCCESS)
	{
		metrics->mean_camera_fused = sdk_metrics.mean_camera_fused;
		metrics->mean_stdev_between_camera = sdk_metrics.mean_stdev_between_camera;

		int count = 0;
		for (auto it = sdk_metrics.camera_individual_stats.begin(); it != sdk_metrics.camera_individual_stats.end(); it++)
		{
			if (count < MAX_FUSED_CAMERAS)
			{
				SL_CameraMetrics camera_metrics;
				memset(&camera_metrics, 0, sizeof(SL_CameraMetrics));

				SL_CameraIdentifier uuid;
				uuid.sn = it->first.sn;
				camera_metrics.uuid = uuid;

				camera_metrics.delta_ts = it->second.delta_ts;
				camera_metrics.is_present = it->second.is_present;
				camera_metrics.ratio_detection = it->second.ratio_detection;
				camera_metrics.received_fps = it->second.received_fps;
				camera_metrics.received_latency = it->second.received_latency;
				camera_metrics.synced_latency = it->second.synced_latency;

				metrics->camera_individual_stats[count] = camera_metrics;

				count++;
			}
		}
	}
	return (SL_FUSION_ERROR_CODE)err;
}

SL_SENDER_ERROR_CODE ZEDFusionController::getSenderState(struct SL_CameraIdentifier* uuid)
{
	SL_SENDER_ERROR_CODE err = SL_SENDER_ERROR_CODE_DISCONNECTED;

	std::map<sl::CameraIdentifier, sl::SENDER_ERROR_CODE> err_codes = fusion.getSenderState();

	sl::CameraIdentifier sdk_uuid;
	sdk_uuid.sn = uuid->sn;

	if (err_codes.find(sdk_uuid) != err_codes.end())
	{
		err = (SL_SENDER_ERROR_CODE)err_codes[sdk_uuid];
	}
	else
	{
		err = SL_SENDER_ERROR_CODE_DISCONNECTED;
	}
	return err;
}

inline void splitIP(std::string ipAdd, std::string& ip, unsigned short& port) {
	std::size_t found_port = ipAdd.find_last_of(":");
	ip = ipAdd;
	port = 30000;
	if (found_port != std::string::npos) {
		ip = ipAdd.substr(0, found_port);
		port = std::atoi(ipAdd.substr(found_port + 1).c_str());
	}
}

inline SL_InputType getInput(sl::InputType::INPUT_TYPE type, sl::String conf) {
	SL_InputType input;

    switch (type)
    {
		case sl::InputType::INPUT_TYPE::GMSL_ID:
		{
			int id = atoi(conf.c_str());
			input.id = id;
			input.input_type = SL_INPUT_TYPE_GMSL;
			input.serial_number = 0;
			break;
		}
		case sl::InputType::INPUT_TYPE::GMSL_SERIAL:
		{
			int serialNumber = atoi(conf.c_str());
			input.serial_number = serialNumber;
			input.input_type = SL_INPUT_TYPE_GMSL;
			break;
		}
		case sl::InputType::INPUT_TYPE::USB_SERIAL:
		{
			int serialNumber = atoi(conf.c_str());
			input.serial_number = serialNumber;
			input.input_type = SL_INPUT_TYPE_USB;
			break;
		}
		case sl::InputType::INPUT_TYPE::STREAM:
		{
			std::string IP_add;
			unsigned short port;
			splitIP(std::string(conf.c_str()), IP_add, port);
			strcpy(input.stream_input_ip, IP_add.c_str());
			input.stream_input_port = port;
			input.input_type = SL_INPUT_TYPE_STREAM;
			break;
		}
		case sl::InputType::INPUT_TYPE::SVO_FILE:
		{
			strcpy(input.svo_input_filename, conf.c_str());
			input.input_type = SL_INPUT_TYPE_SVO;
			break;
		}
		default:
		case sl::InputType::INPUT_TYPE::USB_ID:
		{
			int id = atoi(conf.c_str());
			input.id = id;
			input.input_type = SL_INPUT_TYPE_USB;
			input.serial_number = 0;
			break;
		}
    }
	return input;
}

void ZEDFusionController::readFusionConfigFile(const char* json_config_filename, enum SL_COORDINATE_SYSTEM coord_system, enum SL_UNIT unit, struct SL_FusionConfiguration configs[MAX_FUSED_CAMERAS], int& nb_cameras)
{
	std::string filepath = std::string(json_config_filename);
	auto sdk_configs = sl::readFusionConfigurationFile(filepath, (sl::COORDINATE_SYSTEM)coord_system, (sl::UNIT)unit);

	nb_cameras = std::min(MAX_FUSED_CAMERAS, (int)sdk_configs.size());

	for (int i = 0; i < sdk_configs.size(); i++)
	{
		if (i < MAX_FUSED_CAMERAS)
		{
			sl::FusionConfiguration sdk_config = sdk_configs[i];


			SL_FusionConfiguration fusion_config;
			memset(&fusion_config, 0, sizeof(SL_FusionConfiguration));

			fusion_config.serial_number = sdk_config.serial_number;
			SL_Vector3 position;
			position.x = sdk_config.pose.getTranslation().x;
			position.y = sdk_config.pose.getTranslation().y;
			position.z = sdk_config.pose.getTranslation().z;
			SL_Quaternion rotation;
			rotation.x = sdk_config.pose.getOrientation().x;
			rotation.y = sdk_config.pose.getOrientation().y;
			rotation.z = sdk_config.pose.getOrientation().z;
			rotation.w = sdk_config.pose.getOrientation().w;

			fusion_config.position = position;
			fusion_config.rotation = rotation;

			fusion_config.override_gravity = sdk_config.override_gravity;

			fusion_config.input_type = getInput(sdk_config.input_type.getType(), sdk_config.input_type.getConfiguration());
			fusion_config.comm_param.communication_type = (SL_COMM_TYPE)sdk_config.communication_parameters.getType();
			strcpy(&fusion_config.comm_param.ip_add[0], sdk_config.communication_parameters.getIpAddress().c_str());
			fusion_config.comm_param.ip_port = sdk_config.communication_parameters.getPort();

			configs[i] = fusion_config;
		}
	}
}

void ZEDFusionController::readFusionConfig(const char* fusion_configuration, enum SL_COORDINATE_SYSTEM coord_system, enum SL_UNIT unit, struct SL_FusionConfiguration configs[MAX_FUSED_CAMERAS], int& nb_cameras)
{
	std::string file = std::string(fusion_configuration);
	auto sdk_configs = sl::readFusionConfiguration(file, (sl::COORDINATE_SYSTEM)coord_system, (sl::UNIT)unit);

	nb_cameras = std::min(MAX_FUSED_CAMERAS, (int)sdk_configs.size());

	for (int i = 0; i < sdk_configs.size(); i++)
	{
		if (i < MAX_FUSED_CAMERAS)
		{
			sl::FusionConfiguration sdk_config = sdk_configs[i];

			SL_FusionConfiguration fusion_config;
			memset(&fusion_config, 0, sizeof(SL_FusionConfiguration));

			fusion_config.serial_number = sdk_config.serial_number;
			SL_Vector3 position;
			position.x = sdk_config.pose.getTranslation().x;
			position.y = sdk_config.pose.getTranslation().y;
			position.z = sdk_config.pose.getTranslation().z;
			SL_Quaternion rotation;
			rotation.x = sdk_config.pose.getOrientation().x;
			rotation.y = sdk_config.pose.getOrientation().y;
			rotation.z = sdk_config.pose.getOrientation().z;
			rotation.w = sdk_config.pose.getOrientation().w;

			fusion_config.position = position;
			fusion_config.rotation = rotation;

			fusion_config.override_gravity = sdk_config.override_gravity;

			fusion_config.input_type = getInput(sdk_config.input_type.getType(), sdk_config.input_type.getConfiguration());
			fusion_config.comm_param.communication_type = (SL_COMM_TYPE)sdk_config.communication_parameters.getType();
			strcpy(&fusion_config.comm_param.ip_add[0], sdk_config.communication_parameters.getIpAddress().c_str());
			fusion_config.comm_param.ip_port = sdk_config.communication_parameters.getPort();

			configs[i] = fusion_config;
		}
	}
}

SL_FUSION_ERROR_CODE ZEDFusionController::enablePositionalTracking(struct SL_PositionalTrackingFusionParameters* params)
{
	sl::PositionalTrackingFusionParameters sdk_params;
	sdk_params.enable_GNSS_fusion = params->enable_GNSS_fusion;
	sdk_params.set_gravity_as_origin = params->set_gravity_as_origin;
	sdk_params.tracking_camera_id = sl::CameraIdentifier(params->tracking_camera_id.sn);

	sl::float3 translation(params->base_footprint_to_world_translation.x, params->base_footprint_to_world_translation.y, params->base_footprint_to_world_translation.z);
	sl::float4 orientation(params->base_footprint_to_world_rotation.x, params->base_footprint_to_world_rotation.y, params->base_footprint_to_world_rotation.z, params->base_footprint_to_world_rotation.w);
	sdk_params.base_footprint_to_world_transform.setTranslation(translation);
	sdk_params.base_footprint_to_world_transform.setOrientation(orientation);

	sl::float3 translation_(params->base_footprint_to_baselink_translation.x, params->base_footprint_to_baselink_translation.y, params->base_footprint_to_baselink_translation.z);
	sl::float4 orientation_(params->base_footprint_to_baselink_rotation.x, params->base_footprint_to_baselink_rotation.y, params->base_footprint_to_baselink_rotation.z, params->base_footprint_to_baselink_rotation.w);
	sdk_params.base_footprint_to_baselink_transform.setTranslation(translation_);
	sdk_params.base_footprint_to_baselink_transform.setOrientation(orientation_);

	memcpy(&sdk_params.gnss_calibration_parameters, &params->gnss_calibration_parameters, sizeof(SL_GNSSCalibrationParameters));

	return (SL_FUSION_ERROR_CODE)fusion.enablePositionalTracking(sdk_params);
}

SL_POSITIONAL_TRACKING_STATE ZEDFusionController::getPosition(SL_PoseData* poseData, enum SL_REFERENCE_FRAME reference_frame, struct SL_CameraIdentifier* uuid, enum SL_POSITION_TYPE retrieve_type)
{
	sl::CameraIdentifier sdk_uuid;
	sdk_uuid.sn = uuid->sn;
	sl::Pose sdk_pose;
	sl::POSITION_TYPE sdk_pos_type = (sl::POSITION_TYPE)retrieve_type;
	sl::POSITIONAL_TRACKING_STATE state = fusion.getPosition(sdk_pose, (sl::REFERENCE_FRAME)reference_frame, sdk_uuid, sdk_pos_type);

	memset(poseData, 0, sizeof(SL_PoseData));
	poseData->pose_confidence = sdk_pose.pose_confidence;
	sl::Orientation tempOrientation = sdk_pose.pose_data.getOrientation();
	poseData->rotation.x = tempOrientation.x;
	poseData->rotation.y = tempOrientation.y;
	poseData->rotation.z = tempOrientation.z;
	poseData->rotation.w = tempOrientation.w;

	memcpy(&poseData->pose_covariance[0], &sdk_pose.pose_covariance[0], sizeof(float) * 36);
	memcpy(&poseData->twist[0], &sdk_pose.twist[0], sizeof(float) * 6);
	memcpy(&poseData->twist_covariance[0], &sdk_pose.twist_covariance[0], sizeof(float) * 36);

	poseData->translation.x = sdk_pose.pose_data.getTranslation().x;
	poseData->translation.y = sdk_pose.pose_data.getTranslation().y;
	poseData->translation.z = sdk_pose.pose_data.getTranslation().z;

	poseData->timestamp = sdk_pose.timestamp;
	poseData->valid = sdk_pose.valid;

	return (SL_POSITIONAL_TRACKING_STATE)state;
}

struct SL_FusedPositionalTrackingStatus* ZEDFusionController::getFusedPositionalTrackingStatus()
{
	SL_FusedPositionalTrackingStatus* fused_status = new SL_FusedPositionalTrackingStatus();
	memset(fused_status, 0, sizeof(SL_FusedPositionalTrackingStatus));

	sl::FusedPositionalTrackingStatus sdk_status = fusion.getFusedPositionalTrackingStatus();

	fused_status->gnss_fusion_status = (enum SL_GNSS_FUSION_STATUS)sdk_status.gnss_fusion_status;
	fused_status->gnss_status = (enum SL_GNSS_STATUS)sdk_status.gnss_status;
	fused_status->odometry_status = (enum SL_ODOMETRY_STATUS)sdk_status.odometry_status;
	fused_status->spatial_memory_status = (enum SL_SPATIAL_MEMORY_STATUS)sdk_status.spatial_memory_status;
	fused_status->gnss_mode = (enum SL_GNSS_MODE)sdk_status.gnss_mode;
	fused_status->tracking_fusion_status = (enum SL_POSITIONAL_TRACKING_FUSION_STATUS)sdk_status.tracking_fusion_status;

	return fused_status;
}

void ZEDFusionController::disablePositionalTracking() {
	fusion.disablePositionalTracking();
}

SL_FUSION_ERROR_CODE ZEDFusionController::ingestGNSSData(struct SL_GNSSData* data, bool radian)
{
	sl::GNSSData sdk_gnss;
	sdk_gnss.setCoordinates(data->latitude, data->longitude, data->altitude, radian);

	sdk_gnss.altitude_std = data->altitude_std;
	sdk_gnss.latitude_std = data->latitude_std;
	sdk_gnss.ts = data->ts;

	sdk_gnss.gnss_status = (sl::GNSS_STATUS)data->gnss_status;
	sdk_gnss.gnss_mode = (sl::GNSS_MODE)data->gnss_mode;

	for (int i = 0; i < 9; i++)
	{
		sdk_gnss.position_covariance[i] = data->position_covariance[i];
	}

	return (SL_FUSION_ERROR_CODE)fusion.ingestGNSSData(sdk_gnss);
}

SL_POSITIONAL_TRACKING_STATE ZEDFusionController::getCurrentGNSSData(struct SL_GNSSData* data, bool radian)
{
	SL_POSITIONAL_TRACKING_STATE state = SL_POSITIONAL_TRACKING_STATE_OFF;

	memset(data, 0, sizeof(SL_GNSSData));

	sl::GNSSData sdk_gnss;
	state = (SL_POSITIONAL_TRACKING_STATE)fusion.getCurrentGNSSData(sdk_gnss);

	sdk_gnss.getCoordinates(data->latitude, data->longitude, data->altitude, radian);
	data->altitude_std = sdk_gnss.altitude_std;
	data->latitude_std = sdk_gnss.latitude_std;
	data->longitude = sdk_gnss.longitude_std;
	data->ts = sdk_gnss.ts;

	for (int i = 0; i < 9; i++)
	{
		data->position_covariance[i] = sdk_gnss.position_covariance[i];
	}

	return state;
}

enum SL_GNSS_FUSION_STATUS ZEDFusionController::getGeoPose(struct SL_GeoPose* pose, bool radian)
{
	SL_GNSS_FUSION_STATUS state = SL_GNSS_FUSION_STATUS_OFF;

	memset(pose, 0, sizeof(SL_GeoPose));

	sl::GeoPose sdk_pose;
	state = (SL_GNSS_FUSION_STATUS)fusion.getGeoPose(sdk_pose);

	pose->heading = sdk_pose.heading;
	pose->horizontal_accuracy = sdk_pose.horizontal_accuracy;
	pose->vertical_accuracy = sdk_pose.vertical_accuracy;
	pose->latlng_coordinates.latitude = sdk_pose.latlng_coordinates.getLatitude(radian);
	pose->latlng_coordinates.longitude = sdk_pose.latlng_coordinates.getLongitude(radian);
	pose->latlng_coordinates.altitude = sdk_pose.latlng_coordinates.getAltitude();
	pose->timestamp = sdk_pose.timestamp.getNanoseconds();

	sl::float3 sdk_position = sdk_pose.pose_data.getTranslation();
	pose->translation.x = sdk_position.x;
	pose->translation.y = sdk_position.y;
	pose->translation.z = sdk_position.z;

	sl::float4 sdk_rotation = sdk_pose.pose_data.getOrientation();
	pose->rotation.x = sdk_rotation.x;
	pose->rotation.y = sdk_rotation.y;
	pose->rotation.z = sdk_rotation.z;
	pose->rotation.w = sdk_rotation.w;

	for (int i = 0; i < 36; i++)
	{
		pose->pose_covariance[i] = sdk_pose.pose_covariance[i];
	}

	return state;
}

enum SL_GNSS_FUSION_STATUS ZEDFusionController::geoToCamera(struct SL_LatLng* in, struct SL_PoseData* out, bool radian)
{
	enum SL_GNSS_FUSION_STATUS state = SL_GNSS_FUSION_STATUS_OFF;

	memset(out, 0, sizeof(SL_PoseData));

	sl::LatLng latLng;
	latLng.setCoordinates(in->latitude, in->longitude, in->altitude, radian);

	sl::Pose pose;
	state = (enum SL_GNSS_FUSION_STATUS)fusion.Geo2Camera(latLng, pose);
	out->pose_confidence = pose.pose_confidence;

	for (int i = 0; i < 36; i++)
	{
		out->pose_covariance[i] = pose.pose_covariance[i];
		out->twist_covariance[i] = pose.twist_covariance[i];
	}

	sl::float3 sdk_position = pose.getTranslation();
	out->translation.x = sdk_position.x;
	out->translation.y = sdk_position.y;
	out->translation.z = sdk_position.z;

	sl::float4 sdk_rotation = pose.getOrientation();
	out->rotation.x = sdk_rotation.x;
	out->rotation.y = sdk_rotation.y;
	out->rotation.z = sdk_rotation.z;
	out->rotation.w = sdk_rotation.w;

	out->timestamp = pose.timestamp;

	for (int i = 0; i < 6; i++)
	{
		out->twist[i] = pose.twist[i];
	}

	out->valid = pose.valid;

	return state;
}

enum SL_GNSS_FUSION_STATUS ZEDFusionController::cameraToGeo(struct SL_PoseData* in, struct SL_GeoPose* out, bool radian)
{
	SL_GNSS_FUSION_STATUS state = SL_GNSS_FUSION_STATUS_OFF;

	memset(out, 0, sizeof(SL_GeoPose));

	sl::GeoPose sdk_pose;
	sl::Pose pose;
	for (int i = 0; i < 36; i++)
	{
		pose.pose_covariance[i] = in->pose_covariance[i];
		pose.twist_covariance[i] = in->twist_covariance[i];
	}

	pose.pose_data.setTranslation(sl::float3(in->translation.x, in->translation.y, in->translation.z));
	pose.pose_data.setOrientation(sl::float4(in->rotation.x, in->rotation.y, in->rotation.z, in->rotation.w));

	pose.timestamp = in->timestamp;

	for (int i = 0; i < 6; i++)
	{
		pose.twist[i] = in->twist[i];
	}

	pose.valid = in->valid;

	state = (SL_GNSS_FUSION_STATUS)fusion.Camera2Geo(pose, sdk_pose);

	out->heading = sdk_pose.heading;
	out->horizontal_accuracy = sdk_pose.horizontal_accuracy;
	out->vertical_accuracy = sdk_pose.vertical_accuracy;
	out->latlng_coordinates.latitude = sdk_pose.latlng_coordinates.getLatitude(radian);
	out->latlng_coordinates.longitude = sdk_pose.latlng_coordinates.getLongitude(radian);
	out->latlng_coordinates.altitude = sdk_pose.latlng_coordinates.getAltitude();

	sl::float3 sdk_position = sdk_pose.pose_data.getTranslation();
	out->translation.x = sdk_position.x;
	out->translation.y = sdk_position.y;
	out->translation.z = sdk_position.z;

	sl::float4 sdk_rotation = sdk_pose.pose_data.getOrientation();
	out->rotation.x = sdk_rotation.x;
	out->rotation.y = sdk_rotation.y;
	out->rotation.z = sdk_rotation.z;
	out->rotation.w = sdk_rotation.w;

	for (int i = 0; i < 36; i++)
	{
		out->pose_covariance[i] = sdk_pose.pose_covariance[i];
	}

	return state;
}

SL_FUSION_ERROR_CODE ZEDFusionController::enuToGeo(struct SL_ENU* in, struct SL_LatLng* out)
{
	sl::FUSION_ERROR_CODE err = sl::FUSION_ERROR_CODE::FAILURE;

	sl::ENU enu;
	enu.east = in->east;
	enu.north = in->north;
	enu.up = in->up;

	sl::LatLng latLng;
	err = fusion.ENU2Geo(enu, latLng);

	if (err == sl::FUSION_ERROR_CODE::SUCCESS)
	{
		out->latitude = latLng.getLatitude();
		out->longitude = latLng.getLongitude();
		out->altitude = latLng.getAltitude();
	}
	else
	{
		out->latitude = 0;
		out->longitude = 0;
		out->altitude = 0;
	}

	return (SL_FUSION_ERROR_CODE)err;
}

SL_FUSION_ERROR_CODE ZEDFusionController::geoToEnu(struct SL_LatLng* in, struct SL_ENU* out)
{
	sl::FUSION_ERROR_CODE err = sl::FUSION_ERROR_CODE::FAILURE;

	sl::LatLng latLng;
	latLng.setCoordinates(in->latitude, in->longitude, in->altitude);

	sl::ENU enu;
	err = fusion.Geo2ENU(latLng, enu);

	if (err == sl::FUSION_ERROR_CODE::SUCCESS)
	{
		out->east = enu.east;
		out->north = enu.north;
		out->up = enu.up;
	}
	else
	{
		out->east = 0;
		out->north = 0;
		out->up = 0;
	}

	return (SL_FUSION_ERROR_CODE)err;
}

enum SL_GNSS_FUSION_STATUS ZEDFusionController::getCurrentGNSSCalibrationSTD(float* yaw_std, struct SL_Vector3* position_std)
{
	SL_GNSS_FUSION_STATUS state = SL_GNSS_FUSION_STATUS_OFF;
	sl::float3 sdk_position_std;
	state = (SL_GNSS_FUSION_STATUS)fusion.getCurrentGNSSCalibrationSTD(*yaw_std, sdk_position_std);

	position_std->x = sdk_position_std.x;
	position_std->y = sdk_position_std.y;
	position_std->z = sdk_position_std.z;

	return state;
}

void ZEDFusionController::getGeoTrackingCalibration(struct SL_Vector3* translation, struct SL_Quaternion* rotation)
{
	sl::Transform transform = fusion.getGeoTrackingCalibration();

	sl::float3 sdk_position = transform.getTranslation();
	translation->x = sdk_position.x;
	translation->y = sdk_position.y;
	translation->z = sdk_position.z;

	sl::float4 sdk_rotation = transform.getOrientation();
	rotation->x = sdk_rotation.x;
	rotation->y = sdk_rotation.y;
	rotation->z = sdk_rotation.z;
	rotation->w = sdk_rotation.w;
}