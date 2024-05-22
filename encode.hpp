#pragma once

#include "message_structs.h"
#include "schema_generated.h"
#include <flatbuffers/flatbuffers.h>
#include <algorithm>


using FBB = flatbuffers::FlatBufferBuilder;
using MetadataOffset = flatbuffers::Offset<fb::MsgMetadata>;

/**
 * @brief Encode a message of type T into a flatbuffer object, placing it
 * into the flatbuffer builder fbb and returning its offset within the builder.
 *
 * This function should be specialized to support encoding various message
 * types T.
 *
 * @tparam T the type to encode
 * @param fbb a Flatbuffer builder to store the encoded message in
 * @param msg the message object to encode
 * @param metadata an optional metadata object, created with encode_metadata().
 * Pass 0 to let the metadata field be null.
 * @return flatbuffers::uoffset_t the offset of the encoded message within the
 * Flatbuffer being built in fbb.
 */
template <typename T>
static flatbuffers::uoffset_t encode(
    FBB& fbb, const T& msg, const MetadataOffset& metadata);

// *** utility functions ***
static flatbuffers::Offset<fb::MsgMetadata> encode_metadata(
    FBB& fbb, const std::string& msg_type, const std::string& topic) {
  return fb::CreateMsgMetadataDirect(fbb, msg_type.c_str(), topic.c_str());
}

/**
 * @brief Create a flatbuffer vector from a vector of items by calling
 * encode<T>() on each item.
 *
 * @tparam TEncoded the target flatbuffers type to encode to
 * @tparam T the source message type to encode from
 * @param fbb the flatbuffer builder in which to create the vector
 * @param metadata an optional metadata item to pass to encode() (pass 0 for
 * null)
 * @param src the vector of messages to encode
 * @return flatbuffers::uoffset_t the offset of the vector within fbb
 */
template <typename TEncoded, typename T>
static flatbuffers::uoffset_t encode_vector(
    FBB& fbb, const MetadataOffset& metadata, std::vector<T> src) {
  std::vector<flatbuffers::Offset<TEncoded>> dst(src.size());
  std::transform(
      src.begin(), src.end(), dst.begin(), [&fbb, &metadata](const T& item) {
        return encode<T>(fbb, item, metadata);
      });
  return fbb.CreateVector(dst).o;
}
// *** specializations below ***


/**
 * @brief Create a vector of flatbuffer Offsets of UMRF diff from a vector of UMRF by calling
 * encode() on each item.
 *
 * @tparam TEncoded the target flatbuffers type to encode to
 * @tparam T the source message type to encode from
 * @param fbb the flatbuffer builder in which to create the vector
 * @param metadata an optional metadata item to pass to encode() (pass 0 for
 * null)
 * @param src the vector of messages to encode
 * @return vector of umrfDiff vos std::vector<Flatbuffers::offset<fb::temoto_action_engine::UMRF>>
 */
template <typename TEncoded, typename T>
static std::vector<flatbuffers::Offset<TEncoded>> encode_vector_umrf(
    FBB& fbb, const MetadataOffset& metadata, std::vector<T> src) {
    std::vector<flatbuffers::Offset<TEncoded>> vo_umrf;
    for (size_t i = 0; i < src.size(); i++) vo_umrf[i] = encode(fbb, src[i], metadata);
    return vo_umrf;
}

// std_msgs/Header
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const Header& msg, const MetadataOffset& metadata) {
    auto header_stamp = fb::RosTime(msg.stamp._sec, msg.stamp._nsec);

    return fb::std_msgs::CreateHeaderDirect(
        fbb, metadata, msg.seq, &header_stamp, msg.frame_id.c_str())
        .o;
}

// std::string
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const std::string& msg, const MetadataOffset& metadata) {
    return fbb.CreateString(msg)
    .o;
}

// uint8_t
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const uint8_t& msg, const MetadataOffset& metadata) {
    return fb::std_msgs::CreateUInt8(fbb, 0, msg)
        .o;
}

// UInt8
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const UInt8& msg, const MetadataOffset& metadata) {
    return fb::std_msgs::CreateUInt8(
        fbb,
        metadata, 
        msg.data)
        .o;
}

//std_msgs/String
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const String& msg, const MetadataOffset& metadata) {
    return fb::std_msgs::CreateStringDirect(
        fbb, metadata, msg.data.c_str())
        .o;
}

// std_msgs/Empty
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const Empty& msg, const MetadataOffset& metadata) {
    return fb::std_msgs::CreateEmpty(fbb, metadata)
        .o;
}

// std_msgs/MultiArrayDimension
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const MultiArrayDimension& msg, const MetadataOffset& metadata) {
    return fb::std_msgs::CreateMultiArrayDimensionDirect(
        fbb,
        metadata,
        msg.label.c_str(),
        msg.size,
        msg.stride)
        .o;
}

// std_msgs/MultiArrayLayout
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const MultiArrayLayout& msg, const MetadataOffset& metadata) {
    auto dim = encode_vector<fb::std_msgs::MultiArrayDimension>(fbb, 0, msg.dim);
    return fb::std_msgs::CreateMultiArrayLayout(
        fbb,
        metadata,
        dim,
        msg.data_offset)
        .o;
}

// std_msgs/UInt8MultiArray
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const UInt8MultiArray& msg, const MetadataOffset& metadata) {
    //auto data = encode_vector<fb::std_msgs::UInt8>(fbb, 0, msg.data);
    return fb::std_msgs::CreateUInt8MultiArrayDirect(
        fbb,
        metadata,
        encode(fbb, msg.layout, 0),
        &msg.data)
        .o;
}

// std_msgs/Bool
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const Bool& msg, const MetadataOffset& metadata) {
    return fb::std_msgs::CreateBool(fbb, metadata, msg.data)
        .o;
}

// amrl_msgs/RobofleetSubscription
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const RobofleetSubscription& msg, const MetadataOffset& metadata) {
  return fb::amrl_msgs::CreateRobofleetSubscriptionDirect(
             fbb, metadata, msg.topic_regex.c_str(), msg.action)
      .o;
}

// TODO delete
// amrl_msgs/Localization2DMsg
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const RobotLocationStamped& msg,
    const MetadataOffset& metadata) {
    auto header = encode(fbb, msg.header, 0);

    auto pose = fb::amrl_msgs::CreatePose2Df(
        fbb, 0, msg.x, msg.y, msg.theta);

    return fb::amrl_msgs::CreateLocalization2DMsgDirect(
        fbb, metadata, header, pose, msg.header.frame_id.c_str())
        .o;
}

// geometry_msgs/Point
template <>
flatbuffers::uoffset_t encode(
	FBB& fbb, const Point& msg, const MetadataOffset& metadata) {
	return fb::geometry_msgs::CreatePoint(
        fbb, 
        metadata, 
        msg.x, 
        msg.y, 
        msg.z)
        .o;
}

// geometry_msgs/Point32
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const Point32& msg, const MetadataOffset& metadata) {
    return fb::geometry_msgs::CreatePoint32(
        fbb,
        metadata,
        msg.x,
        msg.y,
        msg.z)
        .o;
}

// geometry_msgs/Vector3
template<>
flatbuffers::uoffset_t encode(
    FBB& fbb, const Vector3& msg, const MetadataOffset& metadata) {
    return fb::geometry_msgs::CreateVector3(
        fbb,
        metadata,
        msg.x,
        msg.y,
        msg.z)
        .o;
}

//// geometry_msgs/Polygon
//template <>
//flatbuffers::uoffset_t encode(
//    FBB& fbb, const Polygon& msg, const MetadataOffset& metadata) {
//    auto points = encode_vector<fb::geometry_msgs::Point32>(fbb, 0, msg.points);
//    return fb::geometry_msgs::CreatePolygon(
//        fbb,
//        metadata,
//        points)
//        .o;
//}

//// geometry_msgs/PolygonStamped
//template <>
//flatbuffers::uoffset_t encode(
//    FBB& fbb, const PolygonStamped& msg, const MetadataOffset& metadata) {
//    return fb::geometry_msgs::CreatePolygonStamped(
//        fbb,
//        metadata,
//        encode(fbb, msg.header, 0),
//        encode(fbb, msg.polygon, 0))
//        .o;
//}

// geometry_msgs/Quaternion
template <>
flatbuffers::uoffset_t encode(
	FBB& fbb, const Quaternion& msg,
	const MetadataOffset& metadata) {
	return fb::geometry_msgs::CreateQuaternion(
		fbb, 
        metadata, 
        msg.x, 
        msg.y,
        msg.z, 
        msg.w)
		.o;
}

// geometry_msgs/Pose
template <>
flatbuffers::uoffset_t encode(
	FBB& fbb, const Pose& msg, const MetadataOffset& metadata) {
	return fb::geometry_msgs::CreatePose(
		fbb,
		metadata,
		encode(fbb, msg.position, 0),
		encode(fbb, msg.orientation, 0))
		.o;
}

// geometry_msgs/PoseStamped
template <>
flatbuffers::uoffset_t encode(
	FBB& fbb, const PoseStamped& msg,
	const MetadataOffset& metadata) {
	return fb::geometry_msgs::CreatePoseStamped(
		fbb,
		metadata,
		encode(fbb, msg.header, 0),
		encode(fbb, msg.pose, 0))
		.o;
}

// geometry_msgs/PoseWithCovariance
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const PoseWithCovariance& msg,
    const MetadataOffset& metadata) {
    return fb::geometry_msgs::CreatePoseWithCovarianceDirect(
        fbb,
        metadata,
        encode(fbb, msg.pose, 0),
        &msg.covariance)
        .o;
}


// geometry_msgs/PoseWithCovarianceStamped
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const PoseWithCovarianceStamped& msg,
    const MetadataOffset& metadata) {
    return fb::geometry_msgs::CreatePoseWithCovarianceStamped(
        fbb,
        metadata,
        encode(fbb, msg.header, 0),
        encode(fbb, msg.pose, 0))
        .o;
}


// geometry_msgs/Transform
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const Transform& msg,
    const MetadataOffset& metadata) {

    auto vector3 = fb::geometry_msgs::CreateVector3(
        fbb, 0, msg.translation.x, msg.translation.y, msg.translation.z);
    return fb::geometry_msgs::CreateTransform(
        fbb,
        metadata,
        vector3,
        encode(fbb, msg.rotation, 0))
        .o;
}

// geometry_msgs/TransformStamped
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const TransformStamped& msg,
    const MetadataOffset& metadata) {
    return fb::geometry_msgs::CreateTransformStampedDirect(
        fbb,
        metadata,
        encode(fbb, msg.header, 0),
        msg.child_frame_id.c_str(),
        encode(fbb, msg.transform, 0))
        .o;
}

// TF2_msg/TFMessage
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const TFMessage& msg,
    const MetadataOffset& metadata) {
    auto transforms = encode_vector<fb::geometry_msgs::TransformStamped>(fbb, 0, msg.transforms);
    return fb::tf2_msgs::CreateTFMessage(
        fbb,
        metadata,
        transforms)
        .o;
}

// geometry_msgs/Twist
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const Twist& msg,
    const MetadataOffset& metadata) {
    auto vector3_linear = fb::geometry_msgs::CreateVector3(
        fbb, 0, msg.linear.x, msg.linear.y, msg.linear.z);
    auto vector3_angular = fb::geometry_msgs::CreateVector3(
        fbb, 0, msg.angular.x, msg.angular.y, msg.angular.z);
    return fb::geometry_msgs::CreateTwist(
        fbb,
        metadata,
        vector3_linear,
        vector3_angular)
        .o;
}

// geometry_msgs/TwistStamped
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const TwistStamped& msg,
    const MetadataOffset& metadata) {    
    return fb::geometry_msgs::CreateTwistStamped(
        fbb,
        metadata,
        encode(fbb, msg.header, 0),
        encode(fbb, msg.twist, 0))
        .o;
}

// geometry_msgs/TwistWithCovariance
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const TwistWithCovariance& msg,
    const MetadataOffset& metadata) {
    return fb::geometry_msgs::CreateTwistWithCovarianceDirect(
        fbb,
        metadata,
        encode(fbb, msg.twist, 0),
        &msg.covariance)
        .o;
}

// geographic_msgs/GeoPoint
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const GeoPoint& msg, const MetadataOffset& metadata) {
    return fb::geographic_msgs::CreateGeoPoint(
        fbb, 
        metadata, 
        msg.latitude, 
        msg.longitude, 
        msg.altitude)
        .o;
}

// geographic_Msgs/GeoPose
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const GeoPose& msg,
    const MetadataOffset& metadata) {
    return fb::geographic_msgs::CreateGeoPose(
        fbb,
        metadata,
        encode(fbb, msg.position, 0),
        encode(fbb, msg.orientation, 0))
        .o;
}

// geographic_Msgs/GeoPoseWithCovariance
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const GeoPoseWithCovariance& msg,
    const MetadataOffset& metadata) {
    return fb::geographic_msgs::CreateGeoPoseWithCovarianceDirect(
        fbb,
        metadata,
        encode(fbb, msg.pose, 0),
        &msg.covariance)
        .o;
}

// geographic_Msgs/GeoPoseWithCovarianceStamped
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const GeoPoseWithCovarianceStamped& msg,
    const MetadataOffset& metadata) {
    return fb::geographic_msgs::CreateGeoPoseWithCovarianceStamped(
        fbb,
        metadata,
        encode(fbb, msg.header, 0),
        encode(fbb, msg.pose, 0))
        .o;
}

/*
* ROS Navigation Messages
*/

// nav_msgs/Path
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const Path& msg,
    const MetadataOffset& metadata) {
    auto poses = encode_vector<fb::geometry_msgs::PoseStamped>(fbb, 0, msg.poses);
    return fb::nav_msgs::CreatePath(
        fbb,
        metadata,
        encode(fbb, msg.header, 0),
        poses)
        .o;
}

// TODO: nav_msgs/MapMetaData
/*
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const MapMetaData& msg,
    const MetadataOffset& metadata) {
    return fb::nav_msgs::CreateMapMetaData(
        fbb,
        metadata,
        .o;
}
*/

// nav_msgs / Odometry
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const Odometry& msg,
    const MetadataOffset& metadata) {
    
    return fb::nav_msgs::CreateOdometryDirect(
        fbb,
        metadata,
        encode(fbb, msg.header, 0),
        msg.child_frame_id.c_str(),
        encode(fbb, msg.pose, 0),
        encode(fbb, msg.twist, 0))
        .o;
}

/*
* sensor_msgs
*/

// sensor_msgs/Image
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const Image& msg,
    const MetadataOffset& metadata) {
    return fb::sensor_msgs::CreateImageDirect(
        fbb,
        metadata,
        encode(fbb, msg.header, 0),
        msg.height,
        msg.width,
        msg.encoding.c_str(),
        msg.is_bigendian,
        msg.step,
        &msg.data)
        .o;
}

// sensor_msgs/CompressedImage
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const CompressedImage& msg,
    const MetadataOffset& metadata) {
    //auto data = encode_vector<fb::sensor_msgs::CompressedImage>(fbb, 0, msg.data);
    return fb::sensor_msgs::CreateCompressedImageDirect(
        fbb,
        metadata,
        encode(fbb, msg.header, 0),
        msg.format.c_str(),
        &msg.data)
        .o;
}

// sensor_msgs/PointField
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const PointField& msg,
    const MetadataOffset& metadata) {
    return fb::sensor_msgs::CreatePointFieldDirect(
        fbb,
        metadata,
        msg.name.c_str(),
        msg.offset,
        msg.datatype,
        msg.count)
        .o;
}

// sensor_msgs/PointCloud2
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const PointCloud2& msg,
    const MetadataOffset& metadata) {
    auto fields = encode_vector<fb::sensor_msgs::PointField>(fbb, 0, msg.fields);
    auto data = encode_vector<uint8_t>(fbb, 0, msg.data);
    return fb::sensor_msgs::CreatePointCloud2(
        fbb,
        metadata,
        encode(fbb, msg.header, 0),
        msg.height,
        msg.width,
        fields,
        msg.is_bigendian,
        msg.point_step,
        msg.row_step,
        data,
        msg.is_dense)
        .o;
}

// sensor_msgs/Imu
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const Imu& msg,
    const MetadataOffset& metadata) {
    return fb::sensor_msgs::CreateImuDirect(
        fbb,
        metadata,
        encode(fbb, msg.header, 0),
        encode(fbb, msg.orientation, 0),
        &msg.orientation_covariance,
        encode(fbb, msg.angular_velocity,0),
        &msg.angular_velocity_covariance,
        encode(fbb, msg.linear_acceleration, 0),
        &msg.linear_acceleration_covariance)
        .o;
}



// TODO: nav_msgs/OccupancyGrid

/*
* AugRE Specific Messages
*/

// augre_msgs/HeaderArrayStamped
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const HeaderArrayStamped& msg,
    const MetadataOffset& metadata) {
    auto data = encode_vector<fb::augre_msgs::HeaderArrayStamped>(fbb, 0, msg.data);
    return fb::augre_msgs::CreateHeaderArrayStamped(
        fbb,
        metadata,
        encode(fbb, msg.header, 0),
        data)
        .o;
}

// augre_msgs/AgentStatus
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const AgentStatus& msg,
    const MetadataOffset& metadata) {
    return fb::augre_msgs::CreateAgentStatusDirect(
        fbb,
        metadata,
        msg.uid.c_str(),
        msg.callsign.c_str(),
        msg.agent_type.c_str(),
        msg.battery,
        msg.commander.c_str(),
        msg.control_status.c_str())
        .o;
}

// augre_msgs/TransformWithCovarianceStamped
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const TransformWithCovarianceStamped& msg,
    const MetadataOffset& metadata) {
    return fb::augre_msgs::CreateTransformWithCovarianceStampedDirect(
        fbb,
        metadata,
        encode(fbb, msg.transform, 0),
        &msg.covariance)
        .o;
}

// augre_msgs/DetectedItem
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const DetectedItem_augre& msg,
    const MetadataOffset& metadata) {
    return fb::augre_msgs::CreateDetectedItemDirect(
        fbb,
        metadata,
        msg.uid.c_str(),
        msg.callsign.c_str(),
        msg.type.c_str(),
        msg.type_label.c_str(),
        msg.how.c_str(),
        msg.how_label.c_str(),
        encode(fbb, msg.pose, 0),
        encode(fbb, msg.cmpr_image, 0),
        msg.url.c_str())
        .o;
}

// augre_msgs/BoundingObject
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const BoundingObject3D& msg,
    const MetadataOffset& metadata) {
    return fb::augre_msgs::CreateBoundingObject3DDirect(
        fbb,
        metadata,
        msg.action,
        msg.shape,
        msg.uid.c_str(),
        msg.size_x,
        msg.size_y,
        msg.size_z,
        msg.radius,
        encode(fbb, msg.centroid, 0))
        .o;
}

// augre_msgs/BoundingObjects
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const BoundingObject3DArray& msg,
    const MetadataOffset& metadata) {
    auto objects = encode_vector<BoundingObject3D>(fbb, 0, msg.objects);
    return fb::augre_msgs::CreateBoundingObject3DArray(
        fbb,
        metadata,
        objects)
        .o;
}

/*
* GTSAM Specific Messages
*/

// asa_db_portal/AzureSpatialAnchor
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const AzureSpatialAnchor& msg,
    const MetadataOffset& metadata) {
    auto AsaId = fbb.CreateString(msg.asa_id);
    auto RepId = fbb.CreateString(msg.rep_id);
    auto NameSpace = fbb.CreateString(msg.ns);
    auto anchor_type = fbb.CreateString(msg.anchor_type);
    auto time_stamp = fb::RosTime(msg.timestamp._sec, msg.timestamp._nsec);
    auto neighbors = encode_vector<std::string>(fbb, 0, msg.neighbors);
    return fb::asa_db_portal::CreateAzureSpatialAnchor(
        fbb, 
        metadata, 
        AsaId, 
        RepId, 
        NameSpace,
        anchor_type,
        &time_stamp, 
        encode(fbb, msg.pose, 0), 
        encode(fbb, msg.geopose, 0), 
        neighbors).o;
}

/*
*  TeMoto
*/
// temoto_action_engine/UmrfGraphDiff
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const UMRFgraphDiff& msg,
    const MetadataOffset& metadata) {
    return fb::temoto_action_engine::CreateUmrfGraphDiffDirect(
        fbb,
        metadata,
        msg.ADD.c_str(),
        msg.SUBTRACT.c_str(),
        msg.operation.c_str(),
        msg.umrf_json.c_str())
        .o;
}
 
// temoto_action_engine/StartUMRF
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const StartUMRF& msg,
    const MetadataOffset& metadata) {
    auto umrf_graph_name = fbb.CreateString(msg.umrf_graph_name);
    auto targets = encode_vector<std::string>(fbb, 0, msg.targets);
    auto umrf_graph_json = fbb.CreateString(msg.umrf_graph_json);
    auto umrf_diff = encode_vector<fb::temoto_action_engine::UmrfGraphDiff>(fbb, 0, msg.umrf_graph_diffs);
    return fb::temoto_action_engine::CreateBroadcastStartUmrfGraph(
        fbb,
        metadata,
        umrf_graph_name,
        msg.name_match_required,
        targets,
        umrf_graph_json,
        umrf_diff)
        .o;
}
 
// temoto_action_engine/StopUMRF
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const StopUMRF& msg,
    const MetadataOffset& metadata) {
    auto umrf_graph_name = fbb.CreateString(msg.umrf_graph_name);
    auto targets = encode_vector<std::string>(fbb, 0, msg.targets);
    return fb::temoto_action_engine::CreateBroadcastStopUmrfGraph(
        fbb,
        metadata,
        umrf_graph_name,
        targets)
        .o;
}

/*
* hri_msgs
*/

// hri_msgs/Gaze
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const Gaze& msg,
    const MetadataOffset& metadata) {
    return fb::hri_msgs::CreateGazeDirect(
        fbb,
        metadata,
        encode(fbb, msg.header, 0),
        msg.sender.c_str(),
        msg.receiver.c_str())
        .o;
}

/*
* audio_common_msgs
*/

// audio_common_msgs/AudioData
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const AudioData& msg,
    const MetadataOffset& metadata) {
    return fb::audio_common_msgs::CreateAudioDataDirect(
        fbb,
        metadata,
        &msg.data)
        .o;
}

// audio_common_msgs/AudioDataStamped
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const AudioDataStamped& msg,
    const MetadataOffset& metadata) {
    return fb::audio_common_msgs::CreateAudioDataStamped(
        fbb,
        metadata,
        encode(fbb, msg.header, 0),
        encode(fbb, msg.audio, 0))
        .o;
}

// audio_common_msgs/AudioInfo
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const AudioInfo& msg,
    const MetadataOffset& metadata) {
    return fb::audio_common_msgs::CreateAudioInfoDirect(
        fbb,
        metadata,
        msg.channels,
        msg.sample_rate,
        msg.sample_format.c_str(),
        msg.bitrate,
        msg.coding_format.c_str())
        .o;
}

