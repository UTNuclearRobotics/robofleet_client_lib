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
        fbb, 0, msg.seq, &header_stamp, msg.frame_id.c_str())
        .o;
}

// std::string
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const std::string& msg, const MetadataOffset& metadata) {
    return fbb.CreateString(msg)
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

// amrl_msgs/RobofleetSubscription
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const RobofleetSubscription& msg, const MetadataOffset& metadata) {
  return fb::amrl_msgs::CreateRobofleetSubscriptionDirect(
             fbb, metadata, msg.topic_regex.c_str(), msg.action)
      .o;
}

// amrl_msgs/RobofleetStatus
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const RobotStatus& msg,
    const MetadataOffset& metadata) {
    return fb::amrl_msgs::CreateRobofleetStatusDirect(
        fbb,
        metadata,
        msg.status.c_str(),
        msg.is_ok,
        msg.battery_level,
        msg.location.c_str())
        .o;
}

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


// TODO: nav_msgs/OccupancyGrid

/*
* AugRE Specific Messages
*/

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