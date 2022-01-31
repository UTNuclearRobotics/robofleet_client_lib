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

// std_msgs/Header
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const Header& msg, const MetadataOffset& metadata) {
    auto header_stamp = fb::RosTime(msg.stamp._sec, msg.stamp._nsec);

    return fb::std_msgs::CreateHeaderDirect(
        fbb, 0, msg.seq, &header_stamp, msg.frame_id.c_str())
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
	return fb::geometry_msgs::CreatePoint(fbb, metadata, msg.x, msg.y, msg.z).o;
}

// geometry_msgs/Quaternion
template <>
flatbuffers::uoffset_t encode(
	FBB& fbb, const Quaternion& msg,
	const MetadataOffset& metadata) {
	return fb::geometry_msgs::CreateQuaternion(
		fbb, metadata, msg.x, msg.y, msg.z, msg.w)
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