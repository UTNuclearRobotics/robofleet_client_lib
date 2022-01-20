#pragma once

#include <flatbuffers/flatbuffers.h>
#include "message_structs.h"
#include "schema_generated.h"

// to add a new message type, specialize this template to decode the message
// and...
template <typename Dst, typename Src>
static Dst decode(const Src* const src);
// specialize this struct to map local types to Flatbuffers types
template <typename Type>
struct flatbuffers_type_for {
  typedef void type;
};

// *** utility functions ***
template <typename T, typename Vsrc, typename Vdst>
static void decode_vector(const Vsrc* const src_vector_ptr, Vdst& dst_vector) {
  dst_vector.resize(src_vector_ptr->size());
  auto src = src_vector_ptr->begin();
  auto dst = dst_vector.begin();

  while (src != src_vector_ptr->end()) {
    *dst = decode<T>(*src);
    ++src;
    ++dst;
  }
}

// *** specializations below ***

template <>
struct flatbuffers_type_for<Time> {
    typedef fb::RosTime type;
};
template <>
Time decode(const fb::RosTime* const src) {
    Time dst;
    dst._sec = src->secs();
    dst._nsec = src->nsecs();
    return dst;
}

template <>
struct flatbuffers_type_for<Header> {
    typedef fb::std_msgs::Header type;
};
template <>
Header decode(const fb::std_msgs::Header* const src) {
    Header dst;
    dst.frame_id = src->frame_id()->str();
    dst.seq = src->seq();
    dst.stamp = decode<Time>(src->stamp());
    return dst;
}


template <>
struct flatbuffers_type_for<RobotStatus> {
  typedef fb::amrl_msgs::RobofleetStatus type;
};
template <>
RobotStatus decode(
    const fb::amrl_msgs::RobofleetStatus* const src) {
  RobotStatus dst;
  dst.battery_level = src->battery_level();
  dst.is_ok = src->is_ok();
  dst.location = src->location()->str();
  dst.status = src->status()->str();
  return dst;
}

template <>
struct flatbuffers_type_for<RobotLocation> {
    typedef fb::amrl_msgs::Localization2DMsg type;
};
template <>
RobotLocation decode(
    const fb::amrl_msgs::Localization2DMsg* const src) {
    RobotLocation dst;
    dst.x = src->pose()->x();
    dst.y = src->pose()->y();
    dst.theta = src->pose()->theta();
    dst.frame = src->header()->frame_id()->str();
    return dst;
}

/*
 * geometry_msgs
 */

template <>
struct flatbuffers_type_for<PoseStamped> {
    typedef fb::geometry_msgs::PoseStamped type;
};
template <>
PoseStamped decode(
    const fb::geometry_msgs::PoseStamped* const src) {
    PoseStamped dst;
    dst.pose.point.x = src->pose()->position()->x();
    dst.pose.point.y = src->pose()->position()->y();
    dst.pose.point.z = src->pose()->position()->z();
    dst.pose.quaternion.x = src->pose()->orientation()->x();
    dst.pose.quaternion.y = src->pose()->orientation()->y();
    dst.pose.quaternion.z = src->pose()->orientation()->z();
    dst.pose.quaternion.w = src->pose()->orientation()->z();
    dst.header.frame_id = src->header()->frame_id()->str();
    return dst;
}
/*
template <>
struct flatbuffers_type_for<GeoPoseStamped> {
    typedef fb::geometry_msgs::GeoPoseStamped type;
};
template <>
GeoPoseStamped decode(
    const fb::geometry_msgs::GeoPoseStamped* const src) {
    GeoPoseStamped dst;
    dst.pose.position.latitude = src->pose()->position()->latitude();
    dst.pose.position.longitude = src->pose()->position()->longitude();
    dst.pose.position.altitude = src->pose()->position()->altitude();
    dst.pose.orientation.x = src->pose()->orientation()->x();
    dst.pose.orientation.y = src->pose()->orientation()->y();
    dst.pose.orientation.z = src->pose()->orientation()->z();
    dst.pose.orientation.w = src->pose()->orientation()->z();
    dst.header.frame_id = src->header()->frame_id()->str();
    return dst;
}
*/
template <>
struct flatbuffers_type_for<NavSatFix> {
    typedef fb::sensor_msgs::NavSatFix type;
};
template <>
NavSatFix decode(
    const fb::sensor_msgs::NavSatFix* const src) {
    NavSatFix dst;
    dst.latitude = src->latitude();
    dst.longitude = src->longitude();
    dst.altitude = src->altitude();
    //dst.position_covariance.resize(src->position_covariance()->size());
    std::copy(src->position_covariance()->begin(), src->position_covariance()->end(), dst.position_covariance.begin());
    dst.position_covariance_type = src->position_covariance_type();
    dst.header.frame_id = src->header()->frame_id()->str();
    dst.status.status = src->status()->status();
    dst.status.service = src->status()->service();
    return dst;
}


template <>
struct flatbuffers_type_for<CompressedImage> {
    typedef fb::sensor_msgs::CompressedImage type;
};
template <>
CompressedImage decode(
    const fb::sensor_msgs::CompressedImage* const src) {
    CompressedImage dst;
    dst.header = decode<Header>(src->header());
    dst.data.resize(src->data()->size());
    std::copy(src->data()->begin(), src->data()->end(), dst.data.begin());
    dst.format = src->format()->str();
    return dst;
}

// detection_msgs/DetectedItem 
template <>
struct flatbuffers_type_for<DetectedItem> {
    typedef fb::detection_msgs::DetectedItem type;
};
template <>
DetectedItem decode(const fb::detection_msgs::DetectedItem* const src) {
    DetectedItem dst;
    dst.name = src->name()->str();
    dst.repID = src->repID()->str();
    dst.anchorID = src->anchorID()->str();
    dst.x = src->x();
    dst.y = src->y();
    dst.z = src->z();
    dst.lat = src->lat();
    dst.lon = src->lon();
    dst.elv = src->elv();
    dst.cmpr_image = decode<CompressedImage>(src->cmpr_image());
    return dst;
}