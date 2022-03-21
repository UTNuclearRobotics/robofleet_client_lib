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

<<<<<<< Updated upstream
=======
// detection_msgs/DetectedItem 
template <>
struct flatbuffers_type_for<DetectedItem> {
    typedef fb::amrl_msgs::DetectedItem type;
};
template <>
DetectedItem decode(const fb::amrl_msgs::DetectedItem* const src) {
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


// TeMoto

template <>
struct flatbuffers_type_for<UMRFgraphDiff> {
    typedef fb::temoto_action_engine::UmrfGraphDiff type;
};
template <>
UMRFgraphDiff decode(
    const fb::temoto_action_engine::UmrfGraphDiff* const src) {
    UMRFgraphDiff dst;
    dst.ADD = src->ADD()->str();
    dst.SUBTRACT = src->SUBTRACT()->str();
    dst.operation = src->operation()->str();
    dst.umrf_json = src->umrf_json()->str();
    return dst;
}



template <>
struct flatbuffers_type_for<StopUMRF> {
    typedef fb::temoto_action_engine::BroadcastStopUmrfGraph type;
};
template <>
StopUMRF decode(
    const fb::temoto_action_engine::BroadcastStopUmrfGraph* const src) {
    StopUMRF dst;
    dst.umrf_graph_name = src->umrf_graph_name()->str();

    dst.targets.resize(src->targets()->size());
    auto src2 = src->targets()->begin();
    auto dst2 = dst.targets.begin();

    while (src2 != src->targets()->end()) {
        //*dst2 = decode<std::string>(*src2);
        ++src2;
        ++dst2;
    }

    return dst;
}



template <>
struct flatbuffers_type_for<StartUMRF> {
    typedef fb::temoto_action_engine::BroadcastStartUmrfGraph type;
};
template <>
StartUMRF decode(
    const fb::temoto_action_engine::BroadcastStartUmrfGraph* const src) {
    StartUMRF dst;
    dst.umrf_graph_name = src->umrf_graph_name()->str();
    dst.name_match_required = src->name_match_required();

    dst.targets.resize(src->targets()->size());
    auto src2 = src->targets()->begin();
    auto dst2 = dst.targets.begin();

    while (src2 != src->targets()->end()) {
        //*dst2 = decode<std::string>(*src2);
        ++src2;
        ++dst2;
    }

    dst.umrf_graph_json = src->umrf_graph_json()->str();

    dst.umrf_graph_diffs.resize(src->umrf_graph_diffs()->size());
    auto src3 = src->umrf_graph_diffs()->begin();
    auto dst3 = dst.umrf_graph_diffs.begin();
    while (src3 != src->umrf_graph_diffs()->end()) {
        *dst3 = decode<UMRFgraphDiff>(*src3);
        ++src3;
        ++dst3;
    }

    return dst;
}
>>>>>>> Stashed changes
