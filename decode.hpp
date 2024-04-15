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

static void decode_string_vector(
    const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>* src_vector_ptr, 
    std::vector<std::string>& dst_vector) {
    auto src = src_vector_ptr->begin();
    dst_vector.reserve(src_vector_ptr->size());
    
    while (src != src_vector_ptr->end()) {
        dst_vector.push_back(src->str());
        ++src;
    }
}

template <>
struct flatbuffers_type_for<String> {
    typedef fb::std_msgs::String type;
};
template <>
String decode(const fb::std_msgs::String* const src) {
    String dst;
    dst.data = src->data()->str();
    return dst;
}

// *** specializations below ***

/*
 * AMRL Messages (DEPRECIATED)
 * Note: See AugRE Messages below
 */

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

 // geometry_msgs/Pose
template <>
struct flatbuffers_type_for<Pose> {
    typedef fb::geometry_msgs::Pose type;
};
template <>
Pose decode(
    const fb::geometry_msgs::Pose* const src) {
    Pose dst;
    dst.position.x = src->position()->x();
    dst.position.y = src->position()->y();
    dst.position.z = src->position()->z();
    dst.orientation.x = src->orientation()->x();
    dst.orientation.y = src->orientation()->y();
    dst.orientation.z = src->orientation()->z();
    dst.orientation.w = src->orientation()->w();
    return dst;
}

 // geometry_msgs/PoseStamped
template <>
struct flatbuffers_type_for<PoseStamped> {
    typedef fb::geometry_msgs::PoseStamped type;
};
template <>
PoseStamped decode(
    const fb::geometry_msgs::PoseStamped* const src) {
    PoseStamped dst;
    dst.header = decode<Header>(src->header());
    dst.pose = decode<Pose>(src->pose());
    return dst;
}


// geometry_msgs/PoseWithCovariance
template <>
struct flatbuffers_type_for<PoseWithCovariance> {
    typedef fb::geometry_msgs::PoseWithCovariance type;
};
template <>
PoseWithCovariance decode(
    const fb::geometry_msgs::PoseWithCovariance* const src) {
    PoseWithCovariance dst;
    dst.pose = decode<Pose>(src->pose());
    //decode_vector<double>(src->covariance(), dst.covariance);
    std::copy(src->covariance()->begin(), src->covariance()->end(), dst.covariance.begin());
    return dst;
}



// geometry_msgs/PoseWithCovarianceStamped
template <>
struct flatbuffers_type_for<PoseWithCovarianceStamped> {
    typedef fb::geometry_msgs::PoseWithCovarianceStamped type;
};
template <>
PoseWithCovarianceStamped decode(
    const fb::geometry_msgs::PoseWithCovarianceStamped* const src) {
    PoseWithCovarianceStamped dst;
    dst.header = decode<Header>(src->header());
    dst.pose = decode<PoseWithCovariance>(src->pose());
    return dst;
}


// geometry_msgs/Transform
template <>
struct flatbuffers_type_for<Transform> {
    typedef fb::geometry_msgs::Transform type;
};
template <>
Transform decode(
    const fb::geometry_msgs::Transform* const src) {
    Transform dst;
    dst.translation.x = src->translation()->x();
    dst.translation.y = src->translation()->y();
    dst.translation.z = src->translation()->z();
    dst.rotation.x = src->rotation()->x();
    dst.rotation.y = src->rotation()->y();
    dst.rotation.z = src->rotation()->z();
    dst.rotation.w = src->rotation()->w();
    return dst;
}    


// geometry_msgs/TransformStamped
template <>
struct flatbuffers_type_for<TransformStamped> {
    typedef fb::geometry_msgs::TransformStamped type;
};
template <>
TransformStamped decode(
    const fb::geometry_msgs::TransformStamped* const src) {
    TransformStamped dst;
    dst.header = decode<Header>(src->header());
    dst.child_frame_id = src->child_frame_id()->str();
    dst.transform = decode<Transform>(src->transform());
    return dst;
}

/*
* Geographic Messages
*/

// geographic_Msgs/GeoPose
template <>
struct flatbuffers_type_for<GeoPose> {
    typedef fb::geographic_msgs::GeoPose type;
};
template <>
GeoPose decode(
    const fb::geographic_msgs::GeoPose* const src) {
    GeoPose dst;
    dst.position.latitude = src->position()->latitude();
    dst.position.longitude = src->position()->longitude();
    dst.position.altitude = src->position()->altitude();
    dst.orientation.x = src->orientation()->x();
    dst.orientation.y = src->orientation()->y();
    dst.orientation.z = src->orientation()->z();
    dst.orientation.w = src->orientation()->z();
    return dst;
}

// geographic_Msgs/GeoPoseStamped
template <>
struct flatbuffers_type_for<GeoPoseStamped> {
    typedef fb::geographic_msgs::GeoPoseStamped type;
};
template <>
GeoPoseStamped decode(
    const fb::geographic_msgs::GeoPoseStamped* const src) {
    GeoPoseStamped dst;
    dst.header = decode<Header>(src->header());
    dst.pose = decode<GeoPose>(src->pose());
    return dst;
}


// geographic_Msgs/GeoPoseWithCovariance
template <>
struct flatbuffers_type_for<GeoPoseWithCovariance> {
    typedef fb::geographic_msgs::GeoPoseWithCovariance type;
};
template <>
GeoPoseWithCovariance decode(
    const fb::geographic_msgs::GeoPoseWithCovariance* const src) {
    GeoPoseWithCovariance dst;
    dst.pose = decode<GeoPose>(src->pose());
    std::copy(src->covariance()->begin(), src->covariance()->end(), dst.covariance.begin());
    //decode_vector<double>(src->covariance(), dst.covariance);
    return dst;
}


// geographic_Msgs/GeoPoseWithCovarianceStamped
template <>
struct flatbuffers_type_for<GeoPoseWithCovarianceStamped> {
    typedef fb::geographic_msgs::GeoPoseWithCovarianceStamped type;
};
template <>
GeoPoseWithCovarianceStamped decode(
    const fb::geographic_msgs::GeoPoseWithCovarianceStamped* const src) {
    GeoPoseWithCovarianceStamped dst;
    dst.header = decode<Header>(src->header());
    dst.pose = decode<GeoPoseWithCovariance>(src->pose());
    return dst;
}


/*
* Sensor Messages
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

// TODO -- Replace with new DetectedItem struct
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


/*
 * augre_msgs
 */

template <>
struct flatbuffers_type_for<AgentStatus> {
    typedef fb::augre_msgs::AgentStatus type;
};
template <>
AgentStatus decode(
    const fb::augre_msgs::AgentStatus* const src) {
    AgentStatus dst;
    dst.uid = src->uid()->str();
    dst.callsign = src->callsign()->str();
    dst.agent_type = src->agent_type()->str();
    dst.battery = src->battery();
    dst.commander = src->commander()->str();
    dst.control_status = src->control_status()->str();
    return dst;
}


template <>
struct flatbuffers_type_for<DetectedItem_augre> {
    typedef fb::augre_msgs::DetectedItem type;
};
template <>
DetectedItem_augre decode(const fb::augre_msgs::DetectedItem* const src) {
    DetectedItem_augre dst;
    dst.uid = src->uid()->str();
    dst.callsign = src->callsign()->str();
    dst.type = src->type()->str();
    dst.type_label = src->type_label()->str();
    dst.how = src->how()->str();
    dst.how_label = src->how_label()->str();
    dst.pose = decode<PoseStamped>(src->pose());
    dst.cmpr_image = decode<CompressedImage>(src->cmpr_image());
    dst.url = src->url()->str();
    return dst;
}


template <>
struct flatbuffers_type_for<TransformWithCovarianceStamped> {
    typedef fb::augre_msgs::TransformWithCovarianceStamped type;
};
template <>
TransformWithCovarianceStamped decode(const fb::augre_msgs::TransformWithCovarianceStamped* const src) {
    TransformWithCovarianceStamped dst;
    dst.transform = decode<TransformStamped>(src->transform());
    std::copy(src->covariance()->begin(), src->covariance()->end(), dst.covariance.begin());
    return dst;
}

/*
* GTSAM Specific Messages
*/

// asa_db_portal/AzureSpatialAnchor

// Reference is NRG/Robofleet_client
    // Line 20 - 22 src/common_conversions.hpp
    // Line 7 -15 src/common_conversions.cpp

    // Line 19-31 robotfleet_client/blob/master/scripts/generate/templates/template_msg_impl

template <>
struct flatbuffers_type_for<AzureSpatialAnchor> {
    typedef fb::asa_db_portal::AzureSpatialAnchor type;
};
template <>
AzureSpatialAnchor decode(const fb::asa_db_portal::AzureSpatialAnchor* const src) {
    AzureSpatialAnchor dst;
    dst.asa_id = src->asa_id()->str();
    dst.rep_id = src->rep_id()->str();
    dst.ns = src->ns()->str();
    dst.timestamp = decode<Time>(src->timestamp());
    dst.pose = decode<PoseWithCovarianceStamped>(src->pose());
    dst.geopose = decode<GeoPoseWithCovarianceStamped>(src->geopose());
    //decode_string_vector(src->neighbors(), dst.neighbors);

    dst.neighbors.resize(src->neighbors()->size());
    auto src2 = src->neighbors()->begin();
    auto dst2 = dst.neighbors.begin();

    while (src2 != src->neighbors()->end()) {
        //*dst2 = decode<std::string>(*src2);
        ++src2;
        ++dst2;
    }


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

// nav_msgs/Path
template <>
struct flatbuffers_type_for<Path> {
    typedef fb::nav_msgs::Path type;
};
template <>
Path decode(const fb::nav_msgs::Path* const src) {
    Path dst;
    dst.header = decode<Header>(src->header());
    dst.poses.resize(src->poses()->size());
    auto src3 = src->poses()->begin();
    auto dst3 = dst.poses.begin();
    while (src3 != src->poses()->end()) {
        *dst3 = decode<PoseStamped>(*src3);
        ++src3;
        ++dst3;
    }
    return dst;
}


// tf2_msgs/TFMessage
template <>
struct flatbuffers_type_for<TFMessage> {
    typedef fb::tf2_msgs::TFMessage type;
};
template <>
TFMessage decode(const fb::tf2_msgs::TFMessage* const src) {
    TFMessage dst;
    
    dst.transforms.resize(src->transforms()->size());
    auto src2 = src->transforms()->begin();
    auto dst2 = dst.transforms.begin();
    while (src2 != src->transforms()->end()) {
        *dst2 = decode<TransformStamped>(*src2);
        ++src2;
        ++dst2;
    }

    return dst;
}

// leg_tracker/Detection
template <>
struct flatbuffers_type_for<Detection> {
    typedef fb::leg_tracker::Detection type;
};
template <>
Detection decode(
    const fb::leg_tracker::Detection* const src) {
    Detection dst;
    dst.position.x = src->position()->x();
    dst.position.y = src->position()->y();
    dst.position.z = src->position()->z();
    dst.confidence = src->confidence();
    dst.label = src->label();
    return dst;
}

// leg_tracker/DetectionArray
template <>
struct flatbuffers_type_for<DetectionArray> {
    typedef fb::leg_tracker::DetectionArray type;
};
template <>
DetectionArray decode(
    const fb::leg_tracker::DetectionArray* const src) {
    DetectionArray dst;
    dst.header = decode<Header>(src->header());

    dst.detections.resize(src->detections()->size());
    auto src2 = src->detections()->begin();
    auto dst2 = dst.detections.begin();
    while (src2 != src->detections()->end()) {
        *dst2 = decode<Detection>(*src2);
        ++src2;
        ++dst2;
    }
    return dst;
}

// leg_tracker/Leg
template <>
struct flatbuffers_type_for<Leg> {
    typedef fb::leg_tracker::Leg type;
};
template <>
Leg decode(
    const fb::leg_tracker::Leg* const src) {
    Leg dst;
    dst.position.x = src->position()->x();
    dst.position.y = src->position()->y();
    dst.position.z = src->position()->z();
    dst.confidence = src->confidence();
    return dst;
}

// leg_tracker/LegArray
template <>
struct flatbuffers_type_for<LegArray> {
    typedef fb::leg_tracker::LegArray type;
};
template <>
LegArray decode(
    const fb::leg_tracker::LegArray* const src) {
    LegArray dst;
    dst.header = decode<Header>(src->header());

    dst.legs.resize(src->legs()->size());
    auto src2 = src->legs()->begin();
    auto dst2 = dst.legs.begin();
    while (src2 != src->legs()->end()) {
        *dst2 = decode<Leg>(*src2);
        ++src2;
        ++dst2;
    }
    return dst;
}

// leg_tracker/Person
template <>
struct flatbuffers_type_for<Person> {
    typedef fb::leg_tracker::Person type;
};
template <>
Person decode(
    const fb::leg_tracker::Person* const src) {
    Person dst;
    dst.pose = decode<Pose>(src->pose());
    dst.id = src->id();
    return dst;
}


// leg_tracker/PersonArray
template <>
struct flatbuffers_type_for<PersonArray> {
    typedef fb::leg_tracker::PersonArray type;
};
template <>
PersonArray decode(
    const fb::leg_tracker::PersonArray* const src) {
    PersonArray dst;
    dst.header = decode<Header>(src->header());

    dst.people.resize(src->people()->size());
    auto src2 = src->people()->begin();
    auto dst2 = dst.people.begin();
    while (src2 != src->people()->end()) {
        *dst2 = decode<Person>(*src2);
        ++src2;
        ++dst2;
    }
    return dst;
}



