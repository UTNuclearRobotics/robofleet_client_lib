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
    dst._sec = src->sec();
    dst._nsec = src->nsec();
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
struct flatbuffers_type_for<Bool> {
    typedef fb::std_msgs::Bool type;
};
template <>
Bool decode(const fb::std_msgs::Bool* const src) {
    Bool dst;
    dst.data = src->data();
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
* ROS Sensor Messages
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

template <>
struct flatbuffers_type_for<PointField> {
    typedef fb::sensor_msgs::PointField type;
};
template <>
PointField decode(
    const fb::sensor_msgs::PointField* const src) {
    PointField dst;
    dst.name = src->name()->str();
    dst.offset = src->offset();
    dst.datatype = src->datatype();
    dst.count = src->count();
    return dst;
}

template <>
struct flatbuffers_type_for<PointCloud2> {
    typedef fb::sensor_msgs::PointCloud2 type;
};
template <>
PointCloud2 decode(
    const fb::sensor_msgs::PointCloud2* const src) {
    PointCloud2 dst;

    // fill struct
    dst.header = decode<Header>(src->header());
    dst.height = src->height();
    dst.width = src->width();
    dst.is_bigendian = src->is_bigendian();
    dst.point_step = src->point_step();
    dst.row_step = src->row_step();
    dst.is_dense = src->is_dense();

    // fill fields vector
    dst.fields.resize(src->fields()->size());
    auto src2 = src->fields()->begin();
    auto dst2 = dst.fields.begin();
    while (src2 != src->fields()->end()) {
        *dst2 = decode<PointField>(*src2);
        ++src2;
        ++dst2;
    }

    // fill data vector
    dst.data.resize(src->data()->size());
    std::copy(src->data()->begin(), src->data()->end(), dst.data.begin());
    return dst;
}


/*
   ROS Navigation Messages
*/

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


// nav_msgs/MapMetaData
template <>
struct flatbuffers_type_for<MapMetaData> {
    typedef fb::nav_msgs::MapMetaData type;
};
template <>
MapMetaData decode(
    const fb::nav_msgs::MapMetaData* const src) {
    MapMetaData dst;
    dst.map_load_time = decode<Time>(src->map_load_time());
    dst.resolution = src->resolution();
    dst.width = src->width();
    dst.height = src->height();
    dst.origin = decode<Pose>(src->origin());
    return dst;
}

// nav_msgs/OccupancyGrid
template <>
struct flatbuffers_type_for<OccupancyGrid> {
    typedef fb::nav_msgs::OccupancyGrid type;
};
template <>
OccupancyGrid decode(
    const fb::nav_msgs::OccupancyGrid* const src) {
    OccupancyGrid dst;
    dst.header = decode<Header>(src->header());
    dst.info = decode<MapMetaData>(src->info());
    dst.data.resize(src->data()->size());
    std::copy(src->data()->begin(), src->data()->end(), dst.data.begin());
    return dst;
}

/*
// nav_msgs/OccupancyGrid returns a ptr
template <>
struct flatbuffers_type_for<OccupancyGrid*> {
    typedef fb::nav_msgs::OccupancyGrid type;
};
template <>
OccupancyGrid* decode(
    const fb::nav_msgs::OccupancyGrid* const src) {
    OccupancyGrid* dst;
    dst->header = decode<Header>(src->header());
    dst->info = decode<MapMetaData>(src->info());
    dst->data.resize(src->data()->size());
    //std::copy(src->data()->begin(), src->data()->end(), dst.data.begin());
    std::copy(src->data()->begin(), src->data()->end(), dst);
    return dst;
}*/

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

template <>
struct flatbuffers_type_for<BoundingObject3D> {
    typedef fb::augre_msgs::BoundingObject3D type;
};
template <>
BoundingObject3D decode(const fb::augre_msgs::BoundingObject3D* const src) {
    BoundingObject3D dst;
    dst.action = src->action();
    dst.shape = src->shape();
    dst.uid = src->uid()->str();
    dst.size_x = src->size_x();
    dst.size_y = src->size_y();
    dst.size_z = src->size_z();
    dst.radius = src->radius();
    dst.centroid = decode<PoseStamped>(src->centroid());
    return dst;
}

template <>
struct flatbuffers_type_for<BoundingObject3DArray> {
    typedef fb::augre_msgs::BoundingObject3DArray type;
};
template <>
BoundingObject3DArray decode(const fb::augre_msgs::BoundingObject3DArray* const src) {
    BoundingObject3DArray dst;

    dst.objects.resize(src->objects()->size());
    auto src2 = src->objects()->begin();
    auto dst2 = dst.objects.begin();

    while (src2 != src->objects()->end()) {
        *dst2 = decode<BoundingObject3D>(*src2);
        ++src2;
        ++dst2;
    }

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
    dst.anchor_type = src->anchor_type()->str();
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
    dst.ADD = src->add()->str();
    dst.SUBTRACT = src->subtract()->str();
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



/*
 * hri_msgs
 */

 // hri_msgs/Gaze
template <>
struct flatbuffers_type_for<Gaze> {
    typedef fb::hri_msgs::Gaze type;
};
template <>
Gaze decode(
    const fb::hri_msgs::Gaze* const src) {
    Gaze dst;
    dst.header = decode<Header>(src->header());
    dst.sender = src->sender()->str();
    dst.receiver = src->receiver()->str();
    return dst;
}

/// <summary>
/// std_msgs::ColorRGBA
/// </summary>

template <>
struct flatbuffers_type_for<ColorRGBA> {
    typedef fb::std_msgs::ColorRGBA type;
};
template <>
ColorRGBA decode(
    const fb::std_msgs::ColorRGBA* const src) {
    ColorRGBA dst;
    dst.r = src->r();
    dst.g = src->g();
    dst.b = src->b();
    dst.a = src->a();

    return dst;
}

/// <summary>
/// geometry_msgs::Point
/// </summary>

template <>
struct flatbuffers_type_for<Point> {
    typedef fb::geometry_msgs::Point type;
};
template <>
Point decode(
    const fb::geometry_msgs::Point* const src) {
    Point dst;
    dst.x = src->x();
    dst.y = src->y();
    dst.z = src->z();

    return dst;
}


/*
 * visualization_msgs
 */

 // Marker
template <>
struct flatbuffers_type_for<Marker> {
    typedef fb::visualization_msgs::Marker type;
};
template <>
Marker decode(
    const fb::visualization_msgs::Marker* const src) {
    Marker dst;
    dst.header = decode<Header>(src->header());
    dst.ns = src->ns()->str();
    dst.id = src->id();
    dst.type = src->type();
    dst.action = src->action();
    dst.pose = decode<Pose>(src->pose());
    dst.scale.x = src->scale()->x();
    dst.scale.y = src->scale()->y();
    dst.scale.z = src->scale()->z();
    dst.color = decode<ColorRGBA>(src->color());
    dst.frame_locked = src->frame_locked();

    dst.points.resize(src->points()->size());
    auto src2 = src->points()->begin();
    auto dst2 = dst.points.begin();
    while (src2 != src->points()->end()) {
        *dst2 = decode<Point>(*src2);
        ++src2;
        ++dst2;
    }

    dst.colors.resize(src->colors()->size());
    auto src3 = src->colors()->begin();
    auto dst3 = dst.colors.begin();
    while (src3 != src->colors()->end()) {
        *dst3 = decode<ColorRGBA>(*src3);
        ++src3;
        ++dst3;
    }

    dst.text = src->text()->str();

    return dst;
}


// MarkerArray

template <>
struct flatbuffers_type_for<MarkerArray> {
    typedef fb::visualization_msgs::MarkerArray type;
};
template <>
MarkerArray decode(
    const fb::visualization_msgs::MarkerArray* const src) {
    MarkerArray dst;

    dst.markers.resize(src->markers()->size());
    auto src2 = src->markers()->begin();
    auto dst2 = dst.markers.begin();
    while (src2 != src->markers()->end()) {
        *dst2 = decode<Marker>(*src2);
        ++src2;
        ++dst2;
    }

    return dst;
}