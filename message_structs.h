#pragma once
#include<cstdint>
#include<string>
#include<vector>
#include<array>

/*
 * ROS message clones
 */
 // std_msgs
struct Time {
	uint32_t _sec;
	uint32_t _nsec;
};

struct Header {
	uint32_t seq;
	Time stamp;
	std::string frame_id;
};

struct String {
	std::string data;
};

struct Empty {

};

struct RobofleetSubscription {
	std::string topic_regex;
	uint8_t action;
};

struct RobotLocation {
	std::string frame;
	float x;
	float y;
	float z;
	float theta;
};

struct RobotLocationStamped {
	Header header;
	float x;
	float y;
	float z;
	float theta;
};

struct RobotStatus {
	std::string status;
	bool is_ok;
	float battery_level;
	std::string location;
};


/*
* geometry_msgs
*/

struct Point {
	float x;
	float y;
	float z;
};

struct Quaternion {
	float x;
	float y;
	float z;
	float w;
};

struct Vector3 {
	double x;
	double y;
	double z;
};

struct Pose {
	Point position;
	Quaternion orientation;
};

struct Pose2D {
	float x;
	float y;
	float yaw;
};

struct PoseStamped {
	Header header;
	Pose pose;
};

struct PoseWithCovariance {
	Pose pose;
	std::vector<double> covariance; // TODO: Swap to a std::vector<float> covariance;
};

struct PoseWithCovarianceStamped {
	Header header;
	PoseWithCovariance pose;
};

struct Twist {
	Vector3 linear;
	Vector3 angular;
};

struct TwistStamped {
	Header header;
	Twist twist;
};

struct TwistWithCovariance {
	Twist twist;
	double covariance[36]; // TODO: Swap to a std::vector<float> covariance;
};

struct TwistWithCovarianceStamped {
	Header header;
	TwistWithCovariance twist;
};

struct Transform {
	Vector3 translation;
	Quaternion rotation;
};

struct TransformStamped {
	Header header;
	std::string child_frame_id;
	Transform transform;
};

/*
* tf2_msgs
*/

struct TFMessage {
	std::vector<TransformStamped> transforms;
};


/*
* geographic_msgs
*/

struct GeoPoint {
	double latitude;
	double longitude;
	double altitude;
};

struct GeoPose {
	GeoPoint position;
	Quaternion orientation;
};

struct GeoPoseStamped {
	Header header;
	GeoPose pose;
};

struct GeoPoseWithCovariance {
	GeoPose pose;
	std::vector<double> covariance;
};

struct GeoPoseWithCovarianceStamped {
	Header header;
	GeoPoseWithCovariance pose;
};

/*
* sensor_msgs
*/

struct NavSatStatus {
	int8 status;
	uint16 service;
};

struct NavSatFix {
	Header header;
	NavSatStatus status;
	double latitude;
	double longitude;
	double altitude;
	//std::vector<double> position_covariance;
	std::array<double,9> position_covariance;
	uint8 position_covariance_type;
};

struct Image {
	Header header;
	uint32 height;
	uint32 width;
	std::string encoding;
	uint8 is_bigendian;
	uint32 step;
	std::vector<uint8_t> data;
};

struct CompressedImage {
	Header header;
	std::string format;
	std::vector<uint8_t> data;
};

struct PointField {
	std::string name;
	uint32 offset;
	uint8 datatype;
	uint32 count;
};

struct PointCloud2 {
	Header header;
	uint32 height;
	uint32 width;
	std::vector<PointField> fields;
	bool is_bigendian;
	uint32 point_step;
	uint32 row_step;
	std::vector<uint8_t> data;
	bool is_dense;
};

/*
* nav_msgs
*/

struct Odometry {
	Header header;
	std::string child_frame_id;
	PoseWithCovariance pose;
	TwistWithCovariance twist;
};

struct Path {
	Header header;
	std::vector<PoseStamped> poses;
};

struct MapMetaData {
	Time map_load_time;
	float resolution;
	uint32_t width;
	uint32_t height;
	Pose origin;
};

struct OccupancyGrid {
	Header header;
	MapMetaData info;
	std::vector<int8_t> data;
};

// modified atak dectection_msgs 
struct DetectedItem {
	std::string name;
	std::string repID;
	std::string anchorID;
	float x;
	float y;
	float z;
	float lat;
	float lon;
	float elv;
	CompressedImage cmpr_image;
};

/*
// augre_msgs
*/
struct AgentStatus {
	std::string uid;
	std::string callsign;
	std::string agent_type;
	float battery;
	std::string commander;
	std::string control_status;
};

// TODO: FIX STRUCT NAME
struct DetectedItem_augre {
	std::string uid;
	std::string callsign;
	std::string type;
	std::string type_label;
	std::string how;
	std::string how_label;
	PoseStamped pose;
	CompressedImage cmpr_image;
	std::string url;
};

struct TransformWithCovarianceStamped {
	TransformStamped transform;
	std::vector<float> covariance;
};

/*
* asa_db_portal msgs
*/

struct AzureSpatialAnchor {
	std::string asa_id;
	std::string rep_id;
	std::string ns;
	std::string anchor_type;
	Time timestamp;
	PoseWithCovarianceStamped pose;
	GeoPoseWithCovarianceStamped geopose;
	std::vector<std::string> neighbors;
};

/*
 *  TeMoto_msgs
 */

// UMRFgraphs
struct UMRFgraphDiff {
	std::string ADD;
	std::string SUBTRACT = "subtract";
	std::string operation;
	std::string umrf_json;
};

struct StartUMRF {
	std::string umrf_graph_name;
	bool name_match_required;
	std::vector<std::string> targets;
	std::string umrf_graph_json;
	std::vector<UMRFgraphDiff> umrf_graph_diffs;
};

struct StopUMRF {
	std::string umrf_graph_name;
	std::vector<std::string> targets;
};

/*
 *  leg_tracker
 */

// Detection
struct Detection {
	Point position;
	float confidence;
	uint32_t label;
};

// Detection Array
struct DetectionArray {
	Header header;
	std::vector<Detection> detections;
};

// Leg
struct Leg {
	Point position;
	float confidence;
};

// Leg Array
struct LegArray {
	Header header;
	std::vector<Leg> legs;
};

// Person
struct Person {
	Pose pose;
	uint32_t id;
};

// Leg Array
struct PersonArray {
	Header header;
	std::vector<Person> people;
};

/*
* hri_msgs
*/

// hrs_msgs/Gaze
struct Gaze {
	Header header;
	std::string sender;
	std::string receiver;
};
