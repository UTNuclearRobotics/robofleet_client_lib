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

// geometry_msgs
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
	double covariance[36];
};

struct PoseWithCovarianceStamped {
	Header header;
	PoseWithCovariance pose;
};

struct Twist {
	Vector3 linear;
	Vector3 angular;
};

struct TwistWithCovariance {
	Twist twist;
	double covariance[36];
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

//geographic_msgs
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

//sensor_msgs
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

struct CompressedImage {
	Header header;
	std::string format;
	std::vector<uint8_t> data;
};

// nav_msgs
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
	std::string name;
	float battery;
	std::string owner;
	bool anchor_localization;
	std::string control_status;
};

//struct DetectedItem {
//	std::string name;
//	std::string rep_id;
//	std::string asa_id;
//	PoseStamped pose;
//	GeoPoseStamped geopose;
//	CompressedImage cmpr_image;
//};

struct TransformWithCovarianceStamped {
	TransformStamped transform;
	std::vector<float> covariance;
};

struct tf {
	std::vector<TransformStamped> transforms;
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

