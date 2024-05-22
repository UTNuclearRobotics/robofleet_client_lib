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

struct UInt8 {
	uint8_t data;
};

struct String {
	std::string data;
};

struct Empty {

};

struct MultiArrayDimension {
	std::string label;
	uint32_t size;
	uint32_t stride;
};

struct MultiArrayLayout {
	std::vector<MultiArrayDimension> dim;
	uint32_t data_offset;
};

struct UInt8MultiArray {
	MultiArrayLayout layout;
	std::vector<uint8_t> data;
};

struct Bool {
	bool data;
};

struct ColorRGBA {
	float r;
	float g;
	float b;
	float a;
};

struct RobofleetSubscription {
	std::string topic_regex;
	uint8_t action;
};


// TODO: to delete - 
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

/*
* geometry_msgs
*/

struct Point {
	float x;
	float y;
	float z;
};

struct Point32 {
	float x;
	float y;
	float z;
};

struct Polygon {
	std::vector<Point32> points;
};

//struct PolygonStamped {
//	Header header;
//	Polygon polygon;
//};

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
	//double covariance[36]; // TODO: Swap to a std::vector<float> covariance;
	std::vector<double> covariance;
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
	std::array<double, 9> position_covariance;
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

struct Imu {
	Header header;
	Quaternion orientation;
	std::vector<double> orientation_covariance;
	Vector3 angular_velocity;
	std::vector<double> angular_velocity_covariance;
	Vector3 linear_acceleration;
	std::vector<double> linear_acceleration_covariance;
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

/*
// augre_msgs
*/
struct HeaderArrayStamped {
	Header header;
	std::vector<Header> data;
};

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

struct BoundingObject3D {
	uint8_t action;
	uint8_t shape;
	std::string uid;
	float size_x;
	float size_y;
	float size_z;
	float radius;
	PoseStamped centroid;
};

struct BoundingObject3DArray {
	std::vector<BoundingObject3D> objects;
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

/*
* visualization_msgs
*/

struct Marker {
	Header header;
	std::string ns;
	uint32_t id;
	uint32_t type;
	uint32_t action;
	Pose pose;
	Vector3 scale;
	ColorRGBA color;
	//duration lifetime;
	bool frame_locked;
	std::vector<Point> points;
	std::vector<ColorRGBA> colors;
	std::string text;
	//std::string mesh_resource;
	//bool mesh_use_embedded_materials;
};

struct MarkerArray {
	std::vector<Marker> markers;
};

/*
* audio_common_msgs
*/
struct AudioData {
	std::vector<uint8_t> data;
};

struct AudioDataStamped {
	Header header;
	AudioData audio;
};

struct AudioInfo {
	uint8_t channels;
	uint32_t sample_rate;
	std::string sample_format;
	uint32_t bitrate;
	std::string coding_format;
};