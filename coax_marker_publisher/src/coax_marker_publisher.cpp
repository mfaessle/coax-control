#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <vicon/Names.h>
#include <vicon/Values.h>
#include <vector>
#include <string>
#include <XMLConfig.h>

using namespace std;


class CoaxMarkerPublisher
{
protected:
	
	ros::Subscriber values_sub;
	//ros::Subscriber names_sub;
	
	ros::Publisher marker1_pub;
	ros::Publisher marker2_pub;
	ros::Publisher marker3_pub;
	ros::Publisher marker4_pub;
	ros::Publisher marker5_pub;
	ros::Publisher stabbar_pub;
	
	bool names_set;
	vector<string> vsk_names;
	vector<float> vsk_values;
	vector<unsigned int> names_index;
	vector<double> markers;
	vector<unsigned int> index;
	XMLConfig config;
	string model;
	string vsk_file;
	
public:
	
	CoaxMarkerPublisher(ros::NodeHandle & n)
	{
		
		//n.param("vsk", vsk_file, string("file.vsk"));
		vsk_file = string("/Users/mfaess/git/CoaX/coax_vsk/vsk/Coax56SB.vsk");
		
		if (ParseVSK(vsk_file, model, vsk_names, vsk_values) != 0)
		{
			ROS_FATAL("%s: Failed to parse vsk file: %s",
					  ros::this_node::getName().c_str(),
					  vsk_file.c_str());
		}
		
		values_sub = n.subscribe("values", 1, &CoaxMarkerPublisher::values_callback, this);
		//names_sub = n.subscribe("names", 1, &CoaxMarkerPublisher::names_callback, this);
		
		marker1_pub = n.advertise<geometry_msgs::Quaternion>("marker1",1);
		marker2_pub = n.advertise<geometry_msgs::Quaternion>("marker2",1);
		marker3_pub = n.advertise<geometry_msgs::Quaternion>("marker3",1);
		marker4_pub = n.advertise<geometry_msgs::Quaternion>("marker4",1);
		marker5_pub = n.advertise<geometry_msgs::Quaternion>("marker5",1);
		stabbar_pub = n.advertise<geometry_msgs::Quaternion>("stabbar",1);
		
		names_set = false;
	}
	~CoaxMarkerPublisher(){
	}
	
	void names_callback(const vicon::Names::ConstPtr &msg)
	{
		if (names_set)
			return;
		
		for (vector<string>::iterator i = vsk_names.begin(); i != vsk_names.end(); ++i) {
			ROS_INFO("loop");
			for (unsigned int j = 0; j < msg->names.size(); j++) {
				string match_name = *i + string(" ");
				if (msg->names[j].find(match_name) != string::npos)
					names_index.push_back(j);
			}
		}
		
		if (4*vsk_names.size() != names_index.size())
		{
			ROS_FATAL("Failed to extract names in vsk file from data");
			ros::shutdown();
		}
		
		names_set = true;
		
		return;
	}
	
	void values_callback(const vicon::Values::ConstPtr &msg)
	{
		//if (!names_set)
		//	return;
		//ROS_INFO("values callback not returned");
		markers = msg->values;
		index = names_index;
		
		geometry_msgs::Quaternion marker1;
		geometry_msgs::Quaternion marker2;
		geometry_msgs::Quaternion marker3;
		geometry_msgs::Quaternion marker4;
		geometry_msgs::Quaternion marker5;
		geometry_msgs::Quaternion stabbar;
		
		marker1.x = markers[1];
		marker1.y = markers[2];
		marker1.z = markers[3];
		if (markers[4] < 0.5) {
			marker1.w = 1; // visible
		} else {
			marker1.w = 0; // not visible
		}
		marker2.x = markers[5];
		marker2.y = markers[6];
		marker2.z = markers[7];
		if (markers[8] < 1e-6) {
			marker2.w = 1; // visible
		} else {
			marker2.w = 0; // not visible
		}
		marker3.x = markers[9];
		marker3.y = markers[10];
		marker3.z = markers[11];
		if (markers[12] < 1e-6) {
			marker3.w = 1; // visible
		} else {
			marker3.w = 0; // not visible
		}
		marker4.x = markers[13];
		marker4.y = markers[14];
		marker4.z = markers[15];
		if (markers[16] < 1e-6) {
			marker4.w = 1; // visible
		} else {
			marker4.w = 0; // not visible
		}
		marker5.x = markers[17];
		marker5.y = markers[18];
		marker5.z = markers[19];
		if (markers[20] < 1e-6) {
			marker5.w = 1; // visible
		} else {
			marker5.w = 0; // not visible
		}
		stabbar.x = markers[21];
		stabbar.y = markers[22];
		stabbar.z = markers[23];
		if (markers[24] < 1e-6) {
			stabbar.w = 1; // visible
		} else {
			stabbar.w = 0; // not visible
		}

		marker1_pub.publish(marker1);
		marker2_pub.publish(marker2);
		marker3_pub.publish(marker3);
		marker4_pub.publish(marker4);
		marker5_pub.publish(marker5);
		stabbar_pub.publish(stabbar);
		
	
		
		return;
	}
	
	int ParseVSK(string &file, string &model,
				 vector<string> &names,
				 vector<float> &values)
	{
		
		if (config.Load(file) != 0)
		{
			ROS_ERROR("%s: Failed to load vsk file: %s",
					  ros::this_node::getName().c_str(),
					  file.c_str());
			return -1;
		}
		
		if (!config.HasElement("/KinematicModel/MarkerSet/Markers/Marker"))
		{
			ROS_ERROR("%s: Failed to locate Markers in vsk file %s",
					  ros::this_node::getName().c_str(),
					  file.c_str());
			return -1;
		}
		
		int i = 0;
		bool model_set = false;
		ROS_DEBUG("%s: Finding markers", ros::this_node::getName().c_str());
		while (true)
		{
			XMLConfig *c =
			config.GetChildrenAsRoot("/KinematicModel/MarkerSet/Markers", i);
			
			if (c == NULL)
				break;
			
			if (!model_set)
			{
				if (!c->HasAttribute("SEGMENT"))
				{
					ROS_ERROR("%s: Failed to locate model name in vsk file %s",
							  ros::this_node::getName().c_str(),
							  file.c_str());
					return -1;
				}
				else
				{
					c->GetAttributeString("SEGMENT", model);
					model_set = true;
				}
			}
			
			if (c->HasAttribute("STATUS"))
			{
				if ((!c->HasAttribute("NAME")) &&
					(!c->HasAttribute("POSITION")))
				{
					ROS_ERROR("%s: Improperly formatted vsk file %s",
							  ros::this_node::getName().c_str(),
							  file.c_str());
					return -1;
				}
				
				string name;
				c->GetAttributeString(string("NAME"), name);
				names.push_back(model + string(":") + name);
				
				float x = c->GetAttributeTupleFloat(string("POSITION"), 0, 0);
				float y = c->GetAttributeTupleFloat(string("POSITION"), 1, 0);
				float z = c->GetAttributeTupleFloat(string("POSITION"), 2, 0);
				
				values.push_back(x);
				values.push_back(y);
				values.push_back(z);
				
				ROS_DEBUG("Name: %s, xyz: %f, %f, %f", name.c_str(), x, y, z);
			}
			
			delete c;
			i++;
		}
		
		return 0;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "coax_marker_publisher");
	
	ros::NodeHandle n("/coax_marker_publisher");
	
	CoaxMarkerPublisher api(n);
	
	
	ros::spin();
	
	return(0);
}
