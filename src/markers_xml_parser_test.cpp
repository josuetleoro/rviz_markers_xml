#include "ros/ros.h"
#include "ros/package.h"

#include <iostream>
#include <exception>

#include "rviz_markers_xml/markers_xml_parser.h"
#include "visualization_msgs/MarkerArray.h"

using namespace std;
using namespace RVizVis;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "xml_parser_test_node");
    ros::NodeHandle nh("~");

    string package_path = ros::package::getPath("rviz_markers_xml");
    string markers_file_path = package_path + "/config/markers.xml";
    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("/markers", 10);

    // Parse the xml file and get the marker array
    MarkersXMLParser *markers_xml_parser;
    try
    {
        markers_xml_parser = new MarkersXMLParser(markers_file_path);
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR("Markers XML parser parsing error: %s", ex.what());
    }

    markers_xml_parser->parseShapes();
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers = markers_xml_parser->getMarkers();
    ROS_INFO("Number of markers: %d", (int)marker_array.markers.size());

    ros::Rate rate(10);
    ros::Time curr_time;
    while (ros::ok())
    {
        curr_time = ros::Time::now();
        // update the timestamps
        // for ( auto marker = begin(marker_array.markers); marker != end(marker_array.markers); marker++)
        for (auto & marker : marker_array.markers)
        {
            marker.header.stamp = curr_time;
        }
        marker_array_pub.publish(marker_array);
        rate.sleep();
    }

    return 0;
}