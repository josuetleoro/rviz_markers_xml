#ifndef MARKERS_XML_PARSER_H
#define MARKERS_XML_PARSER_H

#include <string>
#include <vector>
#include <stdexcept>
#include "tinyxml2.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"

using namespace std;
using namespace tinyxml2;

namespace RVizVis
{
    const double deg2rad = 0.017453293;

    class MarkersXMLParser
    {
    private:
        XMLDocument doc;
        XMLNode *root_node;
        XMLElement *element;
        int total_markers_number;
        int curr_marker_number;
        
        vector<visualization_msgs::Marker> markers_vector;
        string ns;

        void throwErrorWithLine(const string &error_str, const int &line_number)
        {
            string error_msg = "Error in line " + to_string(line_number) + ". " + string(error_str);
            throw runtime_error(error_msg);
        }

        visualization_msgs::Marker parseSphere(XMLElement *shape_element);
        visualization_msgs::Marker parseCube(XMLElement *shape_element);
        visualization_msgs::Marker parseLineStrip(XMLElement *shape_element);
        visualization_msgs::Marker parseDashedLine(XMLElement *shape_element);
        visualization_msgs::Marker parseArrow(XMLElement *shape_element);
        visualization_msgs::Marker parseCubeList(XMLElement *shape_element);
        visualization_msgs::Marker parseText(XMLElement *shape_element);
        visualization_msgs::Marker parseMesh(XMLElement *shape_element);

        string parseAttributeNs(XMLElement *shape_element);
        int parseAttributeId(XMLElement *shape_element);
        string parseAttributeFrameId(XMLElement *shape_element);

        geometry_msgs::Pose parseElementPose(XMLElement *shape_element);
        geometry_msgs::Vector3 parseElementScale3D(XMLElement *shape_element);
        double parseElementScale(XMLElement *shape_element);    
        std_msgs::ColorRGBA parseElementColor(XMLElement *shape_element);
        vector<geometry_msgs::Point> parseElementPoints(XMLElement *shape_element);
        vector<geometry_msgs::Point> parseElementDash(XMLElement *shape_element);
        string parseElementText(XMLElement *shape_element);
        string parseElementUri(XMLElement *shape_element);
        
    public:
        MarkersXMLParser(const string &file_path);
        int parseShapes();
        vector<visualization_msgs::Marker> getMarkers()
        {
            return markers_vector;
        }
    };

}; // namespace RVizVis

#endif