#include "rviz_markers_xml/markers_xml_parser.h"
#include "ros/console.h"
#include <vector>
#include <iostream>
#include <fstream>
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include <stdexcept>
#include <string>
#include <sstream>

namespace RVizVis
{
    MarkersXMLParser::MarkersXMLParser(const string &file_path)
    {
        ifstream file(file_path);
        vector<char> buffer((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
        buffer.push_back('\0');

        doc.LoadFile(file_path.c_str());
        if (doc.Error())
        {
            if (doc.ErrorID() == XML_ERROR_FILE_NOT_FOUND)
                throw runtime_error(string("XML error file not found."));
            else
                throwErrorWithLine(doc.ErrorName(), doc.ErrorLineNum());
        }

        root_node = doc.FirstChildElement("markers");
        if (root_node == nullptr)
        {
            throw runtime_error(string("Invalid shapes file. Could not find root note \"makers\"."));
        }
        XMLElement *temp_element = root_node->FirstChildElement("shape");
        if (temp_element == nullptr)
        {
            throw runtime_error("No shape elements found.");
        }

        total_markers_number = 0;
        for (temp_element; temp_element; temp_element = temp_element->NextSiblingElement("shape"))
        {            
            total_markers_number++;
        }
        ROS_INFO("%d shapes found in markers file", total_markers_number);
        curr_marker_number = 0;

        // Read frame id
        temp_element = root_node->FirstChildElement("frame_id");
        if (temp_element == nullptr)
        {
            throw runtime_error("No frame_id element found.");
        }
        frame_id = temp_element->Attribute("value");
        ROS_INFO("Frame id: %s", frame_id.c_str());
        markers_vector.clear();
    }

    int MarkersXMLParser::parseShapes()
    {
        while (curr_marker_number < total_markers_number)
        {
            curr_marker_number++;
            if (curr_marker_number == 1)
            {
                element = root_node->FirstChildElement("shape");
            }
            else
            {
                element = element->NextSiblingElement("shape");
            }

            const char *type = element->Attribute("type");
            if (type == nullptr) 
            {
                string error_msg = "Attribute type missing in shape element.";
                throwErrorWithLine(error_msg, element->GetLineNum());
            }
            string type_str = type;

            if (type_str.compare("sphere") == 0)
            {
                ROS_INFO("Shape of type sphere found with the following parameters:");
                markers_vector.push_back(parseSphere(element));
            }
            else if (type_str.compare("line_strip") == 0)
            {
                ROS_INFO("Shape of type line_strip found with the following parameters:");
                markers_vector.push_back(parseLineStrip(element));
            }
            else if (type_str.compare("dashed_line") == 0)
            {
                ROS_INFO("Shape of type dashed_line found with the following parameters:");
                markers_vector.push_back(parseDashedLine(element));
            }
            else if (type_str.compare("arrow") == 0)
            {
                ROS_INFO("Shape of type arrow found with the following parameters:");
                markers_vector.push_back(parseArrow(element));
            }
            else if (type_str.compare("text") == 0)
            {
                ROS_INFO("Shape of type text found with the following parameters:");
                markers_vector.push_back(parseText(element));
            }
            else
            {
                string error_msg = "Shape element has unknown type.";
                throwErrorWithLine(error_msg, element->GetLineNum());
            }
        }
        return markers_vector.size();
    }

    visualization_msgs::Marker MarkersXMLParser::parseSphere(XMLElement *shape_element)
    {
        visualization_msgs::Marker sphere;
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.header.frame_id = frame_id;

        // Parse ns and id
        sphere.ns = parseAttributeNs(shape_element);
        sphere.id = parseAttributeId(shape_element);

        // Parse pose
       
        sphere.pose = parseElementPose(shape_element);

        // Parse scale
        sphere.scale = parseElementScale3D(shape_element);

        // Parse color
        sphere.color = parseElementColor(shape_element);
        
        return sphere;
    }

    visualization_msgs::Marker MarkersXMLParser::parseLineStrip(XMLElement *shape_element)
    {
        visualization_msgs::Marker line_strip;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.header.frame_id = frame_id;

        // Parse ns and id
        line_strip.ns = parseAttributeNs(shape_element);
        line_strip.id = parseAttributeId(shape_element);

        // Parse pose       
        line_strip.pose = parseElementPose(shape_element);

        // Parse scale
        line_strip.scale.x = parseElementScale(shape_element);

        // Parse color
        line_strip.color = parseElementColor(shape_element);

        // Parse the points
        line_strip.points = parseElementPoints(shape_element);
        
        return line_strip;
    }

    visualization_msgs::Marker MarkersXMLParser::parseDashedLine(XMLElement *shape_element)
    {
        visualization_msgs::Marker line_list;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.header.frame_id = frame_id;

        // Parse ns and id
        line_list.ns = parseAttributeNs(shape_element);
        line_list.id = parseAttributeId(shape_element);

        // Parse pose       
        line_list.pose = parseElementPose(shape_element);

        // Parse scale
        line_list.scale.x = parseElementScale(shape_element);

        // Parse color
        line_list.color = parseElementColor(shape_element);

        // Parse the dash element
        line_list.points = parseElementDash(shape_element);
                
        return line_list;
    }

    visualization_msgs::Marker MarkersXMLParser::parseArrow(XMLElement *shape_element)
    {
        visualization_msgs::Marker arrow;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.header.frame_id = frame_id;

        // Parse ns and id
        arrow.ns = parseAttributeNs(shape_element);
        arrow.id = parseAttributeId(shape_element);

        // Parse color
        arrow.color = parseElementColor(shape_element);

        // Parse scale
        arrow.scale = parseElementScale3D(shape_element);

        // Check the spec type
        const char *spec = shape_element->Attribute("spec");
        if (spec == nullptr)
        {
            string error_msg = "Attribute spec missing in shape element of type arrow.";
            throwErrorWithLine(error_msg, shape_element->GetLineNum());
        }
        string spec_str = spec;
        // Parse the pose or points accordingly
        if (spec_str.compare("pose") == 0)
        {
            // Parse pose
            arrow.pose = parseElementPose(shape_element);
        }
        else if (spec_str.compare("2p") == 0)
        {
            // Parse points
            arrow.points = parseElementPoints(shape_element);
            if (arrow.points.size() > 2)
            {
                string error_msg = "Maximum number of points is two for shape element of type arrow.";
                throwErrorWithLine(error_msg, shape_element->GetLineNum());
            }
        }
        else
        {
            string error_msg = "Invalid spec attribute in shape element of type arrow.";
            throwErrorWithLine(error_msg, shape_element->GetLineNum());
        }
                
        return arrow;
    }

    visualization_msgs::Marker MarkersXMLParser::parseText(XMLElement *shape_element)
    {
        visualization_msgs::Marker text;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.header.frame_id = frame_id;

        // Parse ns and id
        text.ns = parseAttributeNs(shape_element);
        text.id = parseAttributeId(shape_element);

        // Parse pose
        text.pose = parseElementPose(shape_element);

        // Parse scale
        text.scale.z = parseElementScale(shape_element);
        
        // Parse color
        text.color = parseElementColor(shape_element);     

        // Parse text
        text.text = parseElementText(shape_element);
                
        return text;
    }


    string MarkersXMLParser::parseAttributeNs(XMLElement *shape_element)
    {
        const char *ns = shape_element->Attribute("ns");
        if (ns == nullptr)
        {
            string error_msg = "Attribute ns missing in shape element.";
            throwErrorWithLine(error_msg, shape_element->GetLineNum());
        }
        return string(ns);
    }

    int MarkersXMLParser::parseAttributeId(XMLElement *shape_element)
    {
        int id;
        const char *id_str = shape_element->Attribute("id");
        if (id_str == nullptr)
        {
            string error_msg = "Attribute id missing in shape element.";
            throwErrorWithLine(error_msg, shape_element->GetLineNum());
        }
        try
        {
            id = stoi(id_str);
        }
        catch(const std::exception& e)
        {
            string error_msg = "Invalid id value in shape element. Not a number.";
            throwErrorWithLine(error_msg, shape_element->GetLineNum());
        }
        if (id < 0)
        {
            string error_msg = "Invalid id value in shape element. Negative id value.";
            throwErrorWithLine(error_msg, shape_element->GetLineNum());
        }
        return id;
    }

    geometry_msgs::Pose MarkersXMLParser::parseElementPose(XMLElement *shape_element)
    {
        XMLElement *pose_element = shape_element->FirstChildElement("pose");
        if (pose_element == nullptr)
        {
            string error_msg = "Child element pose missing.";
            throwErrorWithLine(error_msg, shape_element->GetLineNum());
        }

        // ******************** Extract xyz attribute ************************* //
        const char *xyz = pose_element->Attribute("xyz");
        if (xyz == nullptr)
        {
            string error_msg = "Attribute xyz missing in pose element.";
            throwErrorWithLine(error_msg, pose_element->GetLineNum());
        }

        // ******************** Extract rpy attribute ************************* //
        const char *rpy = pose_element->Attribute("rpy");
        if (rpy == nullptr)
        {
            string error_msg = "Attribute rpy missing in pose element.";
            throwErrorWithLine(error_msg, pose_element->GetLineNum());
        }

        // ******************** Parse xyz and rpy ************************* //
        geometry_msgs::Pose pose;
        string token;
        std::vector<double> vec;
        // xyz
        int token_number = 0;
        istringstream ss(xyz);
        while (ss)
        {
            getline(ss, token, ' ');
            try
            {
                vec.push_back(stod(token));
                token_number++;
                if (ss.eof())
                {
                    break;
                }
            }
            catch (const std::exception &e)
            {
                string error_msg = "Invalid value for xyz attribute: Not a number.";
                throwErrorWithLine(error_msg, pose_element->GetLineNum());
            }
        }
        if (token_number < 3)
        {
            string error_msg = "Wrong number of elements [" + to_string(token_number) + "] for xyz attribute, expected 3.";
            throwErrorWithLine(error_msg, pose_element->GetLineNum());
        }

        // rpy
        token_number = 0;
        ss = istringstream(rpy);
        while (ss)
        {
            getline(ss, token, ' ');
            try
            {
                vec.push_back(stod(token));
                token_number++;
                if (ss.eof())
                {
                    break;
                }
            }
            catch (const std::exception &e)
            {
                string error_msg = "Invalid value for rpy attribute: Not a number.";
                throwErrorWithLine(error_msg, pose_element->GetLineNum());
            }
        }
        if (token_number < 3)
        {
            string error_msg = "Wrong number of elements [" + to_string(token_number) + "] for rpy attribute, expected 3.";
            throwErrorWithLine(error_msg, pose_element->GetLineNum());
        }

        ROS_INFO("Pose: %.3f %.3f %.3f %.3f %.3f %.3f", vec.at(0), vec.at(1), vec.at(2), vec.at(3), vec.at(4), vec.at(5));

        pose.position.x = vec.at(0);
        pose.position.y = vec.at(1);
        pose.position.z = vec.at(2);
        // Convert to radians
        pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(vec.at(3) * deg2rad, vec.at(4) * deg2rad, vec.at(5) * deg2rad);
        return pose;
    }

    geometry_msgs::Vector3 MarkersXMLParser::parseElementScale3D(XMLElement *shape_element)
    {
        XMLElement *scale_element = shape_element->FirstChildElement("scale3d");
        if (scale_element == nullptr)
        {
            string error_msg = "Child element scale3d missing.";
            throw runtime_error(error_msg);
        }

        // Extract xyz attribute
        const char *xyz = scale_element->Attribute("xyz");
        if (xyz == nullptr)
        {
            string error_msg = "Attribute xyz missing in scale3d element.";
            throwErrorWithLine(error_msg, scale_element->GetLineNum());
        }

        // Parse xyz
        geometry_msgs::Vector3 scale_vec;
        string token;
        std::vector<double> vec;
        // xyz
        int token_number = 0;
        istringstream ss(xyz);
        while (ss)
        {
            getline(ss, token, ' ');
            try
            {
                vec.push_back(stod(token));
                token_number++;
                if (ss.eof())
                {
                    break;
                }
            }
            catch (const std::exception &e)
            {
                string error_msg = "Invalid value for xyz attribute: Not a number.";
                throwErrorWithLine(error_msg, scale_element->GetLineNum());
            }
        }
        if (token_number < 3)
        {
            string error_msg = "Wrong number of elements [" + to_string(token_number) + "] for xyz attribute, expected 3.";
            throwErrorWithLine(error_msg, scale_element->GetLineNum());
        }

        ROS_INFO("Scale: %.3f %.3f %.3f", vec.at(0), vec.at(1), vec.at(2));

        scale_vec.x = vec.at(0);
        scale_vec.y = vec.at(1);
        scale_vec.z = vec.at(2);
        return scale_vec;
    }

    double MarkersXMLParser::parseElementScale(XMLElement *shape_element)
    {
        XMLElement *scale_element = shape_element->FirstChildElement("scale");
        if (scale_element == nullptr)
        {
            string error_msg = "Child element scale missing.";
            throw runtime_error(error_msg);
        }

        // Extract value attribute
        const char *value = scale_element->Attribute("value");
        if (value == nullptr)
        {
            string error_msg = "Attribute value missing in scale element.";
            throwErrorWithLine(error_msg, scale_element->GetLineNum());
        }

        double scale;
        try
        {
            scale = stod(value);
        }
        catch (const std::exception &e)
        {
            string error_msg = "Invalid value attribute: Not a number.";
            throwErrorWithLine(error_msg, scale_element->GetLineNum());
        }
        return scale;
    }

    std_msgs::ColorRGBA MarkersXMLParser::parseElementColor(XMLElement *shape_element)
    {
        XMLElement *color_element = shape_element->FirstChildElement("color");
        if (color_element == nullptr)
        {
            string error_msg = "Child element color missing.";
            throwErrorWithLine(error_msg, shape_element->GetLineNum());
        }

        // Extract rgba attribute
        const char *rgba = color_element->Attribute("rgba");
        if (rgba == nullptr)
        {
            string error_msg = "Attribute rgba missing in color element.";
            throwErrorWithLine(error_msg, color_element->GetLineNum());
        }

        // Parse rgba
        std_msgs::ColorRGBA color;
        string token;
        std::vector<double> vec;
        // xyz
        int token_number = 0;
        istringstream ss(rgba);
        while (ss)
        {
            getline(ss, token, ' ');
            try
            {
                vec.push_back(stod(token));
                token_number++;
                if (ss.eof())
                {
                    break;
                }
            }
            catch (const std::exception &e)
            {
                string error_msg = "Invalid value for rgba attribute: Not a number.";
                throwErrorWithLine(error_msg, color_element->GetLineNum());
            }
            if (vec.at(vec.size()-1) > 1.0 || vec.at(vec.size()-1) < 0.0)
            {
                string error_msg = "Invalid value for rgba attribute: Value out of range. Valid range [0.0, 1.0]";
                throwErrorWithLine(error_msg, color_element->GetLineNum());
            }
        }
        if (token_number < 4)
        {
            string error_msg = "Wrong number of elements [" + to_string(token_number) + "] in rgba attribute, expected 4.";
            throwErrorWithLine(error_msg, color_element->GetLineNum());
        }

        ROS_INFO("Color: %.3f %.3f %.3f %.3f", vec.at(0), vec.at(1), vec.at(2), vec.at(3));

        color.r = vec.at(0);
        color.g = vec.at(1);
        color.b = vec.at(2);
        color.a = vec.at(3);
        return color;
    }

    vector<geometry_msgs::Point> MarkersXMLParser::parseElementPoints(XMLElement *shape_element)
    {
        // Pre check at least two points are given
        XMLElement *point_element = shape_element->FirstChildElement("point");
        if (point_element == nullptr)
        {
            throw runtime_error("No point element found.");
        }
        point_element = point_element->NextSiblingElement("point");
        if (point_element == nullptr)
        {
            string error_msg = "Minimum number of points is two.";
            throwErrorWithLine(error_msg, shape_element->GetLineNum());
        }

        // Parse the points xyz
        vector<geometry_msgs::Point> points;
        point_element = shape_element->FirstChildElement("point"); // Go back to the first point
        for (point_element; point_element; point_element = point_element->NextSiblingElement("point"))
        {
            // Extract xyz attribute
            const char *xyz = point_element->Attribute("xyz");
            if (xyz == nullptr)
            {
                string error_msg = "Attribute xyz missing in point element.";
                throwErrorWithLine(error_msg, point_element->GetLineNum());
            }
            string token;
            std::vector<double> vec;
            // xyz
            int token_number = 0;
            istringstream ss(xyz);
            while (ss)
            {
                getline(ss, token, ' ');
                try
                {
                    vec.push_back(stod(token));
                    token_number++;
                    if (ss.eof())
                    {
                        break;
                    }
                }
                catch (const std::exception &e)
                {
                    string error_msg = "Invalid value for xyz attribute: Not a number.";
                    throwErrorWithLine(error_msg, point_element->GetLineNum());
                }
            }
            if (token_number < 3)
            {
                string error_msg = "Wrong number of elements [" + to_string(token_number) + "] for xyz attribute, expected 3.";
                throwErrorWithLine(error_msg, point_element->GetLineNum());
            }

            ROS_INFO("Point: %.3f %.3f %.3f", vec.at(0), vec.at(1), vec.at(2));
            geometry_msgs::Point point;
            point.x = vec.at(0);
            point.y = vec.at(1);
            point.z = vec.at(2);
            points.push_back(point);
        }
        return points;
    }

    vector<geometry_msgs::Point> MarkersXMLParser::parseElementDash(XMLElement *shape_element)
    {
        XMLElement *dash_element = shape_element->FirstChildElement("dash");
        if (dash_element == nullptr)
        {
            string error_msg = "Child element dash missing in shape element.";
            throwErrorWithLine(error_msg, shape_element->GetLineNum());
        }

        // Get the dash parameters
        int segments;
        double gap_ratio;
        // segments
        const char *segments_str = dash_element->Attribute("segments");
        if (segments_str == nullptr)
        {
            string error_msg = "Attribute segments missing in dash element.";
            throwErrorWithLine(error_msg, dash_element->GetLineNum());
        }   
        try
        {
            segments = stoi(segments_str);
        }
        catch(const std::exception& e)
        {
            string error_msg = "Invalid segments value in dash element. Not a number.";
            throwErrorWithLine(error_msg, dash_element->GetLineNum());
        }
        ROS_INFO("Segments: %d", segments);
        if (segments <= 1)
        {
            string error_msg = "Invalid segments value in shape element, must be greater than 1.";
            throwErrorWithLine(error_msg, dash_element->GetLineNum());
        }
        // gap_ratio
        const char *gap_ratio_str = dash_element->Attribute("gap_ratio");
        if (gap_ratio_str == nullptr)
        {
            string error_msg = "Attribute gap_ratio missing in dash element.";
            throwErrorWithLine(error_msg, dash_element->GetLineNum());
        }
        try
        {
            gap_ratio = stod(gap_ratio_str);
        }
        catch(const std::exception& e)
        {
            string error_msg = "Invalid gap_ratio value in dash element. Not a number.";
            throwErrorWithLine(error_msg, dash_element->GetLineNum());
        }
        ROS_INFO("Gap ratio: %.2f", gap_ratio);
        if (gap_ratio <= 0.0 || gap_ratio >= 1.0)
        {
            string error_msg = "Invalid gap_ratio value in dash element: Value out of range. Valid range (0.0, 1.0)";
            throwErrorWithLine(error_msg, dash_element->GetLineNum());
        }

        // Read the start and end points
        vector<geometry_msgs::Point> boundary_points, points;
        boundary_points = parseElementPoints(dash_element);
        if (boundary_points.size() > 2)
        {
            string error_msg = "Maximum number of points is two for dashed_line shape type.";
            throwErrorWithLine(error_msg, dash_element->GetLineNum());
        }

        // Calculate lenght and direction of the line
        tf::Point p1, p2;
        vector<tf::Point> tf_points;
        tf::Vector3 dir;
        double length;
        p1 = tf::Point(boundary_points.at(0).x, boundary_points.at(0).y, boundary_points.at(0).z);
        p2 = tf::Point(boundary_points.at(1).x, boundary_points.at(1).y, boundary_points.at(1).z);
        dir = p2 - p1;
        length = dir.length();

        if (length < 0.0001)
        {
            string error_msg = "Lenght of dashed_line smaller than 0.0001. Use points with greater distance between them.";
            throwErrorWithLine(error_msg, dash_element->GetLineNum());
        }
        dir.normalize();
        // ROS_INFO("Lenght: %.2f", length);
        // ROS_INFO("Dir: %.2f %.2f %.2f", dir.getX(), dir.getY(), dir.getZ());
        double seg_plus_gap = length / ((double)segments - gap_ratio);
        double gap_length = seg_plus_gap * gap_ratio;
        double seg_length = seg_plus_gap - gap_length;
        // ROS_INFO("seg_plus_gap: %.2f", seg_plus_gap);
        // ROS_INFO("gap_length: %.2f", gap_length);
        // ROS_INFO("seg_length: %.2f", seg_length);

        // Calculate the points in the dashed line
        tf_points.resize(segments * 2);
        tf_points.at(0) = p1;
        tf_points.at(1) = p1 + seg_length * dir;
        for (int i = 2; i < segments * 2; i+=2)
        {
            tf_points.at(i) = tf_points.at(i - 2) + seg_plus_gap * dir;
            tf_points.at(i + 1) = tf_points.at(i - 1) + seg_plus_gap * dir;
        }

        // Convert back to geometry_msgs points
        for (auto it: tf_points)
        {
            geometry_msgs::Point temp_point;
            tf::pointTFToMsg(it, temp_point);
            points.push_back(temp_point);
        }
        return points;
    }

    string MarkersXMLParser::parseElementText(XMLElement *shape_element)
    {
        XMLElement *scale_element = shape_element->FirstChildElement("text");
        if (scale_element == nullptr)
        {
            string error_msg = "Child element text missing.";
            throw runtime_error(error_msg);
        }

        // Extract value attribute
        const char *value = scale_element->Attribute("value");
        if (value == nullptr)
        {
            string error_msg = "Attribute value missing in text element.";
            throwErrorWithLine(error_msg, scale_element->GetLineNum());
        }
       
        return string(value);
    }

}; // namespace RvizVis

