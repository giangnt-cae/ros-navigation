#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include "costvalue.hpp"

namespace vk_costmap_2d {

double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1);

// Check intersection of one point and one polygon
bool intersectPointAndPolygon(std::vector<geometry_msgs::Point>& polygon, float x, float y);

// Check intersection of two polygons
bool intersectPolygonAndPolygon(std::vector<geometry_msgs::Point>& polygon1,
                                std::vector<geometry_msgs::Point>& polygon2);

// Tinh toan khoang cach nho nhat voi gan nhat tu goc (0.0, 0.0) toi footprint
void calculateMinAndMaxDistances(const std::vector<geometry_msgs::Point>& footprint,
                                 double& min_dist, double& max_dist);

geometry_msgs::Point toPoint(geometry_msgs::Point32 pt);

geometry_msgs::Point32 toPoint32(geometry_msgs::Point pt);

geometry_msgs::Polygon toPolygon(std::vector<geometry_msgs::Point> pts);

std::vector<geometry_msgs::Point> toPointVector(geometry_msgs::Polygon polygon);

// Given a pose and base footprint, build the oriented footprint of the robot
void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,
                        std::vector<geometry_msgs::Point>& oriented_footprint);

void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,
                        geometry_msgs::PolygonStamped & oriented_footprint);

// Padding X & Y
void padFootprint(std::vector<geometry_msgs::Point>& footprint, double padding);

void padFootprintX(std::vector<geometry_msgs::Point>& footprint, double padding);

void padFootprintY(std::vector<geometry_msgs::Point>& footprint, double padding);

// Create a circular footprint from a given radius
std::vector<geometry_msgs::Point> makeFootprintFromRadius(double radius, unsigned int N = 16);

std::vector<geometry_msgs::Point> makeFootprintFromWidthAndHeight(double width, double height);

}