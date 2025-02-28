#include <convert_polygon/convert_to_polygons.hpp>

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "convert_polygon");
    convert_polygon::ConvertToPolygonDBSConcaveHull mypolygons;
    return 0;
}