#ifndef OBSTACLE_LAYER_H_
#define OBSTACLE_LAYER_H_



#include </opt/ros/noetic/include/costmap_2d/costmap_2d.h>
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

namespace obstacle_layer_namespace
{
struct PointInt {
    int x;
    int y;
};
struct Point {
    double x;
    double y;
};

class ObstacleLayer : public costmap_2d::Layer
{
public :
    
    ObstacleLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private :
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
    
    /// \brief             rasterizes line between two map coordinates into a set of cells
    /// \note              since Costmap2D::raytraceLine() is based on the size_x and since we want to rasterize polygons that might also be located outside map bounds we provide a modified raytrace
    ///                    implementation(also Bresenham) based on the integer version presented here : http : //playtechs.blogspot.de/2007/03/raytracing-on-grid.html
    /// \param x0          line start x-coordinate (map frame)
    /// \param y0          line start y-coordinate (map frame)
    /// \param x1          line end x-coordinate (map frame)
    /// \param y1          line end y-coordinate (map frame)
    /// \param[out] cells  new cells in map coordinates are pushed back on this container
    void linetrace(int x0, int y0, int x1, int y1, std::vector<PointInt> &cells);
    Point mark_1; 
    Point mark_2;
    //ros::NodeHandle nh;
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
    };
}
#endif