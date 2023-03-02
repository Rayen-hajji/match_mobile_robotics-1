#ifndef AREA_LAYER_H_
#define AREA_LAYER_H_



#include </opt/ros/noetic/include/costmap_2d/costmap_2d.h>
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

namespace area_layer_namespace
{
struct PointInt {
    int x;
    int y;
};
struct Point {
    double x;
    double y;
};

class AreaLayer : public costmap_2d::Layer
{
public :
    
    AreaLayer();

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
    
    /// \brief                     extracts the boundary of a polygon in terms of map cells
    /// \note                      this method is based on Costmap2D::polygonOutlineCells() but accounts for a self - implemented raytrace algorithm and allows negative map coordinates
    /// \param polygon             polygon defined  by a vector of map coordinates
    /// \param[out] polygon_cells  new cells in map coordinates are pushed back on this container
    void polygonOutlineCells(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells);
    
    Point mark_1, mark_2, mark_3, mark_4; 
    
    //ros::NodeHandle nh;
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
    };
}
#endif
