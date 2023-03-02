#include <area_obstacle/area_layer.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(area_layer_namespace::AreaLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE; 
// using costmap_2d::FREE_SPACE;
namespace area_layer_namespace
{
AreaLayer::AreaLayer() {}

void AreaLayer::onInitialize()
{
    ros::NodeHandle nh("~/"+ name_);
    current_ = true;

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&AreaLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void AreaLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
    enabled_ = config.enabled;
}

void AreaLayer::linetrace(int x0, int y0, int x1, int y1, std::vector<PointInt> &cells)
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    PointInt pt;
    pt.x = x0;
    pt.y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;
    for (; n > 0; --n) {
        cells.push_back(pt);
        if (error > 0.0) {
            pt.x += x_inc;
            error -= dy;
        } else {
            pt.y += y_inc;
            error += dx;
        }
    }
}

void AreaLayer::polygonOutlineCells(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells)
{
    for (unsigned int i = 0; i < polygon.size() - 1; ++i) {
        linetrace(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y, polygon_cells);
    }
    if (!polygon.empty()) {
        unsigned int last_index = polygon.size() - 1;
        // we also need to close the polygon by going from the last point to the first
        linetrace(polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y, polygon_cells);
    }
}

void AreaLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y)
{
    if(!enabled_)
        return;
    // define two point in front of the robot mark_1 and mark_2
    mark_1.x = robot_x + cos(robot_yaw);
    mark_1.y = robot_y + sin(robot_yaw) - 1.0;
    mark_2.x = mark_1.x;
    mark_2.y = robot_y + sin(robot_yaw) + 1.0;
    mark_3.x = robot_x - cos(robot_yaw);
    mark_3.y = robot_y + sin(robot_yaw) + 1.0;
    mark_4.x = robot_x - cos(robot_yaw);
    mark_4.y = robot_y + sin(robot_yaw) - 1.0;

    *min_x = std::min(*min_x, mark_4.x);
    *min_y = std::min(*min_y, mark_1.y);
    *max_x = std::max(*max_x, mark_1.x);
    *max_y = std::max(*max_y, mark_2.y);
    ROS_INFO("updateBound valid");
}

void AreaLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if(!enabled_)
        return;
    unsigned int mx_1, mx_2, mx_3, mx_4;
    unsigned int my_1, my_2, my_3, my_4;
    std::vector<PointInt> cellsToUpdate;
    if ((master_grid.worldToMap(mark_1.x, mark_1.y, mx_1, my_1))&&(master_grid.worldToMap(mark_2.x, mark_2.y, mx_2, my_2))&&(master_grid.worldToMap(mark_3.x, mark_3.y, mx_3, my_3))&&(master_grid.worldToMap(mark_4.x, mark_4.y, mx_4, my_4))){
        const std::vector<PointInt> polygon = {{static_cast<int>(mx_1),static_cast<int>(my_1)},{static_cast<int>(mx_2),static_cast<int>(my_2)},{static_cast<int>(mx_3),static_cast<int>(my_3)},{static_cast<int>(mx_4),static_cast<int>(my_4)}};
        ROS_INFO("WorldToMap valid");
        polygonOutlineCells(polygon , cellsToUpdate);
        for(int i =0; i< cellsToUpdate.size(); i++){
            ROS_INFO("cell[%d]=(%d,%d)",i,cellsToUpdate[i].x,cellsToUpdate[i].y);
            master_grid.setCost(cellsToUpdate[i].x, cellsToUpdate[i].y, LETHAL_OBSTACLE);
        }    
    }
}
//end_namespace
}