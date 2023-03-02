#include <line_obstacle/obstacle_layer.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(obstacle_layer_namespace::ObstacleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE; 
// using costmap_2d::FREE_SPACE;
namespace obstacle_layer_namespace
{

ObstacleLayer::ObstacleLayer() {}

void ObstacleLayer::onInitialize()
{
    ros::NodeHandle nh("~/"+ name_);
    current_ = true;

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&ObstacleLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void ObstacleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
    enabled_ = config.enabled;
}

void ObstacleLayer::linetrace(int x0, int y0, int x1, int y1, std::vector<PointInt> &cells)
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

void ObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y)
{
    if(!enabled_)
        return;
    // define two point in front of the robot mark_1 and mark_2
    mark_1.x = robot_x + cos(robot_yaw);
    mark_1.y = robot_y + sin(robot_yaw) - 1.0;
    mark_2.x = mark_1.x;
    mark_2.y = robot_y + sin(robot_yaw) + 1.0;

    *min_x = std::min(*min_x, mark_1.x);
    *min_y = std::min(*min_y, mark_1.y);
    *max_x = std::max(*max_x, mark_2.x);
    *max_y = std::max(*max_y, mark_2.y);
    ROS_INFO("mark_1=(%f,%f) , mark_2=(%f,%f)",mark_1.x,mark_1.y,mark_2.x,mark_2.y);
    ROS_INFO("updateBound valid");
}

void ObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if(!enabled_)
        return;
    std::vector<PointInt> cellsToUpdate;
    unsigned int mx_1;
    unsigned int my_1;
    unsigned int mx_2;
    unsigned int my_2;
    if ((master_grid.worldToMap(mark_1.x, mark_1.y, mx_1, my_1))&&(master_grid.worldToMap(mark_2.x, mark_2.y, mx_2, my_2))){
        ROS_INFO("mx_1=%d, my_1=%d // mx_2=%d, my_2=%d \n worldToMap valid",mx_1, my_1, mx_2, my_2);
        linetrace(mx_1, my_1, mx_2, my_2, cellsToUpdate);
        for(int i =0; i< cellsToUpdate.size(); i++){
            ROS_INFO("cell[%d]=(%d,%d)",i,cellsToUpdate[i].x,cellsToUpdate[i].y);
            master_grid.setCost(cellsToUpdate[i].x, cellsToUpdate[i].y, LETHAL_OBSTACLE);
        }
    }

//end_namespace
}
}