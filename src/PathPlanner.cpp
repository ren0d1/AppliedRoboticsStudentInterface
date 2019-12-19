#include "PathPlanner.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"

//region algorithms

std::vector<std::pair<cv::Point2d, cv::Point2d>> PathPlanner::exactCellDecomposition(const Polygon& borders, const std::vector<Polygon>& obstacle_list)
{
    if(borders.size() > 1)
    {
        // Retrieve the extreme values of the borders
        double borders_min_X = borders[0].x;
        double borders_min_Y = borders[0].y;
        double borders_max_X = borders[0].x;
        double borders_max_Y = borders[0].y;
        for(int v = 1; v < borders.size(); v++)
        {
            if(borders[v].x < borders_min_X)
                borders_min_X = borders[v].x;

            if(borders[v].y < borders_min_Y)
                borders_min_Y = borders[v].y;

            if(borders[v].x > borders_max_X)
                borders_max_X = borders[v].x;

            if(borders[v].y > borders_max_Y)
                borders_max_Y = borders[v].y;
        }

        std::vector<std::pair<SmartPoint, SmartPoint>> opened_cells_edges{{{{borders_min_X, borders_min_Y}, -1}, {{borders_min_X, borders_max_Y}, -1}}};
        std::vector<Polygon> cells;

        cv::Mat resulting_img = cv::imread("/home/lar2019/workspace/project/images/path_planning/base.jpg");
        double Scale = 512.820496;

        // Compute all rays and assign them to polygons (free cells)
        for(int o = 0; o < obstacle_list.size(); o++)
        {
            double obstacle_min_X = 10000;
            double obstacle_max_X = -10000;
            double obstacle_average_Y = 0;
            //double obstacle_average_Y = obstacle_list[o][0].y;

            bool multi_min = false;
            bool multi_max = false;

            // Retrieve information to define cases
            for(int v = 0; v < obstacle_list[o].size(); v++)
            {


                if(obstacle_list[o][v].x < obstacle_min_X)
                {
                    obstacle_min_X = obstacle_list[o][v].x;
                }else if(obstacle_list[o][v].x == obstacle_min_X){
                    multi_min = true;
                }

                if(obstacle_list[o][v].x > obstacle_max_X)
                {
                    obstacle_max_X = obstacle_list[o][v].x;
                }else if(obstacle_list[o][v].x == obstacle_max_X){
                    multi_max = true;
                }

                obstacle_average_Y += obstacle_list[o][v].y;
            }

            obstacle_average_Y /= obstacle_list[o].size();

            // Compute case and rays for each vertex
            for(int v = 0; v < obstacle_list[o].size(); v++)
            {
                SmartPoint vertex{{obstacle_list[o][v].x, obstacle_list[o][v].y}, o};

                if(!multi_min && vertex.coordinates.x == obstacle_min_X) //2RPL      
                {
                    SmartPoint end_point_up{{0, 0}, -1};
                    bool end_point_up_assigned = false;
                    SmartPoint end_point_down{{0, 0}, -1};
                    bool end_point_down_assigned = false;
                    for(int no = 0; no < obstacle_list.size(); no++)
                    {
                        if(no != o)
                        {
                            cv::Point2d intersection_point_up{};
                            bool has_already_assigned_a_point_up = false;
                            cv::Point2d intersection_point_down{};
                            bool has_already_assigned_a_point_down = false;
                            for(int nv = 0; nv < obstacle_list[no].size() - 1; nv++)
                            {
                                cv::Point2d temp_intersection_point_up{};
                                if(intersection({obstacle_list[o][v].x, obstacle_list[o][v].y}, {obstacle_list[o][v].x, borders_min_Y}, {obstacle_list[no][nv].x, obstacle_list[no][nv].y}, {obstacle_list[no][nv + 1].x, obstacle_list[no][nv + 1].y}, temp_intersection_point_up))
                                {
                                    if(!has_already_assigned_a_point_up)
                                        if(temp_intersection_point_up.y < obstacle_list[o][v].y)
                                        {
                                            intersection_point_up = temp_intersection_point_up;
                                            has_already_assigned_a_point_up = true;
                                        }
                                    else if(intersection_point_up.y < temp_intersection_point_up.y)
                                        if(temp_intersection_point_up.y < obstacle_list[o][v].y)
                                            intersection_point_up = temp_intersection_point_up;
                                }

                                cv::Point2d temp_intersection_point_down{};
                                if(intersection({obstacle_list[o][v].x, obstacle_list[o][v].y}, {obstacle_list[o][v].x, borders_max_Y}, {obstacle_list[no][nv].x, obstacle_list[no][nv].y}, {obstacle_list[no][nv + 1].x, obstacle_list[no][nv + 1].y}, temp_intersection_point_down))
                                {
                                    if(!has_already_assigned_a_point_down)
                                        if(temp_intersection_point_down.y > obstacle_list[o][v].y)
                                        {
                                            intersection_point_down = temp_intersection_point_down;
                                            has_already_assigned_a_point_down = true;
                                        }
                                    else if(intersection_point_down.y < temp_intersection_point_down.y)
                                        if(temp_intersection_point_down.y > obstacle_list[o][v].y)
                                            intersection_point_down = temp_intersection_point_down;
                                }
                            }

                            cv::Point2d temp_intersection_point_up{};
                            if(intersection({obstacle_list[o][v].x, obstacle_list[o][v].y}, {obstacle_list[o][v].x, borders_min_Y}, {obstacle_list[no][obstacle_list[no].size() - 1].x, obstacle_list[no][obstacle_list[no].size() - 1].y}, {obstacle_list[no][0].x, obstacle_list[no][0].y}, temp_intersection_point_up))
                            {
                                if(!has_already_assigned_a_point_up)
                                    if(temp_intersection_point_up.y < obstacle_list[o][v].y)
                                    {
                                        intersection_point_up = temp_intersection_point_up;
                                        has_already_assigned_a_point_up = true;
                                    }
                                else if(intersection_point_up.y < temp_intersection_point_up.y)
                                    if(temp_intersection_point_up.y < obstacle_list[o][v].y)
                                        intersection_point_up = temp_intersection_point_up;
                            }

                            cv::Point2d temp_intersection_point_down{};
                            if(intersection({obstacle_list[o][v].x, obstacle_list[o][v].y}, {obstacle_list[o][v].x, borders_max_Y}, {obstacle_list[no][obstacle_list[no].size() - 1].x, obstacle_list[no][obstacle_list[no].size() - 1].y}, {obstacle_list[no][0].x, obstacle_list[no][0].y}, temp_intersection_point_down))
                            {
                                if(!has_already_assigned_a_point_down)
                                    if(temp_intersection_point_down.y > obstacle_list[o][v].y)
                                    {
                                        intersection_point_down = temp_intersection_point_down;
                                        has_already_assigned_a_point_down = true;
                                    }
                                else if(intersection_point_down.y < temp_intersection_point_down.y)
                                    if(temp_intersection_point_down.y > obstacle_list[o][v].y)
                                        intersection_point_down = temp_intersection_point_down;
                            }

                            if(has_already_assigned_a_point_up)
                            {
                                end_point_up.coordinates = intersection_point_up;
                                end_point_up.obstacle_id = no;
                                has_already_assigned_a_point_up = true;
                            }

                            if(has_already_assigned_a_point_down)
                            {
                                end_point_down.coordinates = intersection_point_down;
                                end_point_down.obstacle_id = no;
                                has_already_assigned_a_point_down = true;
                            }
                        }
                    }

                    if(!end_point_up_assigned)
                        end_point_up.coordinates = {obstacle_list[o][v].x, borders_min_Y};

                    if(!end_point_down_assigned)
                        end_point_down.coordinates = {obstacle_list[o][v].x, borders_max_Y};

                    std::pair<SmartPoint, SmartPoint> up_ray = {vertex, end_point_up};
                    std::pair<SmartPoint, SmartPoint> down_ray = {vertex, end_point_down};

                    int corresponding_up_edge_id = findEdgeId(up_ray, opened_cells_edges, borders_min_Y, borders_max_Y);

                    Polygon cell{};
                    cell.push_back({opened_cells_edges[corresponding_up_edge_id].first.coordinates.x, opened_cells_edges[corresponding_up_edge_id].first.coordinates.y});
                    cell.push_back({opened_cells_edges[corresponding_up_edge_id].second.coordinates.x, opened_cells_edges[corresponding_up_edge_id].second.coordinates.y});
                    cell.push_back({up_ray.first.coordinates.x, up_ray.first.coordinates.y});
                    cell.push_back({up_ray.second.coordinates.x, up_ray.second.coordinates.y});

                    cells.push_back(cell);
                    opened_cells_edges.erase(opened_cells_edges.begin() + corresponding_up_edge_id);

                    int corresponding_down_edge_id = findEdgeId(down_ray, opened_cells_edges, borders_min_Y, borders_max_Y);

                    cell = {};
                    cell.push_back({opened_cells_edges[corresponding_down_edge_id].first.coordinates.x, opened_cells_edges[corresponding_down_edge_id].first.coordinates.y});
                    cell.push_back({opened_cells_edges[corresponding_down_edge_id].second.coordinates.x, opened_cells_edges[corresponding_down_edge_id].second.coordinates.y});
                    cell.push_back({down_ray.first.coordinates.x, down_ray.first.coordinates.y});
                    cell.push_back({down_ray.second.coordinates.x, down_ray.second.coordinates.y});

                    cells.push_back(cell);
                    opened_cells_edges.erase(opened_cells_edges.begin() + corresponding_down_edge_id);

                    std::pair<SmartPoint, SmartPoint> opening_ray = {{{up_ray.first.coordinates.x, up_ray.second.coordinates.y}, up_ray.second.obstacle_id}, {{down_ray.first.coordinates.x, down_ray.second.coordinates.y}, down_ray.second.obstacle_id}};
                    opened_cells_edges.push_back(opening_ray);
                }else if(!multi_max && vertex.coordinates.x == obstacle_max_X) //2RPR
                {
                    SmartPoint end_point_up{{0, 0}, -1};
                    bool end_point_up_assigned = false;
                    SmartPoint end_point_down{{0, 0}, -1};
                    bool end_point_down_assigned = false;
                    for(int no = 0; no < obstacle_list.size(); no++)
                    {
                        if(no != o)
                        {
                            cv::Point2d intersection_point_up{};
                            bool has_already_assigned_a_point_up = false;
                            cv::Point2d intersection_point_down{};
                            bool has_already_assigned_a_point_down = false;
                            for(int nv = 0; nv < obstacle_list[no].size() - 1; nv++)
                            {
                                cv::Point2d temp_intersection_point_up{};
                                if(intersection({obstacle_list[o][v].x, obstacle_list[o][v].y}, {obstacle_list[o][v].x, borders_min_Y}, {obstacle_list[no][nv].x, obstacle_list[no][nv].y}, {obstacle_list[no][nv + 1].x, obstacle_list[no][nv + 1].y}, temp_intersection_point_up))
                                {
                                    if(!has_already_assigned_a_point_up)
                                        if(temp_intersection_point_up.y < obstacle_list[o][v].y)
                                        {
                                            intersection_point_up = temp_intersection_point_up;
                                            has_already_assigned_a_point_up = true;
                                        }
                                    else if(intersection_point_up.y < temp_intersection_point_up.y)
                                        if(temp_intersection_point_up.y < obstacle_list[o][v].y)
                                            intersection_point_up = temp_intersection_point_up;
                                }

                                cv::Point2d temp_intersection_point_down{};
                                if(intersection({obstacle_list[o][v].x, obstacle_list[o][v].y}, {obstacle_list[o][v].x, borders_max_Y}, {obstacle_list[no][nv].x, obstacle_list[no][nv].y}, {obstacle_list[no][nv + 1].x, obstacle_list[no][nv + 1].y}, temp_intersection_point_down))
                                {
                                    if(!has_already_assigned_a_point_down)
                                        if(temp_intersection_point_down.y > obstacle_list[o][v].y)
                                        {
                                            intersection_point_down = temp_intersection_point_down;
                                            has_already_assigned_a_point_down = true;
                                        }
                                    else if(intersection_point_down.y < temp_intersection_point_down.y)
                                        if(temp_intersection_point_down.y > obstacle_list[o][v].y)
                                            intersection_point_down = temp_intersection_point_down;
                                }
                            }

                            cv::Point2d temp_intersection_point_up{};
                            if(intersection({obstacle_list[o][v].x, obstacle_list[o][v].y}, {obstacle_list[o][v].x, borders_min_Y}, {obstacle_list[no][obstacle_list[no].size() - 1].x, obstacle_list[no][obstacle_list[no].size() - 1].y}, {obstacle_list[no][0].x, obstacle_list[no][0].y}, temp_intersection_point_up))
                            {
                                if(!has_already_assigned_a_point_up)
                                    if(temp_intersection_point_up.y < obstacle_list[o][v].y)
                                    {
                                        intersection_point_up = temp_intersection_point_up;
                                        has_already_assigned_a_point_up = true;
                                    }
                                    else if(intersection_point_up.y < temp_intersection_point_up.y)
                                        if(temp_intersection_point_up.y < obstacle_list[o][v].y)
                                            intersection_point_up = temp_intersection_point_up;
                            }

                            cv::Point2d temp_intersection_point_down{};
                            if(intersection({obstacle_list[o][v].x, obstacle_list[o][v].y}, {obstacle_list[o][v].x, borders_max_Y}, {obstacle_list[no][obstacle_list[no].size() - 1].x, obstacle_list[no][obstacle_list[no].size() - 1].y}, {obstacle_list[no][0].x, obstacle_list[no][0].y}, temp_intersection_point_down))
                            {
                                if(!has_already_assigned_a_point_down)
                                    if(temp_intersection_point_down.y > obstacle_list[o][v].y)
                                    {
                                        intersection_point_down = temp_intersection_point_down;
                                        has_already_assigned_a_point_down = true;
                                    }
                                    else if(intersection_point_down.y < temp_intersection_point_down.y)
                                        if(temp_intersection_point_down.y > obstacle_list[o][v].y)
                                            intersection_point_down = temp_intersection_point_down;
                            }

                            if(has_already_assigned_a_point_up)
                            {
                                end_point_up.coordinates = intersection_point_up;
                                end_point_up.obstacle_id = no;
                                end_point_up_assigned = true;
                            }

                            if(has_already_assigned_a_point_down)
                            {
                                end_point_down.coordinates = intersection_point_down;
                                end_point_down.obstacle_id = no;
                                end_point_down_assigned = true;
                            }
                        }
                    }

                    if(!end_point_up_assigned)
                        end_point_up.coordinates = {obstacle_list[o][v].x, borders_min_Y};

                    if(!end_point_down_assigned)
                        end_point_down.coordinates = {obstacle_list[o][v].x, borders_max_Y};

                    std::pair<SmartPoint, SmartPoint> up_ray = {vertex, end_point_up};
                    std::pair<SmartPoint, SmartPoint> down_ray = {vertex, end_point_down};

                    int corresponding_edge_id = findEdgeId(up_ray, opened_cells_edges, borders_min_Y, borders_max_Y);

                    Polygon cell{};
                    cell.push_back({opened_cells_edges[corresponding_edge_id].first.coordinates.x, opened_cells_edges[corresponding_edge_id].first.coordinates.y});
                    cell.push_back({opened_cells_edges[corresponding_edge_id].second.coordinates.x, opened_cells_edges[corresponding_edge_id].second.coordinates.y});
                    cell.push_back({up_ray.second.coordinates.x, up_ray.second.coordinates.y});
                    cell.push_back({down_ray.second.coordinates.x, down_ray.second.coordinates.y});

                    cells.push_back(cell);
                    opened_cells_edges.erase(opened_cells_edges.begin() + corresponding_edge_id);

                    opened_cells_edges.push_back(up_ray);
                    opened_cells_edges.push_back(down_ray);

                    throw std::invalid_argument("Ultra Bullshit");
                }else if(vertex.coordinates.y < obstacle_average_Y) // 1RU
                {
                    SmartPoint end_point_up{{0, 0}, -1};
                    bool end_point_assigned = false;

                    cv::Point2d intersection_point_up{};
                    bool has_already_assigned_a_point = false;

                    for(int no = 0; no < obstacle_list.size(); no++)
                    {
                        if(no != o)
                        {
                            for(int nv = 0; nv < obstacle_list[no].size() - 1; nv++)
                            {
                                cv::Point2d temp_intersection_point_up{};
                                if(intersection({obstacle_list[o][v].x, obstacle_list[o][v].y}, {obstacle_list[o][v].x, borders_min_Y}, {obstacle_list[no][nv].x, obstacle_list[no][nv].y}, {obstacle_list[no][nv + 1].x, obstacle_list[no][nv + 1].y}, temp_intersection_point_up))
                                {
                                    if(!has_already_assigned_a_point){
                                        if(temp_intersection_point_up.y < obstacle_list[o][v].y)
                                        {
                                            intersection_point_up = temp_intersection_point_up;
                                            has_already_assigned_a_point = true;
                                        }
                                    }else if(intersection_point_up.y < temp_intersection_point_up.y){
                                        if(temp_intersection_point_up.y < obstacle_list[o][v].y)
                                        {
                                            intersection_point_up = temp_intersection_point_up;
                                        }
                                    }
                                }
                            }

                            cv::Point2d temp_intersection_point_up{};
                            if(intersection({obstacle_list[o][v].x, obstacle_list[o][v].y}, {obstacle_list[o][v].x, borders_min_Y}, {obstacle_list[no][obstacle_list[no].size() - 1].x, obstacle_list[no][obstacle_list[no].size() - 1].y}, {obstacle_list[no][0].x, obstacle_list[no][0].y}, temp_intersection_point_up))
                            {
                                if(!has_already_assigned_a_point){
                                    if(temp_intersection_point_up.y < obstacle_list[o][v].y)
                                    {
                                        intersection_point_up = temp_intersection_point_up;
                                        has_already_assigned_a_point = true;
                                    }
                                }else if(intersection_point_up.y < temp_intersection_point_up.y){
                                    if(temp_intersection_point_up.y < obstacle_list[o][v].y)
                                    {
                                        intersection_point_up = temp_intersection_point_up;
                                    }
                                }
                            }

                            if(has_already_assigned_a_point)
                            {
                                end_point_up.coordinates = intersection_point_up;
                                end_point_up.obstacle_id = no;
                                end_point_assigned = true;
                            }
                        }
                    }

                    if(!end_point_assigned)
                        end_point_up.coordinates = {obstacle_list[o][v].x, borders_min_Y};

                    std::pair<SmartPoint, SmartPoint> up_ray = {vertex, end_point_up};

                    int corresponding_up_edge_id = findEdgeId(up_ray, opened_cells_edges, borders_min_Y, borders_max_Y);

                    Polygon cell{};
                    cell.push_back({opened_cells_edges[corresponding_up_edge_id].first.coordinates.x, opened_cells_edges[corresponding_up_edge_id].first.coordinates.y});
                    cell.push_back({opened_cells_edges[corresponding_up_edge_id].second.coordinates.x, opened_cells_edges[corresponding_up_edge_id].second.coordinates.y});
                    cell.push_back({up_ray.first.coordinates.x, up_ray.first.coordinates.y});
                    cell.push_back({up_ray.second.coordinates.x, up_ray.second.coordinates.y});

                    cells.push_back(cell);
                    opened_cells_edges.erase(opened_cells_edges.begin() + corresponding_up_edge_id);

                    opened_cells_edges.push_back(up_ray);
                }else if(vertex.coordinates.y > obstacle_average_Y) // 1RD
                {
                    SmartPoint end_point_down{{0, 0}, -1};
                    bool end_point_assigned = false;

                    cv::Point2d intersection_point_down{};
                    bool has_already_assigned_a_point = false;

                    for(int no = 0; no < obstacle_list.size(); no++)
                    {
                        if(no != o)
                        {
                            for(int nv = 0; nv < obstacle_list[no].size() - 1; nv++)
                            {
                                cv::Point2d temp_intersection_point_down{};
                                if(intersection({obstacle_list[o][v].x, obstacle_list[o][v].y}, {obstacle_list[o][v].x, borders_max_Y}, {obstacle_list[no][nv].x, obstacle_list[no][nv].y}, {obstacle_list[no][nv + 1].x, obstacle_list[no][nv + 1].y}, temp_intersection_point_down))
                                {
                                    if(!has_already_assigned_a_point)
                                    {
                                        if(temp_intersection_point_down.y > obstacle_list[o][v].y)
                                        {
                                            intersection_point_down = temp_intersection_point_down;
                                            has_already_assigned_a_point = true;
                                        }
                                    }else if(intersection_point_down.y > temp_intersection_point_down.y){
                                        if(temp_intersection_point_down.y > obstacle_list[o][v].y)
                                        {
                                            intersection_point_down = temp_intersection_point_down;
                                        }
                                    }
                                }
                            }

                            cv::Point2d temp_intersection_point_down{};
                            if(intersection({obstacle_list[o][v].x, obstacle_list[o][v].y}, {obstacle_list[o][v].x, borders_max_Y}, {obstacle_list[no][obstacle_list[no].size() - 1].x, obstacle_list[no][obstacle_list[no].size() - 1].y}, {obstacle_list[no][0].x, obstacle_list[no][0].y}, temp_intersection_point_down))
                            {
                                if(!has_already_assigned_a_point)
                                {
                                    if(temp_intersection_point_down.y > obstacle_list[o][v].y)
                                    {
                                        intersection_point_down = temp_intersection_point_down;
                                        has_already_assigned_a_point = true;
                                    }
                                }else if(intersection_point_down.y > temp_intersection_point_down.y){
                                    if(temp_intersection_point_down.y > obstacle_list[o][v].y)
                                    {
                                        intersection_point_down = temp_intersection_point_down;
                                    }
                                }
                            }

                            if(has_already_assigned_a_point)
                            {
                                end_point_down.coordinates = intersection_point_down;
                                end_point_down.obstacle_id = no;
                                end_point_assigned = true;
                            }
                        }
                    }

                    if(!end_point_assigned)
                        end_point_down.coordinates = {obstacle_list[o][v].x, borders_max_Y};

                    std::pair<SmartPoint, SmartPoint> down_ray = {vertex, end_point_down};

                    int corresponding_down_edge_id = findEdgeId(down_ray, opened_cells_edges, borders_min_Y, borders_max_Y);

                    Polygon cell{};
                    cell.push_back({opened_cells_edges[corresponding_down_edge_id].first.coordinates.x, opened_cells_edges[corresponding_down_edge_id].first.coordinates.y});
                    cell.push_back({opened_cells_edges[corresponding_down_edge_id].second.coordinates.x, opened_cells_edges[corresponding_down_edge_id].second.coordinates.y});
                    cell.push_back({down_ray.first.coordinates.x, down_ray.first.coordinates.y});
                    cell.push_back({down_ray.second.coordinates.x, down_ray.second.coordinates.y});

                    cells.push_back(cell);
                    opened_cells_edges.erase(opened_cells_edges.begin() + corresponding_down_edge_id);

                    opened_cells_edges.push_back(down_ray);
                }
            }
        }

        // Print to check
        for(int c = 0; c < cells.size(); c++)
        {
            cv::line(resulting_img, {cells[c][0].x * Scale, cells[c][0].y * Scale}, {cells[c][1].x * Scale, cells[c][1].y * Scale}, cv::Scalar(180,0,0), 1, 8, 0);
            cv::line(resulting_img, {cells[c][2].x * Scale, cells[c][2].y * Scale}, {cells[c][3].x * Scale, cells[c][3].y * Scale}, cv::Scalar(180,0,0), 1, 8, 0);

            double lx = (cells[c][0].x + cells[c][1].x) / 2;
            double ly = (cells[c][0].y + cells[c][1].y) / 2;

            cv::circle(resulting_img, {lx * Scale, ly * Scale}, 1, cv::Scalar(0,0,180), 1, 8, 0);

            double rx = (cells[c][2].x + cells[c][3].x) / 2;
            double ry = (cells[c][2].y + cells[c][3].y) / 2;

            cv::circle(resulting_img, {rx * Scale, ry * Scale}, 1, cv::Scalar(0,0,180), 1, 8, 0);
            //cv::fillPoly(resulting_img, converted_cell, cv::Scalar(40,190,40));
        }

        std::stringstream ss;
        ss << "/home/lar2019/workspace/project/images/path_planning/";
        std::string folder_path = ss.str();

        std::stringstream img_file;
        img_file << folder_path << "cell_decomp" << ".jpg";
        cv::imwrite(img_file.str(), resulting_img);

        std::cout << "Saved image " << img_file.str() << std::endl;
    }

    return {};
}
//endregion algorithms

//region Helpers

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
bool PathPlanner::intersection(Point o1, Point p1, Point o2, Point p2, cv::Point2d &r)
{
    cv::Point2d converted_o1 = {o1.x, o1.y};
    cv::Point2d converted_p1 = {p1.x, p1.y};
    cv::Point2d converted_o2 = {o2.x, o2.y};
    cv::Point2d converted_p2 = {p2.x, p2.y};

    // Line AB represented as a1x + b1y = c1
    double a1 = converted_p1.y - converted_o1.y;
    double b1 = converted_o1.x - converted_p1.x;
    double c1 = a1*(converted_o1.x) + b1*(converted_o1.y);

    // Line CD represented as a2x + b2y = c2
    double a2 = converted_p2.y - converted_o2.y;
    double b2 = converted_o2.x - converted_p2.x;
    double c2 = a2*(converted_o2.x) + b2*(converted_o2.y);

    double determinant = a1*b2 - a2*b1;

    if (fabs(determinant) > /*EPS*/1e-8)
    {
        double x = (b2*c1 - b1*c2)/determinant;
        double y = (a1*c2 - a2*c1)/determinant;

        if(converted_o2.x > converted_p2.x)
        {
            if(x >= converted_p2.x && x <= converted_o2.x)
            {
                r.x = x;
                r.y = y;

                return true;
            }
        }else{
            if(x >= converted_o2.x && x <= converted_p2.x)
            {
                r.x = x;
                r.y = y;

                return true;
            }
        }
    }

    return false;
}

int PathPlanner::findEdgeId(std::pair<SmartPoint, SmartPoint> ray, std::vector<std::pair<SmartPoint, SmartPoint>> opened_cells_edges, double borders_min_Y, double borders_max_Y)
{
    for(int e = 0; e < opened_cells_edges.size(); e++)
    {
        bool matching_height = opened_cells_edges[e].first.coordinates.y == ray.first.coordinates.y || opened_cells_edges[e].first.coordinates.y == ray.second.coordinates.y || opened_cells_edges[e].second.coordinates.y == ray.second.coordinates.y || opened_cells_edges[e].second.coordinates.y == ray.first.coordinates.y;
        bool is_border = opened_cells_edges[e].first.coordinates.y == borders_min_Y || opened_cells_edges[e].second.coordinates.y == borders_min_Y || opened_cells_edges[e].first.coordinates.y == borders_max_Y || opened_cells_edges[e].second.coordinates.y == borders_max_Y;
        if(matching_height && is_border)
            return e;

        bool is_same_obstacles = ray.first.obstacle_id == (opened_cells_edges[e].first.obstacle_id && ray.second.obstacle_id == opened_cells_edges[e].second.obstacle_id) || (ray.first.obstacle_id == opened_cells_edges[e].second.obstacle_id && ray.second.obstacle_id == opened_cells_edges[e].first.obstacle_id);
        if(is_same_obstacles)
            return e;
    }
}
//endregion Helpers

//region Utility functions (public)

int PathPlanner::planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list,
              const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate,
              const float x, const float y, const float theta, int algorithm)
{
    switch(algorithm)
    {
        case EXACT_CELL:
            exactCellDecomposition(borders, obstacle_list);
            break;

        case MAX_CLEARANCE:
            //max clearance
            break;

        default:
            throw std::invalid_argument("The given algorithm does not exist");
            break;
    }

    return 0;
}
//endregion Utility functions (public)
#pragma GCC diagnostic pop