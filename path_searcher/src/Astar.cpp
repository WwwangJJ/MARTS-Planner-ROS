#include "Astar.h"
#include <functional>
#include <math.h>
using namespace std;
using namespace Eigen;

//TODO 1算法初始化
std::function<double(Eigen::Vector3d, Eigen::Vector3d)>Distance;

/***
 *@函数功能   计算两节点间的欧式距离
 ------------------------------------------------
 *@参数       node1_coord    节点1
 *@参数       node2_coord    节点2
 ------------------------------------------------
 *@返回值     欧式距离值：根号√((x2-x1)^2+(y2-y1)^2)
 */
double EuclideanDistance(Eigen::Vector3d node1_coord,Eigen::Vector3d node2_coord)
{
    double h= std::sqrt(std::pow((node1_coord(0) - node2_coord(0)), 2 ) +
              std::pow((node1_coord(1) - node2_coord(1)), 2 ) +
              std::pow((node1_coord(2) - node2_coord(2)), 2 ));
    return h;
}

/***
*@函数功能   计算两点间的曼哈顿距离
------------------------------------------------
*@参数       node1_coord    节点1
*@参数       node2_coord    节点2
------------------------------------------------
*@返回值    曼哈顿距离值：|(x2-x1)+(y2-y1)|
*/
double ManhattanDistance(Eigen::Vector3d node1_coord,Eigen::Vector3d node2_coord)
{
   double h = std::abs(node1_coord(0) - node2_coord(0) ) +
            std::abs(node1_coord(1) - node2_coord(1) ) +
            std::abs(node1_coord(2) - node2_coord(2) );
    return h;
}

/****************构造函数*****************/
/*
参数：
_distance           Astar算法的启发距离函数
_weight_a           Astar算法的权重a
_weight_b           Astar算法的权重a
_weight_angle       Astar算法的权重angle
_length             绳子的长度
_max_angle          绳子的最大角度
_min_angle          绳子的最小角度
_angle_resolution   绳子角度遍历的分辨率
*/
AstarPathFinder::AstarPathFinder(std::string _distance,double _weight_a,double _weight_b,double _weight_angle,double _length,double _max_angle,double _min_angle,int _angle_resolution):distance(_distance),weight_a(_weight_a),weight_b(_weight_b),weight_angle(_weight_angle),length(_length),max_angle(_max_angle),min_angle(_min_angle),angle_resolution(_angle_resolution)
{
   if (distance=="euclidean")
   {   
       std::cout<<"use euclidean distance"<<std::endl;
       Distance=EuclideanDistance;
   }
   else if (distance=="manhattan")
   {
       std::cout<<"use manhattan distance"<<std::endl;
       Distance=ManhattanDistance;
   }
   std::cout<<"weight a:"<<weight_a<<" weight b:"<<weight_b<<" weight angle:"<<weight_angle<<" length:"<<length<<" max_angle:"<<max_angle<<"PI"<<" min_angle:"<<min_angle<<"PI"<<" angle_resolution:"<<angle_resolution<<std::endl;
};

//TODO 2地图相关操作
/****************初始化点云地图*****************/
/*
参数：
_resolution    分辨率
global_xyz_l   世界坐标系下点云地图三轴最小尺寸
global_xyz_u   世界坐标系下点云地图三轴最大尺寸
max_x_id       栅格坐标系(与世界坐标系相差分辨率)下，整个点云地图的宽
max_y_id       栅格坐标系(与世界坐标系相差分辨率)下，整个点云地图的长
max_z_id       栅格坐标系(与世界坐标系相差分辨率)下，整个点云地图的高
*/
void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u,Eigen::Vector3d search_xyz_l, Eigen::Vector3d search_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);  //世界坐标系下点云地图x轴最小尺寸
    gl_yl = global_xyz_l(1);  //世界坐标系下点云地图y轴最小尺寸
    gl_zl = global_xyz_l(2);  //世界坐标系下点云地图z轴最小尺寸

    gl_xu = global_xyz_u(0);  //世界坐标系下点云地图x轴最大尺寸
    gl_yu = global_xyz_u(1);  //世界坐标系下点云地图y轴最大尺寸
    gl_zu = global_xyz_u(2);  //世界坐标系下点云地图z轴最大尺寸

    search_x_min = search_xyz_l(0);
    search_y_min = search_xyz_l(1);
    search_z_min = search_xyz_l(2);

    search_x_max = search_xyz_u(0);
    search_y_max = search_xyz_u(1);
    search_z_max = search_xyz_u(2);

    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;

    std::cout<< "-------------------------------------------------------------" << std::endl;
    std::cout<< "-------------------------------------------------------------" << std::endl;
    std::cout<< "Map X Min: "       << gl_xl        << " " << "Map X Max: "    << gl_xu        << std::endl;
    std::cout<< "Map Y Min: "       << gl_yl        << " " << "Map Y Max: "    << gl_yu        << std::endl;
    std::cout<< "Map Z Min: "       << gl_zl        << " " << "Map Z Max: "    << gl_zu        << std::endl;
    std::cout<< "Search X Min: "    << search_x_min << " " << "Search X Max: " << search_x_max << std::endl;
    std::cout<< "Search Y Min: "    << search_y_min << " " << "Search Y Max: " << search_y_max << std::endl;
    std::cout<< "Search Z Min: "    << search_z_min << " " << "Search Z Max: " << search_z_max << std::endl;
    std::cout<< "Resolution:"       << resolution   << std::endl;
    std::cout<< "-------------------------------------------------------------" << std::endl;
    std::cout<< "-------------------------------------------------------------" << std::endl;

    /*自己更改的地图形式*/
    data.resize(GLXYZ_SIZE * sizeof(uint8_t),0); //构建一维点云栅格数据存储格式Z->Y->X  0表示空闲，1表示障碍物 
    GridNodeMap.resize(GLX_SIZE);
    for (int i=0;i<GLX_SIZE;i++)
    {
        GridNodeMap[i].resize(GLY_SIZE);
        for (int j=0;j<GLY_SIZE;j++)
        {
            GridNodeMap[i][j].resize(GLZ_SIZE);
            for (int k=0;k<GLZ_SIZE;k++)
            {
                Vector3i tmpIdx(i,j,k);     //栅格索引 
                Vector3d pos = gridIndex2coord(tmpIdx);  //该栅格对应的世界坐标系下的点云位置坐标
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
    /*原先形式*/
/*
    data = new uint8_t[GLXYZ_SIZE];    //构建一维点云栅格数据存储格式Z->Y->X  0表示空闲，1表示障碍物 
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++)
    {
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++)
        {
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++)
            {
                Vector3i tmpIdx(i,j,k);     //栅格索引 
                Vector3d pos = gridIndex2coord(tmpIdx);  //该栅格对应的世界坐标系下的点云位置坐标
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
*/
}

void AstarPathFinder::clearObs()
{
    for (auto& element : data)
    {
        element = 0;
    }
}

/****************设置一维点云数据格式中的障碍物的位置*****************/
/*
参数：
coord_x       世界坐标系下，某个障碍物点云的x值
coord_y       世界坐标系下，某个障碍物点云的y值
coord_z       世界坐标系下，某个障碍物点云的z值
*/
void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    //安全性校验
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;
    //世界坐标系-->栅格坐标系
    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      
    //数据是按照Z->Y->X方向进行数据排序的 1表示该位置为障碍物
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

/****************栅格转世界*****************/
/*
参数：
index    栅格索引
返回值：  该栅格所定应的世界坐标系下点云位置坐标
*/
Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

/****************世界转栅格*****************/
/*
参数：
pt       世界坐标系下点云索引
返回值：  对应的栅格坐标系的栅格索引
*/
Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    /*
        0<=(pt(0) - gl_xl) * inv_resolution)<=GLX_SIZE - 1
        0<=(pt(1) - gl_yl) * inv_resolution)<=GLY_SIZE - 1
        0<=(pt(2) - gl_zl) * inv_resolution)<=GLZ_SIZE - 1
    */
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}


//TODO 3 Astar本体实现
/****************Astar的核心函数*****************/
/*
参数：
start_pt   世界坐标系下点云中起点位置
end_pt     世界坐标系下点云中终点位置
*/
void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    //世界转栅格坐标系
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //栅格转世界坐标系
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //初始化开始节点和终点节点
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //算法开始前，清空open列表 类型：std::multimap<double, GridNodePtr>
    openSet.clear();

    // 定义当前节点和邻居节点对象
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //将开始点push到open列表中
    startPtr -> gScore = 0;
    startPtr -> fScore = weight_b*calHeu(startPtr,endPtr);   
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;

    //f为键值，值为该节点，multimap可以按照键值从小到大自动排序
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );
    
    // 将起点的栅格索引标记为openlist
    GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]] -> id = 1;

    Eigen::Vector3i current_idx;
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    vector<double> angleCostSets;

    // Astar的Loop过程
    while ( !openSet.empty() )
    {
        //通过取multimap刚开始键所定义值，就表示f值最小的节点，当然刚开始currentPtr就是起点
        currentPtr = openSet.begin() -> second; 
        //从openSet中移除该点
        openSet.erase(openSet.begin()); 
        //当前该点的栅格索引
        current_idx = currentPtr->index;
        //将当前点的栅格索引标记为closelist  表示该点已经访问过
        GridNodeMap[current_idx[0]][current_idx[1]][current_idx[2]] -> id = -1; 
        
        //当前点==终点，Astar结束，打印算法所需的时间差
        if( currentPtr->index == goalIdx )
        {
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }

        //寻找Astar中该点的邻居点集  
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets, angleCostSets); 
        
        //遍历当前点所有的邻居点
        for(int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
            neighborPtr = neighborPtrSets[i];
            // 0表示该点不在openlist中，需要将该点加入到openlist中
            if(neighborPtr -> id ==0)
            { 
                neighborPtr -> gScore = currentPtr->gScore + edgeCostSets[i] + angleCostSets[i] * weight_angle;
                neighborPtr -> fScore = weight_a * neighborPtr->gScore + weight_b * calHeu(neighborPtr,endPtr);
                neighborPtr -> cameFrom = currentPtr;
                neighborPtr -> angle = 180 * (max_angle - (max_angle - min_angle) * double(angleCostSets[i])/double(angle_resolution));
                openSet.insert(make_pair(neighborPtr -> fScore, neighborPtr));
                neighborPtr -> id = 1;
                continue;
            }
            // 1表示该点在openlist
            else if(neighborPtr -> id == 1)
            { 
                //对当前的g值进行更新判断
                if(neighborPtr -> gScore > (currentPtr -> gScore + edgeCostSets[i] + angleCostSets[i] * weight_angle))
                {
                    neighborPtr -> gScore = currentPtr -> gScore + edgeCostSets[i] + angleCostSets[i] * weight_angle;
                    neighborPtr -> fScore = weight_a * neighborPtr -> gScore +  weight_b * calHeu(neighborPtr,endPtr);
                    neighborPtr -> angle = 180 * (max_angle - (max_angle - min_angle) * double(angleCostSets[i])/double(angle_resolution));
                    neighborPtr -> cameFrom = currentPtr;
                }
                continue;
            }
            // 表示该点在closelist中，已经访问过了，不做处理
            else
            {
                continue;
            }
        }      
    }
    //搜索时间超过了0.1s，表示搜索失败
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}

/****************计算Astar的启发式H值*****************/
/*
参数：
node1     世界坐标系下点云中点1的指针对象
node2     世界坐标系下点云中点2的指针对象
返回值：   世界坐标系下两点之间的H值
*/
double AstarPathFinder::calHeu(GridNodePtr node1, GridNodePtr node2)
{
    double h;
    Eigen::Vector3d node1_coord = node1->coord;
    Eigen::Vector3d node2_coord = node2->coord;
    h=Distance(node1_coord,node2_coord);

    // if (distance=="euclidean")
    // {
    //     //欧式距离，h值的计算使用的世界坐标系下两个节点的真实距离
    //     h = std::sqrt(std::pow((node1_coord(0) - node2_coord(0)), 2 ) +
    //     std::pow((node1_coord(1) - node2_coord(1)), 2 ) +
    //     std::pow((node1_coord(2) - node2_coord(2)), 2 ));
    // }
    // else if (distance=="manhattan")
    // {
    //     h = std::abs(node1_coord(0) - node2_coord(0) ) +
    //         std::abs(node1_coord(1) - node2_coord(1) ) +
    //         std::abs(node1_coord(2) - node2_coord(2) );
    // }
    return h;
}

/****************寻找Astar中该点的邻居点集*****************/
/*
参数：
neighborPtrSets  合适邻居节点
edgeCostSets     该邻居点与当前点之间的代价值（欧式距离）
angleCostSets    该邻居点变换角度所需的代价值
*/
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets, vector<double> & angleCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    angleCostSets.clear();
  
    if(currentPtr == nullptr)
        std::cout << "Error: Current pointer is null!" << endl;

    Eigen::Vector3i thisNode = currentPtr -> index;
    int this_x = thisNode[0];
    int this_y = thisNode[1];
    int this_z = thisNode[2];
    auto this_coord = currentPtr -> coord;
    int  n_x, n_y, n_z, phi;
    double dist;
    GridNodePtr temp_ptr = nullptr;
    Eigen::Vector3d n_coord;
    
    for(int i = -1;i <= 1;++i )
    {
        for(int j = -1;j <= 1;++j )
        {
            for(int k = -1;k <= 1;++k)
            {
                phi = 0;
                //去除当前点
                if( i == 0 && j == 0 && k == 0)
                    continue; 
                n_x = this_x + i;
                n_y = this_y + j;
                n_z = this_z + k;
                //去除边界点
                if( (n_x < 0) || (n_x > (GLX_SIZE - 1)) || (n_y < 0) || (n_y > (GLY_SIZE - 1) ) || (n_z < 0) || (n_z > (GLZ_SIZE - 1)))
                    continue;
                //去除障碍物点
                // if(isOccupied(n_x, n_y, n_z, 0, 0, resolution))
                //     continue;
                for(phi = 0;phi <= angle_resolution;phi++)
                {
                    double angle = M_PI * (max_angle - (max_angle - min_angle) * double(phi)/double(angle_resolution));
                    if(isOccupied(n_x, n_y, n_z, length, angle, resolution))
                        continue;
                    else
                        break;
                }
                if(phi > angle_resolution)
                    continue;
                // if(isOccupied(n_x, n_y, n_z))
                //     continue; 

                temp_ptr = GridNodeMap[n_x][n_y][n_z];
            
                //去除以访问过的点
                if(temp_ptr->id == -1) continue; 
                n_coord = temp_ptr->coord;

                if(temp_ptr == currentPtr)
                {
                    std::cout << "Error: temp_ptr == currentPtr)" << std::endl;
                }

                if( (std::abs(n_coord[0] - this_coord[0]) < 1e-6) and (std::abs(n_coord[1] - this_coord[1]) < 1e-6) and (std::abs(n_coord[2] - this_coord[2]) < 1e-6 ))
                {
                    std::cout << "Error: Not expanding correctly!" << std::endl;
                    std::cout << "n_coord:" << n_coord[0] << " "<<n_coord[1]<<" "<<n_coord[2] << std::endl;
                    std::cout << "this_coord:" << this_coord[0] << " "<<this_coord[1]<<" "<<this_coord[2] << std::endl;

                    std::cout << "current node index:" << this_x << " "<< this_y<<" "<< this_z << std::endl;
                    std::cout << "neighbor node index:" << n_x << " "<< n_y<<" "<< n_z << std::endl;
                }

                //计算当前点与邻居点之间的代价值
                dist = std::sqrt( (n_coord[0] - this_coord[0]) * (n_coord[0] - this_coord[0])+
                        (n_coord[1] - this_coord[1]) * (n_coord[1] - this_coord[1])+
                        (n_coord[2] - this_coord[2]) * (n_coord[2] - this_coord[2]));

                neighborPtrSets.push_back(temp_ptr); 
                edgeCostSets.push_back(dist);
                angleCostSets.push_back(double(phi));

            }
        }
    }
}

bool AstarPathFinder::isOccupied(
    const int & idx_x, const int & idx_y, const int & idx_z,
    const double & l, const double & phi,
    const double & res) const 
{
    if(((double)idx_x + 0.5) * resolution + gl_xl > search_x_max || ((double)idx_x + 0.5) * resolution + gl_xl < search_x_min)
        return true;
    if(((double)idx_y + 0.5) * resolution + gl_yl > search_y_max || ((double)idx_y + 0.5) * resolution + gl_yl < search_y_min)
        return true;
    if(((double)idx_z + 0.5) * resolution + gl_zl > search_z_max || ((double)idx_y + 0.5) * resolution + gl_zl < search_z_min)
        return true;

    int height = ceil(l * cos(phi) / res);
    // cout<<"i am here"<<height<<endl;
    for(int h = 0;h <= height;h++)
    {
        int len = ceil(3/2 * l * sin(phi) / res * double(h+1)/double(height));
        // cout<<"i am here"<<sin(phi)<<endl;
        for(int x = 0;x <= len;x++)
            for(int y = 0;y <= len;y++)
                {
                    // cout<<"i am here"<<endl;
                    // cout<<"x:"<<x<<" y:"<<y<<" z:"<<h<<endl;
                    if((idx_x + x - len/2) >= 0 && (idx_x + x - len/2) < GLX_SIZE && (idx_y + y - len/2) >= 0 && (idx_y + y - len/2) < GLY_SIZE && (idx_z + h) >= 0 && (idx_z + h) < GLZ_SIZE && (data[(idx_x + x - len/2) * GLYZ_SIZE + (idx_y + y - len/2) * GLZ_SIZE + (idx_z + h)] == 1))
                        {
                            // cout<<"i am here"<<" x:"<<(idx_x + x - len/2)<<" y:"<<(idx_y + y - len/2)<<" z:"<<(idx_z + h)<<endl;
                            return true;
                        }

                }
    }
    return false;
}


/****************障碍物判断*****************/
/*
参数：
idx_x    栅格坐标系下某点的x值
idx_y    栅格坐标系下某点的y值
idx_z    栅格坐标系下某点的z值
返回值：  true是障碍物  false不是障碍物
*/
// inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
// {


//     return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
//             (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
// }

/****************获取Astar的路径*****************/
vector<Vector4d> AstarPathFinder::getPath() 
{   
    Vector4d point;
    vector<Vector4d> path;
    vector<GridNodePtr> gridPath;

    auto ptr = terminatePtr;
    while(ptr -> cameFrom != NULL)
    {
        gridPath.push_back(ptr);
        ptr = ptr->cameFrom;
    }

    for (auto ptr: gridPath)
    {
        point(0) = ptr->coord(0);
        point(1) = ptr->coord(1);
        point(2) = ptr->coord(2);
        point(3) = ptr->angle;
        path.push_back(point);
    }

    //翻转路径点  
    reverse(path.begin(),path.end());

    return path;
}

/****************获取closelist中点的世界坐标下的索引*****************/
vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1) 
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

/****************Astar节点重置*****************/
void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}
void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

