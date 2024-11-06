#ifndef __EMPLANNER_NODE_HPP__
#define	__EMPLANNER_NODE_HPP__
#include "rclcpp/rclcpp.hpp"
#include "emplanner/EMplannerUtils.hpp"
#include "emplanner/matplotlibcpp.h"
#include "emplanner/osqpSolver.h"

class EMplannerNode : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    EMplannerNode(std::string name);
    //折构造函数
    ~EMplannerNode();



   void RoadPreHandleModule(std::vector<waypoint> &pGlobalPath,std::vector<waypoint> &pGlobalLeftEdge,std::vector<waypoint> &pGlobalRightEdge);
   void GetSmootherRefPathModul(std::vector<waypoint> &pGlobalPath,int &pPreRefPathMatchIndex,planPointInfo pVehPos,std::vector<waypoint> &pRefPath);
   void CreateFromVehiclePositionIndex2SModul(planPointInfo pVehPos,std::vector<waypoint> &pRefPath,Eigen::VectorXd &pIndex_S);
   //frenet PlanStartPoint2FrenetModul(planPointInfo pPlanStartPoint,std::vector<waypoint> &pRefPath,Eigen::VectorXd &pIndex_S);

   void PlanStartPoint2FrenetModul(planPointInfo &pPlanStartPoint,std::vector<waypoint> &pRefPath,Eigen::VectorXd &pIndex_S,frenet &gSlPlanStartPoint);
   void MyTestPlanStartPoint2FrenetModul(planPointInfo pPlanStartPoint,std::vector<waypoint> &pRefPath,Eigen::VectorXd &pIndex_S,frenet &gSlPlanStartPoint);



   void ObsArray2FrenetModul(std::vector<planPointInfo> &pLocalObsArray,std::vector<waypoint> &pRefPath,Eigen::VectorXd &pIndex_S,std::vector<frenet>  &pSLObsArray);
   void DPPathPlanModul(const initPlanConfigure &pDpConfig,frenet &pSlPlanStartPoint,std::vector<frenet>  &pSlObsArray,std::vector<waypoint> &pRefPath,Eigen::VectorXd &pIndex_S,
                                std::vector<frenet>  &pSlDpInitPath,std::vector<waypoint>  &pDpNoStitchPath);
   void QPPathPlanModul(qpPlanConfigure &pQpConfig,frenet &pSlPlanStartPoint,std::vector<frenet>  &pSlObsArray,std::vector<waypoint> &pRefPath,Eigen::VectorXd &pIndex_S,
                                std::vector<frenet>  &pSlDpInitPath,double pTime,std::vector<waypoint> &pQpFinalPath);
   void StitchFinalPathModul(std::vector<waypoint> &pStitchPath,std::vector<waypoint> &pQpFinalPath,std::vector<waypoint> &pFinalPath);                             
   void EMplannerMethodModule(std::vector<waypoint> &pGlobalPath,std::vector<planPointInfo> pObsArray,planPointInfo pVehPos);
  

   //全局路径回调函数
   void global_path_callback(const hm_interfaces::msg::NavPath::SharedPtr msg);
  
public:


    // // 声名定时器指针
    // rclcpp::TimerBase::SharedPtr timer;
    // // 声明话题发布者指针
    // rclcpp::Publisher<hm_interfaces::msg::UltrRadarData>::SharedPtr radar_pub;
    // //解码状态机
    // void uradar_rx(unsigned char data);

      // 声明话题发布者指针
//   rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr utc_pub;
//   rclcpp::Publisher<hm_interfaces::msg::CanMsg>::SharedPtr master_can_pub;
  //声明话题订阅者指针
   rclcpp::Subscription<hm_interfaces::msg::NavPath>::SharedPtr global_path_sub;

    std::shared_ptr<EMplannerUtils>  pEMplanner;                 //EMplanner工具指针
    std::shared_ptr<refLineSmoother> pSmoother;                  //参考线平滑处理类指针创建
    std::shared_ptr<qpPathSolver>    pFinalPathSolver;           //二次规划处理类指针创建
    
    initPlanConfigure dp_config;      //动态规划配置参数
    qpPlanConfigure qp_config;        //二次规划配置参数
   
    frenet init_point;                                        //frenet坐标系下的初始点，暂时未使用
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // //全局轨迹信息
    std::vector<waypoint> global_path,global_left_edge,global_right_edge;           //全局路径，道路左边界，道路右边界(笛卡尔坐标系)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // //参考线部分
    planPointInfo vehicle;                               //车辆位置信息(笛卡尔坐标系)
    int global_ref_preMatch_index;                       //参考线在全局路径下的上一次匹配点标识(笛卡尔坐标系)
    // int global_ref_match_index;                          //参考线在全局路径下的匹配点标识(笛卡尔坐标系)
    std::vector<waypoint> local_ref_path;                //局部参考线(笛卡尔坐标系)
    Eigen::VectorXd local_ref_index_S;                   //基于车辆投影点在标识和S距离矩阵
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //规划起始点
    planPointInfo plan_start_point;                      //规划起始点位置信息(笛卡尔坐标系)
    frenet        plan_start_point_frenet;               //规划起始点位置信息(SL坐标系)
    std::vector<waypoint>      local_stitch_path;        //生成的待拼接路径
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //障碍物部分参数
    std::vector<planPointInfo> global_obs_array;              //全局障碍物缓存(笛卡尔坐标系)
    std::vector<planPointInfo> local_obs_array;               //局部障碍物缓存(笛卡尔坐标系)
    std::vector<frenet>        local_obs_array_frenet;        //局部障碍物缓存(SL坐标系)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //动态规划路径
    std::vector<frenet>        dp_init_path_frenet;           //动态规划粗解(SL坐标系)    
    std::vector<waypoint>      dp_no_stitch_path;             //动态规划粗解(笛卡尔坐标系)    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //二次规划
    std::vector<waypoint>      qp_final_path;                //动态规划最终路径(笛卡尔坐标系)   

    std::vector<waypoint>      local_final_path;            //拼接后的最终路径(笛卡尔坐标系) 
    std::vector<waypoint>      local_pre_path;              //记录的拼接后的最终路径(笛卡尔坐标系) 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////








};

#endif