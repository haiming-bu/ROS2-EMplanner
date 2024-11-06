
#include "emplanner/emplanner_node.hpp"


namespace plt = matplotlibcpp;
using namespace std;
/************************************************************************************************
 * 函数名：URadarNode::URadarNode(std::string name) : Node(name)
 * 描述：节点构造函数
 * 输入：无
 * 输出：无
 * 调用：外部调用
 ************************************************************************************************/

EMplannerNode::EMplannerNode(std::string name) : Node(name)
{
    RCLCPP_INFO(this->get_logger(), "节点%s已启动", name.c_str());


    /////////////////////////////////////////////////////////////////////
    //动态规划参数初始化
    this->dp_config.w_cost_obs = 1e6;           //障碍物代价
    this->dp_config.w_cost_dl = 300;            //光滑性代价
    this->dp_config.w_cost_ddl = 2000;
    this->dp_config.w_cost_dddl = 10000;
    this->dp_config.w_cost_ref = 20;           //参考线距离代价
    this->dp_config.rows = 9;                  //采样行数
    this->dp_config.cols = 6;                  //采样列数
    this->dp_config.sample_s = 10;             //纵向采样长度
    this->dp_config.sample_l = 1;              //横向采样长度
    //////////////////////////////////////////////////////////////////////
    //二次规划配置参数初始化
    this->qp_config.size = 60; //60
    this->qp_config.w_cost_dl = 100000; //25000 for original   //光滑性代价1
    this->qp_config.w_cost_ddl = 50;                           //光滑性代价2
    this->qp_config.w_cost_dddl = 20;                          //光滑性代价3
    this->qp_config.w_cost_ref = 2;                            //参考线代价
    this->qp_config.w_cost_end_l = 15;                         //终点的状态代价(希望path的终点状态为(0,0,0))
    this->qp_config.w_cost_end_dl = 15;
    this->qp_config.w_cost_end_ddl = 15;
    this->qp_config.w_cost_center = 1200; //1200             //在凸空间中央代价
    this->qp_config.host_d1 = 3;                             //质心到前轴的距离
    this->qp_config.host_d2 = 3;                             //质心到后轴的距离
    this->qp_config.host_w = 1.63;                           //host的宽度
    this->qp_config.obs_width = 2;
    this->qp_config.obs_length = 5;
    this->qp_config.delta_dl_max = 10;
    this->qp_config.delta_ddl_max = 20;
    this->qp_config.dl_max = 10;
    this->qp_config.ddl_max = 3;
    //////////////////////////////////////////////////////////////////////
    this->pSmoother=std::make_shared<refLineSmoother>(2,2,3,-0.2,0.2,181);        //参考线平滑初始化，参考线必须为181个点
    this->pFinalPathSolver=std::make_shared<qpPathSolver>(this->qp_config,this->init_point);  //二次规划器初始化
    this->pEMplanner=std::make_shared<EMplannerUtils>();



    //读取障碍物信息
    this->pEMplanner->ReadObs(this->global_obs_array,"/home/bhm/ros2_ws/bc_tank_ws/src/emplanner/data/obstacles.txt");
    //全局路径预处理
    this->pEMplanner->ReadTraje(this->global_path,"/home/bhm/ros2_ws/bc_tank_ws/src/emplanner/data/waypoints.txt");            //轨迹读取
    RCLCPP_INFO(this->get_logger(), "GLOABLE-SIZE(%d)!\n",this->global_path.size());
    //路径预处理
    this->RoadPreHandleModule(this->global_path,this->global_left_edge,this->global_right_edge);

 
    //设定整车初始位置
    this->vehicle.x = 55;
    this->vehicle.y = 448; //460
    this->vehicle.time = 0;
  
  //  //参考线上一个匹配点，初始默认为-1
    this->global_ref_preMatch_index =-1;

   //
   global_path_sub = this->create_subscription<hm_interfaces::msg::NavPath>("/global_path", 100, std::bind(&EMplannerNode::global_path_callback, this, std::placeholders::_1));
    // // 创建发布者
    // // ser_pub = this->create_publisher<XXX>("ser_data", 10);
    // radar_pub = this->create_publisher<hm_interfaces::msg::UltrRadarData>("/ultr_radar_data", 20);
    // // 创建定时器，100ms为周期，定时发布
    // timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&URadarNode::timer_callback, this));

}
/************************************************************************************************
 * 函数名：Nav619Node::~Nav619Node()
 * 描述：节点折构造函数
 * 输入：无
 * 输出：无
 * 调用：外部调用
 ************************************************************************************************/
EMplannerNode::~EMplannerNode()
{
      //关闭串口
    //   ser.close();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************功能模块********************************************************************************/

/************************************************************************************************
 * 函数名：RoadPreHandleModule
 * 描述：道路预处理模块
 * 输入：pGlobalPath:全局路径
 * 输出：pGlobalLeftEdge:路径左边界
 *      pGlobalRightEdge:路径右边界
 * 调用：外部调用
 ************************************************************************************************/
void EMplannerNode::RoadPreHandleModule(std::vector<waypoint> &pGlobalPath,std::vector<waypoint> &pGlobalLeftEdge,std::vector<waypoint> &pGlobalRightEdge)
{
    //路径重采样
    this->pEMplanner->ResampleTraje(PATH_RESAMPLE_INTERVAL_VAL,pGlobalPath);
    for(int i =0;i<pGlobalPath.size();++i)
    {  
        //填充路径点点位方向和曲率
        this->pEMplanner->GetDirAndK(pGlobalPath, i);
    }
    //生成路径左右边界
    this->pEMplanner->CreateEdge(pGlobalPath,pGlobalLeftEdge,pGlobalRightEdge);

}


/************************************************************************************************
 * 函数名：GetRefLineAndSmoothereHandleOpt
 * 描述：获取平滑后的参考线
 * 输入：pGlobalPath:全局路径
 *      pPreRefPathMatchIndex:上一次的参考线
 *      pVehPos:车辆的实时位置
 * 输出：pRefPath:平滑后的参考线
 * 调用：外部调用
 ************************************************************************************************/
void EMplannerNode::GetSmootherRefPathModul(std::vector<waypoint> &pGlobalPath,int &pPreRefPathMatchIndex,planPointInfo pVehPos,std::vector<waypoint> &pRefPath)
{
    std::vector<waypoint> lOriRefPath;   //未平滑的参考线
    //清除参考线缓冲区
    pRefPath.clear();

    //获取参考线匹配点
    int lMatchIndex = this->pEMplanner->GetMatchPoint(pVehPos,pPreRefPathMatchIndex,pGlobalPath);
    //记录匹配点标识
    pPreRefPathMatchIndex = lMatchIndex;
    //获取参考轨迹共181个点位
    this->pEMplanner->GetTrajectoryInitRefLine(lMatchIndex,pGlobalPath,lOriRefPath);
    //更新待平滑处理的参考线
    this->pSmoother->updateRefLine(lOriRefPath);
    //获取平滑处理后的参考线
    this->pSmoother->getNewRefLine(pRefPath);                  
    for(int i =0;i<pRefPath.size();++i)
    {
        this->pEMplanner->GetDirAndK(pRefPath, i);
    }

}


/************************************************************************************************
 * 函数名：CreateFromVehiclePositionIndex2SOpt(void)
 * 描述：生成从当前车辆位置开始的index标识到SL坐标系的S对应弧长关系
 * 输入：
 * 输出：
 * 调用：外部调用
 ************************************************************************************************/
void EMplannerNode::CreateFromVehiclePositionIndex2SModul(planPointInfo pVehPos,std::vector<waypoint> &pRefPath,Eigen::VectorXd &pIndex_S)
{
  // covert start point and obstacles to frenet coordinate
  //获取当前位置在参考线上的匹配点标识
  int lMatchIndex = this->pEMplanner->GetMatchPoint(pVehPos,-1,pRefPath);
  //创建整车当前位置的匹配点
  waypoint lVehMatchPoint = pRefPath.at(lMatchIndex);
  //创建整车当前位置的投影点
  waypoint lVehProjectPoint;
  //获取投影点
  this->pEMplanner->GetProjectPoint(pVehPos,lVehMatchPoint,lVehProjectPoint);

  this->pEMplanner->Index2S(lMatchIndex,lVehProjectPoint,pRefPath,pIndex_S);
  //    cout<<index_S<<endl;
}


/************************************************************************************************
 * 函数名：PlanStartPoint2FrenetOpt(void)
 * 描述：将规划起点坐标由二维坐标系转换为frenet坐标系的值
 * 输入：pPlanStartPoint:规划起始点
 *      pRefPath:参考线
 *      pIndex_S:基于参考线的标识与弧长对应关系表
 * 输出：lSlPlanStartPoint 规划起点的frenet坐标系坐标值
 * 调用：外部调用
 ************************************************************************************************/
// void EMplannerNode::PlanStartPoint2FrenetModul(planPointInfo &pPlanStartPoint,std::vector<waypoint> &pRefPath,Eigen::VectorXd &pIndex_S,frenet &gSlPlanStartPoint)
void EMplannerNode::PlanStartPoint2FrenetModul(planPointInfo &pPlanStartPoint,std::vector<waypoint> &pRefPath,Eigen::VectorXd &pIndex_S,frenet &gSlPlanStartPoint)
{ 

  //获取规划起点在参考线上的匹配点标识
  int lMatchIndex = this->pEMplanner->GetMatchPoint(pPlanStartPoint,-1,pRefPath);
  //获取规划起始点的匹配点
  waypoint PlanStartPointMatchPoint = pRefPath.at(lMatchIndex);     
  waypoint PlanStartPointProjectPoint;
  //获取规划起点的投影点坐标    
  this->pEMplanner->GetProjectPoint(pPlanStartPoint,PlanStartPointMatchPoint,PlanStartPointProjectPoint);
  // printf( "PSP-GOOD-S-ORI  S[%f] L[%f] SDOT[%f] LDOT[%f] LDIFF[%f] S2DOT[%f] L2DOT[%f] L2DIFF[%f]!\n",gSlPlanStartPoint.s,gSlPlanStartPoint.l,
  //       gSlPlanStartPoint.s_dot,gSlPlanStartPoint.l_dot,gSlPlanStartPoint.l_diff,gSlPlanStartPoint.s_2dot,gSlPlanStartPoint.l_2dot,gSlPlanStartPoint.l_2diff);
  //二维坐标系数据转frenet坐标系坐标
  this->pEMplanner->Cardesian2Frenet(pPlanStartPoint,lMatchIndex,PlanStartPointProjectPoint,pRefPath,pIndex_S,gSlPlanStartPoint);
 
  // printf( "PSP-GOOD-2  X[%f] Y[%f] DIR[%f] K[%f] VELX[%f] VELY[%f] AX[%f] AY[%f]!\n",pPlanStartPoint.x,pPlanStartPoint.y,
  //           pPlanStartPoint.dirAngle,pPlanStartPoint.k,pPlanStartPoint.velocity_x,pPlanStartPoint.velocity_y ,pPlanStartPoint.accel_x,pPlanStartPoint.accel_y);
  // return  lSlPlanStartPoint;

}

/************************************************************************************************
 * 函数名：PlanStartPoint2FrenetOpt(void)
 * 描述：将规划起点坐标由二维坐标系转换为frenet坐标系的值
 * 输入：pPlanStartPoint:规划起始点
 *      pRefPath:参考线
 *      pIndex_S:基于参考线的标识与弧长对应关系表
 * 输出：lSlPlanStartPoint 规划起点的frenet坐标系坐标值
 * 调用：外部调用
 ************************************************************************************************/
// void EMplannerNode::PlanStartPoint2FrenetModul(planPointInfo &pPlanStartPoint,std::vector<waypoint> &pRefPath,Eigen::VectorXd &pIndex_S,frenet &gSlPlanStartPoint)
void EMplannerNode::MyTestPlanStartPoint2FrenetModul(planPointInfo pPlanStartPoint,std::vector<waypoint> &pRefPath,Eigen::VectorXd &pIndex_S,frenet &gSlPlanStartPoint)
{ 

  // frenet lSlPlanStartPoint;
  //获取规划起点在参考线上的匹配点标识
  int lMatchIndex = this->pEMplanner->GetMatchPoint(pPlanStartPoint,-1,pRefPath);
  //获取规划起始点的匹配点
  waypoint PlanStartPointMatchPoint = pRefPath.at(lMatchIndex);     
  waypoint PlanStartPointProjectPoint;
  //获取规划起点的投影点坐标    
  this->pEMplanner->GetProjectPoint(pPlanStartPoint,PlanStartPointMatchPoint,PlanStartPointProjectPoint);
  //二维坐标系数据转frenet坐标系坐标
  this->pEMplanner->Cardesian2Frenet(pPlanStartPoint,lMatchIndex,PlanStartPointProjectPoint,pRefPath,pIndex_S,gSlPlanStartPoint);


  // printf( "PSP-TEST-2  X[%f] Y[%f] DIR[%f] K[%f] VELX[%f] VELY[%f] AX[%f] AY[%f]!\n",pPlanStartPoint.x,pPlanStartPoint.y,
  //           pPlanStartPoint.dirAngle,pPlanStartPoint.k,pPlanStartPoint.velocity_x,pPlanStartPoint.velocity_y ,pPlanStartPoint.accel_x,pPlanStartPoint.accel_y);
  // return  lSlPlanStartPoint;


}

/************************************************************************************************
 * 函数名：ObsArray2FrenetOpt(void)
 * 描述：将二维障碍物缓存数据转换为frenet坐标系下数据
 * 输入：pLocalObsArray:局部的障碍物信息缓存
 *      pRefPath:参考线
 *      pIndex_S:标识与弧长对应关系
 * 输出：pSLObsArray:障碍物frenet坐标系坐标值缓存
 * 调用：外部调用
 ************************************************************************************************/
void EMplannerNode::ObsArray2FrenetModul(std::vector<planPointInfo> &pLocalObsArray,std::vector<waypoint> &pRefPath,Eigen::VectorXd &pIndex_S,std::vector<frenet>  &pSLObsArray)
{
  //障碍物容器清零
  pSLObsArray.clear();
  for(auto &obs:pLocalObsArray)
  {
      //获取障碍物在参考线上的匹配点标识
      int lMatchIndex = this->pEMplanner->GetMatchPoint(obs,-1,pRefPath);
      //障碍物匹配点
      waypoint lObsMatchPoint = pRefPath.at(lMatchIndex);  
      //障碍物投影点
      waypoint lObsProjectPoint;   
      //获取投影点坐标
      this->pEMplanner->GetProjectPoint(obs,lObsMatchPoint,lObsProjectPoint);
      frenet lObsFrenet;
      this->pEMplanner->Cardesian2Frenet(obs,lMatchIndex,lObsProjectPoint,pRefPath,pIndex_S,lObsFrenet);
      pSLObsArray.push_back(lObsFrenet);
  }
}


/************************************************************************************************
 * 函数名：DPPathPlanOpt(void)
 * 描述：动态规划
 * 输入：pDpConfig:动态规划配置参数
 *      pSlPlanStartPoint:SL坐标系下的规划起始点
 *      pSlObsArray：SL坐标系下的障碍物缓存
 *      pRefPath：笛卡尔坐标系下的参考线
 *      pIndex_S:标识与弧长的对应关系
 * 输出：pSlDpInitPath:SL坐标系下的动态规划路径
 *       pDpNoStitchPath:笛卡尔坐标系下的动态规划路径
 * 调用：外部调用
 ************************************************************************************************/
void EMplannerNode::DPPathPlanModul(const initPlanConfigure &pDpConfig,frenet &pSlPlanStartPoint,std::vector<frenet>  &pSlObsArray,std::vector<waypoint> &pRefPath,Eigen::VectorXd &pIndex_S,
                                std::vector<frenet>  &pSlDpInitPath,std::vector<waypoint>  &pDpNoStitchPath)
{

  // get init plan value
  //动态规划粗解清零
  pSlDpInitPath.clear();        //frenet坐标系
  pDpNoStitchPath.clear();   //笛卡尔坐标系

  //通过撒点采用动态规划获取路径粗解
  this->pEMplanner->GetInitPlanValue(pSlObsArray,pSlPlanStartPoint,pDpConfig,pSlDpInitPath);
  //初始路径插值处理，1m间隔处理
  this->pEMplanner->TrajectoryInterp(pSlDpInitPath);
  //SL坐标系数据转笛卡尔坐标系数据
  this->pEMplanner->Frenet2Cardesian(pSlDpInitPath,pRefPath,pIndex_S,pDpNoStitchPath);
}


/************************************************************************************************
 * 函数名：QPPathPlanOpt(void)
 * 描述：路径二次规划
 * 输入：pQpConfig:二次规划参数
 *      pSlPlanStartPoint:SL坐标系下的规划起始点
 *      pSlObsArray:SL坐标系下的障碍物缓存
 *      pRefPath:参考线
 *      pIndex_S:标识与弧长对应列矩阵
 *      pSlDpInitPath:SL坐标系下的动态规划轨迹
 *      pTime:绝对时间
 * 输出：pQpFinalPath:笛卡尔坐标系下生成的二次规划路径
 * 调用：外部调用
 ************************************************************************************************/
void EMplannerNode::QPPathPlanModul(qpPlanConfigure &pQpConfig,frenet &pSlPlanStartPoint,std::vector<frenet>  &pSlObsArray,std::vector<waypoint> &pRefPath,Eigen::VectorXd &pIndex_S,
                                std::vector<frenet>  &pSlDpInitPath,double pTime,std::vector<waypoint> &pQpFinalPath)
{
  Eigen::VectorXd low_bound,upper_bound;
  this->pEMplanner->GetCovexBound(pSlDpInitPath,pSlObsArray,pQpConfig,low_bound,upper_bound);


  this->pFinalPathSolver->updateBound(low_bound,upper_bound,pSlPlanStartPoint);
  // final_path_solver.updateBoundWithDpPath(low_bound,upper_bound,init_path,spp_frenet);
  std::vector<frenet> lSlQpPath;
  this->pFinalPathSolver->getNewPath(lSlQpPath,pSlPlanStartPoint);
  //笛卡尔坐标系下的路径清零
  pQpFinalPath.clear();
  this->pEMplanner->Frenet2Cardesian(lSlQpPath,pRefPath,pIndex_S,pQpFinalPath);
  this->pEMplanner->TimeAndVelocity(pQpFinalPath,pTime+0.1);

}


/************************************************************************************************
 * 函数名：StitchFinalPathModul(std::vector<waypoint> &pStitchPath,std::vector<waypoint> &pQpFinalPath,std::vector<waypoint> &pFinalPath)
 * 描述：轨迹拼接
 * 输入：pStitchPath:规划起始点时算出的待拼接轨迹
 *      pQpFinalPath:二次规划轨迹
 * 输出：pFinalPath:最终轨迹
 * 调用：外部调用
 ************************************************************************************************/
void EMplannerNode::StitchFinalPathModul(std::vector<waypoint> &pStitchPath,std::vector<waypoint> &pQpFinalPath,std::vector<waypoint> &pFinalPath)
{
  pFinalPath.clear();
  copy(pStitchPath.begin(),pStitchPath.end(), back_inserter(pFinalPath));
  pFinalPath.insert(pFinalPath.end(),pQpFinalPath.begin(),pQpFinalPath.end());

}






/************************************************************************************************
 * 函数名：StitchFinalPathOpt(void)
 * 描述：最后的轨迹拼接
 * 输入：
 * 输出：
 * 调用：外部调用
 ************************************************************************************************/
void EMplannerNode::EMplannerMethodModule(std::vector<waypoint> &pGlobalPath,std::vector<planPointInfo> pObsArray,planPointInfo pVehPos)
{

    //获取规划起始点及待拼接路径
    this->pEMplanner->GetPlanStartPoint(this->local_pre_path,pVehPos,this->plan_start_point,this->local_stitch_path);
    //局部障碍物筛选
    this->pEMplanner->LocalObsSelect(pVehPos,pObsArray,this->local_obs_array);
    // //生成平滑后的参考线
    this->GetSmootherRefPathModul(pGlobalPath,this->global_ref_preMatch_index,pVehPos,this->local_ref_path);
    // //生成基于平滑参考线的标识与弧长对应表
    this->CreateFromVehiclePositionIndex2SModul(pVehPos,this->local_ref_path,this->local_ref_index_S);

  //  frenet gMyTestSlPlanStartPoint;
    //生成规划起始点的SL坐标系点位值
    this->PlanStartPoint2FrenetModul(this->plan_start_point,this->local_ref_path,this->local_ref_index_S,this->plan_start_point_frenet);    //查找异常原因

  //  this->MyTestPlanStartPoint2FrenetModul(this->plan_start_point,this->local_ref_path,this->local_ref_index_S,gMyTestSlPlanStartPoint);
    // printf( "PSP-REAL  ADDR[%d] X[%f] Y[%f] DIR[%f] K[%f] VELX[%f] VELY[%f] AX[%f] AY[%f]!\n",&this->plan_start_point,this->plan_start_point.x,this->plan_start_point.y,
    //         this->plan_start_point.dirAngle,this->plan_start_point.k,this->plan_start_point.velocity_x,this->plan_start_point.velocity_y ,this->plan_start_point.accel_x,this->plan_start_point.accel_y);
    // this->MyTestPlanStartPoint2FrenetModul(this->plan_start_point,this->local_ref_path,this->local_ref_index_S,this->plan_start_point_frenet);

    //生成障碍物点的SL坐标系点位值
    this->ObsArray2FrenetModul(this->local_obs_array,this->local_ref_path,this->local_ref_index_S,this->local_obs_array_frenet);
    //动态规划
    this->DPPathPlanModul(this->dp_config,this->plan_start_point_frenet,this->local_obs_array_frenet,this->local_ref_path,this->local_ref_index_S,this->dp_init_path_frenet,this->dp_no_stitch_path);
    //二次规划
    this->QPPathPlanModul(this->qp_config,this->plan_start_point_frenet,this->local_obs_array_frenet,this->local_ref_path,this->local_ref_index_S,this->dp_init_path_frenet,pVehPos.time,this->qp_final_path);
    //轨迹拼接
    this->StitchFinalPathModul(this->local_stitch_path,this->qp_final_path,this->local_final_path);


}

/************************************************************************************************
 * 函数名：gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
 * 描述：GPS数据获取回调函数
 * 输入：无
 * 输出：无
 * 调用：外部调用
 ************************************************************************************************/
void EMplannerNode::global_path_callback(const hm_interfaces::msg::NavPath::SharedPtr msg)
{
 
  waypoint lPointTemp;
  this->global_path.reserve(msg->path_x.size());
  for(int i;i<msg->path_x.size();i++){
    lPointTemp.id=msg->id;
    lPointTemp.x=msg->path_x[i];
    lPointTemp.y=msg->path_y[i];
    lPointTemp.dirAngle = 0; // {-Pi,Pi}
    lPointTemp.k = 0;

    lPointTemp.velocity = 0;
    lPointTemp.accel = 0;
    lPointTemp.time = -1;

    this->global_path.push_back(lPointTemp);
  }

}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EMplannerNode>("EMplannerNode");
  //创建线程
//   std::thread Thread1(VehMotionTask,node);
  double ldCurStamp;

  //设置循环的执行频率
  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok())
  {
    // 在这里添加你的逻辑代码
    auto stamp = rclcpp::Clock().now();
    ldCurStamp = stamp.seconds();
    //printf("RCLCPP-CLOCK[%0.5f]\n",ldCurStamp);
    int control = 0;  
    while(control<node->global_path.size()){


      node->EMplannerMethodModule(node->global_path,node->global_obs_array,node->vehicle);

      plt::cla();
      plt::plotTrajectory(node->global_path);
      plt::plotTrajectory(node->global_left_edge,"green");
      plt::plotTrajectory(node->global_right_edge,"green");
      plt::plotTrajectory(node->local_ref_path,"black");
      plt::plotTrajectory(node->qp_final_path,"purple");
      plt::plotTrajectory(node->dp_no_stitch_path,"y");
//        plt::plotTrajectory(final_path,"r");
       plt::plotTrajectory(node->local_obs_array,".r");
       plt::plot(std::vector<double> {node->vehicle.x},std::vector<double> {node->vehicle.y},"vc");


      plt::xlim(node->vehicle.x - 40,node->vehicle.x + 80);
      plt::ylim(node->vehicle.y - 30,node->vehicle.y + 80);
      plt::title("plot frenet_path");
      plt::grid(true);
      plt::pause(0.1);

      control = node->global_ref_preMatch_index+150;
      node->vehicle = node->pEMplanner->CovertFromWaypoint(node->qp_final_path.at(0));
      
      node->local_pre_path=node->local_final_path;

      //node->vehicle = node->pEMplanner->CovertFromWaypoint(node->local_ref_path.at(50));
      // node->pre_path = node->pEMplanner->local_final_path;
      //node->pre_path = node->dp_no_stitch_path;
      // node->local_pre_path=node->dp_no_stitch_path;
      RCLCPP_INFO(node->get_logger(), "EMPLANNER TEST GLOABLE-SIZE(%d)!\n",node->global_path.size());
    }

    rclcpp::spin_some(node);
    // 按照设定的频率暂停
    loop_rate.sleep();
  }

  // rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



