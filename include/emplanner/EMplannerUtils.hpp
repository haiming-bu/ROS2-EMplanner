#ifndef __EMPLANNERUTILS_HPP__
#define	__EMPLANNERUTILS_HPP__

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include "iostream"
#include <Eigen/Dense>
#include <iomanip>
#include <algorithm>


#define PATH_RESAMPLE_INTERVAL_VAL      (1)     //路径重采样间隔



enum frame {base_link,world};

/* used in cardesian2Frenet(),
   normal:only calculate s and l;
   firstInfo: s,l,s_dot,l_dot,l_diff
   secondInfo: s, l, s_dot,l_dot,s_2dot,l_2dot,l_diff,l_2diff;
  */
enum cardesian2FrentMode {normal,firstInfo,secondInfo};

//路径点参数定义
typedef  struct waypoint {
public:
    double x = 0;
    double y = 0;
    double dirAngle = 0; // {-Pi,Pi}
    double k = 0;

    double velocity = 0;
    double accel = 0;
    double time = -1;


    // waypoint (double x_ = 0,double y_ = 0,double dirAngle_ = 0,double k_ = 0,double velocity_=0, double accel_= 0,double time_= -1)
    // :x(x_),y(y_),dirAngle(dirAngle_),k(k_),velocity(velocity_),accel(accel_),time(time_)
    // {};

    // waypoint(const waypoint &point)
    // :x(point.x),y(point.y),dirAngle(point.dirAngle),k(point.k),velocity(point.velocity),accel(point.accel),time(point.time)
    // {};

}waypoint;

//自然坐标系参数定义
typedef struct frenet{
public:
    int    id=0;
    double s = 0;
    double l = 0; // it must be an odd number
    double s_dot = 0;
    double l_dot = 0;
    double l_diff = 0;

    double s_2dot = 0;
    double l_2dot = 0;
    double l_2diff = 0;

    // frenet( double s_ = 0,double l_ = 0,double s_dot_ = 0,double l_dot_ = 0, double l_diff_ = 0,double s_2dot_ = 0,double l_2dot_ = 0,double l_2diff_ = 0)
    // :s(s_),l(l_),s_dot(s_dot_),l_dot(l_dot_),l_diff(l_diff_),s_2dot(s_2dot_),l_2dot(l_2dot_),l_2diff(l_2diff_)
    // {};

    // frenet(const frenet &frenet_point)
    // :s(frenet_point.s),l(frenet_point.l),s_dot(frenet_point.s_dot),l_dot(frenet_point.l_dot),l_diff(frenet_point.l_diff),s_2dot(frenet_point.s_2dot),l_2dot(frenet_point.l_2dot),l_2diff(frenet_point.l_2diff)
    // {};
}frenet;


//规划点信息定义
typedef struct planPointInfo{
public:
    double x = 0;
    double y = 0;
    double dirAngle = 0;
    double k = 0; // should the carLocationInfo have the member -- curvature value ?

    double velocity_x = 0;
    double velocity_y = 0;
    double accel_x = 0;
    double accel_y = 0;

    double time = -1;
    frame frame_id = world;
    // planPointInfo(double x_ = 0,double y_ = 0,double dirAngle_ = 0,double k_ = 0,double velocity_x_ = 0,double velocity_y_=0,double accel_x_ =0,double accel_y_ = 0,double  time_ = -1,frame frame_ = world)
    // :x(x_),y(y_),dirAngle(dirAngle_),k(k_),velocity_x(velocity_x_),velocity_y(velocity_y_),accel_x(accel_x_),accel_y(accel_y_),time(time_),frame_id(frame_)
    // {};

    // planPointInfo(const planPointInfo &vehicle_)
    // :x(vehicle_.x),y(vehicle_.y),dirAngle(vehicle_.dirAngle),k(vehicle_.k),velocity_x(vehicle_.velocity_x),velocity_y(vehicle_.y),accel_x(vehicle_.accel_x),accel_y(vehicle_.accel_y),time(vehicle_.time),frame_id(vehicle_.frame_id)
    // {};

}planPointInfo;



//代价元素参数定义
typedef struct costElement{
    double miniCost = 1e8;     //最小代价
    int preRow = -1;           //上一行
    int preCol = -1;           //上一列
    frenet pose;               //点位
}costElement;

//动态规划配置
typedef struct initPlanConfigure{
    int rows = 0;
    int cols = 0;
    double sample_s = 0;
    double sample_l = 0;

    double w_cost_obs = 0;
    double w_cost_dl = 0;
    double w_cost_ddl = 0;
    double w_cost_dddl = 0;
    double w_cost_ref = 0;

}initPlanConfigure;


//二次规划配置
typedef struct qpPlanConfigure{
    double w_cost_dl = 0;
    double w_cost_ddl = 0;
    double w_cost_dddl = 0;
    double w_cost_ref = 0;
    double w_cost_center = 0;

    double w_cost_end_l = 0;
    double w_cost_end_dl = 0;
    double w_cost_end_ddl = 0;

    int obs_width = 0;
    int obs_length = 0;

    double host_d1 = 0;
    double host_d2 = 0;
    double host_w = 0;

    double delta_dl_max = 0;
    double delta_ddl_max = 0;
    double dl_max = 0;
    double ddl_max = 0;

    int size = 0;

}qpPlanConfigure;



class EMplannerUtils {
	public:
		EMplannerUtils();     //构造函数
		~EMplannerUtils();    //折构造函数
		
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/********************************************************实用工具***************************************************************************/
   void TimeAndVelocity(std::vector<waypoint> &init_path_cardesian, double startPoint_time);
   waypoint CovertFromPlanPointInfo(const planPointInfo &point);
   planPointInfo CovertFromWaypoint(const waypoint &point);   
   double NormalizeAngle(const double angle);
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
    //全局轨迹处理
    void ReadTraje(std::vector<waypoint> &vecTraj,const std::string &fileName);            //轨迹读取
    void ReadObs(std::vector<planPointInfo> &obs_array,const std::string &fileName);       //静态障碍物读取
    void WriteTraje(std::vector<waypoint> &vecTraj, const std::string &fileName);
    void WriteEigen(const Eigen::VectorXd &vec,const std::string &file_name);
    //重采样
    double CalcPathLength(const std::vector<waypoint> &vecTraj);
    void CalcCurvePara(std::vector<waypoint> &curve_points);
    void ResampleOnStraight(std::vector<waypoint> &vecTraj,std::vector<waypoint> &curve_points,double resample_interval);
    void ResampleOnCurve(std::vector<waypoint> &vecTraj,std::vector<waypoint> &curve_points,double resample_interval);
    void ResampleTraje(double resample_interval, std::vector<waypoint> &vecTraj);          //轨迹重采样
    void CreateEdge(const std::vector<waypoint> &global_path, std::vector<waypoint> &upper_edge, std::vector<waypoint> &low_edge);  
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
   //通用函数
    void GetDirAndK(std::vector<waypoint> &vecTraj, int index);                            //获取轨迹点位方向及曲率
    
    bool ControlWeak(const planPointInfo &current_vehicle,const std::vector<waypoint> &pre_trajectory);
    //获取规划起始点及待拼接轨迹    
    void GetPlanStartPoint(const std::vector<waypoint> &pre_trajectory,const planPointInfo &current_vehicle,planPointInfo &start_plan_point,std::vector<waypoint> &stitch_trajectory);   
    void LocalObsSelect(const planPointInfo &currentvehicle,const std::vector<planPointInfo> &obstacle_array_original, std::vector<planPointInfo> &obstacle_array_selected);     
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
    int  GetMatchPoint(planPointInfo hostPoint, int prePointIdx,const std::vector<waypoint> &vecTraj);                                      //获取匹配点
    void GetTrajectoryInitRefLine(int matchPoint_index, const std::vector<waypoint> &vecTraj,std::vector<waypoint> &frenet_trajectory);     //获取全局轨迹的初始参考线 
    void GetProjectPoint(planPointInfo hostPoint, const waypoint &matchPoint, waypoint &projectPoint);                                      //获取投影点
    void GetProjectPoint(const frenet &frenet_point,  const std::vector<waypoint> &ref_trajectory, const Eigen::VectorXd &index_S,waypoint &projectPoint);


    double GetDistance(const waypoint &p1,const waypoint &p2);
    double CalcSFromIndex2S(int matchPoint_index_local,const waypoint &project_point,const std::vector<waypoint> &frenet_trajectory,const Eigen::VectorXd &index_S);
    void Index2S(int matchPoint_index_local,const waypoint &project_point,const std::vector<waypoint> &frenet_trajectory,Eigen::VectorXd &index_S);   //在参考线下标识与弧长关系
    
    
    void Cardesian2Frenet(const planPointInfo &cardesian_point, int match_point_index, const waypoint &project_point, const std::vector<waypoint> &ref_trajectory,const Eigen::VectorXd &index_S,frenet &frenet_pos,cardesian2FrentMode mode = secondInfo);
    void Frenet2Cardesian(const std::vector<frenet> &dense_init_path, const std::vector<waypoint> &ref_trajectory,const Eigen::VectorXd &index_S, std::vector<waypoint> &init_path_cardesian);
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
    //动态规划
   void CalcQuinticCoeff(const frenet &start_point, const frenet &end_point, Eigen::VectorXd &coeff);
   double CalcObsCost(double w_cost_obs, double square_dist);
   double CalcNeighborCost(const std::vector<frenet> &obstacle_array,const frenet &start_point, const frenet &end_point, const initPlanConfigure &config);
   void GetInitPlanValue(const std::vector<frenet> &obstacle_array,const frenet &start_point, const initPlanConfigure &config, std::vector<frenet> &init_path);
   void TrajectoryInterp(std::vector<frenet> &init_path);                  //路径增密
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
   //二次规划
   int GetMatchSIndex(const std::vector<frenet> &init_path,double s);
   void GetCovexBound(const std::vector<frenet> &init_path,const std::vector<frenet> &obstacle_array,const qpPlanConfigure &config,Eigen::VectorXd &low_bound, Eigen::VectorXd &upper_bound);


	private:

};

#endif