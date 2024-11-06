//////////////////////////////////////////////////////////////////////////////////	
/********************************************************************************/
//EMplanner 工具类
//作者：步海明
//版本：V1.0
//创建日期：2024/10/11
/********************************************************************************/
//////////////////////////////////////////////////////////////////////////////////
#include "emplanner/EMplannerUtils.hpp"
#include "math.h"

using namespace std;
/************************************************************************************************
 * 函数名：EMplanner
 * 描述：构造函数
 * 输入：
 * 输出：
 * 调用：
 ************************************************************************************************/  
EMplannerUtils::EMplannerUtils(/* args */)
{

}
/************************************************************************************************
 * 函数名：~EMplanner()
 * 描述：折构造函数
 * 输入：
 * 输出：
 * 调用：
 ************************************************************************************************/  
EMplannerUtils::~EMplannerUtils()
{

}
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/************************************************************************************************
 * 函数名：TimeAndVelocity(std::vector<waypoint> &init_path_cardesian, double startPoint_time)
 * 描述： 该函数用于更新路径点时间标识
 * 输入：
 * 输出：
 * 调用：外部调用
 * 说明：
 ************************************************************************************************/  
void EMplannerUtils::TimeAndVelocity(std::vector<waypoint> &init_path_cardesian, double startPoint_time)
{
    if(init_path_cardesian.empty())
        return;
    int count = 0;
    for(auto &waypoint_:init_path_cardesian)
    {
        waypoint_.time = startPoint_time + count*0.02;
        count++;
    }
}

/************************************************************************************************
 * 函数名：CovertFromPlanPointInfo(const planPointInfo &point)
 * 描述： planPointInfo数据转waypoint数据
 * 输入：
 * 输出：
 * 调用：外部调用
 * 说明：
 ************************************************************************************************/ 
waypoint EMplannerUtils::CovertFromPlanPointInfo(const planPointInfo &point)
{
    waypoint result;
    result.x = point.x;
    result.y = point.y;
    result.dirAngle = point.dirAngle;
    result.k = point.k;
    result.time = point.time;
    return result;
}
/************************************************************************************************
 * 函数名：CovertFromWaypoint(const waypoint &point)
 * 描述： waypoint数据转planPointInfo数据
 * 输入：
 * 输出：
 * 调用：外部调用
 * 说明：
 ************************************************************************************************/ 
planPointInfo EMplannerUtils::CovertFromWaypoint(const waypoint &point)
{
    planPointInfo result;
    result.x = point.x;
    result.y = point.y;
    result.dirAngle = point.dirAngle;
    result.k = point.k;
    result.time = point.time;
    return result;
}
/************************************************************************************************
 * 函数名：NormalizeAngle(const double angle)
 * 描述： 角度规整
 * 输入：
 * 输出：
 * 调用：外部调用
 * 说明：
 ************************************************************************************************/ 
double EMplannerUtils::NormalizeAngle(const double angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0) {
        a += M_PI;
    } else {
        a -= M_PI;
    }
    return a;
}



/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/************************************************************************************************
 * 函数名：ReadTraje
 * 描述：从文件读取全局轨迹
 * 输入：
 * 输出：
 * 调用：外部调用
 ************************************************************************************************/  
void EMplannerUtils::ReadTraje(std::vector<waypoint> &vecTraj,const std::string &fileName){
    std::ifstream infile(fileName,std::ios::in);
    if(!infile.is_open())
    {
        std::cout<<"can not open fine"<<std::endl;
        return;
    }
    waypoint Point;
    std::string line;
    std::stringstream ss;
    while(getline(infile,line))
    {
        ss<<line;
        ss>>Point.x>>Point.y>>Point.dirAngle>>Point.k;
        vecTraj.push_back(Point);
        ss.clear();
    }
    infile.close();
}

/************************************************************************************************
 * 函数名：ReadObs(std::vector<planPointInfo> &obs_array,const std::string &fileName)
 * 描述：从文件读取全局障碍物位置
 * 输入：
 * 输出：
 * 调用：外部调用
 ************************************************************************************************/  
void EMplannerUtils::ReadObs(std::vector<planPointInfo> &obs_array,const std::string &fileName)
{
    std::ifstream infile(fileName,std::ios::in);
    if(!infile.is_open())
    {
        std::cout<<"can not open fine"<<std::endl;
        return;
    }
    planPointInfo Point;
    std::string line;
    std::stringstream ss;
    while(getline(infile,line))
    {
        ss<<line;
        ss>>Point.x>>Point.y>>Point.dirAngle>>Point.k;
        obs_array.push_back(Point);
        ss.clear();
    }
    infile.close();
}

/************************************************************************************************
 * 函数名：WriteTraje(std::vector<waypoint> &vecTraj, const std::string &fileName)
 * 描述：写轨迹
 * 输入：
 * 输出：
 * 调用：外部调用
 ************************************************************************************************/  
void EMplannerUtils::WriteTraje(std::vector<waypoint> &vecTraj, const std::string &fileName){
    std::ofstream out_file(fileName,std::ios::trunc);
    for(auto el:vecTraj)
    {
        out_file<<std::setprecision(8)<<el.x<<' '<<el.y<<' '<<el.dirAngle<<' '<<el.k<<std::endl;
    }
    out_file.close();
}
/************************************************************************************************
 * 函数名：WriteEigen(const Eigen::VectorXd &vec,const std::string &file_name)
 * 描述：写特征
 * 输入：
 * 输出：
 * 调用：外部调用
 ************************************************************************************************/  
void EMplannerUtils::WriteEigen(const Eigen::VectorXd &vec,const std::string &file_name)
{
    std::ofstream out_file(file_name,std::ios::trunc);
    out_file<<vec;
    out_file.close();
}

/************************************************************************************************
 * 函数名：double EMplanner::CalcPathLength(const std::vector<waypoint> &vecTraj)
 * 描述：计算路径长度
 * 输入：vecTraj:初始轨迹
 * 输出：轨迹长度
 * 调用：外部调用
 ************************************************************************************************/ 
double EMplannerUtils::CalcPathLength(const std::vector<waypoint> &vecTraj)
{
    double length = 0;
    for(int i = 0;i<vecTraj.size()-1;++i)
    {
        Eigen::Vector2d point_vector = {vecTraj.at(i+1).x - vecTraj.at(i).x,vecTraj.at(i+1).y - vecTraj.at(i).y};
        length += point_vector.norm();
    }
    return length;
}

/************************************************************************************************
 * 函数名：CalcCurvePara(std::vector<waypoint> &curve_points)
 * 描述：计算曲线参数
 * 输入：
 * 输出：
 * 调用：外部调用
 ************************************************************************************************/ 
void EMplannerUtils::CalcCurvePara(std::vector<waypoint> &curve_points){
    if(curve_points.size() != 3)
        return;
    waypoint &p0 = curve_points.at(0), &p1 = curve_points.at(1),&p2 = curve_points.at(2);
    double d =2*((p0.y-p2.y)*(p0.x-p1.x)-(p0.y-p1.y)*(p0.x-p2.x));
    if(fabs(d)<1e-8)
    {
        p1.k = 0;
        std::vector<double> Line = {(p1.x - p0.x),(p1.y - p0.y)};
        p1.dirAngle = atan2(Line.at(1),Line.at(0));
        return;

    }
    double a = p0.y*p0.y - p1.y*p1.y + p0.x*p0.x-p1.x*p1.x;
    double b = p0.y*p0.y - p2.y*p2.y + p0.x*p0.x-p2.x*p2.x;

    double cx = ((p0.y - p2.y)*a - (p0.y - p1.y)*b)/d;
    double cy = ((p0.x - p2.x)*a - (p0.x - p1.x)*b)/(-d);

    double dx = cx - p1.x;
    double dy = cy - p1.y;
    double R = sqrt(dx*dx+dy*dy);
    p1.k = 1/R;
    p0.k = 1/R;

    std::vector<double> differVector = {p2.x - p1.x,p2.y - p1.y};
    std::vector<double> dirVec = {-dy/R,dx/R};
    if((differVector.at(0)*dirVec.at(0)+ differVector.at(1)*dirVec.at(1))<0)
    {
        dirVec.at(0) = -dirVec.at(0);
        dirVec.at(1) = -dirVec.at(1);
    }
    p1.dirAngle = atan2(dirVec.at(1),dirVec.at(0));

    // calculate k and dir for p0
    dx = cx - p0.x;
    dy = cy - p0.y;
    differVector = {p1.x - p0.x,p1.y - p0.y};
    dirVec = {-dy/R,dx/R};
    if((differVector.at(0)*dirVec.at(0)+ differVector.at(1)*dirVec.at(1))<0)
    {
        dirVec.at(0) = -dirVec.at(0);
        dirVec.at(1) = -dirVec.at(1);
    }
    p0.dirAngle = atan2(dirVec.at(1),dirVec.at(0));
}

/************************************************************************************************
 * 函数名：ResampleOnStraight(std::vector<waypoint> &vecTraj,std::vector<waypoint> &curve_points,double resample_interval)
 * 描述：直线重采样
 * 输入：
 * 输出：
 * 调用：外部调用
 ************************************************************************************************/ 
void EMplannerUtils::ResampleOnStraight(std::vector<waypoint> &vecTraj,std::vector<waypoint> &curve_points,double resample_interval){
    if(curve_points.size() != 3)
        return;
    waypoint prePoint(vecTraj.back());
    Eigen::Vector2d vecDiff = {curve_points.at(1).x - prePoint.x,curve_points.at(1).y - prePoint.y};
    double dist = vecDiff.norm();
    double coeff = resample_interval/dist;
    vecDiff[0] *= coeff;
    vecDiff[1] *= coeff;
    for(;dist>resample_interval;dist -= resample_interval)
    {
        prePoint.x += vecDiff[0];
        prePoint.y += vecDiff[1];
        vecTraj.push_back(prePoint);
    }
}
/************************************************************************************************
 * 函数名：EMplanner::ResampleOnCurve(std::vector<waypoint> &vecTraj,std::vector<waypoint> &curve_points,double resample_interval)
 * 描述：曲线重采样
 * 输入：
 * 输出：
 * 调用：外部调用
 ************************************************************************************************/ 
void EMplannerUtils::ResampleOnCurve(std::vector<waypoint> &vecTraj,std::vector<waypoint> &curve_points,double resample_interval)
{
    if(curve_points.size() != 3)
        return;
    waypoint prePoint(curve_points.at(0));
    double R = 1/curve_points.at(1).k;
    int dir = 0;

    // judge whither clockwise or anticlockwise
    Eigen::Vector2d p0 = {cos(prePoint.dirAngle), sin(prePoint.dirAngle)};
    Eigen::Vector2d p1 = {cos(curve_points.at(1).dirAngle), sin(curve_points.at(1).dirAngle)};

    double cross = p0[0]*p1[1]- p0[1]*p1[0];
    double dot = p0.dot(p1);
    double theta = acos(dot/(p0.norm()*p1.norm()));
    if(cross>0)
    {
        dir = 1;
    }
    else
    {
        dir = -1;//clockwise
    }

    double dist = fabs(theta)*R;
    double theta_diff = resample_interval*curve_points.at(1).k;
    for(;dist>resample_interval;dist -= resample_interval){

        if(vecTraj.size() == vecTraj.capacity())
            break;

        Eigen::Vector2d vec = {cos(prePoint.dirAngle+dir*theta_diff/2), sin(prePoint.dirAngle+dir*theta_diff/2)};
        vec = 2*R* sin(theta_diff/2)*vec;
        prePoint.dirAngle += dir*theta_diff;
        prePoint.x += vec[0];
        prePoint.y += vec[1];
        vecTraj.push_back(prePoint);
    }
}






/************************************************************************************************
 * 函数名：ResampleTraje(double resample_interval, std::vector<waypoint> &vecTraj)
 * 描述：以设定间隔，进行轨迹重采样
 * 输入：resample_interval:采样间隔
 *      vecTraj:初始轨迹
 * 输出：vecTraj:重采样的轨迹
 * 调用：外部调用
 ************************************************************************************************/  
void EMplannerUtils::ResampleTraje(double resample_interval, std::vector<waypoint> &vecTraj)
{
    std::vector<waypoint> original_trajectory(vecTraj);
    vecTraj.clear();
    vecTraj.push_back(original_trajectory.at(0));
    vecTraj.reserve(ceil(1.5*CalcPathLength(original_trajectory)/resample_interval));
    for(int i = 1;i<original_trajectory.size();++i)
    {
        std::vector<waypoint> curve_points = {vecTraj.back(),
                                              original_trajectory.at(i),
                                              i<original_trajectory.size()-1? original_trajectory.at(i+1):original_trajectory.back()};
        CalcCurvePara(curve_points);
        if(curve_points.at(1).k == 0)
        {
            ResampleOnStraight(vecTraj,curve_points,resample_interval);
        }
        else{
            ResampleOnCurve(vecTraj,curve_points,resample_interval);
        }
    }

}



/************************************************************************************************
 * 函数名：CreateEdge(const std::vector<waypoint> &global_path, std::vector<waypoint> &upper_edge, std::vector<waypoint> &low_edge)
 * 描述：生成轨迹边界
 * 输入：global_path:待生成轨迹
 * 输出：upper_edge:左边界
 *      low_edge:右边界
 * 调用：外部调用
 ************************************************************************************************/  
void EMplannerUtils::CreateEdge(const std::vector<waypoint> &global_path, std::vector<waypoint> &upper_edge, std::vector<waypoint> &low_edge)
{
    upper_edge.reserve(global_path.size());
    low_edge.reserve(global_path.size());

    for(const auto & point : global_path)
    {
        waypoint temp_point;
        Eigen::Vector2d pos = {point.x,point.y};
        Eigen::Vector2d tor = {-sin(point.dirAngle), cos(point.dirAngle)};
        Eigen::Vector2d upper_point = pos+6*tor;
        temp_point.x = upper_point[0];
        temp_point.y = upper_point[1];
        temp_point.dirAngle = point.dirAngle;
        temp_point.k = point.k;
        upper_edge.push_back(temp_point);

        upper_point = pos-6*tor;
        temp_point.x = upper_point[0];
        temp_point.y = upper_point[1];
        temp_point.dirAngle = point.dirAngle;
        temp_point.k = point.k;
        low_edge.push_back(temp_point);
    }
}


/************************************************************************************************
 * 函数名：GetDirAndK(std::vector<waypoint> &vecTraj, int index)
 * 描述：获取轨迹点的方向及曲率
 *      该函数计算path的切线方向与X轴的夹角和曲率
 *      heading =arctan(dy/dx);
 *      kappa = dheading/ds
 *      ds=(dx^2+dy^2)^0.5
 * 输入：
 * 输出：
 * 调用：外部调用
 ************************************************************************************************/  
void EMplannerUtils::GetDirAndK(std::vector<waypoint> &vecTraj, int index){
    if(index == 0 || index == vecTraj.size()-1)
    {
        return;
    }
    waypoint &p0 = vecTraj.at(index-1), &p1 = vecTraj.at(index),&p2 = vecTraj.at(index+1);
    //计算叉积(P0-P1)X(P0-P2)为什么*2
    double d =2*((p0.y-p2.y)*(p0.x-p1.x)-(p0.y-p1.y)*(p0.x-p2.x));
    if(fabs(d)<1e-8)
    {
        vecTraj.at(index).k = 0;
        double module = (p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y);
        std::vector<double> Line = {(p2.x - p1.x)/module,(p2.y - p1.y)/module};
        vecTraj.at(index).dirAngle = atan2(Line.at(1),Line.at(0));
        return;

    }
    double a = p0.y*p0.y - p1.y*p1.y + p0.x*p0.x-p1.x*p1.x;
    double b = p0.y*p0.y - p2.y*p2.y + p0.x*p0.x-p2.x*p2.x;
    //Cx,Cy为圆中心点坐标
    double cx = ((p0.y - p2.y)*a - (p0.y - p1.y)*b)/d;
    double cy = ((p0.x - p2.x)*a - (p0.x - p1.x)*b)/(-d);

    double dx = cx - p1.x;
    double dy = cy - p1.y;
    double R = sqrt(dx*dx+dy*dy);
    vecTraj.at(index).k = 1/R;

    std::vector<double> differVector = {p2.x - p1.x,p2.y - p1.y};
    std::vector<double> dirVec = {-dy/R,dx/R};
    if((differVector.at(0)*dirVec.at(0)+ differVector.at(1)*dirVec.at(1))<0)
    {
        dirVec.at(0) = -dirVec.at(0);
        dirVec.at(1) = -dirVec.at(1);
    }


    vecTraj.at(index).dirAngle = atan2(dirVec.at(1),dirVec.at(0));
    if(index == 1)
    {
        dx = cx - p0.x;
        dy = cy - p0.y;
        R = sqrt(dx*dx+dy*dy);
        vecTraj.at(index-1).k = 1/R;
        differVector = {p1.x - p0.x,p1.y - p0.y};
        dirVec = {-dy/R,dx/R};
        if((differVector.at(0)*dirVec.at(0)+ differVector.at(1)*dirVec.at(1))<0)
        {
            dirVec.at(0) = -dirVec.at(0);
            dirVec.at(1) = -dirVec.at(1);
        }
        vecTraj.at(index-1).dirAngle = atan2(dirVec.at(1),dirVec.at(0));
    }
    if(index == vecTraj.size()-2)
    {
        dx = cx - p2.x;
        dy = cy - p2.y;
        R = sqrt(dx*dx+dy*dy);
        vecTraj.at(index+1).k = 1/R;
        differVector = {p2.x - p1.x,p2.y - p1.y};
        dirVec = {-dy/R,dx/R};
        if((differVector.at(0)*dirVec.at(0)+ differVector.at(1)*dirVec.at(1))<0)
        {
            dirVec.at(0) = -dirVec.at(0);
            dirVec.at(1) = -dirVec.at(1);
        }
        vecTraj.at(index+1).dirAngle = atan2(dirVec.at(1),dirVec.at(0));
    }
}


/************************************************************************************************
 * 函数名：ControlWeak(const planPointInfo &current_vehicle,const std::vector<waypoint> &pre_trajectory)
 * 描述： 控制强弱检测
 * 输入：current_vehicle:车辆当前位置
 * 输出：pre_trajectory: 上一次的规划路径
 * 调用：外部调用
 * 说明：纵向误差大于1.5，横向误差大于0.5，认为控制效果弱
 ************************************************************************************************/ 
bool EMplannerUtils::ControlWeak(const planPointInfo &current_vehicle,const std::vector<waypoint> &pre_trajectory)
{
    int match_index = -1;
    for(int i = 0;i<pre_trajectory.size();i++)
    {
        if(pre_trajectory.at(i).time <= current_vehicle.time && pre_trajectory.at(i+1).time > current_vehicle.time ) {
            match_index = i;
            break;
        }
    }
    double heading = pre_trajectory.at(match_index).dirAngle;
    Eigen::Vector2d tangential = {cos(heading), sin(heading)};   //切向量
    Eigen::Vector2d normal = {-sin(heading),cos(heading)};       //法向量
    Eigen::Vector2d error_vec = {current_vehicle.x - pre_trajectory.at(match_index).x,current_vehicle.y - pre_trajectory.at(match_index).y};
    double lon_error = error_vec.dot(tangential); // Longitudinal error
    double lat_error = error_vec.dot(normal); // lateral error
    if(lon_error > 2.5 || lat_error > 0.5)
        return true;
    else
        return false;
}


/************************************************************************************************
 * 函数名：GetPlanStartPoint(const std::vector<waypoint> &pre_trajectory,const planPointInfo &current_vehicle,planPointInfo &start_plan_point,std::vector<waypoint> &stitch_trajectory)
 * 描述： 获取规划起始点
 * 输入：pre_trajectory:上一次的规划轨迹
 *      current_vehicle:车辆的当前位置
 * 输出：start_plan_point:规划的起始点
 *      stitch_trajectory:待拼接的轨迹
 * 调用：外部调用
 * 说明：将除起始点的预测20个点缝合到新轨迹上，该函数计算规划起点及拼接轨迹信息
 ************************************************************************************************/ 
void EMplannerUtils::GetPlanStartPoint(const std::vector<waypoint> &pre_trajectory,const planPointInfo &current_vehicle,planPointInfo &start_plan_point,std::vector<waypoint> &stitch_trajectory)
{
    double dt = 0.1;
    stitch_trajectory.clear();
    stitch_trajectory.reserve(20);
    if(pre_trajectory.empty())
    {
        start_plan_point.x = current_vehicle.x;
        start_plan_point.y = current_vehicle.y;
        start_plan_point.velocity_x = 0;
        start_plan_point.velocity_y = 0;
        start_plan_point.dirAngle = current_vehicle.dirAngle;
        start_plan_point.accel_x = 0;
        start_plan_point.accel_y = 0;
        start_plan_point.k  = 0;
        start_plan_point.time = current_vehicle.time+dt;
        return;
    }
    else
    {
        if(ControlWeak(current_vehicle,pre_trajectory))
        {  //控制效果较弱时
            double vx_global = current_vehicle.velocity_x*cos(current_vehicle.dirAngle) - current_vehicle.velocity_y*sin(current_vehicle.dirAngle);
            double vy_global = current_vehicle.velocity_x*sin(current_vehicle.dirAngle) + current_vehicle.velocity_y*cos(current_vehicle.dirAngle);
            double ax_global = current_vehicle.accel_x*cos(current_vehicle.dirAngle) - current_vehicle.accel_y*sin(current_vehicle.dirAngle);
            double ay_global = current_vehicle.accel_x*sin(current_vehicle.dirAngle) + current_vehicle.accel_y*cos(current_vehicle.dirAngle);

            start_plan_point.x = current_vehicle.x + vx_global*dt+0.5*ax_global*dt*dt;
            start_plan_point.y = current_vehicle.y + vy_global*dt+0.5*ay_global*dt*dt;
            start_plan_point.velocity_x = vx_global+ax_global*dt;
            start_plan_point.velocity_y = vy_global+ay_global*dt;

            start_plan_point.dirAngle = atan2(start_plan_point.velocity_y,start_plan_point.velocity_x);

            start_plan_point.accel_x = current_vehicle.accel_x;
            start_plan_point.accel_y = current_vehicle.accel_y;
            start_plan_point.k  = 0;
            start_plan_point.time = current_vehicle.time+dt;
            return;

        } else
        {
            //控制效果较强时
            int expert_Index = -1;
            for(int i = 0;i<pre_trajectory.size();++i)
            {
                if(pre_trajectory.at(i).time<=(current_vehicle.time+dt) && pre_trajectory.at(i+1).time>(current_vehicle.time+dt))
                {
                    expert_Index = i;
                    break;
                }

            }
            start_plan_point.x = pre_trajectory.at(expert_Index).x;
            start_plan_point.y = pre_trajectory.at(expert_Index).y;
            start_plan_point.dirAngle = pre_trajectory.at(expert_Index).dirAngle;
            //当前加速度指的是轨迹的切向速度，切向加速度
            start_plan_point.velocity_x = pre_trajectory.at(expert_Index).velocity*cos(start_plan_point.dirAngle);
            start_plan_point.velocity_y = pre_trajectory.at(expert_Index).velocity*sin(start_plan_point.dirAngle);
            start_plan_point.k  = pre_trajectory.at(expert_Index).k;

            //converting waypoint acceleration to ax_global and ay_global needs tangential acceleration and normal acceleration
            Eigen::Vector2d tangentail = {cos(start_plan_point.dirAngle), sin(start_plan_point.dirAngle)};
            Eigen::Vector2d normal = {-sin(start_plan_point.dirAngle),cos(start_plan_point.dirAngle)};
            Eigen::Vector2d tangential_accel = pre_trajectory.at(expert_Index).accel*tangentail;
            Eigen::Vector2d normal_accel = pow(pre_trajectory.at(expert_Index).velocity,2)*start_plan_point.k*normal;

            start_plan_point.accel_x = (tangential_accel+normal_accel)[0];
            start_plan_point.accel_y = (tangential_accel+normal_accel)[1];

            start_plan_point.time = pre_trajectory.at(expert_Index).time;

            //stitch 20 points of pre_trajectory to current period plan
            if (expert_Index>=20)
                std::copy(pre_trajectory.begin()+expert_Index-20,pre_trajectory.begin()+expert_Index,std::back_inserter(stitch_trajectory));
            else
                std::copy(pre_trajectory.begin(),pre_trajectory.begin()+expert_Index,std::back_inserter(stitch_trajectory));
            return;
        }
    }
    
    
}


/************************************************************************************************
 * 函数名：ObsSelect(const planPointInfo &currentvehicle,const std::vector<planPointInfo> &obstacle_array_original, std::vector<planPointInfo> &obstacle_array_selected)
 * 描述： 筛选纵向[-10,60]，横向[-10,10]的障碍物
 * 输入：currentvehicle:车辆当前位置
 *      obstacle_array_original:原始的障碍物缓存
 * 输出：obstacle_array_selected:筛选后的障碍物缓存
 * 调用：外部调用
 * 说明：
 ************************************************************************************************/ 
void EMplannerUtils::LocalObsSelect(const planPointInfo &currentvehicle,const std::vector<planPointInfo> &obstacle_array_original, std::vector<planPointInfo> &obstacle_array_selected)
{
    obstacle_array_selected.clear();
    obstacle_array_selected.reserve(32);
    Eigen::Vector2d tor = {cos(currentvehicle.dirAngle),sin(currentvehicle.dirAngle)};
    Eigen::Vector2d nor = {-sin(currentvehicle.dirAngle),cos(currentvehicle.dirAngle)};
    for(const auto &obstacle : obstacle_array_original)
    {
        Eigen::Vector2d pose_error = {obstacle.x - currentvehicle.x,obstacle.y - currentvehicle.y};
        double lon_error = pose_error.dot(tor);
        double lat_error = pose_error.dot(nor);
        if((lon_error > -10 && lon_error<60) && (lat_error > -10 && lat_error<10))
            obstacle_array_selected.push_back(obstacle);
    }
}


/************************************************************************************************
 * 函数名：GetMatchPoint(planPointInfo hostPoint, int prePointIdx,const std::vector<waypoint> &vecTraj)
 * 描述：匹配点获取
 * 输入：hostPoint:当前位置
 *      prePointIdx:上次找到的匹配点标识
 *      vecTraj:搜索轨迹
 * 输出：minIdx:匹配点标识 
 * 调用：外部调用
 ************************************************************************************************/  
int EMplannerUtils::GetMatchPoint(planPointInfo hostPoint, int prePointIdx,const std::vector<waypoint> &vecTraj)
{
    if(prePointIdx == -1)
    {
        std::vector<double> distVec;
        for(int i = 0;i<vecTraj.size()-1;++i)
        {
            double tempDist = pow(hostPoint.x - vecTraj.at(i).x,2)+ pow(hostPoint.y - vecTraj.at(i).y,2);
            distVec.push_back(tempDist);
        }
        auto itr = std::min_element(distVec.begin(),distVec.end());
        return static_cast<int>(std::distance(distVec.begin(),itr));
    }
    else{

        double preDist = 0,dist = 0,nextDist = 0;
        int process = 0;
        int minIdx = 0;
        int dir = 0;

        // determine direction
        //确定方向
        std::vector<double> vec = {hostPoint.x - vecTraj.at(prePointIdx).x, hostPoint.y - vecTraj.at(prePointIdx).y};
        std::vector<double> matchDir = {cos(vecTraj.at(prePointIdx).dirAngle), sin(vecTraj.at(prePointIdx).dirAngle)};
        double innerProd = vec.at(0)*matchDir.at(0)+vec.at(1)*vec.at(1);
        dir = innerProd > 0? 1:(innerProd < 0)?-1:0;

        // find match point forward or backward
        if(dir != 0)
        {
            int idx = prePointIdx;
            preDist = 1e10;
            dist = pow(hostPoint.x - vecTraj.at(prePointIdx).x,2)+ pow(hostPoint.y - vecTraj.at(prePointIdx).y,2);
            nextDist = pow(hostPoint.x - vecTraj.at(prePointIdx+dir).x,2)+ pow(hostPoint.y - vecTraj.at(prePointIdx+dir).y,2);
            while (idx >= 0 && idx <vecTraj.size())
            {
                preDist = dist;
                dist = pow(hostPoint.x - vecTraj.at(idx).x,2)+ pow(hostPoint.y - vecTraj.at(idx).y,2);
                if( dist<preDist)
                {
                    process = 0;
                    minIdx = idx;
                }
                else if(dist > preDist)
                {
                    process++;
                }
                else{}
                if (process >= 5) break;
                idx = idx + dir;
            }
        } else{
            minIdx = prePointIdx;
        }
        return minIdx;
    }
}




/************************************************************************************************
 * 函数名：GetTrajectoryInitRefLine(int matchPoint_index, const std::vector<waypoint> &vecTraj,std::vector<waypoint> &frenet_trajectory)
 * 描述：从全局轨迹中截取初始参考线
 * 输入：matchPoint_index:匹配点
 *      vecTra:待截取的全局轨迹
 * 输出：frenet_trajectory:截取后的初始参考线
 * 调用：外部调用
 * 说明：匹配点前取30个点，匹配点后取150个点，算上匹配点，截取后的初始参考点共181个点
 ************************************************************************************************/ 
void EMplannerUtils::GetTrajectoryInitRefLine(int matchPoint_index, const std::vector<waypoint> &vecTraj,std::vector<waypoint> &frenet_trajectory)
{

    frenet_trajectory.clear();
    frenet_trajectory.reserve(181);
    if (vecTraj.size()<181)
    {
        std::cout<<"the global waypoints is too short!"<<std::endl;
        return;
    }
    int startIndex = 0;
    if(matchPoint_index<29)
        startIndex = 0;
    else if(vecTraj.size() - matchPoint_index<151)
        startIndex = static_cast<int>(vecTraj.size()) - 181;
    else
        startIndex = matchPoint_index - 30;

    // use std::copy, it copys [start,end) to new vector.
    std::copy(vecTraj.begin()+startIndex,vecTraj.begin()+startIndex+181,std::back_inserter(frenet_trajectory));
}




/************************************************************************************************
 * 函数名：GetProjectPoint(planPointInfo hostPoint, const waypoint &matchPoint, waypoint &projectPoint)
 * 描述：获取投影点
 * 输入：hostPoint:车辆当前位置
 *      matchPoint:找到的匹配点
 * 输出：projectPoint:投影点
 * 调用：外部调用
 ************************************************************************************************/  
void EMplannerUtils::GetProjectPoint(planPointInfo hostPoint, const waypoint &matchPoint, waypoint &projectPoint)
{
    Eigen::Vector2d hostPos = {hostPoint.x,hostPoint.y},matchPointPos = {matchPoint.x,matchPoint.y};
    Eigen::Vector2d d = hostPos - matchPointPos;
    Eigen::Vector2d tao = {cos(matchPoint.dirAngle), sin(matchPoint.dirAngle)};
    Eigen::Vector2d projectPose = matchPointPos+tao*(d.transpose()*tao);
    projectPoint.k = matchPoint.k;
    projectPoint.dirAngle = matchPoint.dirAngle+matchPoint.k*static_cast<double>((d.transpose()*tao));
    projectPoint.x = projectPose[0];
    projectPoint.y = projectPose[1];
}

/************************************************************************************************
 * 函数名：GetProjectPoint(const frenet &frenet_point,  const std::vector<waypoint> &ref_trajectory, const Eigen::VectorXd &index_S,waypoint &projectPoint)
 * 描述：获取投影点
 * 输入：hostPoint:车辆当前位置
 *      matchPoint:找到的匹配点
 * 输出：projectPoint:投影点
 * 调用：外部调用
 ************************************************************************************************/  
void EMplannerUtils::GetProjectPoint(const frenet &frenet_point,  const std::vector<waypoint> &ref_trajectory, const Eigen::VectorXd &index_S,waypoint &projectPoint)
{
    int match_index = -1;
    for(int i = 0;i<index_S.size()-1;++i)
    {
        if(index_S[i]<frenet_point.s && index_S[i+1]>frenet_point.s)
            match_index = i;
    }
    if(match_index == -1)
        return;
    waypoint match_point = ref_trajectory.at(match_index);
    Eigen::Vector2d r_n = {match_point.x,match_point.y};
    Eigen::Vector2d tao_n = {cos(match_point.dirAngle),sin(match_point.dirAngle)};
    double ds = frenet_point.s - index_S[match_index];

    Eigen::Vector2d r_r = r_n + ds*tao_n;
    projectPoint.x = r_r[0];
    projectPoint.y = r_r[1];
    projectPoint.dirAngle = match_point.dirAngle + ds*match_point.k;
    projectPoint.k = (match_point.k + ref_trajectory.at(match_index+1).k)/2;
}



/************************************************************************************************
 * 函数名：GetDistance(const waypoint &p1,const waypoint &p2)
 * 描述：获取两点间距离长度
 * 输入：
 * 输出：距离长度
 * 调用：外部调用
 * 说明：
 ************************************************************************************************/  
double EMplannerUtils::GetDistance(const waypoint &p1,const waypoint &p2)
{
    return sqrt(pow(p1.x - p2.x,2)+ pow(p1.y-p2.y,2));
}

/************************************************************************************************
 * 函数名：CalcSFromIndex2S(int matchPoint_index_local,const waypoint &project_point,const std::vector<waypoint> &frenet_trajectory,const Eigen::VectorXd &index_S)
 * 描述：通过Index2S计算投影点对应的弧长，同时采用向量内积判断投影点是在匹配点的前面还是后面
 * 输入：matchPoint_index_local:匹配点标识
 *      project_point:投影点
 *      frenet_trajectory:参考线
 *      index_S：标识与弧长对应关系缓存
 * 输出：投影点弧长
 * 调用：外部调用
 * 说明：
 ************************************************************************************************/  
double EMplannerUtils::CalcSFromIndex2S(int matchPoint_index_local,const waypoint &project_point,const std::vector<waypoint> &frenet_trajectory,const Eigen::VectorXd &index_S)
{
    double heading = frenet_trajectory.at(matchPoint_index_local).dirAngle;
    Eigen::Vector2d tangent = {cos(heading), sin(heading)};
    Eigen::Vector2d dir = {project_point.x - frenet_trajectory.at(matchPoint_index_local).x,project_point.y - frenet_trajectory.at(matchPoint_index_local).y};
    double s_temp = this->GetDistance(frenet_trajectory.at(matchPoint_index_local),project_point);
    double s0 = dir.dot(tangent) >0?index_S[matchPoint_index_local]+s_temp:index_S[matchPoint_index_local]-s_temp;
    return s0;
}


/************************************************************************************************
 * 函数名：Index2S(int matchPoint_index_local,const waypoint &project_point,const std::vector<waypoint> &frenet_trajectory,Eigen::VectorXd &index_S)
 * 描述：在截取的参考线基础上，生成index标识和轨迹对应弧长的关系
 * 输入：matchPoint_index_local:匹配点标识
 *      project_point:投影点
 *      frenet_trajectory:参考线
 * 输出：index_S:标识与弧长对应关系
 * 调用：外部调用
 * 说明：投影点位置S为0，前向为正，后向为负
 ************************************************************************************************/  
void EMplannerUtils::Index2S(int matchPoint_index_local,const waypoint &project_point,const std::vector<waypoint> &frenet_trajectory,Eigen::VectorXd &index_S)
{
    int size = static_cast<int>(frenet_trajectory.size());
    index_S.resize(size);
    index_S[0] = 0;
    for(int i = 1;i<frenet_trajectory.size();++i)
    {
        index_S[i] = this->GetDistance(frenet_trajectory.at(i),frenet_trajectory.at(i-1))+index_S[i-1];
    }
    //s0为实际起点到坐标原点的弧长
    //获取投影点project_point对应的弧长S0
    double s0 = this->CalcSFromIndex2S(matchPoint_index_local,project_point,frenet_trajectory,index_S);
    //计算完s0再用原index2s减去s0
    index_S = index_S - s0*Eigen::VectorXd::Ones(index_S.size());
}





/************************************************************************************************
 * 函数名：void Cardesian2Frenet(const planPointInfo &cardesian_point, int match_point_index, const waypoint &project_point, const std::vector<waypoint> &ref_trajectory,const Eigen::VectorXd &index_S,frenet &frenet_pos,cardesian2FrentMode mode)
 * 描述：笛卡尔坐标系转自然坐标系
 * 输入：cardesian_point:笛卡尔坐标系下待转换点
 *      match_point_index:笛卡尔坐标系下待转换点的匹配点标识
 *      project_point:笛卡尔坐标系下待转换点投影点
 *      ref_trajectory:参考线
 *      index_S:index与弧长关系缓存
 * 输出：frenet_pos:笛卡尔坐标系下待转换点生成的SL坐标系下的点坐标
 * 调用：外部调用
 ************************************************************************************************/  
void EMplannerUtils::Cardesian2Frenet(const planPointInfo &cardesian_point, int match_point_index, const waypoint &project_point, const std::vector<waypoint> &ref_trajectory,const Eigen::VectorXd &index_S,frenet &frenet_pos,cardesian2FrentMode mode)
{
    // printf( "pPlanStartPoint cardesian_point  X[%f] Y[%f] DIR[%f] K[%f] VELX[%f] VELY[%f] AX[%f] AY[%f]!\n",cardesian_point.x,cardesian_point.y,
    //         cardesian_point.dirAngle,cardesian_point.k,cardesian_point.velocity_x,cardesian_point.velocity_y ,cardesian_point.accel_x,cardesian_point.accel_y);

    if(ref_trajectory.empty())
    {
        std::cout<<"reference line is empty!"<<std::endl;
        return;
    }
    Eigen::Vector2d r_h = {cardesian_point.x,cardesian_point.y};
    Eigen::Vector2d r_r = {project_point.x,project_point.y};
    Eigen::Vector2d tao_r = {cos(project_point.dirAngle),sin(project_point.dirAngle)};
    Eigen::Vector2d n_r = {-sin(project_point.dirAngle), cos(project_point.dirAngle)};
    Eigen::Vector2d n_h = {-sin(cardesian_point.dirAngle), cos(cardesian_point.dirAngle)};


    frenet_pos.s = this->CalcSFromIndex2S(match_point_index,project_point,ref_trajectory,index_S);

    frenet_pos.l = (r_h - r_r).dot(n_r);

    if(mode == firstInfo || mode == secondInfo)
    {
        Eigen::Vector2d v = {cardesian_point.velocity_x,cardesian_point.velocity_y};

        frenet_pos.l_dot = v.dot(n_r);
        frenet_pos.s_dot = (1/(1-frenet_pos.l*project_point.k))* v.dot(tao_r);
        if(fabs(frenet_pos.s_dot)<1e-8)
            frenet_pos.l_diff = 0;
        else
            frenet_pos.l_diff = frenet_pos.l_dot/ frenet_pos.s_dot;
        if(mode == secondInfo)
        {
            Eigen::Vector2d a = {cardesian_point.accel_x,cardesian_point.accel_y};

            frenet_pos.l_2dot = a.dot(n_r) - project_point.k*(1-project_point.k*frenet_pos.l)*pow(frenet_pos.s_dot,2);
            // because d_k/ds is small, use 0 replace
            frenet_pos.s_2dot =(1/(1-frenet_pos.l*project_point.k))*(a.dot(tao_r)+2*project_point.k*frenet_pos.l_diff*pow(frenet_pos.s_dot,2));
            if(fabs(frenet_pos.s_dot)<1e-8)
                frenet_pos.l_2diff = 0;
            else
                frenet_pos.l_2diff = (1/pow(frenet_pos.s_dot,2))*(frenet_pos.l_2dot - frenet_pos.l_diff*frenet_pos.s_2dot);
        }
        return;
    }
}


/************************************************************************************************
 * 函数名：Frenet2Cardesian(const std::vector<frenet> &dense_init_path, const std::vector<waypoint> &ref_trajectory,const Eigen::VectorXd &index_S, std::vector<waypoint> &init_path_cardesian)
 * 描述：自然坐标系转笛卡尔坐标系
 * 输入：dense_init_path:SL坐标系下待转化路径
 *      ref_trajectory:参考线
 *      index_S:标识与弧长对应列表
 * 输出：init_path_cardesian:笛卡尔坐标系下的路径点
 * 调用：外部调用
 * 说明:dk_r/ds is thought to be 0 近似认为kappa == 0
 ************************************************************************************************/  
void EMplannerUtils::Frenet2Cardesian(const std::vector<frenet> &dense_init_path, const std::vector<waypoint> &ref_trajectory,const Eigen::VectorXd &index_S, std::vector<waypoint> &init_path_cardesian)
{
    if(dense_init_path.empty())
        return;
    init_path_cardesian.clear();
    init_path_cardesian.reserve(dense_init_path.size());
    for(const auto &frenet_point:dense_init_path)
    {
        waypoint project_point,cur_point;
        this->GetProjectPoint(frenet_point,ref_trajectory,index_S,project_point);
        Eigen::Vector2d r_r = {project_point.x,project_point.y};
        Eigen::Vector2d tau_r = {cos(project_point.dirAngle),sin(project_point.dirAngle)};
        Eigen::Vector2d n_r = {-sin(project_point.dirAngle), cos(project_point.dirAngle)};

        Eigen::Vector2d r_h = r_r + frenet_point.l*n_r;
        cur_point.x = r_h[0];
        cur_point.y = r_h[1];
        cur_point.dirAngle = project_point.dirAngle+ atan2(frenet_point.l_diff,(1-frenet_point.l*project_point.k));
        cur_point.dirAngle = NormalizeAngle(cur_point.dirAngle);

        double d_theta = cur_point.dirAngle - project_point.dirAngle;
        cur_point.k = ((frenet_point.l_2diff+project_point.k*frenet_point.l_diff* tan(d_theta))*
                      pow(cos(d_theta),2)*(1/(1 - project_point.k*frenet_point.l))+project_point.k)*
                              cos(d_theta)*(1/(1 - project_point.k*frenet_point.l));
        init_path_cardesian.push_back(cur_point);
    }
}


/*-----------------------------------------------------------------动态规划------------------------------------------------------------------------*/


/************************************************************************************************
 * 函数名：CalcQuinticCoeff(const frenet &start_point, const frenet &end_point, Eigen::VectorXd &coeff)
 * 描述： 五次多项式计算
 * 输入：start_point:起始点
 *      end_point:截止点
 * 输出：coeff:五次多项式结果
 * 调用：外部调用
 * 说明：
 ************************************************************************************************/ 
void EMplannerUtils::CalcQuinticCoeff(const frenet &start_point, const frenet &end_point, Eigen::VectorXd &coeff)
{
    // l = a0 + a1*s +a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5;
    // l' = a1 + 2*a2*s + 3*a3*s^2 + 4*a4*s^3 + 5*a5*s^4
    // l'' = 2*a2 + 6*a3*s + 12*a4*s^2 + 20*a5*s^3

    Eigen::Matrix<double,6,6> A;
    Eigen::VectorXd B(6);
    double S_2 = start_point.s * start_point.s;
    double S_3 = S_2 * start_point.s;
    double S_4 = S_3 * start_point.s;
    double S_5 = S_4 * start_point.s;

    double E_2 = end_point.s * end_point.s;
    double E_3 = E_2 * end_point.s;
    double E_4 = E_3 * end_point.s;
    double E_5 = E_4 * end_point.s;

    A<<1, start_point.s,   S_2,   S_3, S_4, S_5,
       0,  1, 2*start_point.s, 3*S_2,  4*S_3, 5*S_4,
       0, 0, 2, 6*start_point.s, 12*S_2, 20*S_3,
       1, end_point.s,   E_2,   E_3, E_4, E_5,
       0, 1, 2*end_point.s, 3*E_2,  4*E_3, 5*E_4,
       0, 0, 2, 6*end_point.s, 12*E_2, 20*E_3;
    
    B<<start_point.l, start_point.l_diff,start_point.l_2diff,end_point.l,end_point.l_diff,end_point.l_2diff;
    coeff = A.inverse()*B;
}

/************************************************************************************************
 * 函数名：CalcObsCost(double w_cost_obs, double square_dist)
 * 描述： 障碍物距离代价计算
 * 输入：w_cost_obs:
 *      square_dist:
 * 输出：障碍物距离代价
 * 调用：外部调用
 * 说明：
 ************************************************************************************************/ 
double  EMplannerUtils::CalcObsCost(double w_cost_obs, double square_dist)
{
    // if dist>4 cost = 0; if dist in [3,4], cost = 1000/square_dist; if dist<3, cost = w_cost_obs
    double cost = 0;
    if(square_dist >= 9 && square_dist <= 16)
        cost = 1000/square_dist;
    else if(square_dist < 9)
        cost = w_cost_obs;
    else
        cost = 0;
    return cost;

}


/************************************************************************************************
 * 函数名：CalcNeighborCost(const std::vector<frenet> &obstacle_array,const frenet &start_point, const frenet &end_point, const initPlanConfigure &config)
 * 描述： 计算相邻节点的代价
 * 输入：obstacle_array:障碍物缓存
 *      start_point:起始点
 *      end_point:截止点
 *      config:动态规划参数
 * 输出：相邻代价
 * 调用：外部调用
 * 说明：
 ************************************************************************************************/ 
double EMplannerUtils::CalcNeighborCost(const std::vector<frenet> &obstacle_array,const frenet &start_point, const frenet &end_point, const initPlanConfigure &config)
{
    Eigen::VectorXd coeff(6);
    CalcQuinticCoeff(start_point,end_point,coeff);    //五次多项式计算
    
    //use 10 points to calculate the cost
    double cost_sum = 0;
    for(int i = 0;i<10;++i)
    {
        double ds = start_point.s + i*config.sample_s/10;
        double ds_2 = ds*ds, ds_3 = ds_2*ds, ds_4 = ds_3*ds, ds_5 = ds_4*ds;
        double l = coeff[0] + coeff[1]*ds + coeff[2]*ds_2 + coeff[3]*ds_3 + coeff[4]*ds_4 + coeff[5]*ds_5;
        double l_diff = coeff[1] + 2*coeff[2]*ds + 3*coeff[3]*ds_2 + 4*coeff[4]*ds_3 + 5*coeff[5]*ds_4;
        double l_2diff = 2*coeff[2] + 6*coeff[3]*ds + 12*coeff[4]*ds_2 + 20*coeff[5]*ds_3;
        double l_3diff = 6*coeff[3] + 24*coeff[4]*ds + 60*coeff[5]*ds_2;
        //光滑代价
        double cost_smooth = config.w_cost_dl*pow(l_diff,2) + config.w_cost_ddl*pow(l_2diff,2)+config.w_cost_dddl*pow(l_3diff,2);
        //参考线距离代价
        double cost_ref = config.w_cost_ref* pow(l,2);
        //障碍物代价
        double cost_collision = 0;
        for(const auto &obstacle:obstacle_array)
        {
            Eigen::Vector2d pose_error = {obstacle.s - ds, obstacle.l - l};
            double square_dist = pose_error.dot(pose_error); // Not Euclidean distance, just approximate
            double cost_collision_once = this->CalcObsCost(config.w_cost_obs,square_dist);
            cost_collision += cost_collision_once;
        }

        cost_sum = cost_sum + cost_smooth + cost_ref + cost_collision;
    }

}


/************************************************************************************************
 * 函数名：GetInitPlanValue(const std::vector<frenet> &obstacle_array,const frenet &start_point, const initPlanConfigure &config, std::vector<frenet> &init_path)
 * 描述： 动态规划，通过撒点采用动态规划获取路径粗解,开辟凸空间
 * 输入：obstacle_array:障碍物缓存
 *      start_point:规划起始点，SL坐标系
 *      config:动态规划参数
 * 输出：init_path:动态规划的粗解
 * 调用：外部调用
 ************************************************************************************************/  
void EMplannerUtils::GetInitPlanValue(const std::vector<frenet> &obstacle_array,const frenet &start_point, const initPlanConfigure &config, std::vector<frenet> &init_path)
{
    init_path.clear();
    std::vector<std::vector<costElement> > cost_matrix;    //代价元素矩阵
    cost_matrix.resize(config.rows);
    for(int i = 0;i<config.rows;++i)
    {
        cost_matrix[i].resize(config.cols);
    }
    for(int i = 0;i<config.rows;i++)
    {
        double l = (config.rows/2 - i)*config.sample_l;
        for(int j = 0;j<config.cols;j++)
        {
            cost_matrix[i][j].pose.l = l;
            cost_matrix[i][j].pose.s = start_point.s+(j+1)*config.sample_s;
            cost_matrix[i][j].pose.l_diff = 0;
            cost_matrix[i][j].pose.l_2diff = 0;
        }
    }

    // calculate the first column cost
    for(int i = 0;i<config.rows;++i)
    {   //无反馈？
        double cost = CalcNeighborCost(obstacle_array,start_point,cost_matrix[i][0].pose,config);
        cost_matrix[i][0].miniCost = cost;
    }

    for(int j = 1;j<config.cols;++j)
    {
        for(int i = 0;i<config.rows;++i)
        {
            for(int k = 0;k<config.rows;++k)
            {
                double cost_temp = CalcNeighborCost(obstacle_array,cost_matrix[k][j-1].pose,cost_matrix[i][j].pose,config);
                //起点到上一节点的最小代价
                double pre_mini_cost = cost_matrix[k][j-1].miniCost;
                double cost_cur = pre_mini_cost+cost_temp;
                if(cost_cur < cost_matrix[i][j].miniCost)
                {   //搜寻代价最小的最优节点，记录上一列节点行和列编号
                    cost_matrix[i][j].miniCost = cost_cur;
                    cost_matrix[i][j].preRow = k;
                    cost_matrix[i][j].preCol = j-1;
                }
            }
        }
    }
    //找到cost最后一列中，cost最小的
    int mini_cost_row = 0;
    int mini_cost_col = config.cols -1;
    double mini_cost_last = 1e8;
    for(int i = 0;i<config.rows;++i)
    {

        if(cost_matrix[i][mini_cost_col].miniCost < mini_cost_last)
        {
            mini_cost_last = cost_matrix[i][mini_cost_col].miniCost;
            mini_cost_row = i;
        }
    }

    init_path.push_back(cost_matrix[mini_cost_row][mini_cost_col].pose);
    for(int i = 0;i<config.cols-1;++i)
    {
        mini_cost_row = cost_matrix[mini_cost_row][mini_cost_col].preRow;
        mini_cost_col = cost_matrix[mini_cost_row][mini_cost_col].preCol;
        init_path.push_back(cost_matrix[mini_cost_row][mini_cost_col].pose);
    }
    init_path.push_back(start_point);
    std::reverse(init_path.begin(),init_path.end());
}


/************************************************************************************************
 * 函数名：TrajectoryInterp(std::vector<frenet> &init_path)
 * 描述： 增密路径点，每1m采样一个点，一共采60个
 * 输入：init_path:待增密路径
 * 输出：init_path:增米后的路径
 * 调用：外部调用
 ************************************************************************************************/  
void EMplannerUtils::TrajectoryInterp(std::vector<frenet> &init_path)
{
    if (init_path.empty())
        return;
    std::vector<frenet> ori_path = init_path;
    init_path.clear();
    init_path.reserve(61);
    double ds = 1;
    init_path.push_back(ori_path.at(0));
    double s_cur = ori_path.at(0).s + ds;
    int count = 1;
    for(int i = 0;i<ori_path.size()-1;++i)
    {
        Eigen::VectorXd coeff(6);
        frenet p1 = ori_path.at(i), p2 = ori_path.at(i+1);
        this->CalcQuinticCoeff(p1,p2,coeff);
        while(s_cur<ori_path.at(i+1).s)
        {
            frenet point_curr;
            point_curr.s = s_cur;
            double s_2 = s_cur*s_cur , s_3 = s_2*s_cur , s_4 = s_3*s_cur , s_5 = s_4*s_cur ;
            point_curr.l= coeff[0] + coeff[1]*s_cur + coeff[2]*s_2 + coeff[3]*s_3 + coeff[4]*s_4 + coeff[5]*s_5;
            point_curr.l_diff = coeff[1] + 2*coeff[2]*s_cur + 3*coeff[3]*s_2 + 4*coeff[4]*s_3 + 5*coeff[5]*s_4;
            point_curr.l_2diff = 2*coeff[2] + 6*coeff[3]*s_cur + 12*coeff[4]*s_2 + 20*coeff[5]*s_3;
            init_path.push_back(point_curr);
            s_cur += ds;
            count++;
            if(count>61)
                return;
        }
    }
}





/*-----------------------------------------------------------------二次规划------------------------------------------------------------------------*/


/************************************************************************************************
 * 函数名：GetMatchSIndex(const std::vector<frenet> &init_path,double s)
 * 描述： 该函数将找到init_path上的所有点中，与s最近的点，并返回该点在init_path的编号
 * 输入：init_path:SL坐标系下的路径点
 *      s:SL坐标系下的待搜寻点
 * 输出：与s最近的点，并返回该点在init_path的编号
 * 调用：外部调用
 * 说明：init_path上的点是动态规划的结果，必然大于等于0，小于59；而s是障碍物在整个参考线上的投影所得到的坐标，所以
        s有可能小于0，也有可能大于59
 ************************************************************************************************/  
int EMplannerUtils::GetMatchSIndex(const std::vector<frenet> &init_path,double s)
{
    int count = 0,index = -1;
    if(s<init_path.at(0).s)
        return index = 0;
    else if(s>init_path.back().s)
        return index = 59;
    else
    {
        for(auto &point:init_path)
        {
            if(point.s>s)
                break;
            count++;
        }
        if((init_path.at(count).s - s) > (s - init_path.at(count-1).s))
            return index = count;
        else
            return index = count-1;
    }
}

/************************************************************************************************
 * 函数名：GetCovexBound(const std::vector<frenet> &init_path,const std::vector<frenet> &obstacle_array,const qpPlanConfigure &config,Eigen::VectorXd &low_bound, Eigen::VectorXd &upper_bound)
 * 描述： 该函数将输出每个动态规划离散点S所对应的L的上下边界
 * 输入：init_path:SL坐标系下的路径点
 *      obstacle_array:SL坐标系下的障碍物点位缓存
 *      config:动态规划参数
 * 输出：low_bound:L下边界
 *      upper_bound:L上边界
 * 调用：外部调用
 * 说明：一般真实障碍物投影到frent后，长宽会变得扭曲，这里近似用直角坐标系的长宽值代替frenet坐标下的值
 *      obstacle_length = 5, obstacle_width = 2
 ************************************************************************************************/  

void EMplannerUtils::GetCovexBound(const std::vector<frenet> &init_path,const std::vector<frenet> &obstacle_array,const qpPlanConfigure &config,Eigen::VectorXd &low_bound, Eigen::VectorXd &upper_bound)
{
    if(init_path.empty())
        return;
    //无障碍的情况下，车辆边界默认为±6
    low_bound = -6*Eigen::VectorXd::Ones(60); //right bound of vehicle
    upper_bound = 6*Eigen::VectorXd::Ones(60); //left bound of vehicle
    for(auto &obs:obstacle_array)
    {
        //计算障碍物头部和尾部的S
        double obs_s_min = obs.s - config.obs_length/2.0;
        double obs_s_max = obs.s + config.obs_length/2.0;
        //找到obs_s_min,obs_s_max在dp_path_s中的位置
        int index_s_min = GetMatchSIndex(init_path,obs_s_min);
        int index_s_max = GetMatchSIndex(init_path,obs_s_max);
        int index_obs = GetMatchSIndex(init_path,obs.s);
        if(index_s_min==0 || index_s_max == 0)
          //障碍物太远了
            continue;
        
        if(init_path.at(index_obs).l>obs.l) // dp-path on left of obstacles
        {   //向左绕
            for(int i = index_s_min;i<=index_s_max;++i)
                low_bound[i] =std::max(obs.l + config.obs_width/2.0,low_bound[i]);
        }
        else // dp-path on right of obstacles
        {   
            //向右绕
            for(int i = index_s_min;i<=index_s_max;++i)
                upper_bound[i] = std::min(obs.l - config.obs_width/2.0,upper_bound[i]);
        }
    }
}



