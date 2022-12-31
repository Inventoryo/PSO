#ifndef SRC_PARAMETER_H
#define SRC_PARAMETER_H
//common
#include <vector>
#include <string>
using namespace std;

static const double PI = 3.14159265358979323846;
static const double DT = 1;
static const double G  = 9.8;

static const int DIRECTION[8][2] ={ { -1,-1 },{ 0,-1 },{ 1,-1 },{ 1, 0 },{ 1, 1 },{ 0, 1 },{ -1, 1 },{ -1, 0 } };

static const int MAX_SIMULATION_STEP = 5000;
static const int RUN_MODE = 0;//0:PSO, 1:ACO
//map
static const int RESOLUTION = 100;
static const int WIDTH  = 500;//
static const int HEIGHT = 500;//


//target.
static const int slow_target_num   = 1;
static const int middel_target_num = 0;
static const int fast_target_num   = 0;
static const int TARGET_NUM = slow_target_num + middel_target_num + fast_target_num;

//uav
static const int slow_uav_num   = 3;
static const int middel_uav_num = 0;
static const int fast_uav_num   = 3;
static const int extra_uav_num  = 0;
static const int uav_num = slow_uav_num + middel_uav_num + fast_uav_num + extra_uav_num;
static const int ADD_TIME = 0;// 300;

static const double communication_R = 20000;
static const double forget_time     = 5000.0;

//particle
static const int USED_LAYERS = 1;//
static const int MAX_LAYER_NUM = 8;//
static const int PARTICLE_NUM  = 100;
static const int MAX_ITERATION = 100;
static const double tao = 1;
static const double WEIGHT = 1;
static const double c1 = 1;
static const double c2 = 1;
static const double  W1 = 1.0;
static const double  W2 = 1.0;
static const double  W3 = 0.0;

//output
const string uav_path			 = "C:\\Users\\Administrator\\Desktop\\PSO\\data\\uav.txt";
const string target_path		 = "C:\\Users\\Administrator\\Desktop\\PSO\\data\\target.txt";
const string traj_Point_path     = "C:\\Users\\Administrator\\Desktop\\PSO\\data\\traj_Point.txt";
const string area_Point_path     = "C:\\Users\\Administrator\\Desktop\\PSO\\data\\area_Point.txt";
const string particle_state_path = "C:\\Users\\Administrator\\Desktop\\PSO\\data\\particle_state.txt";
const string uav_state_path      = "C:\\Users\\Administrator\\Desktop\\PSO\\data\\uav_state.txt";
const string best_state_path     = "C:\\Users\\Administrator\\Desktop\\PSO\\data\\best_state.txt";
const string cove_rate_PSO       = "C:\\Users\\Administrator\\Desktop\\PSO\\data\\cove_rate_PSO.txt";
const string time_consume        = "C:\\Users\\Administrator\\Desktop\\PSO\\data\\time_consume.txt";

//obstacle
const int radar_num = 1;

#endif