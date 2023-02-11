#ifndef SRC_PARAMETER_H
#define SRC_PARAMETER_H
//common
#include <vector>
#include <string>
using namespace std;

#define PI 3.14159265358979323846
#define G  9.8
#define DT 1.0

#define ADD_TIME 0

#define COMMUNICATION_R 2000000.0
#define FORGET_TIME 50000.0

static const int DIRECTION[8][2] ={ { -1,-1 },{ 0,-1 },{ 1,-1 },{ 1, 0 },{ 1, 1 },{ 0, 1 },{ -1, 1 },{ -1, 0 } };

//particle
#define MAX_LAYER_NUM  8

//static const int USED_LAYERS = 1;
// static const int PARTICLE_NUM  = 100;
// static const int MAX_ITERATION = 100;
// static const double tao = 1;
// static const double WEIGHT = 1;
// static const double c1 = 1;
// static const double c2 = 1;
// static const double  W1 = 1.0;
// static const double  W2 = 1.0;
// static const double  W3 = 0.0;

//output
// const string OUT_FILE_PATH       = "D:/temp/PSO/mingw_build/result/";
// const string uav_path			 = OUT_FILE_PATH + "/uav.txt";
// const string target_path		 = OUT_FILE_PATH + "/target.txt";
// const string traj_Point_path     = OUT_FILE_PATH + "/traj_Point.txt";
// const string area_Point_path     = OUT_FILE_PATH + "/area_Point.txt";
// const string particle_state_path = OUT_FILE_PATH + "/particle_state.txt";
// const string uav_state_path      = OUT_FILE_PATH + "/uav_state.txt";
// const string best_state_path     = OUT_FILE_PATH + "/best_state.txt";
// const string cove_rate_PSO       = OUT_FILE_PATH + "/cove_rate_PSO.txt";
// const string time_consume        = OUT_FILE_PATH + "/time_consume.txt";

//obstacle
const int radar_num = 1;

#endif