#ifndef SRC_PARTICLESWARM_H
#define SRC_PARTICLESWARM_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <set>
#include <time.h>

#include "common/utility.h"
#include "common/dubins.h"
#include "common/parameter.h"
#include "aco_ws/aco.h"
#include "pso_ws/pso.h"

extern "C" {
	#include "D:/temp/PSO/mingw_build/iniparser/include/iniparser.h"
}

using namespace std;

class MultipleSearchAndTrack{
public:
	MultipleSearchAndTrack(){};
	~MultipleSearchAndTrack(){};
	void run();
	void init(const string& config_file_path);
	void initParams(const string& config_file_path);
	void destory();
	void show(utility::UAV *uav, utility::TARGET *target); //return the current state of uav swarm and target group
private:

	void updateUAVStatesInDubinsState(utility::UAV * uav);
	void updateArtificalPotentialFieldStateImpl(utility::UAV * uav, utility::State& goal, utility::State&next);
	utility::point2D computeRepulsion(utility::point2D& start, utility::point2D& obstacle);
	double computeEnage(utility::State& start, utility::point2D& obstacle,double zero_dist);
	double computeAttEnage(utility::State& start, utility::State& end, double zero_dist);
	utility::point2D computeAttraction(utility::point2D& start, utility::point2D& goal);
	void updateTargetStates();
	void updateUAVStates();
	void informationShare();
	void updateMission();

	float computeUncetanty(utility::State& state, double search_r, int id);
	double sigmod(double x){ return 1.0 / (1 + exp(-x)); };
	double computeAngle(utility::State& start, utility::State& goal);

	vector<utility::MAP> global_map_;

	vector<utility::TARGET> target_;
	vector<utility::UAV> uav_;
	utility::RADAR *radar_;
	utility::OBSTACLE *obs_;
	ACO aco_;
	PSO pso_;
	//PIO pio_;
	//output
	ofstream output_uav_;
	ofstream output_target_;
	ofstream output_traj_Point_;
	ofstream output_area_Point_;
	ofstream output_coverage_rate_;
	ofstream output_time_;

	//others
	utility::nion union_;//Disjoint Set
	vector<vector<double>> *target_state_;
	vector<bool> *tracked_;
	HybridAStar::DubinsPath *path_;
	int cunt = 0;
	int opt_cunt = 0;
	double  duration_=0;

	int run_mode_;
	int max_simulation_step_;

	//uav params
	int uav_num_total_;
	int uav_num_slow_;
	int uav_num_middle_;
	int uav_num_fast_;
	int uav_num_extra_;

	//target params
	int target_num_total_;
	int target_num_slow_;
	int target_num_middle_;
	int target_num_fast_;

	//map params
	int resolution_;
	int width_;
	int height_;

	//file path


};

#endif
