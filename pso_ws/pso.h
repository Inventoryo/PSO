#ifndef SRC_PSO_H
#define SRC_PSO_H

#include <time.h>
#include "../common/parameter.h"
#include "../common/utility.h"
#include <fstream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

extern "C" {
	#include "../3rdParty/iniparser/src/iniparser.h"
}

struct acc_grid
{
	double cover_rate = 0;
	bool is_bord = false;
	bool occupy = false;
};

class particle
{
public:
	particle() {};
	~particle() {};
	utility::State  state;
	double Pbest_fitness;
	utility::State Pbest_state;
};

class PSO
{
public:
	PSO(){};
	~PSO(){};
	void create(const string& config_file_path);
	void destory();
	void getNextPoint(utility::MAP &global_map, vector<utility::RADAR> &radar, vector<utility::TARGET> &target, 
								  vector<utility::UAV> &uav, int cur_T, int uav_idx);

private:
	void initParams(const string& config_file_path);
	void spreadSwarm();
	void updateSwarmStates();
	void updateParticleStates(particle* particle, int iteration);
	double calculateFitness(particle* particle);

	particle *swarm_;
	int border_ ;

	double Gbest_fitness_;
	utility::State Gbest_state_;

	vector<utility::UAV> *uav_;
	utility::UAV *cur_uav_;
	utility::MAP global_map_;
	vector<utility::RADAR>* radar_;
	vector<utility::TARGET> *target_;
	double min_R_;
	int cur_T_;
	utility::point2D area_guider_;
	utility::point2D base_angle_;

	ofstream output_particle_state;
	ofstream output_uav_state;
	ofstream output_best_state;
	vector<cv::Mat> layers_;

	vector<pair<vector<int>, int>> infulence_;
	
	double tao_;
	double weight_;
	double c1_;
	double c2_;
	double w2_;
	double eta_;
	double max_iteration_;
	int particle_num_ ;
	int used_layer_num_;

};

#endif

