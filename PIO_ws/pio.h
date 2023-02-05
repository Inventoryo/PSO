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

class PIO
{
public:
	PIO(){};
	~PIO(){};
	void create(const string& config_file_path);
	void destory();
	void getNextPoint(utility::MAP &global_map, utility::RADAR *radar,
								  utility::OBSTACLE *obs, vector<utility::TARGET> &target, 
								  vector<utility::UAV> &uav, int cur_T, int uav_idx);

private:
	void initParams(const string& config_file_path);
	void spreadSwarm();
	void updateSwarmStates();
	void updateParticleStatesAtMapMode(particle* particle, int iteration);
	void updateParticleStatesAtLandmarkMode(particle* particle, int iteration, utility::point2D& center);
	double calculateFitness(particle* particle);

	utility::point2D findBestPoints(const vector<cv::Mat> &layers, const utility::State & uav_state, int border, int resolution);
	double getUncertanty(const vector<cv::Mat> &layers, const double x, const double y, int border, int resulution, int used_layer_num);

	particle *swarm_;
	int border_ ;

	double Gbest_fitness_;
	utility::State Gbest_state_;

	vector<utility::UAV> *uav_;
	utility::UAV *cur_uav_;
	utility::MAP global_map_;
	utility::RADAR *radar_;
	vector<utility::TARGET> *target_;
	utility::OBSTACLE *obs_;
	double min_R_;
	int cur_T_;
	utility::point2D area_guider_;
	double base_angle_;

	ofstream output_particle_state;
	ofstream output_uav_state;
	ofstream output_best_state;
	vector<cv::Mat> layers_;
	
	double tao_;
	double weight_;
	double c1_;
	double c2_;
	double w2_;
	double eta_;
	double max_iteration_;
	int particle_num_ ;
	int used_layer_num_;

	double R_;

};

#endif
