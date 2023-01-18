
#include "pso.h"

using namespace std;
#define DEBUG 1
#define DEBUG_IDX 0

int Fdrections[9][2] = { { -1, -1 }, { 0, -1 }, { 1, -1 },
						 { -1,  0 }, { 0,  0 }, { 1,  0 },
						 { -1,  1 }, { 0,  1 }, { 1,  1 }, };

utility::point2D findBestPoints(const vector<cv::Mat> &layers, const utility::State & uav_state, int border, int resolution) {

	int layer_idx = MAX_LAYER_NUM - 2;
	int x_idx = uav_state.position.x / resolution;
	x_idx = min(max(0, x_idx), layers[0].cols - 1) + border;
	x_idx >>= layer_idx ;

	int y_idx = uav_state.position.y / resolution;
	y_idx = min(max(0, y_idx), layers[0].rows - 1) + border;
	y_idx >>= layer_idx ;

	int uav_dir = 4 * (uav_state.velocity.y + 3 * PI / 4) / PI;
	uav_dir = uav_dir % 8;

	int max_val = 0;
	int best_x, best_y;
	int temp_x, temp_y;

	for (int dir_idx = 0; dir_idx < 8; dir_idx++) {
		temp_x = x_idx + DIRECTION[dir_idx][0];
		temp_x = min(max(0, temp_x), (int)layers[layer_idx].cols - 1);
		temp_y = y_idx + DIRECTION[dir_idx][1];
		temp_y = min(max(0, temp_y), (int)layers[layer_idx].rows - 1);
		int dir_gain = (8 - abs(uav_dir - dir_idx));
		if (max_val <= dir_gain * layers[layer_idx].at<uint16_t>(temp_y, temp_x)) {
			max_val = dir_gain * layers[layer_idx].at<uint16_t>(temp_y, temp_x);
			best_x = temp_x;
			best_y = temp_y;
		}
	}
	x_idx = best_x ;//top layer best
	y_idx = best_y ;

	/*
	while (layer_idx--) {
		max_val = 0;
		for (int j = 0; j < 4; j++) {
			temp_x = x_idx + Fdrections[j][0];
			temp_x = min(max(0, temp_x), (int)layers[layer_idx].cols - 1);
			temp_y = y_idx + Fdrections[j][1];
			temp_y = min(max(0, temp_y), (int)layers[layer_idx].rows - 1);
			if (max_val <= layers[layer_idx].at<uint16_t>(temp_y, temp_x)) {
				max_val = layers[layer_idx].at<uint16_t>(temp_y, temp_x);
				best_x = temp_x;
				best_y = temp_y;
			}
		}
		x_idx = best_x << 1 ;
		y_idx = best_y << 1 ;
	}
	*/
	x_idx = (best_x + 0.5) * pow(2, layer_idx) - border;
	y_idx = (best_y + 0.5) * pow(2, layer_idx) - border;
	return utility::point2D(x_idx * resolution, y_idx * resolution);
}

double getUncertanty(const vector<cv::Mat> &layers, const double x, const double y, int border, int resulution, int used_layer_num){
	double res = 0 ;
	int x_idx = x / resulution;
	x_idx = x_idx < 0 ? 0 : x_idx >= layers[0].cols ? layers[0].cols - 1 : x_idx;
	x_idx += border;
	int y_idx = y / resulution;
	y_idx = y_idx < 0 ? 0 : y_idx >= layers[0].rows ? layers[0].rows - 1 : y_idx;
	y_idx += border;

	int weight = 1;// << used_layer_num;
	double sum_w = 0;
	for(int i = 0; i < used_layer_num; i++){
		sum_w += weight;
		res += weight * layers[i].at<uint16_t>(y_idx, x_idx);
		x_idx >>= 1;
		y_idx >>= 1;
		weight <<= 1;
	}
	return res / sum_w;
}

void PSO::initParams(const string& config_file_path){
	dictionary *dict = nullptr;
	dict = iniparser_load(config_file_path.c_str());
	if(dict == nullptr){
		printf("fail to parser from %s\n", config_file_path.c_str());
		return;
	}

	tao_    = iniparser_getdouble(dict, "pso_param:tao", -1);
	weight_ = iniparser_getdouble(dict, "pso_param:weight", -1);
	c1_ = iniparser_getdouble(dict, "pso_param:c1", -1);
	c2_ = iniparser_getdouble(dict, "pso_param:c2", -1);
	w2_ = iniparser_getdouble(dict, "pso_param:w2", -1);
	eta_ = iniparser_getdouble(dict, "pso_param:eta", -1);
	max_iteration_ = iniparser_getint(dict, "pso_param:max_iteration", -1);
	particle_num_  = iniparser_getint(dict, "pso_param:particle_num", -1);
	used_layer_num_  = iniparser_getint(dict, "pso_param:used_layer_num", -1);

	global_map_.height_ = iniparser_getint(dict, "map_param:height", -1);
	global_map_.width_ = iniparser_getint(dict, "map_param:width", -1);
	global_map_.resolution_ = iniparser_getint(dict, "map_param:resolution", -1);


	//file_path
	const char *particle_state_path = iniparser_getstring(dict, "file_path:particle_state_path", NULL);
	output_particle_state.open(particle_state_path, ios::binary);

	const char *uav_state_path = iniparser_getstring(dict, "file_path:uav_state_path", NULL);
	output_uav_state.open(uav_state_path, ios::binary);

	const char *best_state_path = iniparser_getstring(dict, "file_path:best_state_path", NULL);
	output_best_state.open(best_state_path, ios::binary);

	return;
}

void PSO::create(const string& config_file_path)
{
	initParams(config_file_path);
	border_ = 64;
	swarm_ = (particle*)malloc(particle_num_ * sizeof(particle));
	int height = global_map_.height_ + 2 * border_;
	int width  = global_map_.width_  + 2 * border_;
	for(uint8_t layer = 0 ;layer < MAX_LAYER_NUM;layer++){
		cv::Mat mat(height, width, CV_16UC1, cv::Scalar(0));
		layers_.push_back(mat);
		height = (height + 1) / 2;//ceil
		width  = (width + 1) / 2;
	}
	cv::namedWindow("111", cv::WINDOW_AUTOSIZE);
	return;
};

void PSO::destory()
{
	output_particle_state.close();
	output_uav_state.close();
	output_best_state.close();
	free(swarm_);
	return;
};

void PSO::getNextPoint(utility::MAP &global_map, utility::RADAR *radar, utility::OBSTACLE *obs, vector<utility::TARGET> &target, vector<utility::UAV> &uav, int cur_T, int uav_idx)
{
	global_map_ = global_map;
	radar_ = radar;
	obs_ = obs;
	uav_ = &uav;
	cur_T_ = cur_T;
	target_ = &target;
	cur_uav_ = &(uav[uav_idx]);
	min_R_ = pow(cur_uav_->state.velocity.x, 2) / cur_uav_->Acc_limite_y[1];

	for(int i = 0; i < global_map_.height_; i++){
		for(int j = 0; j < global_map_.width_; j++){
			layers_[0].at<uint16_t>(i + border_, j + border_) = cur_T > (global_map.map_[i * global_map_.width_ + j].search_time + FORGET_TIME) ? FORGET_TIME : (cur_T - global_map.map_[i * global_map_.width_ + j].search_time);
		}
	}

	for (int layer_idx = 0; layer_idx < layers_.size() - 1; layer_idx++) 
		cv::pyrDown(layers_[layer_idx], layers_[layer_idx + 1], cv::Size(layers_[layer_idx + 1].cols, layers_[layer_idx + 1].rows));

		//cv::resize(layers_[layer_idx], layers_[layer_idx + 1], cv::Size(layers_[layer_idx + 1].cols, layers_[layer_idx + 1].rows), 0, 0, CV_INTER_LINEAR);

	if (uav_idx == DEBUG_IDX)
		printf("debug points\n");

	area_guider_ = findBestPoints(layers_, cur_uav_->state, border_, global_map_.resolution_);
	base_angle_ = atan2(area_guider_.y - cur_uav_->state.position.y, area_guider_.x - cur_uav_->state.position.x);//-pi~pi
	base_angle_ = base_angle_ >= 0 ? base_angle_ : 2 * PI + base_angle_;//0~2*pi

#if DEBUG
	if(uav_idx == DEBUG_IDX){
		cv::Mat temp;
		temp.copySize(layers_[0]);
		cv::convertScaleAbs(layers_[0], temp, 0.2);

		for(int i = 0; i < target_->size(); i++){
			int x_idx = target_->at(i).state.position.x / global_map_.resolution_;
			int y_idx = target_->at(i).state.position.y / global_map_.resolution_;
			cv::circle(temp, cv::Point(x_idx, y_idx), 10, (0, 0, 255));
		}

		cv::imshow("111", temp);
		cv::waitKey(10);
	}

#endif

	spreadSwarm();
	updateSwarmStates();

	return;
};

void PSO::spreadSwarm() {
	srand((unsigned)time(NULL));
	particle* temp_particle = swarm_;
	Gbest_fitness_ = 0;
	int j = 0;
	while (j < particle_num_) {
		
		double length = 1.1 * cur_uav_->search_r + (cur_uav_->state.velocity.x) * (rand() % 1000) / 1000.0;
		double theta = 2 * PI * ((rand() % 1000) / 1000.0);
		temp_particle->state.position.x = length * cos(theta) + cur_uav_->state.position.x;
		temp_particle->state.position.y = length * sin(theta) + cur_uav_->state.position.y;
		
		if(!global_map_.isInBound(temp_particle->state.position) || radar_->isInRader(temp_particle->state.position) || obs_->isInObstacle(temp_particle->state.position))
			continue;

		temp_particle->state.velocity.x = cur_uav_->Vel_limite[0];//+ (cur_uav_->Vel_limite[1] - cur_uav_->Vel_limite[0]) * ((rand() % 1000) / 1000.0);//���ٶ�
		temp_particle->state.velocity.y   = 2 * PI*((rand() % 1000) / 1000.0);
		temp_particle->state.accelerate.x = 0;
		temp_particle->state.accelerate.y = 0;
		temp_particle->Pbest_state        = temp_particle->state;
		temp_particle->Pbest_fitness      = 0;
		if (Gbest_fitness_ <= temp_particle->Pbest_fitness) {
			Gbest_state_   = temp_particle->state;
			Gbest_fitness_ = temp_particle->Pbest_fitness;
		}
		temp_particle++;
		j++;
	}
	return;
}

double PSO::calculateFitness(particle* particle){

	if (radar_->isInRader(particle->state.position))
		return -100000;
	if(obs_->isInObstacle(particle->state.position))
		return -100000;

	//fitness = (1-w)*fitness_1 + w*fitness_2
	double fitness = 0;
	double fitness_1 = 0;
	double fitness_2 = 0;
	double fitness_3 = 0;
	//fitness1
	//double t = global_map_->computeUncetanty(particle->state, cur_uav_->search_r, cur_T_);
	double t = getUncertanty(layers_, particle->state.position.x, particle->state.position.y, border_, global_map_.resolution_, used_layer_num_) / FORGET_TIME;
	//double p  = (1 - exp(-tao * t)) ;
	double d = utility::dubinsDistance(cur_uav_->state, particle->state, min_R_);
	d = (d - cur_uav_->search_r) / global_map_.resolution_ ;
	fitness_1 =  t / (1.0 + d * d );

	double c;

	for (int target_id = 0; target_id < target_->size(); target_id++) { //target influence
		if (cur_T_ - cur_uav_->target_state[target_id].second < FORGET_TIME) {//target has been spotted
			c = 1.0 + 1.0 /( cur_uav_->Vel_limite[1] * cur_uav_->Vel_limite[1] );
			d = utility::dist(cur_uav_->target_state[target_id].first.position, particle->state.position);
			d /= 2 * cur_uav_->search_r;

			if(d < 1.0) fitness_2 += 1.0 * c / ( 10.0 + d * d *d );
			else if ( d < 10.0 ) fitness_2 += 1.0 * c / ( 10.0 + d * d );
			else fitness_2 += 1.0 * c / ( 10.0 + d );
		}
	}

	int cur_uav_num = uav_->size();
  	for (int uav_id = 0; uav_id < cur_uav_num; uav_id++) {  //uav avoidance
		if(uav_id == cur_uav_->id) continue;
	    c = 1.0 + 1.1 / ( cur_uav_->Vel_limite[1] * (*uav_)[uav_id].Vel_limite[1] );
	 	d = utility::dist((*uav_)[uav_id].state.position, particle->state.position);
		d /= ((*uav_)[uav_id].search_r + cur_uav_->search_r);

		if(d < 1.0) fitness_2 -= 1.0 * c / ( 10.0 + d * d *d );
		else if ( d < 10.0 ) fitness_2 -= 1.0 * c / ( 10.0 + d * d );
		else fitness_2 -= 1.0 * c / ( 10.0 + d );
	}

	//fitness3
	double angle2 = atan2(particle->state.position.y - cur_uav_->state.position.y , particle->state.position.x - cur_uav_->state.position.x);//-pi~pi
	angle2 = angle2 >= 0 ? angle2 : 2 * PI + angle2;//0~2pi
	angle2 = abs(utility::Mod2pi(angle2 - base_angle_));
	angle2 = angle2 <= PI ? angle2 : 2 * PI - angle2;//0~pi
	fitness_3 = 1.0/(1.0 + exp(angle2));

	fitness = eta_ * fitness_1 + (1 - eta_) * fitness_3 + w2_ * fitness_2 ;

	return fitness;
};

void PSO::updateParticleStates(particle* particle, int iteration)
{
	double r1 = rand() % 1000 / 1000.0;
	double r2 = rand() % 1000 / 1000.0;
	double line_X = particle->state.velocity.x * cos(particle->state.velocity.y);//
	double line_Y = particle->state.velocity.x * sin(particle->state.velocity.y);//
	double temp_weight = (double)(max_iteration_ - iteration) / max_iteration_ * weight_ + 0.5 * weight_;
	//double temp_weight = 1.0;
	line_X = temp_weight * line_X + c1_ * r1 *(particle->Pbest_state.position.x - particle->state.position.x) + c2_ * r2 * (Gbest_state_.position.x - particle->state.position.x) ;//+ F_sum.x*DT;
	line_Y = temp_weight * line_Y + c1_ * r1 *(particle->Pbest_state.position.y - particle->state.position.y) + c2_ * r2 * (Gbest_state_.position.y - particle->state.position.y) ;//+ F_sum.y*DT;

	particle->state.velocity.x = sqrt(line_X * line_X + line_Y * line_Y);//linear velocity
	if(particle->state.velocity.x <= 0.0001 )//stoped
		particle->state.velocity.y = 0;
	else
		particle->state.velocity.y = atan2(line_Y, line_X) >= 0 ? atan2(line_Y, line_X) : 2*PI + atan2(line_Y, line_X);

	particle->state.velocity.x = min(cur_uav_->Vel_limite[1], max((double)cur_uav_->Vel_limite[0], (double)particle->state.velocity.x));

	particle->state.position.x += particle->state.velocity.x * DT * cos(particle->state.velocity.y);
	particle->state.position.y += particle->state.velocity.x * DT * sin(particle->state.velocity.y);

	if (particle->state.position.x < 0) {
		particle->state.position.x = 0;
		particle->state.velocity.y = PI * (0.5 - (rand() % 1000) / 1000.0);//
	}
	else if (particle->state.position.x >= global_map_.width_ * global_map_.resolution_) {
		particle->state.position.x = global_map_.width_ * global_map_.resolution_;
		particle->state.velocity.y = PI * (0.5 + (rand() % 1000) / 1000.0);//
	}
	if (particle->state.position.y < 0) {
		particle->state.position.y = 0;
		particle->state.velocity.y = PI * ((rand() % 1000) / 1000.0);//
	}
	else if (particle->state.position.y > global_map_.height_ * global_map_.resolution_) {
		particle->state.position.y = global_map_.height_ * global_map_.resolution_;
		particle->state.velocity.y = PI * (1 + (rand() % 1000) / 1000.0);//
	}
	return;
};

void PSO::updateSwarmStates() {
	Gbest_fitness_ = 0;

	double temp_fitness;
	
	int iteration = 0;
	int p_idx = 0;
	for (iteration = 0; iteration < max_iteration_; iteration++) {
		for (p_idx = 0; p_idx < particle_num_; p_idx++) {
			particle *temp_part = swarm_ + p_idx;
			updateParticleStates(temp_part, iteration);
			temp_fitness = calculateFitness(temp_part);

			//update local best
			if (temp_fitness >= temp_part->Pbest_fitness) {
				temp_part->Pbest_fitness = temp_fitness;
				temp_part->Pbest_state = temp_part->state;
			}

			//update global best
			if (temp_fitness >= Gbest_fitness_) {
				Gbest_fitness_ = temp_fitness;
				Gbest_state_   = temp_part->state;
			}

			if (cur_uav_->id == DEBUG_IDX && (particle_num_ % 5) ==0 && (iteration % 10) == 0 && cur_T_ <= 500)
				output_particle_state << temp_part->state.position.x << " " << temp_part->state.position.y << " ";
		}
		if (cur_uav_->id == DEBUG_IDX && (iteration % 10) == 0 && cur_T_ <= 500) {
			double d = utility::dubinsDistance(cur_uav_->state, Gbest_state_, min_R_);

			output_particle_state << endl;
			output_uav_state  << cur_uav_->state.position.x << " " << cur_uav_->state.position.y << " " << cur_uav_->state.velocity.y << endl;
			output_best_state << Gbest_state_.position.x << " " << Gbest_state_.position.y << " " 
							  << area_guider_.x << " " << area_guider_.y << " "
							  << Gbest_fitness_ << " " << d << endl;
		}
	}
	
	printf("Gbest_fitness_ = %lf\n", Gbest_fitness_);
	cur_uav_->traj_Point = Gbest_state_;
	cur_uav_->area_guider = area_guider_;
	//uav_->updatePrevPoses();
	return;
};
