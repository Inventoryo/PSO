
#include "pso.h"

using namespace std;
#define DEBUG 0
#define DEBUG_IDX 0

static bool SHOW = 1;

int Fdrections[9][2] = { { -1, -1 }, { 0, -1 }, { 1, -1 },
						 { -1,  0 }, { 0,  0 }, { 1,  0 },
						 { -1,  1 }, { 0,  1 }, { 1,  1 }};

utility::point2I projectToCertainLayer(const utility::point2D &position,
	const int resolution, const int layer, const int border,
	const int height, const int width){
	utility::point2I res;
	int x_idx = position.x / resolution;
	x_idx = min(max(0, x_idx), width - 1) + border;

	int y_idx = position.y / resolution;
	y_idx = min(max(0, y_idx), height - 1) + border;
	res.x = x_idx >> layer;
	res.y = y_idx >> layer;
	return res;
}

utility::point2D findBestPoints(const vector<cv::Mat> &layers, vector<utility::UAV> &uav, int uav_idx,  int border, int resolution) {

	utility::UAV &cur_uav = uav[uav_idx];
	int top_layer_idx = layers.size() - 1;
	int height = layers[0].rows;
	int width = layers[0].cols;

	utility::point2I tp_idx_cp = projectToCertainLayer(cur_uav.state.position, resolution, top_layer_idx, border, height, width);
	utility::point2I tp_idx_cag = projectToCertainLayer(cur_uav.area_guider, resolution, top_layer_idx, border, height, width);

	//step 1 find best area
	vector<vector<bool>> visited(layers[top_layer_idx].cols, vector<bool>(layers[top_layer_idx].rows, false));
	// for(auto c:uav){
	// 	if(c.id == uav_idx)
	// 		continue;
	// 	utility::point2I idx_cag = projectToCertainLayer(c.area_guider, resolution, top_layer_idx, border, height, width);
	// 	visited[idx_cag.y][idx_cag.x] = true;
	// }

	// if(!visited[tp_idx_cag.y][tp_idx_cag.x] && !tp_idx_cp.isSamePoint(tp_idx_cag))
	// 	return cur_uav.area_guider;

	int uav_dir = (4.0 * (cur_uav.state.velocity.y + 3.0 * PI / 4.0) / PI + 0.5);
	uav_dir = uav_dir % 8;

	double max_val = 0;
	int temp_x, temp_y;
	int best_x, best_y;
	int best_dir;

	int cur_idx_x = tp_idx_cp.x;
	int cur_idx_y = tp_idx_cp.y;
	//find best grid at top layer
	for (int j = 0; j < 8; j++) {
		temp_x = cur_idx_x + DIRECTION[j][0];
		temp_y = cur_idx_y + DIRECTION[j][1];
		if(temp_x < 0 ||temp_x >= (int)layers[top_layer_idx].cols || 
		   temp_y < 0 ||temp_y >= (int)layers[top_layer_idx].rows)
			continue;
		double gain = abs(uav_dir - j);
		gain = gain < 4 ? 8 - gain : gain;
		double val = gain * layers[top_layer_idx].at<uint16_t>(temp_y, temp_x);
		if(!visited[temp_y][temp_x] &&  val > max_val){
			max_val = val;
			best_x = temp_x;
			best_y = temp_y;
			best_dir = j;
		}
	}

	// Search down to the best
	int cur_layer_idx = top_layer_idx - 1;
	cur_idx_x = best_x * 2 + 1;
	cur_idx_y = best_y * 2 + 1;
	while (cur_layer_idx >= 1) {
		double max_val = 0;
		int temp_x, temp_y;
		for (int j = 0; j < 9; j++) {
			temp_x = cur_idx_x + Fdrections[j][0];
			temp_y = cur_idx_y + Fdrections[j][1];

			if(temp_x < 0 ||temp_x >= (int)layers[cur_layer_idx].cols ||
		    	temp_y < 0 ||temp_y >= (int)layers[cur_layer_idx].rows)
				continue;

			utility::point2D temp_p = utility::point2D((temp_x + 0.5) * pow(2, cur_layer_idx) * resolution, 
													   (temp_y + 0.5) * pow(2, cur_layer_idx) * resolution);

			double dd = utility::dist(temp_p, cur_uav.state.position) / (pow(2, cur_layer_idx) * resolution);
			double d_gain = 1.0 / (1.0 + dd * dd);
			if (max_val <= layers[cur_layer_idx].at<uint16_t>(temp_y, temp_x) * d_gain) {
				max_val = layers[cur_layer_idx].at<uint16_t>(temp_y, temp_x) * d_gain;
				best_x = temp_x;
				best_y = temp_y;
			}
		}
		cur_idx_x = (best_x) * 2;
		cur_idx_y = (best_y) * 2;

		cur_layer_idx--;
	}

	cur_idx_x = (best_x + 0.5) * pow(2.0, cur_layer_idx + 1) - border;
	cur_idx_y = (best_y + 0.5) * pow(2.0, cur_layer_idx + 1) - border;
	printf("best_x:%d, best_y:%d, cur_idx_x:%d, cur_idx_y:%d, uav_dir:%d, best_dir:%d\n", best_x, best_y, cur_idx_x, cur_idx_y, uav_dir, best_dir);
	return utility::point2D(cur_idx_x * resolution, cur_idx_y * resolution);
}

double getUncertanty(const vector<cv::Mat> &layers, const utility::point2D &position, int border, int resulution, int used_layer_num){
	int height = layers[0].rows;
	int width = layers[0].cols;

	double uncertanty = 0;
	double weight = 1.0;
	double sum_w = 0;
	for(int i = 0; i < used_layer_num; i++){
		utility::point2I p = projectToCertainLayer(position, resulution, i, border, height, width);
		sum_w += weight;
		uncertanty += weight * layers[i].at<uint16_t>(p.y, p.x);
		weight *= 0.5;
	}
	return uncertanty / sum_w;
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

	printf("tao_=%f, weight_=%f, c1_=%f, c2_=%f, w2_=%f, eta_=%f\n", tao_, weight_, c1_, c2_, w2_, eta_);
	printf("max_iteration_=%f, particle_num_=%f, used_layer_num_=%f\n", max_iteration_, particle_num_, used_layer_num_);


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
	border_ = 0;
	swarm_ = (particle*)malloc(particle_num_ * sizeof(particle));
	int height = global_map_.height_ + 2 * border_;
	int width  = global_map_.width_  + 2 * border_;
	for(uint8_t layer = 0 ;layer < MAX_LAYER_NUM;layer++){
		cv::Mat mat(height, width, CV_16UC1, cv::Scalar(0));
		layers_.push_back(mat);
		height = (height + 1) / 2;//ceil
		width  = (width + 1) / 2;
	}

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

void PSO::getNextPoint(utility::MAP &global_map, vector<utility::RADAR> &radar, vector<utility::TARGET> &target, vector<utility::UAV> &uav, int cur_T, int uav_idx)
{
	global_map_ = global_map;
	radar_ = &radar;
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

	area_guider_ = findBestPoints(layers_, uav, uav_idx, border_, global_map_.resolution_);
	base_angle_ = area_guider_ - cur_uav_->state.position;
	base_angle_ = base_angle_ / base_angle_.distance();
	// base_angle_ = base_angle_ >= 0 ? base_angle_ : 2 * PI + base_angle_;//0~2*pi

	infulence_.clear();
	for(int i = 0; i < target_->size(); i++){
		if(cur_T - cur_uav_->target_state[i].second > FORGET_TIME) continue;

		int d = utility::dist(cur_uav_->target_state[i].first.position, cur_uav_->state.position);
		d /= 2000;
		if(d < cur_uav_->k_d){
			vector<int> uav_idx;
			for(int j = 0; j < uav.size();j++){
				if(j == cur_uav_->id) continue;
				int d = utility::dist(cur_uav_->target_state[i].first.position, uav[j].state.position);
				d /= 2000;
				if(d < uav[j].k_d);
					uav_idx.push_back(j);
			}
			infulence_.push_back(make_pair(uav_idx, i));
		}
	}

#if DEBUG
	if(uav_idx == DEBUG_IDX){
		cv::Mat temp;
		temp.copySize(layers_[0]);
		cv::convertScaleAbs(layers_[0], temp, 0.2);

		for(int i = 0; i < target_->size(); i++){
			int x_idx = target_->at(i).state.position.x / global_map_.resolution_ + border_;
			int y_idx = target_->at(i).state.position.y / global_map_.resolution_ + border_;
			cv::circle(temp, cv::Point(x_idx, y_idx), 5, (125, 125, 125), 5);
		}
		cv::namedWindow("111", cv::WINDOW_AUTOSIZE);
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
	int max_ite = 1000;
	while (j < particle_num_ && max_ite--) {
		double length = 1.1 * cur_uav_->search_r + (cur_uav_->state.velocity.x) * (rand() % 1000) / 1000.0;
		double theta = 2 * PI * ((rand() % 1000) / 1000.0);
		temp_particle->state.position.x = length * cos(theta) + cur_uav_->state.position.x;
		temp_particle->state.position.y = length * sin(theta) + cur_uav_->state.position.y;
		
		bool bad_location = false;
		if(!global_map_.isInBound(temp_particle->state.position, 1000.0))
			bad_location = true;
		else {
			for(auto radar:(*radar_)){
				if(radar.isInRader(temp_particle->state.position, 200.0)){
					bad_location = true;
					break;
				}
			}
		}

		if(bad_location)
			continue;

		temp_particle->state.velocity.x = cur_uav_->Vel_limite[0];//+ (cur_uav_->Vel_limite[1] - cur_uav_->Vel_limite[0]) * ((rand() % 1000) / 1000.0);//
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

	bool bad_location = false;
	if(!global_map_.isInBound(particle->state.position, 1000.0))
		bad_location = true;
	else {
		for(auto radar:(*radar_)){
			if(radar.isInRader(particle->state.position, 1000.0)){
				bad_location = true;
				break;
			}
		}
	}

	if (bad_location)
		return -100000;


	//fitness = (1-w)*fitness_1 + w*fitness_2
	double fitness = 0;
	double fitness_1 = 0;
	double fitness_2 = 0;
	double fitness_3 = 0;
	//fitness1
	//double t = global_map_->computeUncetanty(particle->state, cur_uav_->search_r, cur_T_);
	double t = getUncertanty(layers_, particle->state.position, border_, global_map_.resolution_, used_layer_num_) / FORGET_TIME;
	//double p  = (1 - exp(-tao * t)) ;
	double d = utility::dubinsDistance(cur_uav_->state, particle->state, min_R_);
	d = (d - cur_uav_->search_r) / cur_uav_->search_r ;
	fitness_1 =  t / (0.1 + d * d ) * 0.1;

	for (int uav_id = 0; uav_id < uav_->size(); uav_id++) {  //uav avoidance
		if(uav_id == cur_uav_->id) continue;
	 	d = utility::dist((*uav_)[uav_id].state.position, particle->state.position);
		d /= cur_uav_->search_r + (*uav_)[uav_id].search_r;
		if(d < 1) fitness_1 -= 1.0 / ( 0.001 + d * d );

	}

	for(auto pa:infulence_){
		int target_id = pa.second;
		d = utility::dist(cur_uav_->target_state[target_id].first.position, particle->state.position);
		d /= 2000;
		if(d < 1) fitness_2 += 1.0 * cur_uav_->k_att / ( 0.1 + sqrt(d));
		else if(d < cur_uav_->k_d) fitness_2 += 1.0 * cur_uav_->k_att / ( 0.1 + d * d);
		for(auto u_idx:pa.first){
			utility::UAV &u = (*uav_)[u_idx];
			d = utility::dist(u.traj_Point.position, particle->state.position);
			d /= 2000;

			if(d < 1) fitness_2 -= 1.0 * u.k_rep / ( 0.1 + sqrt(d) );
			else if(d < cur_uav_->k_d) fitness_2 -= 1.0 * u.k_rep / ( 0.1 + d * d);
		}
	}

	//fitness3
	utility::point2D particle_angle = particle->state.position - cur_uav_->state.position;
	particle_angle = particle_angle / particle_angle.distance();
	double angle = acos(particle_angle.x * base_angle_.x + particle_angle.y * base_angle_.y);

	fitness_3 = 1.0/(1.0 + exp(angle));

	fitness = eta_ * fitness_1 + (1.0 - eta_) * fitness_3 + w2_ * fitness_2 ;

	// if(SHOW){
	// 	SHOW = false;
	// 	printf("Gbest_fitness_ = %lf, fitness_1 = %lf, fitness_2 = %lf, fitness_3 = %lf, angle = %lf\n", Gbest_fitness_, fitness_1, fitness_2, fitness_3, angle);
	// }

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
	Gbest_fitness_ = -10000.0;

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

			if (cur_uav_->id == DEBUG_IDX && (p_idx % 5) ==0 && (iteration % 10) == 0 && cur_T_ <= 5000){
				SHOW = true;
				output_particle_state << temp_part->state.position.x << " " << temp_part->state.position.y << " ";
			}
		}
		if (cur_uav_->id == DEBUG_IDX && (iteration % 10) == 0 && cur_T_ <= 5000) {
			double d = utility::dubinsDistance(cur_uav_->state, Gbest_state_, min_R_);

			output_particle_state << endl;
			output_uav_state  << cur_uav_->state.position.x << " " << cur_uav_->state.position.y << " " << cur_uav_->state.velocity.y << endl;
			output_best_state << Gbest_state_.position.x << " " << Gbest_state_.position.y << " " 
							  << area_guider_.x << " " << area_guider_.y << " "
							  << Gbest_fitness_ << " " << d << endl;
		}
	}
	
	//printf("iteration = %d, p_idx = %d, Gbest_fitness_ = %lf\n", iteration, p_idx, Gbest_fitness_);
	cur_uav_->traj_Point = Gbest_state_;
	cur_uav_->area_guider = area_guider_;
	//uav_->updatePrevPoses();
	return;
};
