
#include "multiple_search_and_track.h"
#include <thread>
#define REPLANNING_NUM 1

#define DESTORY_MODE 1

void trim(string& src, char decimal){
	int idx = 0;
	if(!src.empty()){
		while((idx = src.find(decimal, idx)) != string::npos)
			src.erase(idx, 1);
	}
	return;
}

void StringTArray(string& src, const char split, vector<double>& dst){
	if(src.empty())
		return;

	trim(src, ' ');
	dst.clear();
	istringstream iss(src);
	string token;

	while(getline(iss, token, split)){

		dst.push_back(std::stod(token.c_str()));
	}
	return;
}

void initTargetFromString(string& status, const double* limite, vector<utility::TARGET>& targets){
	if(status.empty()){
		return;
	}

	vector<double> status_v;
	StringTArray(status, ',', status_v);

	for(int i = 0; i < status_v.size(); i += 4){
		utility::TARGET temp_target;
		temp_target.state.position.x   = status_v[i];
		temp_target.state.position.y   = status_v[i + 1];
		temp_target.state.velocity.x   = status_v[i + 2];
		temp_target.state.velocity.y   = status_v[i + 3];
		temp_target.state.accelerate.x = 0;//
		temp_target.state.accelerate.y = 0;//
		temp_target.Vel_limite[0]      = limite[0];
		temp_target.Vel_limite[1]      = limite[1];
		temp_target.Acc_limite_x[0]    = limite[2];
		temp_target.Acc_limite_x[1]    = limite[3];
		temp_target.Acc_limite_y[0]    = limite[4];
		temp_target.Acc_limite_y[1]    = limite[5];
		targets.push_back(temp_target);
	}
	return;
}

void initUAVFromString(string& status, const double* limite, vector<utility::UAV>& uavs, int start_id){
	if(status.empty()){
		return;
	}

	vector<double> status_v;
	StringTArray(status, ',', status_v);

	for(int i = 0; i < status_v.size(); i += 4){
		utility::UAV uav_temp;
		uav_temp.state.position.x   = status_v[i];
		uav_temp.state.position.y   = status_v[i + 1];
		uav_temp.state.velocity.x   = status_v[i + 2];
		uav_temp.state.velocity.y   = status_v[i + 3];
		uav_temp.state.accelerate.x = 0;
		uav_temp.state.accelerate.y = 0;

		uav_temp.area_guider = uav_temp.state.position;

		uav_temp.Vel_limite[0]   = limite[0];
		uav_temp.Vel_limite[1]   = limite[1];
		uav_temp.Acc_limite_x[0] = limite[2];
		uav_temp.Acc_limite_x[1] = limite[3];
		uav_temp.Acc_limite_y[0] = limite[4];
		uav_temp.Acc_limite_y[1] = limite[5];
		uav_temp.h               = limite[6];
		uav_temp.search_r        = limite[7];
		uav_temp.k_att = limite[8];
		uav_temp.k_rep = limite[9];
		uav_temp.k_d   = limite[10];

		uav_temp.id = uavs.size() + start_id;
		uav_temp.coverd_area_cnt  =  1;
		uav_temp.track_target_num = -1;

		uavs.push_back(uav_temp);
	}
	return;
}

void initRadarFromString(string& status, vector<utility::RADAR>& radar){
	if(status.empty()){
		return;
	}

	vector<double> status_v;
	StringTArray(status, ',', status_v);

	for(int i = 0; i < status_v.size(); i += 3){
		utility::RADAR radar_temp;
		radar_temp.center_point.x = status_v[i];
		radar_temp.center_point.y = status_v[i+1];
		radar_temp.R			  = status_v[i+2];
		radar.push_back(radar_temp);
	}
	return;
}

void MultipleSearchAndTrack::initParams(const string& config_file_path){
	dictionary *dict = nullptr;
	dict = iniparser_load(config_file_path.c_str());
	if(dict == nullptr){
		printf("fail to parser from %s\n", config_file_path.c_str());
		return;
	}

	//common param
	run_mode_   = iniparser_getint(dict, "common_param:run_mode", -1);
	max_simulation_step_   = iniparser_getint(dict, "common_param:max_simulation_step", -1);

	//uav
	string uav_status_fast = iniparser_getstring(dict, "uav_param:uav_status_fast", "");
	double uav_limit_fast[11] = {60., 90., -0.6, 0.6, -G * tan(20.0 * PI / 180.0), G * tan(20.0 * PI / 180.0), 800, 2000, 2.0, 2.1, 2.0};
	initUAVFromString(uav_status_fast, uav_limit_fast, uav_, 0);

	string uav_status_middle = iniparser_getstring(dict, "uav_param:uav_status_middle", "");
	double uav_limit_middle[11] = {30., 50., -0.5, 0.5, -G * tan(25.0 * PI / 180.0), G * tan(25.0 * PI / 180.0), 400, 1500, 5.0, 5.1, 5.0};
	initUAVFromString(uav_status_middle, uav_limit_middle, uav_, 0);

	string uav_status_slow = iniparser_getstring(dict, "uav_param:uav_status_slow", "");
	//vel_limit, Acc_limite_x, Acc_limite_y, h, search_r
	double uav_limit_slow[11] = {0., 30., -0.4, 0.4, -G * tan(30.0 * PI / 180.0), G * tan(30.0 * PI / 180.0), 400, 3000, 5.0, 5.1, 5.0};
	initUAVFromString(uav_status_slow, uav_limit_slow, uav_, 0);

	string uav_status_extre = iniparser_getstring(dict, "uav_param:uav_status_extre", "");
	double uav_limit_extre[11] = {30., 50., -0.5, 0.5, -G * tan(25.0 * PI / 180.0), G * tan(25.0 * PI / 180.0), 400, 1500, 5.0, 5.1, 5.0};
	initUAVFromString(uav_status_extre, uav_limit_extre, extre_uav_, uav_.size());
	extre_step_ = iniparser_getint(dict, "uav_param:extre_step", -1);

	//target
	string target_status_slow = iniparser_getstring(dict, "target_param:target_status_slow", "");
	//vel_limit, Acc_limite_x, Acc_limite_y,
	double target_limit_slow[6] = {0.,15., -0.4, 0.4, -G * tan( 20.0 * PI / 180.0 ), G * tan( 20.0 * PI / 180.0 )};
	initTargetFromString(target_status_slow, target_limit_slow, target_);

	string target_status_middle = iniparser_getstring(dict, "target_param:target_status_middle", "");
	double target_limit_middle[6] = {0., 15., -0.4, 0.4, -G * tan( 20.0 * PI / 180.0 ), G * tan( 20.0 * PI / 180.0 )};
	initTargetFromString(target_status_middle, target_limit_middle, target_);

	string target_status_fast = iniparser_getstring(dict, "target_param:target_status_fast", "");
	double target_limit_fast[6] = {0., 15., -0.4, 0.4, -G * tan( 20.0 * PI / 180.0 ), G * tan( 20.0 * PI / 180.0 )};
	initTargetFromString(target_status_fast, target_limit_fast, target_);

	//map
	resolution_   = iniparser_getint(dict, "map_param:resolution", -1);
	width_  = iniparser_getint(dict, "map_param:width", -1);
	height_ = iniparser_getint(dict, "map_param:height", -1);

	//radar
	string radar_params = iniparser_getstring(dict, "radar_param:radar_params", "");
	initRadarFromString(radar_params, radar_);

	//file_path
	const char *uav_path = iniparser_getstring(dict, "file_path:uav_path", "");
	output_uav_.open(uav_path, ios::binary);

	const char *target_path = iniparser_getstring(dict, "file_path:target_path", "");
	output_target_.open(target_path, ios::binary);

	const char *traj_Point_path = iniparser_getstring(dict, "file_path:traj_Point_path", "");
	output_traj_Point_.open(traj_Point_path, ios::binary);

	const char *area_Point_path = iniparser_getstring(dict, "file_path:area_Point_path", "");
	output_area_Point_.open(area_Point_path, ios::binary);

	const char *cove_rate_PSO = iniparser_getstring(dict, "file_path:cove_rate_PSO", "");
	output_coverage_rate_.open(cove_rate_PSO, ios::binary);

	const char *time_consume = iniparser_getstring(dict, "file_path:time_consume", "");
	output_time_.open(time_consume, ios::binary);

	const char *destroyed_target_num = iniparser_getstring(dict, "file_path:destroyed_target_num", "");
	output_target_num_.open(destroyed_target_num, ios::binary);

	return;
};

void MultipleSearchAndTrack::updateUAVStatesInDubinsState(utility::UAV* uav) {
	double minR = uav->state.velocity.x * uav->state.velocity.x / (uav->Acc_limite_y[1]);
	double l = dist(uav->traj_Point.position, uav->state.position);
	double theTa1 = acos((uav->traj_Point.position.x - uav->state.position.x) / l);
	theTa1 = (uav->traj_Point.position.y - uav->state.position.y)>0 ? theTa1 : 2 * PI - theTa1;
	double theTa2 = uav->state.velocity.y - theTa1;
	cout << "theTa2 =" << theTa2 * 180 / PI << endl;
	if (theTa2 > PI)
		theTa2 -= 2 * PI;
	else if (theTa2 <-PI)
		theTa2 += 2 * PI;
	double theTi, d;

	if (theTa2<-PI / 2) {
		theTa2 += PI;
		d = sqrt((l - minR * sin(theTa2))*(l - minR * sin(theTa2)) + minR * cos(theTa2)*minR*cos(theTa2));
		theTi = asin(minR*cos(theTa2) / d) + asin(minR / d) + theTa1;
	}
	else if (theTa2<0) {
		theTa2 *= -1;
		d = sqrt((l - minR * sin(theTa2))*(l - minR * sin(theTa2)) + minR * cos(theTa2)*minR*cos(theTa2));
		theTi = asin(minR / d) - asin(minR*cos(theTa2) / d) + theTa1;
	}
	else if (theTa2>PI / 2) {
		theTa2 = PI - theTa2;
		d = sqrt((l - minR * sin(theTa2))*(l - minR * sin(theTa2)) + minR * cos(theTa2)*minR*cos(theTa2));
		theTi = -asin(minR / d) - asin(minR*cos(theTa2) / d) + theTa1;
	}
	else {
		d = sqrt((l - minR * sin(theTa2))*(l - minR * sin(theTa2)) + minR * cos(theTa2)*minR*cos(theTa2));
		theTi = -asin(minR / d) + asin(minR*cos(theTa2) / d) + theTa1;
	}

	if (isnan(theTa1)) {
		cout << "theTa1 is nan" << endl;
		theTi = theTa2;
	}
	cout << "uav->state.velocity.y = " << uav->state.velocity.y << endl;
	cout << "theTi = " << theTi << endl;
	double q0[3] = { uav->state.position.x,uav->state.position.y,uav->state.velocity.y };//start point state
	double q1[3] = { uav->traj_Point.position.x,uav->traj_Point.position.y,theTi };//end point state;

	HybridAStar::dubins_init(q0, q1, minR, path_);
	double velocity = uav->state.velocity.x;
	double t = DT * velocity, total_lenth = HybridAStar::dubins_path_length(path_);

	while (!uav->path_.empty())
		uav->path_.pop();

	cout << "total_lenth" << total_lenth << endl;
	while (t <= total_lenth) {
		//cout << "t" << t <<endl;
		double temp_point[3];
		HybridAStar::dubins_path_sample(path_, t, temp_point);
		utility::State temp;
		temp.position.x = temp_point[0];
		temp.position.y = temp_point[1];
		temp.velocity.x = velocity;
		temp.velocity.y = temp_point[2];
		uav->path_.push(temp);
		t += velocity * DT;
	}
	return;
}

utility::point2D MultipleSearchAndTrack::computeRepulsion(utility::point2D& start, utility::point2D& obstacle, double R) {
	utility::point2D F_rep = start- obstacle;
	double D = F_rep.distance();
	F_rep = F_rep / D;

	D /= 1000;
	R /= 1000;

	double scale = 0;
	if(D > R){
		D -= R;
		if(D < 2.0)
			scale = 1.0 / (D+0.00001) * (D+0.00001);
	}

	return F_rep * scale;
};

double MultipleSearchAndTrack::computeEnage(utility::State  & start, utility::point2D& obstacle,double zero_dist) {
	utility::point2D F_rep = obstacle -start.position;

	double theta = utility::Mod2pi(F_rep.angle() - start.velocity.y);
	double res = 0;
	theta = 1 + cos(theta);
	if (F_rep.distance() < resolution_)
		res = theta;
	else if (F_rep.distance() < zero_dist)
		res = theta / (F_rep.distance() / resolution_);
	return res;
};

double MultipleSearchAndTrack::computeAttEnage(utility::State& start, utility::State& end, double zero_dist) {

	double d = utility::dubinsDistance(start, end, 1);
	if (d > zero_dist)
		d =  0;
	else if (d > 0.0000001)
		d = 1 / d;
	return d;
};

utility::point2D MultipleSearchAndTrack::computeAttraction(utility::point2D& start, utility::point2D& goal) {
	utility::point2D F_att = goal - start;
	return F_att / 10.0;
};

void MultipleSearchAndTrack::updateArtificalPotentialFieldStateImpl(utility::UAV *uav, utility::State &goal, utility::State &next) {
	double k_att = 1.0, k_ref = 80.0;

	//compute Attraction
	utility::point2D F_att = computeAttraction(uav->state.position, goal.position);

	utility::point2D F_rep(0, 0);
	//compute Radar
	for(auto radar:radar_){
		F_rep = F_rep + computeRepulsion(uav->state.position, radar.center_point, radar.R);
	}

	for(auto u:uav_){
		if(u.id != uav->id)
			F_rep = F_rep + computeRepulsion(uav->state.position, u.state.position, 0);
	}

	//printf("F_rep.distance() = %f, F_att.distance() = %f\n", F_rep.distance(), F_att.distance());

	//sum
	utility::point2D F = F_att * k_att + F_rep * k_ref;
	double dur_x[2] = { max(uav->Vel_limite[0], uav->state.velocity.x + uav->Acc_limite_x[0] * DT) ,min(uav->Vel_limite[1], uav->state.velocity.x + uav->Acc_limite_x[1] * DT) };
	double dur_y[2] = { uav->Acc_limite_y[0] * DT, uav->Acc_limite_y[1] * DT };

	double angle_best, lenth_best;
	angle_best = F.angle() - uav->state.velocity.y;
	if (angle_best > PI)
		angle_best -= 2 * PI;
	else if (angle_best <-PI)
		angle_best += 2 * PI;

	lenth_best = min(uav->Vel_limite[1], max(uav->Vel_limite[0], (F*DT).distance()));
		
	utility::point2D Vel_best, F_body;
	Vel_best.x = min(dur_x[1], max(dur_x[0], lenth_best * cos(angle_best)));
	Vel_best.y = min(dur_y[1], max(dur_y[0], lenth_best * sin(angle_best)));

	F_body.x = (Vel_best.x - uav->state.velocity.x) / DT;
	F_body.y = Vel_best.y / DT;

	F.x = F_body.x * cos(uav->state.velocity.y) - F_body.y * sin(uav->state.velocity.y);
	F.y = F_body.x * sin(uav->state.velocity.y) + F_body.y * cos(uav->state.velocity.y);

	next.position.x = uav->state.position.x + uav->state.velocity.x * cos(uav->state.velocity.y) * DT + 0.5 * F.x * DT * DT;
	next.position.y = uav->state.position.y + uav->state.velocity.x * sin(uav->state.velocity.y) * DT + 0.5 * F.y * DT * DT;
	double tmep[2] = { uav->state.velocity.x * cos(uav->state.velocity.y) + DT * F.x ,uav->state.velocity.x* sin(uav->state.velocity.y) + DT * F.y };
	next.velocity.x = sqrt(tmep[0] * tmep[0] + tmep[1] * tmep[1]);
	next.velocity.y = atan2(tmep[1], tmep[0]) >= 0 ? atan2(tmep[1], tmep[0]) : 2 * PI + atan2(tmep[1], tmep[0]);
	return;
}

void MultipleSearchAndTrack::init(const string& config_file_path) {

	initParams(config_file_path);

	target_state_ = new vector<vector<double>>(uav_.size() + extre_uav_.size(), vector<double>(target_.size(), 0));
	tracked_ = new vector<bool>(target_.size(), false);
	path_ = new HybridAStar::DubinsPath();
	
	//map
	printf("initializing map\n");
	global_map_.resize(uav_.size() + extre_uav_.size());

	//setParam
	aco_.setParam(10, 500, 2, 4, 100, 0.8);
	pso_.create(config_file_path);

	srand((unsigned)time(NULL));

	for ( int i = 0; i < uav_.size(); i++) {
		utility::UAV& uav_temp = uav_[i];
		for (int j = 0; j < target_.size(); j++) {
			utility::State temp_state;
			temp_state.position.x = 0;
			temp_state.position.y = 0;
			uav_temp.target_state.push_back(make_pair(temp_state, -FORGET_TIME));
			uav_temp.Tj[j] = 1;
		}

		global_map_[uav_[i].id].init(width_, height_, resolution_, -FORGET_TIME);
		global_map_[uav_[i].id].updateMap(&uav_temp, cunt);
		union_.parent.push_back(uav_temp.id);
	}

	printf("initialize success\n");
};

void MultipleSearchAndTrack::updateTargetStates() {
	printf("updateing targets state\n");

	vector<utility::TARGET>::iterator temp_target = target_.begin();
	int target_tag = 0;
	for ( ; target_tag < target_.size(); temp_target++, target_tag++) {
		double  Acc_X = temp_target->Acc_limite_x[0] + (temp_target->Acc_limite_x[1] - temp_target->Acc_limite_x[0]) *(rand() % 1000 / 1000.0);
		double  Acc_Y = temp_target->Acc_limite_y[0] + (temp_target->Acc_limite_y[1] - temp_target->Acc_limite_y[0]) *(rand() % 1000 / 1000.0);
		temp_target->state.position.x += temp_target->state.velocity.x * cos(temp_target->state.velocity.y) * DT + 0.5 * Acc_X * DT * DT;//
		temp_target->state.position.y += temp_target->state.velocity.x * sin(temp_target->state.velocity.y) * DT + 0.5 * Acc_Y * DT * DT;//

		double line_X = temp_target->state.velocity.x * cos(temp_target->state.velocity.y) + Acc_X * DT;//
		double line_Y = temp_target->state.velocity.x * sin(temp_target->state.velocity.y) + Acc_Y * DT;

		double line_vel = sqrt(line_X * line_X + line_Y * line_Y);
		temp_target->state.velocity.x = min(temp_target->Vel_limite[1], max(temp_target->Vel_limite[0], line_vel));

		if(line_vel > 0.00001)
			temp_target->state.velocity.y = atan2(line_Y, line_X) >= 0 ? atan2(line_Y, line_X): 2 * PI + atan2(line_Y, line_X);
		else
			temp_target->state.velocity.y = 2 * PI * (rand() % 1000 / 1000.0);
		// if(cunt % 20 == 0)//random movement
		// 	temp_target->state.velocity.y = 2 * PI * ((rand() % 1000) / 1000.0);//

		if (temp_target->state.position.x <= 0) {
			temp_target->state.position.x = 0;
			temp_target->state.velocity.y = PI * (0.5 - (rand() % 1000) / 1000.0);//
		}
		else if (temp_target->state.position.x >= width_ * resolution_) {
			temp_target->state.position.x = width_ * resolution_;
			temp_target->state.velocity.y = PI * (0.5 + (rand() % 1000) / 1000.0);//
		}

		if (temp_target->state.position.y <= 0) {
			temp_target->state.position.y = 0;
			temp_target->state.velocity.y = PI * ((rand() % 1000) / 1000.0);//
		}
		else if (temp_target->state.position.y >= height_ * resolution_) {
			temp_target->state.position.y = height_ * resolution_;
			temp_target->state.velocity.y = PI * (1 + (rand() % 1000) / 1000.0);//
		}

		bool istracking = false;
		for(int i = 0; i < uav_.size(); i++){
			if (dist(uav_[i].state.position, temp_target->state.position) <= 0.5 * uav_[i].search_r) {
				istracking = true;//target been tracked
				cout << "target " << target_tag + 1 << " is successfully tracked by uav " << i + 1 << endl;
				break;
			}
		}
#if !DESTORY_MODE
		if (istracking){
			(*tracked_)[target_tag] = true;
			for(int i = 0; i < uav_.size(); i++)
				uav_[i].Tj[target_tag] = -1;
		}else{
			(*tracked_)[target_tag] = false;
			for(int i = 0; i < uav_.size(); i++)
				uav_[i].Tj[target_tag] = 1;
		}
#endif 
		output_target_ << temp_target->state.position.x << " " << temp_target->state.position.y << " ";
	}
	output_target_ << endl;
	return;
};

void MultipleSearchAndTrack::updateUAVStates() {
	printf("updateing UAVs state\n");
	vector<utility::UAV>::iterator cur_uav = uav_.begin();
	int cur_uav_num = uav_.size();
	int i = 0;
	for (; i < cur_uav_num; i++, cur_uav++) {

		clock_t start, finish;
		double  duration;
		if (cur_uav->track_target_num == -1 && cunt % REPLANNING_NUM == 0) {//
			printf("updateing UAVs(%d) state\n", i);
			start = clock();
			switch (run_mode_){
			case 0:{
				pso_.getNextPoint(global_map_[cur_uav->id], radar_, target_, uav_, cunt, i);
				break;
			};
			case 1:{
				utility::point2I pose_in, pose_out;
				pose_in.x = uav_[i].state.position.x / resolution_;
				pose_in.x = pose_in.x < 0 ? 0 : pose_in.x>width_ - 1 ? width_ - 1 : pose_in.x;
				pose_in.y = uav_[i].state.position.y / resolution_;
				pose_in.y = pose_in.y<0 ? 0 : pose_in.y>height_ - 1 ? height_ - 1 : pose_in.y;
				int dir = 4 * (uav_[i].state.velocity.y + 3 * PI / 4) / PI;
				dir = dir % 8;

				pose_out = aco_.run(global_map_[cur_uav->id], pose_in, dir, uav_[i].search_r / resolution_, i, cunt);

				uav_[i].traj_Point.position.x = pose_out.x * resolution_;
				uav_[i].traj_Point.position.y = pose_out.y * resolution_;
				break;
			};
			default:
				break;
			}

			finish = clock();
			duration = (double)(finish - start);
			output_time_ << duration << endl;
		}

		updateArtificalPotentialFieldStateImpl( &(*cur_uav), uav_[i].traj_Point, uav_[i].state);

		output_uav_ << uav_[i].state.position.x << " " << uav_[i].state.position.y << " " << uav_[i].search_r << " ";
		output_traj_Point_ << uav_[i].traj_Point.position.x << " " << uav_[i].traj_Point.position.y << " ";
		//output_area_Point_ << uav_[i].area_guider.x << " " << uav_[i].area_guider.y << " ";
		global_map_[cur_uav->id].updateMap(&(*cur_uav), cunt);

		for (int j = 0; j < target_.size(); j++) {//target inf0
			if (dist(target_[j].state.position, uav_[i].state.position)<uav_[i].search_r) {
				uav_[i].target_state[j].first = target_[j].state;
				uav_[i].target_state[j].second = cunt;
#if DESTORY_MODE
				(*tracked_)[j] = true;
				//cout << "target" << j + 1 << " has been destoryed by uav" << i + 1 << endl;
#endif
			}
		}
	}

	if(cunt < extre_step_){
		for(auto c:extre_uav_){
			output_uav_ << c.state.position.x << " " << c.state.position.y << " " << c.search_r << " ";
			output_traj_Point_ << c.traj_Point.position.x << " " << c.traj_Point.position.y << " ";
		}
	}

	output_uav_ << endl;
	output_traj_Point_ << endl;
	output_time_ << endl;
	//output_area_Point_ << endl;
	return;
}

void MultipleSearchAndTrack::informationShare() {
	printf("exchanging information\n");
	int cur_uav_num = uav_.size();

	for (int i = 0; i < cur_uav_num; i++) {//reset
		union_.parent[i] = i;
		uav_[i].coverd_area_cnt = 0;
	}
	for (int i = 0; i < cur_uav_num; i++) {
		for (int j = i; j < cur_uav_num; j++) {
			if (dist(uav_[i].state.position, uav_[j].state.position) <= COMMUNICATION_R) {//union
				union_.join(i, j);
				for (int k = 0; k < target_.size(); k++) {//exchange target info
					if (uav_[i].target_state[k].second < uav_[j].target_state[k].second)
						uav_[i].target_state[k] = uav_[j].target_state[k];
					else
						uav_[j].target_state[k] = uav_[i].target_state[k];
					int x = uav_[i].target_state[k].first.position.x / resolution_;
					int y = uav_[i].target_state[k].first.position.y / resolution_;
					if(x<0||x>=width_ || y<0||y>=height_)
						continue;
					// if (global_map_[j](x, y).search_time > uav_[j].target_state[k].second + 1) {
					// 	uav_[i].target_state[k].second = -FORGET_TIME;
					// 	uav_[j].target_state[k].second = -FORGET_TIME;
					// }
				}
			}
		}
	}

	union_.setup_barrel();//setup barrel

	//dense map
	int uav_idx;
	for (int i = 0; i < height_; i++) {
		for (int j = 0; j < width_; j++) {
			for (int k = 0; k < union_.barrel.size(); k++) {
				int max_count = -FORGET_TIME;
				for (int l = 0 ; l < union_.barrel[k].size(); l++) {
					uav_idx = union_.barrel[k][l];
					max_count = std::max(max_count, global_map_[uav_idx](i,j).search_time);//find the lowest search time				}
				}
				for (int l = 0; l < union_.barrel[k].size(); l++) {
					uav_idx = union_.barrel[k][l];
					global_map_[uav_idx](i,j).search_time = max_count;//set the search time
					global_map_[uav_idx].search_time_.at<uint16_t>(i, j) = max_count;
					if (cunt - max_count < FORGET_TIME)
						uav_[union_.barrel[k][l]].coverd_area_cnt += (double)(FORGET_TIME + max_count - cunt) / FORGET_TIME;//count the coverd area
				}
			}
		}
	}
	double cove_rate = (double)uav_[0].coverd_area_cnt / (height_ * width_);
	output_coverage_rate_ << cove_rate << endl;

	return;
}

void MultipleSearchAndTrack::updateMission() {
	printf("updateing mission\n");
	double d_uav_target = 0;
	int t_last = 0;
	double coef = 0;
	int cur_uav_num = uav_.size();
	for (int i = 0; i < cur_uav_num; i++) {
		for (int j = 0; j < target_.size(); j++) {//
			d_uav_target = utility::dist(uav_[i].state.position, target_[j].state.position) ;
			t_last = cunt - uav_[i].target_state[j].second;
			coef = ( target_[j].Vel_limite[1] + 1.0) / uav_[i].Vel_limite[1];
			if (t_last >= FORGET_TIME )
				(*target_state_)[i][j] = 0;//
			else {
				d_uav_target /= 10000.0;
				(*target_state_)[i][j] = d_uav_target <= 1.0 ? coef / (1.0 + d_uav_target ) : coef / (1 + d_uav_target * d_uav_target * d_uav_target);
			}
			cout << "target_state[" << i << "][" << j << "]=" << (*target_state_)[i][j] << endl;
		}
	}
	set<int> row, col;
	for (int l = 0; l < target_.size(); l++) {
		double maxTargetState = 0;
		int uav_cnt, target_cnt;
		for (int i = 0; i < cur_uav_num; i++) {//
			for (int j = 0; j < target_.size(); j++) {
				if (row.find(i) == row.end() && col.find(j) == col.end() && (*target_state_)[i][j] > maxTargetState) {
					maxTargetState = (*target_state_)[i][j];
					uav_cnt = i;
					target_cnt = j;
				}
			}
		}
		if (maxTargetState > 0.001) {//
			printf("uav_cnt = %d target_cnt = %d\n", uav_cnt + 1, target_cnt + 1);
			if (dist(uav_[uav_cnt].state.position, target_[target_cnt].state.position) < 500) {
				(*tracked_)[target_cnt] = true;
				//uav_[uav_cnt].state.velocity.x = target_[target_cnt].state.velocity.x;
				printf("target %d is successfully tracked by uav %d\n", target_cnt + 1, uav_cnt + 1);
			}

			for (int i = 0; i < cur_uav_num; i++) {
				if (i == uav_cnt) {
					uav_[i].Tj[target_cnt] = 1;//
				}
				else
					uav_[i].Tj[target_cnt] = 0;
			}
			row.insert(uav_cnt); col.insert(target_cnt);
		}
	}
	for (int i = 0; i < cur_uav_num; i++) {
		int j = 0;
		for (; j < target_.size(); j++) {
			//cout << "uav[" << i << "].TJ[" << j << "]= " << uav[i].Tj[j] << " "<< uav[i].covered_target_id[j].second<< endl;
			if (uav_[i].Tj[j] == 1 && cunt - uav_[i].target_state[j].second < FORGET_TIME) {
				uav_[i].track_target_num = j;
				uav_[i].traj_Point = uav_[i].target_state[j].first;
				break;
			}
		}
		if (j >= target_.size()) {
			uav_[i].track_target_num = -1;
			printf("uav %d is free\n", i + 1);
		}//
	}
}

void MultipleSearchAndTrack::addUAV(){

	for ( int i = 0; i < extre_uav_.size(); i++) {
		utility::UAV& uav_temp = extre_uav_[i];
		for (int j = 0; j < target_.size(); j++) {
			utility::State temp_state;
			temp_state.position.x = 0;
			temp_state.position.y = 0;
			uav_temp.target_state.push_back(make_pair(temp_state, -FORGET_TIME));
			uav_temp.Tj[j] = 1;
		}

		global_map_[uav_temp.id].init(width_, height_, resolution_, -FORGET_TIME);
		global_map_[uav_temp.id].updateMap(&uav_temp, cunt);
		union_.parent.push_back(uav_temp.id);
	}

	for( auto c:extre_uav_)
		uav_.push_back(c);
	return;
}

void MultipleSearchAndTrack::removeUAV(int uav_idx){

	int reduce = 0;
	for ( auto ite = uav_.begin();ite!=uav_.end();ite++) {
		if(ite->id == uav_idx){
			uav_.erase(ite);
			union_.parent.erase(union_.parent.begin()+ uav_idx);
			reduce = 1;
		}
		ite->id -= reduce;
	}

	return;
}

void MultipleSearchAndTrack::run() {
	char status = 'r';
	while (cunt < max_simulation_step_)
	{
		// int key = cv::waitKey(10);

		// if(key == 'r')
		// 	status = 'r';
		// else if(key == 'p')
		// 	status = 'p';

		// if(status == 'p') continue;

		printf("\nprocessing step = %d\n", cunt);

		if(cunt == extre_step_)
			addUAV();

		if (uav_.size() > 1)
			informationShare();

		updateTargetStates();

		updateUAVStates();

		int destoryed_num = 0;
		for (int i = 0; i < target_.size(); i++) {//
			if ((*tracked_)[i])
				destoryed_num++;
		}

#if DESTORY_MODE
		//printf("destroyed_target_num = %d\n", destoryed_num);
#else
		// updateMission();
#endif
		show();

		output_target_num_ << destoryed_num << endl;
		// if (destoryed_num >= target_.size())
		// 	break;

		cunt++;
		
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	//std::cout << "target_state:" << endl << target_state << endl;
	//std::cout << "All target has been tracked!The total number of step is " << cunt << std::endl;
}

void MultipleSearchAndTrack::destory()
{
	pso_.destory();
	output_uav_.close();
	output_target_.close();
	output_traj_Point_.close();
	output_area_Point_.close();
	output_coverage_rate_.close();
	output_time_.close();
	output_target_num_.close();

	delete(target_state_);
	delete(tracked_);
	delete(path_);
	return;
};

void MultipleSearchAndTrack::show() {

	utility::MAP& global_map = this->global_map_[0];
	int resolution = global_map.resolution_;
	int height = global_map.height_;
	int width = global_map.width_;
	cv::Mat bottom_map(height, width, CV_16UC1);

	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			bottom_map.at<uint16_t>(height - i - 1, j) = cunt > (global_map.map_[i * width + j].search_time + FORGET_TIME) ? 
				FORGET_TIME : (cunt - global_map.map_[i * width + j].search_time);
		}
	}

	cv::Mat temp;
	temp.copySize(bottom_map);
	cv::convertScaleAbs(bottom_map, temp, 0.2);

	for(int i = 0; i < target_.size(); i++){
		int x_idx = target_.at(i).state.position.x / resolution;
		int y_idx = height - 1 - target_.at(i).state.position.y / resolution;
		cv::circle(temp, cv::Point(x_idx, y_idx), 2, (100, 100, 100), 5);
	}

	for(int i = 0; i < uav_.size(); i++){
		int x_idx = uav_.at(i).traj_Point.position.x / resolution;
		int y_idx = height - 1 - uav_.at(i).traj_Point.position.y / resolution;
		cv::circle(temp, cv::Point(x_idx, y_idx), 3, (150, 150, 150), 5);
	}

	for(int i = 0; i < uav_.size(); i++){
		int x_idx = uav_.at(i).area_guider.x / resolution;
		int y_idx = height - 1 - uav_.at(i).area_guider.y / resolution;
		cv::circle(temp, cv::Point(x_idx, y_idx), 5, (200, 200, 200), 5);
	}

	for(auto radar:radar_){
		int x_idx = radar.center_point.x / resolution;
		int y_idx = height - 1 - radar.center_point.y / resolution;
		int r = radar.R / resolution;
		cv::circle(temp, cv::Point(x_idx, y_idx), r, (200, 200, 200), 5);
	}

	cv::imshow("111", temp);
	cv::waitKey(10);

	return;
}