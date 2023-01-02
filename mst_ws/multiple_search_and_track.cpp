
#include "multiple_search_and_track.h"

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
	uav_num_slow_   = iniparser_getint(dict, "uav_param:uav_num_slow", -1);
	uav_num_middle_ = iniparser_getint(dict, "uav_param:uav_num_middle", -1);
	uav_num_fast_   = iniparser_getint(dict, "uav_param:uav_num_fast", -1);
	uav_num_extra_  = iniparser_getint(dict, "uav_param:uav_num_extra", -1);
	uav_num_total_  = uav_num_slow_ + uav_num_middle_ + uav_num_fast_ + uav_num_extra_;

	//target
	target_num_slow_   = iniparser_getint(dict, "target_param:target_num_slow", -1);
	target_num_middle_ = iniparser_getint(dict, "target_param:target_num_middle", -1);
	target_num_fast_   = iniparser_getint(dict, "target_param:target_num_fast", -1);
	target_num_total_  = target_num_slow_ + target_num_middle_ + target_num_fast_;

	//map
	resolution_   = iniparser_getint(dict, "map_param:resolution", -1);
	width_  = iniparser_getint(dict, "map_param:width", -1);
	height_ = iniparser_getint(dict, "map_param:height", -1);

	//file_path
	const char *uav_path = iniparser_getstring(dict, "file_path:uav_path", NULL);
	output_uav_.open(uav_path, ios::binary);

	const char *target_path = iniparser_getstring(dict, "file_path:target_path", NULL);
	output_target_.open(target_path, ios::binary);

	const char *traj_Point_path = iniparser_getstring(dict, "file_path:traj_Point_path", NULL);
	output_traj_Point_.open(traj_Point_path, ios::binary);

	const char *area_Point_path = iniparser_getstring(dict, "file_path:area_Point_path", NULL);
	output_area_Point_.open(area_Point_path, ios::binary);

	const char *cove_rate_PSO = iniparser_getstring(dict, "file_path:cove_rate_PSO", NULL);
	output_coverage_rate_.open(cove_rate_PSO, ios::binary);

	const char *time_consume = iniparser_getstring(dict, "file_path:time_consume", NULL);
	output_time_.open(time_consume, ios::binary);

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

void MultipleSearchAndTrack::updateArtificalPotentialFieldStateImpl(utility::UAV *uav, utility::State &goal, utility::State &next) {
	double k_att = 1, k_ref = 0;//100000000.0;

	//compute Attraction
	utility::point2D F_att = computeAttraction(uav->state.position, goal.position);

	//compute Repulsion
	utility::point2D obstacle_point;
	if (uav->state.position.x > obs_->point_lists[1].x)
		obstacle_point.x = obs_->point_lists[1].x;
	else if (uav->state.position.x < obs_->point_lists[0].x)
		obstacle_point.x = obs_->point_lists[0].x;
	else
		obstacle_point.x = uav->state.position.x;
	if (uav->state.position.y > obs_->point_lists[3].y)
		obstacle_point.y = obs_->point_lists[3].y;
	else if (uav->state.position.y < obs_->point_lists[0].y)
		obstacle_point.y = obs_->point_lists[0].y;
	else
		obstacle_point.y = uav->state.position.y;
	utility::point2D F_rep = computeRepulsion(uav->state.position, obstacle_point);

	//compute Radar
	utility::point2D Radar_point;
	Radar_point = radar_->center_point + (uav->state.position - radar_->center_point)*(radar_->R / (uav->state.position - radar_->center_point).distance());
	F_rep = F_rep + computeRepulsion(uav->state.position, Radar_point);

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

utility::point2D MultipleSearchAndTrack::computeRepulsion(utility::point2D& start, utility::point2D& obstacle) {
	utility::point2D F_rep = start- obstacle;

	if (F_rep.distance() > 10000)
		F_rep = F_rep * 0;
	else if (F_rep.distance() > 0.0000001) 
		F_rep = F_rep * (1 / pow(F_rep.distance(), 3));
		
	return F_rep;
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
	return F_att;
};

void MultipleSearchAndTrack::init(const string& config_file_path) {

	initParams(config_file_path);

	target_state_ = new vector<vector<double>>(uav_num_total_, vector<double>(target_num_total_, 0));
	tracked_ = new vector<bool>(target_num_total_, false);
	path_ = new HybridAStar::DubinsPath();
	global_map_.resize(uav_num_total_);
	//map
	printf("initializing map\n");

	//printf("(*global_map_)[0][0].search_time = %d\n", (*global_map_)[0][0].search_time);
	//setParam
	aco_.setParam(10, 500, 2, 4, 100, 0.8);
	pso_.create(config_file_path);

	//obstacle
	printf("initializing obstacles\n");
	srand((int(time(0))));
	radar_ = new utility::RADAR();
	radar_->center_point.x = 40000;
	radar_->center_point.y = 30000;
	radar_->R = 6000;

	obs_ = new utility::OBSTACLE();

	utility::point2D a, b, c, d;
	a.x = 15000;
	a.y = 15000;
	obs_->point_lists.push_back(a);
	b.x = a.x + 5000;
	b.y = a.y ;
	obs_->point_lists.push_back(b);
	c.x = a.x + 5000;
	c.y = a.y + 5000;
	obs_->point_lists.push_back(c);
	d.x = a.x ;
	d.y = a.y + 5000;
	obs_->point_lists.push_back(d);

	//slow target
	printf("initializing targets\n");
	int i = 0;
	target_.resize(target_num_total_);
	vector<utility::TARGET>::iterator temp_target = target_.begin();
	for (i = 0; i < target_num_slow_; ) {
		temp_target->Vel_limite[0]      = 0;
		temp_target->Vel_limite[1]      = 15;
		temp_target->Acc_limite_x[0]    = -0.4;
		temp_target->Acc_limite_x[1]    =  0.4;
		temp_target->Acc_limite_y[0]    = -G * tan( 20.0 * PI / 180.0 );
		temp_target->Acc_limite_y[1]    =  G * tan( 20.0 * PI / 180.0 );
		temp_target->state.position.x   = 200000 + i * 60000;//PosMin.x + (PosMax.x - PosMin.x)*(rand() % 1000 / 1000.0);
		temp_target->state.position.y   = 100000 ;// PosMin.y + (PosMax.y - PosMin.y)*(rand() % 1000 / 1000.0);
		temp_target->state.velocity.x   = 10;//
		temp_target->state.velocity.y   = PI/2; //2 * PI*(rand() % 1000 / 1000.0);//
		temp_target->state.accelerate.x = 0;//
		temp_target->state.accelerate.y = 0;//
		if (obs_->isInObstacle(temp_target->state.position))
			continue;

		temp_target++;
		i++;
	}

	for (i = 0; i < target_num_middle_; ) {
		temp_target->Vel_limite[0]      = 0;
		temp_target->Vel_limite[1]      = 15;
		temp_target->Acc_limite_x[0]    = -0.4;
		temp_target->Acc_limite_x[1]    =  0.4;
		temp_target->Acc_limite_y[0]    = -G * tan(20.0 * PI / 180.0);
		temp_target->Acc_limite_y[1]    =  G * tan(20.0 * PI / 180.0);
		temp_target->state.position.x   = 45000 ;
		temp_target->state.position.y   = 45000 + i * 15000;;
		temp_target->state.velocity.x   = 14;
		temp_target->state.velocity.y   = 2 * PI*(rand() % 1000 / 1000.0);
		temp_target->state.accelerate.x = 0;
		temp_target->state.accelerate.y = 0;
		if (obs_->isInObstacle(temp_target->state.position))
			continue;

		temp_target++;
		i++;
	}

	for (i = 0; i < target_num_fast_; ) {
		temp_target->Vel_limite[0]      = 0;
		temp_target->Vel_limite[1]      = 30;
		temp_target->Acc_limite_x[0]    = -0.4;
		temp_target->Acc_limite_x[1]    =  0.4;
		temp_target->Acc_limite_y[0]    = -G * tan(20.0 * PI / 180.0);
		temp_target->Acc_limite_y[1]    =  G * tan(20.0 * PI / 180.0);
		temp_target->state.position.x   = 60000;
		temp_target->state.position.y   = 75000 + i * 18000;
		temp_target->state.velocity.x   = 25;
		temp_target->state.velocity.y   = 0;
		temp_target->state.accelerate.x = 0;
		temp_target->state.accelerate.y = 0;

		if (obs_->isInObstacle(temp_target->state.position))
			continue;

		temp_target++;
		i++;
	}

	//UAV
	printf("initializing UAVS\n");
	i = 0;
	utility::UAV * uav_temp = new utility::UAV();
	for ( ; i < uav_num_slow_; ) {
		uav_temp->h  = 400;
		uav_temp->id = i;
		uav_temp->search_r = 3000;
		uav_temp->Vel_limite[0]   = 0;
		uav_temp->Vel_limite[1]   = 25;
		uav_temp->Acc_limite_x[0] = -0.4;
		uav_temp->Acc_limite_x[1] =  0.4;
		uav_temp->Acc_limite_y[0] = -G * tan(30.0 * PI / 180.0);
		uav_temp->Acc_limite_y[1] =  G * tan(30.0 * PI / 180.0);

		uav_temp->coverd_area_cnt  =  1;
		uav_temp->track_target_num = -1;
		
		uav_temp->state.position.x   = 0;//0;
		uav_temp->state.position.y   = 7000 + 14000 * i;//PosMin.y + (PosMax.y - PosMin.y)*(rand() % 1000 / 1000.0);
		uav_temp->state.velocity.x   = 30;// uav_temp->Vel_limite[0];// + (uav_temp->Vel_limite[1] - uav_temp->Vel_limite[0]) * (rand() % 1000 / 1000.0);//���ٶ�
		uav_temp->state.velocity.y   = 0;//
		uav_temp->state.accelerate.x = 0;//
		uav_temp->state.accelerate.y = 0;//
		
		if (obs_->isInObstacle(uav_temp->state.position) || radar_->isInRader(uav_temp->state.position)) {
			continue;
		}

		for (int j = 0; j < target_num_total_; j++) {
			utility::State temp_state;
			temp_state.position.x = 0;
			temp_state.position.y = 0;
			uav_temp->target_state.push_back(make_pair(temp_state, -FORGET_TIME));
			uav_temp->Tj[j] = 1;
		}

		global_map_[i].init(width_, height_, resolution_, -FORGET_TIME);
		global_map_[i].updateMap(uav_temp, cunt);

		uav_.push_back(*uav_temp);
		union_.parent.push_back(i);
		i++;
	}

	for (; i < uav_num_slow_ + uav_num_middle_; ) {//
		uav_temp->h  = 600;
		uav_temp->id = i;
		uav_temp->search_r = 5000;
		uav_temp->Vel_limite[0]   = 0;
		uav_temp->Vel_limite[1]   = 55.5;
		uav_temp->Acc_limite_x[0] = -0.6;
		uav_temp->Acc_limite_x[1] =  0.6;
		uav_temp->Acc_limite_y[0] = -G * tan(20.0 * PI / 180.0);
		uav_temp->Acc_limite_y[1] =  G * tan(20.0 * PI / 180.0);
		
		uav_temp->track_target_num = -1;
		uav_temp->coverd_area_cnt  =  1;

		uav_temp->state.position.x   = 0;//50000;//PosMin.x + (PosMax.x - PosMin.x)*(rand()%1000 / 1000.0);//λ��
		uav_temp->state.position.y   = 100000 * (i - uav_num_slow_ + 1) / (uav_num_middle_ + 1); //50000 * (i - slow_uav_num + 1) / (middel_uav_num + 1);
		uav_temp->state.velocity.x   = 35;// uav_temp->Vel_limite[0];// + (uav_temp->Vel_limite[1] - uav_temp->Vel_limite[0]) * (rand() % 1000 / 1000.0);//
		uav_temp->state.velocity.y   = 0;//
		uav_temp->state.accelerate.x = 0;//
		uav_temp->state.accelerate.y = 0;//

		if (obs_->isInObstacle(uav_temp->state.position) || radar_->isInRader(uav_temp->state.position)) {
			continue;
		}

		for (int j = 0; j < target_num_total_; j++) {
			utility::State temp_state;
			temp_state.position.x = 0;
			temp_state.position.y = 0;
			uav_temp->target_state.push_back(make_pair(temp_state, -FORGET_TIME));
			uav_temp->Tj[j] = 1;
		}

		global_map_[i].init(width_, height_, resolution_, -FORGET_TIME);
		global_map_[i].updateMap(&(*uav_temp), cunt);

		uav_.push_back(*uav_temp);
		union_.parent.push_back(i);
		i++;
	}

	for (; i < uav_num_slow_ + uav_num_middle_ + uav_num_fast_; ) {//
		uav_temp->h  = 1000;
		uav_temp->id = i;
		uav_temp->search_r = 3000;
		uav_temp->Vel_limite[0]   = 60;
		uav_temp->Vel_limite[1]   = 90;
		uav_temp->Acc_limite_x[0] = -0.6;
		uav_temp->Acc_limite_x[1] =  0.6;
		uav_temp->Acc_limite_y[0] = -G * tan(20.0 * PI / 180.0);
		uav_temp->Acc_limite_y[1] =  G * tan(20.0 * PI / 180.0);
		
		uav_temp->track_target_num = -1;
		uav_temp->coverd_area_cnt  =  1;

		uav_temp->state.position.x   = 0;//0;
		uav_temp->state.position.y   = 14000 + 14000 * (i - uav_num_slow_ - uav_num_middle_); 
		uav_temp->state.velocity.x   = 30;// uav_temp->Vel_limite[0];// +(uav_temp->Vel_limite[1] - uav_temp->Vel_limite[0]) * (rand() % 1000 / 1000.0);//
		uav_temp->state.velocity.y   = 0;// 2 * pi*(rand() % 1000 / 1000.0);//
		uav_temp->state.accelerate.x = 0;//
		uav_temp->state.accelerate.y = 0;//

		if (obs_->isInObstacle(uav_temp->state.position) || radar_->isInRader(uav_temp->state.position)) {
			continue;
		}

		for (int j = 0; j < target_num_total_; j++) {
			utility::State temp_state;
			temp_state.position.x = 0;
			temp_state.position.y = 0;
			uav_temp->target_state.push_back(make_pair(temp_state, -FORGET_TIME));
			uav_temp->Tj[j] = 1;
		}
		global_map_[i].init(width_, height_, resolution_, -FORGET_TIME);
		global_map_[i].updateMap(&(*uav_temp), cunt);

		uav_.push_back(*uav_temp);
		union_.parent.push_back(i);
		i++;
	}

	for (; i < uav_num_slow_ + uav_num_middle_ + uav_num_fast_ + uav_num_extra_; ) {//
		uav_temp->h = 1000;
		uav_temp->id = i;
		uav_temp->search_r = 5000;
		uav_temp->Vel_limite[0] = 60;
		uav_temp->Vel_limite[1] = 90;
		uav_temp->Acc_limite_x[0] = -0.6;
		uav_temp->Acc_limite_x[1] = 0.6;
		uav_temp->Acc_limite_y[0] = -G * tan(20.0 * PI / 180.0);
		uav_temp->Acc_limite_y[1] = G * tan(20.0 * PI / 180.0);

		uav_temp->track_target_num = -1;
		uav_temp->coverd_area_cnt = 1;

		uav_temp->state.position.x = 25000;
		uav_temp->state.position.y = 50000;//
		uav_temp->state.velocity.x = uav_temp->Vel_limite[0];// +(uav_temp->Vel_limite[1] - uav_temp->Vel_limite[0]) * (rand() % 1000 / 1000.0);//
		uav_temp->state.velocity.y = - PI / 2.0;// 2 * pi*(rand() % 1000 / 1000.0);//
		uav_temp->state.accelerate.x = 0;//
		uav_temp->state.accelerate.y = 0;//

		if (obs_->isInObstacle(uav_temp->state.position) || radar_->isInRader(uav_temp->state.position)) {
			continue;
		}

		for (int j = 0; j < target_num_total_; j++) {
			utility::State temp_state;
			temp_state.position.x = 0;
			temp_state.position.y = 0;
			uav_temp->target_state.push_back(make_pair(temp_state, -FORGET_TIME));
			uav_temp->Tj[j] = 1;
		}
		global_map_[i].init(width_, height_, resolution_, -FORGET_TIME);
		global_map_[i].updateMap(&(*uav_temp), cunt);

		uav_.push_back(*uav_temp);
		union_.parent.push_back(i);
		i++;
	}


	printf("initialize success\n");
};

void MultipleSearchAndTrack::updateTargetStates() {
	printf("updateing targets state\n");

	vector<utility::TARGET>::iterator temp_target = target_.begin();
	int target_tag = 0;
	for ( ; target_tag < target_num_total_; temp_target++, target_tag++) {
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
		if(cunt % 20 == 0)//random movement
			temp_target->state.velocity.y = 2 * PI * ((rand() % 1000) / 1000.0);//

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
		for(int i = 0; i < uav_num_total_; i++)
			if (dist(uav_[i].state.position, temp_target->state.position) <= 10 * resolution_) {
				istracking = true;//target been tracked
				break;
			}
		if (!istracking)
			for(int i = 0; i < uav_num_total_; i++)
				uav_[i].Tj[target_tag] = 1;
		output_target_ << temp_target->state.position.x << " " << temp_target->state.position.y << " ";
	}
	output_target_ << endl;
	return;
};

void MultipleSearchAndTrack::updateUAVStates() {
	printf("updateing UAVs state\n");
	vector<utility::UAV>::iterator cur_uav = uav_.begin();
	int cur_uav_num = cunt < ADD_TIME ? uav_num_total_ - uav_num_extra_ : uav_num_total_;
	int i = 0;
	for (; i < cur_uav_num; i++, cur_uav++) {

		clock_t start, finish;
		double  duration;
		if (cur_uav->track_target_num == -1 && cunt % 30 == 0) {//
			start = clock();

			switch (run_mode_){
			case 0:{
				pso_.getNextPoint(global_map_[i], radar_, obs_, target_, uav_, cunt, i);
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
				printf("dir = %d\n", dir);

				pose_out = aco_.run(global_map_[i], pose_in, dir, uav_[i].search_r / resolution_, i, cunt);

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
		global_map_[i].updateMap(&(*cur_uav), cunt);

		for (int j = 0; j < target_num_total_; j++) {//target inf0
			if (dist(target_[j].state.position, uav_[i].state.position)<uav_[i].search_r) {
				uav_[i].target_state[j].first = target_[j].state;
				uav_[i].target_state[j].second = cunt;
				cout << "target" << j + 1 << " has been found by uav" << i + 1 << endl;
				if(dist(target_[j].state.position, uav_[i].state.position) < 5 * resolution_){
					(*tracked_)[j] = true;
					cout << "target " << j + 1 << " is successfully tracked by uav " << i + 1 << endl;
					for (int i = 0; i < cur_uav_num; i++)
						if(uav_[i].id!=i)
							uav_[i].Tj[j] = 0;
						else
							uav_[i].Tj[j] = 1;
				}
			}
		}
	}

	for (; i < uav_num_total_; i++) {
		output_uav_ << 0 << " " << 0 << " " << 1 << " ";
		output_traj_Point_ << 0 << " " << 0 << " ";
	}

	output_uav_ << endl;
	output_traj_Point_ << endl;
	output_time_ << endl;
	//output_area_Point_ << endl;
	return;
}

void MultipleSearchAndTrack::informationShare() {
	printf("exchanging information\n");
	int cur_uav_num = cunt < ADD_TIME ? uav_num_total_ - uav_num_extra_ : uav_num_total_;

	for (int i = 0; i < cur_uav_num; i++) {//reset
		union_.parent[i] = i;
		uav_[i].coverd_area_cnt = 0;
	}
	for (int i = 0; i < cur_uav_num; i++) {
		for (int j = i; j < cur_uav_num; j++) {
			if (dist(uav_[i].state.position, uav_[j].state.position) <= COMMUNICATION_R) {//union
				union_.join(i, j);
				for (int k = 0; k < target_num_total_; k++) {//exchange target info
					if (uav_[i].target_state[k].second < uav_[j].target_state[k].second)
						uav_[i].target_state[k] = uav_[j].target_state[k];
					else
						uav_[j].target_state[k] = uav_[i].target_state[k];
					int x = uav_[i].target_state[k].first.position.x / resolution_;
					int y = uav_[i].target_state[k].first.position.y / resolution_;
					if(x<0||x>=width_ || y<0||y>=height_)
						continue;
					if (global_map_[j](x, y).search_time > uav_[j].target_state[k].second + 1) {////����˴�û��Ŀ��
						uav_[i].target_state[k].second = -FORGET_TIME;
						uav_[j].target_state[k].second = -FORGET_TIME;
					}
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
	int cur_uav_num = cunt < ADD_TIME ? uav_num_total_ - uav_num_extra_ : uav_num_total_;
	for (int i = 0; i < cur_uav_num; i++) {
		for (int j = 0; j < target_num_total_; j++) {//
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
	for (int l = 0; l < target_num_total_; l++) {
		double maxTargetState = 0;
		int uav_cnt, target_cnt;
		for (int i = 0; i < cur_uav_num; i++) {//
			for (int j = 0; j < target_num_total_; j++) {
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
		for (; j < target_num_total_; j++) {
			//cout << "uav[" << i << "].TJ[" << j << "]= " << uav[i].Tj[j] << " "<< uav[i].covered_target_id[j].second<< endl;
			if (uav_[i].Tj[j] == 1 && cunt - uav_[i].target_state[j].second < FORGET_TIME) {
				uav_[i].track_target_num = j;
				uav_[i].traj_Point = uav_[i].target_state[j].first;
				break;
			}
		}
		if (j >= target_num_total_) {
			uav_[i].track_target_num = -1;
			printf("uav %d is free\n", i + 1);
		}//
	}
}

void MultipleSearchAndTrack::run() {
	while (cunt < max_simulation_step_)
	{

		printf("\nprocessing step%d\n", cunt);

		if (uav_num_total_ > 1)
			informationShare();

		//updateTargetStates();

		updateUAVStates();

		updateMission();

		int i = 0;
		for (; i<target_num_total_; i++) {//
			if (!(*tracked_)[i])
				break;
		}
		if (i >= target_num_total_)
			break;

		cunt++;
	}
	//std::cout << "target_state:" << endl << target_state << endl;
	std::cout << "All target has been tracked!The total number of step is " << cunt << std::endl;
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

	delete(target_state_);
	delete(tracked_);
	delete(path_);
	return;
};
void MultipleSearchAndTrack::show(utility::UAV *uav, utility::TARGET *target) {
	for (int i = 0; i < uav_.size(); i++) {
		uav[i] = uav_[i];
	}
	for (int j = 0; j < target_.size(); j++) {
		target[j] = target_[j];
	}
	return;
}