clc;
clear;


cunt = load("../draw_data/最终实验数据/时间分析/pso/cunt.txt");
tracked_num_pso = load("../draw_data/最终实验数据/时间分析/pso/tracked_number.txt");
tracked_num_topdown = load("../draw_data/最终实验数据/时间分析/top_down/tracked_number.txt");
tracked_num_bottomup = load("../draw_data/最终实验数据/时间分析/bottom_up/tracked_number.txt");
tracked_num_hybrid = load("../draw_data/最终实验数据/时间分析/hybrid/tracked_number.txt");
tracked_num_aco = load("../draw_data/最终实验数据/时间分析/aco/tracked_number.txt");


sim_steps = 1000;

x_idx = 1:1000;%length(cunt);

figure(1);

p1 = plot(x_idx, tracked_num_pso(1:sim_steps, 1),'r');
hold on;
p2 = plot(x_idx, tracked_num_topdown(1:sim_steps, 1),'g');
p3 = plot(x_idx, tracked_num_bottomup(1:sim_steps, 1),'b');
p4 = plot(x_idx, tracked_num_hybrid(1:sim_steps, 1),'y');
p5 = plot(x_idx, tracked_num_aco(1:sim_steps, 1),'m');

% xlable("simulation steps");
% ylable("fitness value");
% h1 = legend([p1, p2, p3, p4, p5], "ACO", "PSO", "BottomUP_PSO", "TopDown_PSO", "DACMP_PSO");