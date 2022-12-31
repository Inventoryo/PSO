clc;
clear;

time_aco = load("../draw_data/最终实验数据/time_consume/aco.txt");
time_pso = load("../draw_data/最终实验数据/time_consume/pso.txt");
time_bottomup_pso = load("../draw_data/最终实验数据/time_consume/bottom_up.txt");
time_top_down_pso = load("../draw_data/最终实验数据/time_consume/top_down.txt");
time_dacmp_pso = load("../draw_data/最终实验数据/time_consume/dacmp.txt");


sim_steps = length(time_aco(:,1));

x_idx = 1:sim_steps;

figure(1);

p1 = plot(x_idx, time_aco(1:sim_steps, 1),'r');
hold on;
p2 = plot(x_idx, time_pso(1:sim_steps, 1),'g');
p3 = plot(x_idx, time_bottomup_pso(1:sim_steps, 1),'b');
p4 = plot(x_idx, time_top_down_pso(1:sim_steps, 1),'y');
p5 = plot(x_idx, time_dacmp_pso(1:sim_steps, 1),'m');

xlable("simulation steps");
ylable("time, [ms]");
h1 = legend([p1, p2, p3, p4, p5], "ACO", "PSO", "BottomUP_PSO", "TopDown_PSO", "DACMP_PSO");