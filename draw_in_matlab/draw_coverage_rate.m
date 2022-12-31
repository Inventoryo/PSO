clc;
clear;
cla;  

cove_rate_PSO_V1=load('../draw_data/最终实验数据/稳定性/robust/1111_9v6/cove_rate_PSO.txt'); 
cove_rate_PSO_V2=load('../draw_data/最终实验数据/稳定性/robust/10v6.txt'); 
%  cove_rate_PSO_V1=load('../draw_data/最终实验数据/任务分配/1111_9v6/cove_rate_PSO.txt'); %9v15覆盖率
%  cove_rate_PSO_V1=load('../data/cove_rate_PSO.txt'); 
 
%  cove_rate_PSO_V1=load('../draw_data/最终实验数据/search_performance/1102_6V0_PSO/cove_rate_PSO.txt');
% % cove_rate_PSO_V1=load('../draw_data/最终实验数据/search_performance/0505_6V0_PIO/cove_rate_PSO.txt');
%  cove_rate_PSO_V2=load('../draw_data/最终实验数据/search_performance/1102_6V0_BottomUp/cove_rate_PSO.txt'); 
%  cove_rate_PSO_V3=load('../draw_data/最终实验数据/search_performance/1102_6V0_TopDown/cove_rate_PSO.txt'); 
%  cove_rate_PSO_V4=load('../draw_data/0506_6V0_PIO_效果好/cove_rate_PSO.txt'); 
%  cove_rate_PSO_V5=load('../draw_data/最终实验数据/search_performance/1102_6V0_ACO/cove_rate_PSO.txt'); 
% % cove_rate_PSO_V7=load('../draw_data/1007_9V0_layer7/cove_rate_PSO.txt'); 

% cove_rate_PSO_V8=load('../draw_data/1005_5v0_2stage_1_0/cove_rate_PSO.txt'); 
x_idx = 1:700;
figure(1);
first_row = 1;
last_row  = 700;
% average = sum(cove_rate_PSO_V1(first_row:last_row)) / (last_row - first_row);
% average_V1 = linspace(average, average, last_row);
% average = sum(cove_rate_PSO_V2(first_row:last_row)) / (last_row - first_row);
% average_V2 = linspace(average, average, last_row);
% average = sum(cove_rate_PSO_V3(first_row:last_row)) / (last_row - first_row);
% average_V3 = linspace(average, average, last_row);
% average = sum(cove_rate_PSO_V4(first_row:last_row)) / (last_row - first_row);
% average_V4 = linspace(average, average, last_row);
% average = sum(cove_rate_PSO_V5(first_row:last_row)) / (last_row - first_row);
% average_V5 = linspace(average, average, last_row);
% average = sum(cove_rate_PSO_V6(first_row:last_row)) / (last_row - first_row);
% average_V6 = linspace(average, average, last_row);

p1 = plot(x_idx(first_row:last_row), cove_rate_PSO_V1(first_row:last_row),'r-');
hold on
p2 = plot(x_idx(first_row:last_row), cove_rate_PSO_V2(first_row:last_row),'c-');
% p3 = plot(x_idx(first_row:last_row), cove_rate_PSO_V3(first_row:last_row),'b-');
% p4 = plot(x_idx(first_row:last_row), cove_rate_PSO_V4(first_row:last_row),'g-');
% p5 = plot(x_idx(first_row:last_row), cove_rate_PSO_V5(first_row:last_row),'k-');
% p6 = plot(x_idx(first_row:last_row), cove_rate_PSO_V6(first_row:last_row),'r-');

% p6 = plot(x_idx(1:last_row), cove_rate_PSO_V6(1:last_row),'m');
% p7 = plot(x_idx(1:last_row), cove_rate_PSO_V7(1:last_row),'c');
% p8 = plot(x_idx(1:last_row), cove_rate_PSO_V8(1:last_row),'c');

% p11 = plot(x_idx(first_row:last_row), average_V1(first_row:last_row),'r-.');
% p21 = plot(x_idx(first_row:last_row), average_V2(first_row:last_row),'m-.');
% p31 = plot(x_idx(first_row:last_row), average_V3(first_row:last_row),'b-.');
% p41 = plot(x_idx(first_row:last_row), average_V4(first_row:last_row),'g-.');
% p51 = plot(x_idx(first_row:last_row), average_V5(first_row:last_row),'k-.');
% p61 = plot(x_idx(1:last_row), average_V6(1:last_row),'r');
legend([p1, p2], 'without a new UAV', 'with a new UAV');
% legend([p1, p2, p3, p4, p5, p6], 'PSO', 'PIO', 'PSO-BottomUp', 'PSO-TopDown', 'DACMP','ACO');
% legend([p1, p2, p3, p4, p5, p6], 'V1', 'V2', 'V3', 'V4', 'V5', 'V6');
% legend([p1, p2], 'ACO', 'PSO');
xlabel('simulation steps');
ylabel('coverage rate(%)');
% legend([p1, p11, p2, p21, p3, p31, p4, p41, p5, p51], 'ACO', 'Average of ACO','PSO', 'Average of PSO','PSO-BottomUp', 'Average of PSO-BottomUp', 'PSO-TopDown', 'Average of PSO-TopDown','DACMP', 'Average of DACMP');
hold off;

% iteration_time_ACO=load('data/iteration_time_ACO.txt');
% iteration_time_PSO=load('data/iteration_time_PSO.txt');  
% x=1:1000;
% figure(4);
% p1 = plot(x,iteration_time_ACO(1:1000),'r');
% hold on
% p2 = plot(x,iteration_time_PSO(1:1000),'g');
% legend([p1,p2],'ACO', 'PSO','TARGET');
% hold off;
