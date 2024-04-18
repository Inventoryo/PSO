clc;
clear;
cla;  


 L1=load('./mingw_build/draw_data/L1/1/cove_rate_PSO.txt');
 L2=load('./mingw_build/draw_data/L2/1/cove_rate_PSO.txt'); 
 L3=load('./mingw_build/draw_data/L3/1/cove_rate_PSO.txt'); 
 L4=load('./mingw_build/draw_data/L7/2/cove_rate_PSO.txt'); 
 L5=load('./mingw_build/draw_data/L5/1/cove_rate_PSO.txt'); 
 L6=load('./mingw_build/draw_data/L6/2/cove_rate_PSO.txt'); 
 L7=load('./mingw_build/draw_data/L7/1/cove_rate_PSO.txt'); 
x_idx = 1:1500;
figure(1);
first_row = 1;
last_row  = 1500;
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

p1 = plot(x_idx(first_row:last_row), L1(first_row:last_row),'r-');
hold on
p2 = plot(x_idx(first_row:last_row), L2(first_row:last_row),'c-');
p3 = plot(x_idx(first_row:last_row), L3(first_row:last_row),'b-');
p4 = plot(x_idx(first_row:last_row), L4(first_row:last_row),'g-');
p5 = plot(x_idx(first_row:last_row), L5(first_row:last_row),'k-');
p6 = plot(x_idx(first_row:last_row), L6(first_row:last_row),'m-');
p7 = plot(x_idx(first_row:last_row), L7(first_row:last_row),'y-');

% p11 = plot(x_idx(first_row:last_row), average_V1(first_row:last_row),'r-.');
% p21 = plot(x_idx(first_row:last_row), average_V2(first_row:last_row),'m-.');
% p31 = plot(x_idx(first_row:last_row), average_V3(first_row:last_row),'b-.');
% p41 = plot(x_idx(first_row:last_row), average_V4(first_row:last_row),'g-.');
% p51 = plot(x_idx(first_row:last_row), average_V5(first_row:last_row),'k-.');
% p61 = plot(x_idx(1:last_row), average_V6(1:last_row),'r');
%legend([p1, p2], 'without a new UAV', 'with a new UAV');
legend([p1, p2, p3, p4, p5, p6, p7], 'L1', 'L2', 'L3', 'L4', 'L5','L6', 'L7');
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