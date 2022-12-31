
clc;
clear;
cla;  

while(1)
%     cla;
    target         = load('../data/target.txt');
    uav            = load('../data/uav.txt');
    Gbest_position = load('../data/traj_Point.txt');
    area_Point     = load('../data/area_Point.txt');
    plot (50000,50000);
    hold on;
    axis equal;
    plot (0,0); 

      rectangle('Position',[5000-2400,30300-2400,4800,4800],'Curvature',[1,1],'FaceColor',[0.41176,0.41176,0.41176]),axis equal;
      rectangle('Position',[30000-4000,15000-4000,8000,8000],'Curvature',[1,1],'FaceColor',[0.41176,0.41176,0.41176]),axis equal;
      rectangle('Position',[40000-4000,31000-4000,8000,8000],'Curvature',[1,1],'FaceColor',[0.41176,0.41176,0.41176]),axis equal;
      rectangle('Position',[5000-3000,15000-3000,6000,6000],'Curvature',[1,1],'FaceColor',[0.41176,0.41176,0.41176]),axis equal;
      rectangle('Position',[30000,45000,3000,3000],'FaceColor',[0,0,0]);
      
    text(40000,30000,'rader','color','g');
    text(20000,20000,'rader','color','g');
    text(5000,30000,'rader','color','g');
    last_row = 160;
   
    search_r = 2000;
%     
    p1 = plot(uav(1:last_row,1),uav(1:last_row,2),'r');
    rectangle('Position',[uav(last_row,1)-search_r,uav(last_row,2)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor','r');
    text(uav(last_row,1),uav(last_row,2)-1500,['U',num2str(1),''],'color','r');
    
    p2 = plot(uav(1:last_row,4),uav(1:last_row,5),'g');
    rectangle('Position',[uav(last_row,4)-search_r,uav(last_row,5)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor','g');
    text(uav(last_row,4),uav(last_row,5)-1500,['U',num2str(2),''],'color','g');
    
    p3 = plot(uav(1:last_row,7),uav(1:last_row,8),'b');
    rectangle('Position',[uav(last_row,7)-search_r,uav(last_row,8)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor','b');
    text(uav(last_row,7),uav(last_row,8)-1500,['U',num2str(3),''],'color','b');
   
    p4 = plot(uav(1:last_row,10),uav(1:last_row,11),'c');
    rectangle('Position',[uav(last_row,10)-search_r,uav(last_row,11)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor','c');
    text(uav(last_row,10),uav(last_row,11)-1500,['U',num2str(4),''],'color','c');
    
    p5 = plot(uav(1:last_row,13),uav(1:last_row,14),'m');
    rectangle('Position',[uav(last_row,13)-search_r,uav(last_row,14)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor','m');
    text(uav(last_row,13),uav(last_row,14)-1500,['U',num2str(5),''],'color','m');
   
    search_r = 3000;
    
    
    
    p6 = plot(uav(1:last_row,16),uav(1:last_row,17),'k');
    rectangle('Position',[uav(last_row,16)-search_r,uav(last_row,17)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor','k');
    text(uav(last_row,16),uav(last_row,17)-1500,['U',num2str(6),''],'color','k');
    
    p7 = plot(uav(1:last_row,19),uav(1:last_row,20),'color',[0.126549,0.69804,0.66667]);
%     rectangle('Position',[uav(last_row,19)-search_r,uav(last_row,20)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor',[0.126549,0.69804,0.66667]),axis equal;
    rectangle('Position',[uav(last_row,19)-search_r,uav(last_row,20)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor',[0.126549,0.69804,0.66667]),axis equal;
    text(uav(last_row,19),uav(last_row,20)-1500,['U',num2str(7),''],'color',[0.126549,0.69804,0.66667]);
    
    p8 = plot(uav(1:last_row,22),uav(1:last_row,23),'color',[0.64706,0.16471,0.16471]);
    rectangle('Position',[uav(last_row,22)-search_r,uav(last_row,23)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor',[0.64706,0.16471,0.16471]),axis equal;
    text(uav(last_row,22),uav(last_row,23)-1500,['U',num2str(8),''],'color',[0.64706,0.16471,0.16471]);
    
    p9 = plot(uav(1:last_row,25),uav(1:last_row,26),'color',[0.18431,0.3098,0.3098]);
    rectangle('Position',[uav(last_row,25)-search_r,uav(last_row,26)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor',[0.18431,0.3098,0.3098]),axis equal;
    text(uav(last_row,25),uav(last_row,26)-1500,['U',num2str(9),''],'color',[0.18431,0.3098,0.3098]);
%     
% %     p91 = plot(uav(301:last_row,28),uav(301:last_row,29),'color',[0.28431,0.3098,0.3098]);
% %     rectangle('Position',[uav(last_row,28)-search_r,uav(last_row,29)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor',[0.28431,0.3098,0.3098]),axis equal;
% %     text(uav(last_row,28),uav(last_row,29)-1500,['U',num2str(10),''],'color',[0.28431,0.3098,0.3098]);
%     
     p10 = plot(uav(1:last_row,28),uav(1:last_row,29),'color',[0.28431,0.3098,0.3098]);
    rectangle('Position',[uav(last_row,28)-search_r,uav(last_row,29)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor',[0.28431,0.3098,0.3098]),axis equal;
    text(uav(last_row,28),uav(last_row,29)-1500,['U',num2str(10),''],'color',[0.28431,0.3098,0.3098]);
    
    search_r = 5000;
    
     p11 = plot(uav(1:last_row,31),uav(1:last_row,32),'color','r');
    rectangle('Position',[uav(last_row,31)-search_r,uav(last_row,32)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor','r'),axis equal;
    text(uav(last_row,31),uav(last_row,32)-1500,['U',num2str(11),''],'color','r');
    
     p12 = plot(uav(1:last_row,34),uav(1:last_row,35),'color','g');
    rectangle('Position',[uav(last_row,34)-search_r,uav(last_row,35)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor','g'),axis equal;
    text(uav(last_row,34),uav(last_row,35)-1500,['U',num2str(12),''],'color','g');
    
     p13 = plot(uav(1:last_row,37),uav(1:last_row,38),'color','b');
    rectangle('Position',[uav(last_row,37)-search_r,uav(last_row,38)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor','b'),axis equal;
    text(uav(last_row,37),uav(last_row,38)-1500,['U',num2str(13),''],'color','b');
    
     p14 = plot(uav(1:last_row,40),uav(1:last_row,41),'color','c');
    rectangle('Position',[uav(last_row,40)-search_r,uav(last_row,41)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor','c'),axis equal;
    text(uav(last_row,40),uav(last_row,41)-1500,['U',num2str(14),''],'color','c');
    
     p15 = plot(uav(1:last_row,43),uav(1:last_row,44),'color','m');
    rectangle('Position',[uav(last_row,43)-search_r,uav(last_row,44)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor','m'),axis equal;
    text(uav(last_row,43),uav(last_row,44)-1500,['U',num2str(15),''],'color','m');
    
   % legend([p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,p15],'UAV1', 'UAV2','UAV3', 'UAV4','UAV5', 'UAV6','UAV7','UAV8', 'UAV9','UAV10','UAV11','UAV12','UAV13','UAV14','UAV15');
    
    p010 = plot(target(1:10:last_row,1),target(1:10:last_row,2),'r.');
    text(target(last_row,1),target(last_row,2)+1500,['T',num2str(1),''],'color','r');
    p011 = plot(target(1:10:last_row,3),target(1:10:last_row,4),'g.');
    text(target(last_row,3),target(last_row,4)+1500,['T',num2str(2),''],'color','g');
    p012 = plot(target(1:10:last_row,5),target(1:10:last_row,6),'b.');
    text(target(last_row,5),target(last_row,6)+1500,['T',num2str(3),''],'color','b');
    p013 = plot(target(1:10:last_row,7),target(1:10:last_row,8),'c.');
    text(target(last_row,7),target(last_row,8)+1500,['T',num2str(4),''],'color','c');
    p014 = plot(target(1:10:last_row,9),target(1:10:last_row,10),'m.');
    text(target(last_row,9),target(last_row,10)+1500,['T',num2str(5),''],'color','m');
    p015 = plot(target(1:10:last_row,11),target(1:10:last_row,12),'k.');
    text(target(last_row,11),target(last_row,12)+1500,['T',num2str(6),''],'color','k');
    p016 = plot(target(1:10:last_row,13),target(1:10:last_row,14),'c.');
    text(target(last_row,13),target(last_row,14)+1500,['T',num2str(7),''],'color','c');
    p017 = plot(target(1:10:last_row,15),target(1:10:last_row,16),'m.');
    text(target(last_row,15),target(last_row,16)+1500,['T',num2str(8),''],'color','m');
    p018 = plot(target(1:10:last_row,17),target(1:10:last_row,18),'k.');
    text(target(last_row,17),target(last_row,18)+1500,['T',num2str(9),''],'color','k');
%     p019 = plot(target(1:10:last_row,19),target(1:10:last_row,20),'k.');
%     text(target(last_row,19),target(last_row,20)+1500,['T',num2str(10),''],'color','c');
%     p020 = plot(target(1:10:last_row,21),target(1:10:last_row,22),'k.');
%     text(target(last_row,21),target(last_row,22)+1500,['T',num2str(11),''],'color','m');
%     p021 = plot(target(1:10:last_row,23),target(1:10:last_row,24),'k.');
%     text(target(last_row,23),target(last_row,24)+1500,['T',num2str(12),''],'color','k');
    axis equal;
%   hl = legend([p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,p15,p010,p011,p012,p013,p014,p015,p016,p017,p018],'U1', 'U2','U3', 'U4','U5', 'U6','U7','U8', 'U9','U10','U11','U12','U13','U14','U15','T1','T2','T3','T4','T5','T6','T7','T8','T9','Location','eastoutside');
%   set(hl,'Orientation','horizon');
    
    hold off;
   color ='rgbcmykrgbcmykrgbcmyk';   
   
    pause(10);
end




    
