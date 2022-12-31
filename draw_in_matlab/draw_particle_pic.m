
clc;
clear;


while(1)
    cla;
    target = load('data/target.txt');
    uav = load('data/uav.txt');
    Gbest_position=load('data/traj_Point.txt');
    figure(1);
    
     hold on;
    color ={'r','r','g','g','b','b','c','c','m','m','y','y','k','k'};
    for j =1:length(uav(:,1))
        plot (60000,60000);
        hold on;
        axis equal;
        plot (0,0);
        plot(uav(1:j,1),uav(1:j,2),'r');
        text(uav(j,1),uav(j,2)-2000,['(uav',num2str(0),')'],'color','r');
        text(uav(j,1),uav(j,2),['(target',num2str(uav(j,3)),')'],'color','r');
        rectangle('Position',[uav(j,1)-2000,uav(j,2)-1000,4000,4000],'Curvature',[1,1]),axis equal;
        
        plot(uav(1:j,4),uav(1:j,5),'g');
        text(uav(j,4),uav(j,5)-2000,['(uav',num2str(1),')'],'color','g');
        text(uav(j,4),uav(j,5),['(target',num2str(uav(j,6)),')'],'color','g');
        rectangle('Position',[uav(j,4)-2000,uav(j,5)-2000,4000,4000],'Curvature',[1,1]),axis equal;
        
        plot(uav(1:j,7),uav(1:j,8),'b');
        text(uav(j,7),uav(j,8)-2000,['(uav',num2str(2),')'],'color','b');
        text(uav(j,7),uav(j,8),['(target',num2str(uav(j,9)),')'],'color','b');
        rectangle('Position',[uav(j,7)-2000,uav(j,8)-2000,4000,4000],'Curvature',[1,1]),axis equal;
        
        plot(uav(1:j,10),uav(1:j,11),'c');
        text(uav(j,10),uav(j,11)-2000,['(uav',num2str(3),')'],'color','c');
        text(uav(j,10),uav(j,11),['(target',num2str(uav(j,12)),')'],'color','c');
        rectangle('Position',[uav(j,10)-3000,uav(j,11)-3000,6000,6000],'Curvature',[1,1]),axis equal;
        
        plot(uav(1:j,13),uav(1:j,14),'m');
        text(uav(j,13),uav(j,14)-2000,['(uav',num2str(4),')'],'color','m');
        text(uav(j,13),uav(j,14),['(target',num2str(uav(j,15)),')'],'color','m');
        rectangle('Position',[uav(j,13)-3000,uav(j,14)-3000,6000,6000],'Curvature',[1,1]),axis equal;
        
        plot(uav(1:j,16),uav(1:j,17),'y'); 
        text(uav(j,16),uav(j,17)-2000,['(uav',num2str(5),')'],'color','y');
        text(uav(j,16),uav(j,17),['(target',num2str(uav(j,18)),')'],'color','y');
        rectangle('Position',[uav(j,16)-3000,uav(j,17)-3000,6000,6000],'Curvature',[1,1]),axis equal;
        
        plot(uav(1:j,19),uav(1:j,20),'k');
        text(uav(j,19),uav(j,20)-2000,['(uav',num2str(6),')'],'color','k');
        text(uav(j,19),uav(j,20),['(target',num2str(uav(j,21)),')'],'color','k');
        rectangle('Position',[uav(j,19)-5000,uav(j,20)-5000,10000,10000],'Curvature',[1,1]),axis equal;
        
        plot(uav(1:j,22),uav(1:j,23),'r');
        text(uav(j,22),uav(j,23)-2000,['(uav',num2str(7),')'],'color','r');
        text(uav(j,22),uav(j,23),['(target',num2str(uav(j,24)),')'],'color','r');
        rectangle('Position',[uav(j,22)-5000,uav(j,23)-5000,10000,10000],'Curvature',[1,1]),axis equal;
        
        plot(uav(1:j,25),uav(1:j,26),'g');
        text(uav(j,25),uav(j,26)-2000,['(uav',num2str(8),')'],'color','g');
        text(uav(j,25),uav(j,26),['(target',num2str(uav(j,27)),')'],'color','g');
        rectangle('Position',[uav(j,25)-5000,uav(j,26)-5000,10000,10000],'Curvature',[1,1]),axis equal;
        
        plot(Gbest_position(j,1),Gbest_position(j,2),'ro');
        plot(Gbest_position(j,3),Gbest_position(j,4),'go');
        plot(Gbest_position(j,5),Gbest_position(j,6),'bo');
        plot(Gbest_position(j,7),Gbest_position(j,8),'co');
        plot(Gbest_position(j,9),Gbest_position(j,10),'mo');
        plot(Gbest_position(j,11),Gbest_position(j,12),'yo');
        plot(Gbest_position(j,13),Gbest_position(j,14),'ko');
        plot(Gbest_position(j,15),Gbest_position(j,16),'ro');
        plot(Gbest_position(j,17),Gbest_position(j,18),'go');
        %for i=1:2:6
           % plot(uav(1:j,i),uav(1:j,i+1));
        %end
        %for i=1:2:6
            %plot(Gbest_position(1:j,i),Gbest_position(1:j,i+1),'o');
        %end
        
        plot(target(j,1),target(j,2),'rp');
        text(target(j,1),target(j,2),['(target',num2str(0),')'],'color','g');
        
        plot(target(j,3),target(j,4),'gp');
        text(target(j,3),target(j,4),['(target',num2str(1),')'],'color','g');

        plot(target(j,5),target(j,6),'bp');
        text(target(j,5),target(j,6),['(target',num2str(2),')'],'color','g');
        
%         plot(target(j,7),target(j,8),'p');
%         text(target(j,7),target(j,8),['(target',num2str(3),')'],'color','g');
%         
%         plot(target(j,9),target(j,10),'p');
%         text(target(j,9),target(j,10),['(target',num2str(4),')'],'color','g');
%         
%         plot(target(j,11),target(j,12),'p');
%         text(target(j,11),target(j,12),['(target',num2str(5),')'],'color','g');
% 
%         plot(target(j,13),target(j,14),'p');
%         text(target(j,13),target(j,14),['(target',num2str(6),')'],'color','g');
%         
%         plot(target(j,15),target(j,16),'p');
%         text(target(j,15),target(j,16),['(target',num2str(7),')'],'color','g');
%         
%         plot(target(j,17),target(j,18),'p');
%         text(target(j,17),target(j,18),['(target',num2str(8),')'],'color','g');

        plot(30000,30000,'ro');
        text(30000,30000,'rader','color','g');
        rectangle('Position',[30000-5000,30000-5000,10000,10000],'Curvature',[1,1]),axis equal;
        rectangle('Position',[15000,10000,5000,5000]);
        
        grid on;
        hold off;
        pause(0.0000005);
    end
    
    title("particle position and target position")%БъЬт
    hold off;
    pause(10);
end




    
