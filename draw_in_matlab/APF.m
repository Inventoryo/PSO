% [X,Y]= meshgrid(1:100:50000, 1:100:50000);
% a_scale = 1/500000000;
% attr_potential = a_scale * ((X-30000).^2 + (Y-20000).^2);
% map = [1:100:50000; 1:100:50000];
% rho = bwdist(map)/100+1;
% influence = 1.5;
% r_scale = 1000;
% rep_potential = r_scale *((1./rho - 1/influence)).^2;
% %potential_field = attr_potential + rep_potential;
% Z= attr_potential;
% mesh(X,Y,Z);
attr_potential=zeros();
rep_potential_1=zeros();
rep_potential_2=zeros();
rep_potential_3=zeros();
potential_field=zeros();
for i = 1:100
    for j = 1:100
        a_scale = 0.1;
        attr_potential(i,j) = a_scale * 0.5 *  ((i-0)*(i-0) + (j-0)*(j-0));
        r_scale = 1000;
        dist1 = sqrt((i-30)*(i-30)+(j-30)*(j-30));
        dist2 = sqrt((i-80)*(i-80)+(j-60)*(j-60));
        dist3 = sqrt((i-60)*(i-60)+(j-80)*(j-80));
        if(dist1 < 5)
           rep_potential_1(i,j) = 405;%0.5 * sqrt((1/sqrt((i-15)*(i-15)+(j-10)*(j-10))+1/20));
        elseif(dist1>5 && dist1<=15)        
           rep_potential_1(i,j) = 0.5 * r_scale *(1/sqrt((i-30)*(i-30)+(j-30)*(j-30))-1/10)*(1/sqrt((i-30)*(i-30)+(j-30)*(j-30))-1/10);
        else
            rep_potential_1(i,j) =0;
        end
        
         if(dist2 < 5)
           rep_potential_2(i,j) = 405;%0.5 * sqrt((1/sqrt((i-15)*(i-15)+(j-10)*(j-10))+1/20));
        elseif(dist2>5 && dist2<=15)        
           rep_potential_2(i,j) = 0.5 * r_scale * (1/dist2-1/10)*(1/dist2-1/10);
        else
            rep_potential_2(i,j) =0;
         end
         
         if(dist3 < 5)
           rep_potential_3(i,j) = 405;%0.5 * sqrt((1/sqrt((i-15)*(i-15)+(j-10)*(j-10))+1/20));
        elseif(dist3>5 && dist3<=15)        
           rep_potential_3(i,j) = 0.5 * r_scale * (1/dist3-1/10)*(1/dist3-1/10);
        else
            rep_potential_3(i,j) =0;
         end
        
        potential_field(i,j) = attr_potential(i,j)+ rep_potential_1(i,j)+rep_potential_2(i,j) + rep_potential_3(i,j);
    end
end
[x,y]=meshgrid(1:100,1:100);
meshz(x,y, potential_field);