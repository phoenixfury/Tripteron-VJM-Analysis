%% First try

data = zeros(13, 13, 13);
data2 = zeros(13, 13, 13);
Force = [0; 0; 100; 0; 0; 0];
pointsize = 45;
theta = zeros(13,3);
tic
for z = 1: 13
    for y = 1: 13
        for x = 1:13
        p_global = [x* 0.1, y*0.1, z*0.1];

        Kc = Kc_def( p_global, theta);
        
        [pre_data  kcc]= ComputeDeflection( p_global, Force);
        
        dt_VJM = Kc\Force;
        deflection = sqrt(pre_data(1)^2 + pre_data(2)^2 + pre_data(3)^2);
        
        D_VJM = sqrt(dt_VJM(1)^2 + dt_VJM(2)^2 + dt_VJM(3)^2);
        data(x, y, z) = D_VJM; 
        
        
        data2(x, y, z) = deflection; 
%         figure(4)
        scatter3(x*0.1, y*0.1, z*0.1,pointsize, abs(data(x,y,z) - data2(x,y,z)));
        hold on
%         figure(5)
%         scatter3(x*0.1, y*0.1, z*0.1,pointsize, data2(x,y,z));
%         hold on
%         figure(6)
%         scatter3(x*0.1, y*0.1, z*0.1,pointsize, data(x,y,z));
%         hold on
        end
    end
end
toc
colorbar


    