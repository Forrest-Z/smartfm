clear all, close all
write_fig = 0;
disp(' ')

tic 

load('GP_speed_x','x_mean','x_var', 'XmaxVar_speed');
load('GP_speed_y','y_mean','y_var', 'YmaxVar_speed');

% to speed-up the calculation without wasting time on other areas;
MaxVar_calc_flag = 0;

[size_x, size_y] = size(x_mean);

thetha_mean = 0*zeros(size_x, size_y);
thetha_var = 0*zeros(size_x, size_y);

for xx = 1:size_x
    display( xx);
    for yy=1:size_y
        
        if (x_var(xx,yy) < XmaxVar_speed - 0.0001) && (y_var(xx,yy) < YmaxVar_speed - 0.0001)
            [thetha_mean(xx,yy), thetha_var(xx,yy)] = projected_normal_function(x_mean(xx,yy), y_mean(xx,yy), x_var(xx,yy), y_var(xx,yy));
        else
            if(MaxVar_calc_flag==0)
                [thetha_mean(xx,yy), thetha_var(xx,yy)] = projected_normal_function(x_mean(xx,yy), y_mean(xx,yy), x_var(xx,yy), y_var(xx,yy));
                MaxVar_thetha_mean = thetha_mean(xx,yy);
                MaxVar_thetha_var  = thetha_var(xx,yy);
                MaxVar_calc_flag=1;
            else
                thetha_mean(xx,yy)  = MaxVar_thetha_mean;
                thetha_var(xx,yy)   = MaxVar_thetha_var;
            end
        end
    end
end

maxMean_vector = max(thetha_mean); maxMean_speed = max(maxMean_vector);
minMean_vector = min(thetha_mean); minMean_speed = min(minMean_vector);
maxVar_vector = max(thetha_var); maxVar_speed = max(maxVar_vector);
minVar_vector = min(thetha_var); minVar_speed = min(minVar_vector);

thetha_mean_img = mat2gray(thetha_mean, [minMean_speed, maxMean_speed]);
imwrite(thetha_mean_img, 'PN_thetha_mean.jpg', 'jpg');
thetha_var_img = mat2gray(thetha_var, [minVar_speed maxVar_speed]);
imwrite(thetha_var_img, 'PN_thetha_var.jpg', 'jpg');

save('PN_thetha_params.txt', 'maxMean_speed', 'minMean_speed', 'maxVar_speed', 'minVar_speed', '-ASCII')
save('PN_workspace');
toc