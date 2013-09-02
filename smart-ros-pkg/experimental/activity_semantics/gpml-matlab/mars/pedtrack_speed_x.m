clear all, close all
write_fig = 0;
disp(' ')
tic

% 1st step: read the environment image, to initialize the size;
imOnXY=imread('binary_image.jpg');
[size_x,size_y,img_width]=size(imOnXY);
for xx = 1:fix(size_x/2)
    for yy=1:size_y
        tmp = imOnXY(xx,yy,:);
        imOnXY(xx,yy,:)=imOnXY(size_x-xx, yy, :);
        imOnXY(size_x-xx, yy, :) = tmp;
    end
end
a = 0*zeros(size_x, size_y);

filename_cluster1 = 'map_data_both.txt';
% open the file
fid = fopen(filename_cluster1);
% read the file headers, find N (one value)
N = fscanf(fid, '%d', 1);
%N = 300;
total = 0;

%offset_x = 424;
%offset_y = 973-516-89;

offset_x = 0;
offset_y = 0;

% read each set of measurements
for n = 1:N
   x_tmp = fscanf(fid, '\n%d\t', 1);
   y_tmp = fscanf(fid, '%d\t', 1);
   times = fscanf(fid, '%d\t', 1);
    for nn=1:times
        total = total+1;
        speed(total) = fscanf(fid, '%f\t', 1);
        thetha(total)= fscanf(fid, '%f\t', 1);
        x(total)=x_tmp-offset_x;
        y(total)=y_tmp-offset_y;
        z(total)= cos(thetha(total))*speed(total);       
    end
end
% close the file
fclose(fid);

n = total; D = 2;
train_data_x = zeros(n,D);
train_data_y = zeros(n,1);
for i=1:n
    %malegebi:matlab is different with OpenCV;
    %train_data_x(i,1)= (x(i)-353);
    %train_data_x(i,2)= (y(i)-343);
    train_data_x(i,1)= y(i);
    train_data_x(i,2)= x(i);
    train_data_y(i) = z(i);
end

meanfunc = {@meanConst};
covfunc = @covSEiso; 
likfunc = @likGauss; 
hyp.cov = log([5.0; 0.7]); hyp.mean = [0]; hyp.lik = log(0.01);
hyp = minimize(hyp, @gp, -1000, @infExact, meanfunc, covfunc, likfunc, train_data_x, train_data_y);
%grid_size = 400;

width  = size_y;
height = size_x;

predict_z = zeros(height*width, D);

for i=1:height
    for j=1:width
        predict_z((i-1)*width+j,1)=i;
        predict_z((i-1)*width+j,2)=j;
    end
end

disp('[m s2] = gp(hyp, @infExact, meanfunc, covfunc, likfunc, x, y, z);')

[m s2] = gp(hyp, @infExact, meanfunc, covfunc, likfunc, train_data_x, train_data_y, predict_z);
[gridx, gridy] = meshgrid(1:height, 1:width);
%do not quite understand here;
m = reshape(m, width, height);
s2 = reshape(s2, width, height);
%m = 1.0-m;
surface(gridy, gridx, m, 'FaceAlpha', 0.5, 'LineStyle', 'none', 'FaceColor', 'interp');
hold on;
surface(a, imOnXY, 'FaceColor', 'texturemap', 'EdgeColor', 'none', 'CDataMapping', 'direct')

m_deputy=m';
s2_deputy=s2';
for xx = 1:fix(size_x/2)
    for yy=1:size_y
        tmp = m_deputy(xx,yy,:);
        m_deputy(xx,yy,:)=m_deputy(size_x-xx, yy, :);
        m_deputy(size_x-xx, yy, :) = tmp;
        tmp = s2_deputy(xx,yy,:);
        s2_deputy(xx,yy,:)=s2_deputy(size_x-xx, yy, :);
        s2_deputy(size_x-xx, yy, :) = tmp;
    end
end

maxMean_vector = max(m); maxMean_speed = max(maxMean_vector);
minMean_vector = min(m); minMean_speed = min(minMean_vector);
maxVar_vector = max(s2); maxVar_speed = max(maxVar_vector);
minVar_vector = min(s2); minVar_speed = min(minVar_vector);

XmaxMean_speed = maxMean_speed;
XminMean_speed = minMean_speed;
XmaxVar_speed  = maxVar_speed;
XminVar_speed  = minVar_speed;
x_mean =  m_deputy; x_var = s2_deputy;
x_hyp = hyp;

save('GP_speed_x','x_mean','x_var', 'x_hyp', 'XmaxMean_speed', 'XminMean_speed', 'XmaxVar_speed', 'XminVar_speed');

direction_mean = mat2gray(m_deputy, [minMean_speed, maxMean_speed]);
imwrite(direction_mean, 'both_Xspeed.jpg', 'jpg');
direction_var = mat2gray(s2_deputy, [minVar_speed maxVar_speed]);
imwrite(direction_var, 'both_Xspeed_var.jpg', 'jpg');

toc
%save('cluster1_Xspeed_params.txt', 'maxMean_speed', 'minMean_speed', 'maxVar_speed', 'minVar_speed', '-ASCII')
    