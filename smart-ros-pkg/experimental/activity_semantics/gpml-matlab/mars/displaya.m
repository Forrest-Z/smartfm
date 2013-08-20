%Generate a surface to plot in 3d
imOnXY=imread('ipm_color_image.png');
[x,y,width]=size(imOnXY);
x
y
width
z = peaks(120);
%Generate an image to plot on the xy plane (z=0)
%imOnXY=rand([size(peaks,1),size(peaks,2),3]);
figure(1);
%Show the image
a = 0*peaks(120)-10;
hold on
surface(a,imOnXY,'FaceColor','texturemap','EdgeColor','none','CDataMapping','direct')
surface(z,'FaceAlpha',0.5,'LineStyle','none','FaceColor','interp');
axis on
view(-35,45)