clear
clc
clf
step_amount = 30;

xg = 5;
yg = -2;
psi = 50.00001*pi/180;
% psi = tan(yg/xg);
%% USE TWO DIFFERENT SPLINE FUNCTIONS DEPENDING ON XY AND PSI
xshort = 0.0001;
yshort = tan(psi)*xshort;
% if psi<90*pi/180
    x2 = xg-xshort;
    y2 = yg-yshort;
% else
%     x2 = xg+xshort;
%     y2 = yg+yshort;
% end

xplot = xg-xshort*500;
yplot = yg-yshort*500;


x1 = x2*psi*180/pi/100;
y1 = -0;

% scale = psi/100*180/pi;
% scale1 = scale;
% scale2 = scale1*scale;
% x1 = x2*scale1;
% y1 = y2*scale2;

xtmp = x2-xg;
ytmp = y2-yg;
alpha = atan(ytmp/xtmp)*180/pi;

x_sample = [0, 0.01, x1, x2, xg];
y_sample = [0, 0,    y1, y2, yg];
x_lin = linspace(0, xg, 1000);
y_lin = spline(x_sample, y_sample, x_lin);

x_sample2 = [0, 0.01, x2, xg];
y_sample2 = [0, 0,    y2, yg];
x_lin2 = linspace(0, xg, 1000);
y_lin2 = spline(x_sample2, y_sample2, x_lin2);


x_distance = diff(x_lin);
y_distance = diff(y_lin);

dist = sqrt(x_distance.^2 + y_distance.^2);
total_dist = sum(dist);


x_distance2 = diff(x_lin2);
y_distance2 = diff(y_lin2);

dist2 = sqrt(x_distance2.^2 + y_distance2.^2);
total_dist2 = sum(dist2);




if total_dist < total_dist2
    working_spline = [x_lin; y_lin];
    dist = total_dist;
else
    working_spline = [x_lin2; y_lin2];
    dist = total_dist2;
end

step_length = 0.15;

swing = [];

support = [];
for i = 1:step_amount
%    if i ~= step_amount
%        idx = floor(length(working_spline)/step_amount)*i;
%    else
%        idx = length(working_spline);
%    end
   if i == 1
       idx = 1;
       x_tmp = working_spline(1, idx);
       y_tmp = working_spline(2, idx);
        
   else
       x_tmp = support(end, 1) + step_length*cos(support(end,3));
%        y_tmp = support(end, 2) + step_length*sin(support(end,3));
       idx = dsearchn(working_spline(1,:)',x_tmp);
   end
  
%    if i == 37
%        a = 3;
%    end
          y_tmp = working_spline(2, idx);

   yaw = atan((working_spline(2,idx+1)-working_spline(2,idx))/(working_spline(1,idx+1)-working_spline(1,idx)));
   i
   support  = [support; x_tmp, y_tmp, yaw];
%    support  = [support; x_tmp, y_tmp, yaw];
%    swing    = [swing; x_tmp, y_tmp+(-1)^(i+1)*0.1, yaw];

end
for i = 1:length(support)
   support(i,1) = support(i,1)-(-1)^i*sin(support(i,3))*0.1;
   support(i,2) = support(i,2)+(-1)^i*cos(support(i,3))*0.1;
end
plot(xg,yg,'kx')
grid on
grid minor
hold on
axis equal
plot(x1,y1,'bx')
plot(xplot,yplot,'gx')
plot(0,0,'ko')
plot(x_lin, y_lin, 'b.')
plot(x_lin2, y_lin2, 'g.')
plot(support(:,1), support(:,2),'rx')

% plot(swing(:,1), swing(:,2),'cx')
for i  = 1:step_amount
    xshort = 0.05;
    yshort = tan(support(i,3))*xshort;
    plot([support(i,1) support(i,1)+xshort], [support(i,2) support(i,2)+yshort],'r','linewidth',2)
%     plot([swing(i,1) swing(i,1)+xshort], [swing(i,2) swing(i,2)+yshort],'c','linewidth',2)
end

if total_dist < total_dist2
    legend('goal','first', 'second','origin', ['Shorter: ' num2str(total_dist)], ['Longer: ' num2str(total_dist2)],'Location','southeast')
else
    legend('goal','first', 'second','origin',['Longer: ' num2str(total_dist)], ['Shorter: ' num2str(total_dist2)],'Location','southeast')
end


foot_distance_x = diff(support(:,1));
foot_distance_y = diff(support(:,2));

foot_distance = sqrt(foot_distance_x.^2+foot_distance_y.^2)
