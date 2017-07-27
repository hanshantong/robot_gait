clear
close all
load('data.txt')
% load('data_3S_NO_CONTROL.txt')
% data = data_3S_NO_CONTROL;
% load('data_ssp_after_offset_soft_left.txt')
% data = data_ssp_after_offset_soft_left;
% load('data_ssp_after_offset_soft_right.txt')
% data = data_ssp_after_offset_soft_right;
% load('data_left_ssp_no_scaling.txt')
% data = data_left_ssp_no_scaling;
% load('data_ssp_no_scaling_left_leg.txt')
% data = data_ssp_no_scaling_left_leg;
% load('data_ssp_no_scaling_right_leg.txt')
% data = data_ssp_no_scaling_right_leg;
% load('data_good_walk.txt')
% data = data_good_walk;
% load('data_walk_ok.txt')
% data = data_walk_ok;
plot_q          = 1;
plot_torque     = 0;
plot_zmp        = 0;
plot_est        = 0;
plot_est_state  = 0;
plot_fk         = 0;
plot_ref        = 1;
plot_f          = 0;
plot_body_angle = 0;
plot_phase      = 0;
plot_yaw        = 0;
scale   = 1;
begin   = 1;     
% stop = 1000;  
stop    = size(data,1);
t       = data(begin:stop,1);
 
robot_width             = 0.18;
foot_length_front       = 0.155;
% foot_length_back        = 0.095;
foot_width_inner        = 0.05;
foot_width_outer        = 0.09;

q1_ref = data(begin:stop,2)*scale;
q1_tau = data(begin:stop,4);
q2_ref = data(begin:stop,5)*scale;
q2_tau = data(begin:stop,7);
q3_ref = data(begin:stop,8)*scale;
q3_tau = data(begin:stop,10);
q4_ref = data(begin:stop,11)*scale;
q4_tau = data(begin:stop,13);
q5_ref = data(begin:stop,14)*scale;
q5_tau = data(begin:stop,16);
q6_ref = data(begin:stop,17)*scale;
q6_tau = data(begin:stop,19);
q7_ref = data(begin:stop,20)*scale;
q7_tau = data(begin:stop,22);
q8_ref = data(begin:stop,23)*scale;
q8_tau = data(begin:stop,25);
q9_ref = data(begin:stop,26)*scale;
q9_tau = data(begin:stop,28);
q10_ref = data(begin:stop,29)*scale;
q10_tau = data(begin:stop,31);
q11_ref = data(begin:stop,32)*scale;
q11_tau = data(begin:stop,34);
q12_ref = data(begin:stop,35)*scale;
q12_tau = data(begin:stop,37);

q1 = data(begin:stop,3)*scale;
q2 = data(begin:stop,6)*scale;
q3 = data(begin:stop,9)*scale;
q4 = data(begin:stop,12)*scale;
q5 = data(begin:stop,15)*scale;
q6 = data(begin:stop,18)*scale;
q7 = data(begin:stop,21)*scale;
q8 = data(begin:stop,24)*scale;
q9 = data(begin:stop,27)*scale;
q10 = data(begin:stop,30)*scale;
q11 = data(begin:stop,33)*scale;
q12 = data(begin:stop,36)*scale;
Ts = 5e-3;
q1d = diff(q1)/Ts;
q2d = diff(q2)/Ts;
q3d = diff(q3)/Ts;
q4d = diff(q4)/Ts;
q5d = diff(q5)/Ts;
q6d = diff(q6)/Ts;

q7d = diff(q7)/Ts;
q8d = diff(q8)/Ts;
q9d = diff(q9)/Ts;
q10d = diff(q10)/Ts;
q11d = diff(q11)/Ts;
q12d = diff(q12)/Ts;

d = 0.006;

Fxl = data(begin:stop,38);
Fyl = -data(begin:stop,39);
Fzl = data(begin:stop,40);
Txl = -data(begin:stop,41);
Tyl = data(begin:stop,42);
Tzl = data(begin:stop,43);


Fxr = data(begin:stop,44);
Fyr = -data(begin:stop,45);
Fzr = data(begin:stop,46);
Txr = -data(begin:stop,47);
Tyr = data(begin:stop,48);
Tzr = data(begin:stop,49);

% ZMP_x = ZMP_x(1:2000);
% ZMP_y = ZMP_y(1:2000);

phi = data(begin:stop,50);
theta = data(begin:stop,51);
psi = data(begin:stop,52);

x_est = data(begin:stop,53);
vx_est = data(begin:stop,54);
ax_est = data(begin:stop,55);
y_est = data(begin:stop,56);
vy_est = data(begin:stop,57);
ay_est = data(begin:stop,58);
zmpx_calc = data(begin:stop,59);
zmpy_calc = data(begin:stop,60);
comx_fk = data(begin:stop,61);
comy_fk = data(begin:stop,62);
comvy_fk = diff(comy_fk)/5e-3;
comay_fk = diff(comvy_fk)/5e-3;
supx_fk = data(begin:stop,63);
supy_fk = data(begin:stop,64);
supz_fk = data(begin:stop,117);
swingx_fk = data(begin:stop,65);
swingy_fk = data(begin:stop,66);
swingz_fk = data(begin:stop,118);
comx_ref = data(begin:stop,67);
comy_ref = data(begin:stop,68);
x_offs = data(begin:stop,69);
y_offs = data(begin:stop,70);
x_output1 = data(begin:stop,71);
x_output2 = data(begin:stop,72);
y_output1 = data(begin:stop,73);
y_output2 = data(begin:stop,74);

comvy_ref = data(begin:stop,75);
comay_ref = data(begin:stop,76);
y_mpc = data(begin:stop,77);
vy_mpc = data(begin:stop,78);
ay_mpc = data(begin:stop,79);

com_phi = data(begin:stop,80)*180/pi;
com_theta = data(begin:stop,81)*180/pi;
com_psi = data(begin:stop,82)*180/pi;
sup_phi = data(begin:stop,83)*180/pi;
sup_theta = data(begin:stop,84)*180/pi;
sup_psi = data(begin:stop,85)*180/pi;
swing_phi = data(begin:stop,86)*180/pi;
swing_theta = data(begin:stop,87)*180/pi;
swing_psi = data(begin:stop,88)*180/pi;

bodyx = data(begin:stop,89);
bodyy = data(begin:stop,90);
bodyz = data(begin:stop,91);

lfootx = data(begin:stop,92);
lfooty = data(begin:stop,93);
lfootz = data(begin:stop,94);

rfootx = data(begin:stop,95);
rfooty = data(begin:stop,96);

rfootz = data(begin:stop,97);

prefy  = data(begin:stop,98);
phase = data(begin:stop, 99);
idx = [phase == 0];
idx = find(idx);
idx = idx(1:50:end);

idx1 = [phase == 1];
idx1 = find(idx1);
idx1 = idx1(1:10:end);

offset_l_phi = data(begin:stop, 100)*180/pi;
offset_l_theta = data(begin:stop, 101)*180/pi;
offset_l_psi = data(begin:stop, 102)*180/pi;

offset_r_phi = data(begin:stop, 103)*180/pi;
offset_r_theta = data(begin:stop, 104)*180/pi;
offset_r_psi = data(begin:stop, 105)*180/pi;

Tipping_scale = data(1,106);
prefx = data(begin:stop,107);
body_theta = data(begin:stop, 109)*180/pi;
body_phi   = data(begin:stop, 110)*180/pi;

philref = data(begin:stop, 111)*180/pi;
thetalref = data(begin:stop, 112)*180/pi;
psilref = data(begin:stop, 113)*180/pi;

phirref = data(begin:stop, 114)*180/pi;
thetaref = data(begin:stop, 115)*180/pi;
psirref = data(begin:stop, 116)*180/pi;

ZMP_xl = (-Tyl-Fxl*d)./Fzl+prefx;
ZMP_yl = (Txl-Fyl*d)./Fzl+0.09;

ZMP_xr = (-Tyr-Fxr*d)./Fzr+prefx;
ZMP_yr = (Txr-Fyr*d)./Fzr-0.09;

ZMP_x = (ZMP_xl.*Fzl+ZMP_xr.*Fzr)./(Fzl+Fzr);
ZMP_y = (ZMP_yl.*Fzl+ZMP_yr.*Fzr)./(Fzl+Fzr);

ref_psi_support = data(begin:stop, 119);
ref_psi_swing = data(begin:stop, 120);

leftSupport = data(begin:stop, 121);
ref_psi_body = data(begin:stop, 122);

x_sup = data(begin:stop, 123);
y_sup = data(begin:stop, 124);
z_sup = data(begin:stop, 125);

x_swing = data(begin:stop, 126);
y_swing = data(begin:stop, 127);
z_swing = data(begin:stop, 128);

walking_mode = data(begin:stop, 129);
t = 1:length(t);
if plot_q == 1
    figure('units','normalized','outerposition',[-1 0 1 1],'name','Joint Angles')
    subplot(3,2,1)
    plot(t,q1,'-')
    hold on
    grid on
    grid minor
    
    plot(t,q2,'-')
    plot(t,q3,'-')
    plot(t,q1_ref,'-')
    hold on
    grid on
    plot(t,q2_ref,'-')
    plot(t,q3_ref,'-')
    legend('q1','q2','q3','q1 ref','q2 ref', 'q3 ref')
    subplot(3,2,3)
    plot(t,q4)
    hold on
    grid on
    grid minor
    
    plot(t,q4_ref,'-')
        plot(t, walking_mode*10)

    legend('q4','q4 ref')
    
    subplot(3,2,5)
    plot(t,q5,'-')
    grid on
    hold on
    grid minor
    
    plot(t,q6,'-')
    plot(t,q5_ref,'-')
    hold on
    plot(t,q6_ref,'-')
    legend('q5','q6','q5 ref', 'q6 ref')
    
    subplot(3,2,2)
    plot(t,q7,'-')
    hold on
    grid on
    
    grid minor
    plot(t,q8,'-')
    plot(t,q9,'-')
    plot(t,q7_ref,'-')
    grid on
    hold on
    plot(t,q8_ref,'-')
    plot(t,q9_ref,'-')
    legend('q7','q8','q9','q7 ref','q8 ref', 'q9 ref')
    
    subplot(3,2,4)
    plot(t,q10,'-')
    hold on
    grid on
    grid minor
    plot(t,q10_ref,'-')
    legend('q10','q10 ref')
    
    subplot(3,2,6)
    plot(t,q11,'-')
    hold on
    grid on
    grid minor
    
    plot(t,q12,'-')
    plot(t,q11_ref,'-')
    grid on
    hold on
    plot(t,q12_ref,'-')
    legend('q11','q12','q11 ref','q12 ref')
end

if plot_torque == 1
    figure('units','normalized','outerposition',[-1 0 1 1],'name','Joint Torques')
    subplot(3,2,1)
    plot(t,q1_tau,'.')
    hold on
    grid on
    plot(t,q2_tau,'-')
    plot(t,q3_tau,'-')
    legend('q1','q2','q3')
    
    subplot(3,2,3)
    plot(t,q4_tau)
    grid on
    legend('q4')
    
    subplot(3,2,5)
    plot(t,q5_tau,'-')
    grid on
    hold on
    plot(t,q6_tau,'-')
    legend('q5','q6')
    
    subplot(3,2,2)
    plot(t,q7_tau,'-')
    hold on
    grid on
    plot(t,q8_tau,'-')
    plot(t,q9_tau,'-')
    legend('q7','q8','q9')
    
    subplot(3,2,4)
    plot(t,q10_tau,'-')
    grid on
    legend('q10')
    
    subplot(3,2,6)
    plot(t,q11_tau,'-')
    hold on
    grid on
    plot(t,q12_tau,'-')
    legend('q11','q12')
end

if plot_zmp == 1
    figure('units','normalized','outerposition',[-1 0 1 1],'name','ZMP')
    subplot(2,1,1)
    plot(t, ZMP_x)
    hold on
    grid on
    grid minor
    plot(t, ZMP_xl,'.')
    plot(t, ZMP_xr,'.')
    legend('ZMP','ZMP left', 'ZMP right')
    ylim([0 0.4])
    
    subplot(2,1,2)
    plot(t, ZMP_y)
    hold on
    grid on
    grid minor
    plot(t, ZMP_yl,'.')
    plot(t, ZMP_yr,'.')
    ylim([-0.2 0.2])
    legend('ZMP','ZMP left', 'ZMP right')
end

if plot_est == 1
    figure('units','normalized','outerposition',[-1 0 1 1],'name','Estimation')
    subplot(2,1,1)
    plot(t, x_est,'-');
    hold on
    grid on
    grid minor
    plot(t,comx_ref,'-')
    plot(t, comx_fk,'-')
    plot(t, zmpx_calc,'.')
    plot(t, x_output1,'-')
    plot(t, prefx,'.')
    
    legend('x state (no offset) estimation', 'X ref', 'X fk', 'ZMP X', ...
        'X output (with offset) estimation','ZMP x ref')
    
    subplot(2,1,2)
    %         yyaxis left
    
    plot(t, y_est,'-')
    hold on
    grid on
    grid minor
    %     plot(t, y_output1,'-')
    plot(t,comy_ref,'-')
    plot(t, comy_fk,'-')
    plot(t, prefy,'.')
    plot(t, zmpy_calc,'--')
    plot(t, y_output2,'r')
    %     a = gca;
    %     plot([idx, idx], [a.YLim], 'g')
    
    
    %     plot([t(1) t(end)], [0.049 0.049], 'c')
    %         plot(t, lfootz,'.')
    %     yyaxis right
    %     plot(t, Fzl)
    %     plot([t(1) t(end)], [0.196 0.196], 'c')
    
    legend('y state (no offset) estimation','reference com y',...
        'Y fk', 'Reference ZMP Y', ...
        'Real (measured) zmp y','Model ZMP (estimated - LIP)','FLC phase')
    %     subplot(3,1,3)
    %     plot(t, x_offs)
    %     hold on
    %     grid on
    %     plot(t, y_offs)
    %     legend('X_offset', 'Y_offset')
%     figure
%     idx_phase_1 = phase == 1;
%     idx_phase_0 = phase == 0;
%     Fzl_tmp = Fzl(idx_phase_0);
%     Fzr_tmp = Fzr(idx_phase_0);
%     
%     idx_l_touching = Fzl_tmp < -20;
%     idx_r_touching = Fzr_tmp < -20;
%     ZMP_xl_tmp = ZMP_xl(idx_phase_0);
%     ZMP_yl_tmp = ZMP_yl(idx_phase_0);
%     ZMP_xr_tmp = ZMP_xr(idx_phase_0);
%     ZMP_yr_tmp = ZMP_yr(idx_phase_0);
%     
%     idx_ = phase == 2 | phase == 3;
%     plot(zmpx_calc(idx_),zmpy_calc(idx_),'.')
%     hold on
%     grid on
%     grid minor
%     plot(zmpx_calc(idx_phase_1),zmpy_calc(idx_phase_1),'r.')
%     plot(zmpx_calc(idx_phase_0),zmpy_calc(idx_phase_0),'g.')
% %     plot(ZMP_xl_tmp(idx_l_touching), ZMP_yl_tmp(idx_l_touching),'b.')
% %     plot(ZMP_xr_tmp(idx_r_touching), ZMP_yr_tmp(idx_r_touching),'b.')
%     rectangle('Pos', [0.00-foot_length_back robot_width/2-foot_width_inner foot_length_back+foot_length_front foot_width_inner+foot_width_outer])    
%     plot(0.00, robot_width/2,'rx')
%     rectangle('Pos', [0.0-foot_length_back -robot_width/2-foot_width_outer foot_length_back+foot_length_front foot_width_inner+foot_width_outer])
%     plot(0.0, -robot_width/2,'rx')
%     for i = 1:4
%         x_tmp = 0.15*i-foot_length_back;
%         if mod(i,2) == 1
%             y_tmp = robot_width/2-foot_width_inner;
%             plot(0.15*i, robot_width/2,'rx')
%         else
%             y_tmp = -robot_width/2-foot_width_outer;
%             plot(0.15*i, -robot_width/2,'rx')
%         end
%         
%         rectangle('Pos', [x_tmp y_tmp  foot_length_back+foot_length_front foot_width_inner+foot_width_outer])
%         
%         
%     end
%     legend('Single support', 'Foot landing control', 'Double Support')
%     %     rectangle('Pos', [-2 0 1 2])
end



if plot_est_state == 1
    zmp_est = [1 0 -0.6/9.81]*[y_est, vy_est, ay_est]';
    
    figure('units','normalized','outerposition',[-1 0 1 1],'name','Estimation')
    subplot(3,1,1)
    plot(t,comy_ref,'-')
    hold on
    plot(t,y_est,'-');
    plot(t, zmp_est)
    plot(t, ZMP_y)
    plot(t, prefy,'-','linewidth',0.5)
    plot(t, comy_fk,'-')
    
    legend('Comy reference', 'Y est','estimated zmp','real zmp','reference zmp','COM fk')
    subplot(3,1,2)
    plot(t, comvy_ref,'-')
    hold on
    plot(t, vy_est,'-')
    legend('Ref vy', 'Est vy')
    subplot(3,1,3)
    plot(t, comay_ref,'-')
    hold on
    %     plot(t, ay_mpc)
    plot(t,ay_est ,'-')
    legend('Ref ay',  'Est ay')
end

% supy_fk = supy_fk+comy_fk;
% swingy_fk = swingy_fk +comy_fk;
if plot_f == 1
    figure('units','normalized','outerposition',[-1 0 1 1],'name','Force')
    subplot(3,1,1)
    plot(t, Fzl)
    grid on
    grid minor
    hold on
    plot(t,Fzr)
%     a = gca;
%     plot([idx, idx], [a.YLim], 'g')
    legend('Fzl','Fzr')
    title('Fz')
    subplot(3,1,2)
    plot(t, Tyl)
    grid on
    grid minor
    hold on
    plot(t,Tyr)
%     a = gca;
%     plot([idx, idx], [a.YLim], 'g')
    title('Ty')
    legend('Tyl','Tyr')
    subplot(3,1,3)
    plot(t, Txl)
    grid on
    grid minor
    hold on
    plot(t,Txr)
%     a = gca;
%     plot([idx, idx], [a.YLim], 'g')
    legend('Txl','Txr')
    title('Tx')
    
end
if plot_fk == 1
    figure('units','normalized','outerposition',[-1 0 1 1],'name','Position FK')
    subplot(2,1,1)
    plot(t, supx_fk,'.')
    hold on
    grid on
    grid minor
    plot(t, swingx_fk,'.')
    plot(t, comx_fk,'.')
    plot(t, comx_ref,'.')
    plot(t, lfootx,'.')
    plot(t, rfootx,'.')
    legend('Sup x', 'Swing x','COM','Ref COM','Left foot', 'Right foot')
    subplot(2,1,2)
    plot(t, supy_fk,'.')
    hold on
    plot(t, swingy_fk,'.')
    plot(t, comy_fk,'.')
    plot(t,comy_ref,'.')
    plot(t, lfooty,'.')
    plot(t, rfooty,'.')
    
    legend('Sup y', 'Swing y', 'COM', 'Ref COM','Left foot','Right foot')
    
    figure('units','normalized','outerposition',[-1 0 1 1],'name','FK Euler Angles')
    %     plottools
    subplot(3,1,1)
    plot(t, com_phi)
    hold on
    grid on
    grid minor
    plot(t, sup_phi,'.','linewidth',2)
    plot(t, swing_phi,'.','linewidth',2)
    plot(t,philref,'.')
    plot(t,phirref,'.')
    plot(t,offset_l_phi,'.')
    plot(t,offset_r_phi,'.')
    %     a = gca;
    %         plot([idx, idx], [a.YLim], 'g')
    
    legend('COM', 'SUP', 'SWING','Phiref_l','Phiref_r','Offset Phiref_l',' Offset Phiref_r')
    
    subplot(3,1,2)
    plot(t, com_theta)
    hold on
    grid on
    grid minor
    plot(t, sup_theta,'.','linewidth',2)
    plot(t, swing_theta,'.','linewidth',2)
    plot(t,thetalref,'.')
    plot(t,thetaref,'.')
    plot(t,offset_l_theta,'.')
    plot(t,offset_r_theta,'.')
    %         a = gca;
    %         plot([idx, idx], [a.YLim], 'g')
    legend('COM', 'SUP', 'SWING','Thetaref_l','Thetaref_r','Offset Phiref_l',' Offset Phiref_r')
    
    subplot(3,1,3)
    plot(t, com_psi)
    hold on
    grid on
    grid minor
    plot(t, sup_psi,'.','linewidth',2)
    plot(t, swing_psi,'.','linewidth',2)
    plot(t,psilref,'.')
    plot(t,psirref,'.')
    plot(t,offset_l_psi,'.')
    plot(t,offset_r_psi,'.')
    %     a = gca;
    %     plot([idx, idx], [a.YLim], 'g')
    legend('COM', 'SUP', 'SWING','Psiref_l','Psiref_r','Offset Phiref_l',' Offset Phiref_r')
    
end

if plot_ref == 1
    figure('units','normalized','outerposition',[-1 0 1 1],'name','Reference foot trajectories')
    subplot(5,1,1)
    plot(t, bodyx,'.')
    hold on
%     grid on
%     plot(t, lfootx,'.')
%     plot(t, rfootx,'.')
    plot(t, x_sup,'--')
    plot(t, x_swing,'--')
    plot(t,prefx)
    legend('body','lfoot','rfoot')
    title('x axis')
    subplot(5,1,2)
    plot(t, bodyy,'.')
    hold on
    grid on
%     plot(t, lfooty,'.')
%     plot(t, rfooty,'.')
    plot(t, y_sup,'.')
    plot(t, y_swing,'.')
    plot(t, prefy)
%     plot(t, leftSupport*0.1,'.')
    legend('body','lfoot','rfoot', 'reference zmp')
    title('y axis')
    subplot(5,1,3)
    plot(t, lfootz,'.')
    hold
    grid on
    grid minor
    plot(t, rfootz,'.')
%     plot(t, swingz_fk,'.')
%     plot(t, supz_fk,'.')
    %     a = gca;
    %     plot([idx1, idx1], [a.YLim], 'g--')
    %     plottools
    legend('lfoot reference','rfoot reference','Swing fk','Sup fk')
    title('z axis')
    subplot(5,1,4)
%     plot(t,phase,'.')
    plot(rfootx, rfootz,'.')
    hold on%     plot(t, leftSupport*0.1,'.')
    grid on
    grid minor
    plot(lfootx, lfootz,'.')
     subplot(5,1,5)
%     plot(t,phase,'.')
    plot(rfooty, rfootz,'.')
    hold on%     plot(t, leftSupport*0.1,'.')
    grid on
    grid minor
    plot(lfooty, lfootz,'.')
end

if plot_body_angle == 1
    
    figure('units','normalized','outerposition',[-1 0 1 1],'name','Body angles')
    subplot(3,1,1)
    plot(t, phi,'.')
    hold on
    grid on
    grid minor
    plot(t, body_phi,'.','linewidth',2)
%     a = gca;
%     plot([idx, idx], [a.YLim], 'g')
%     ylim([-10 10])
    legend('IMU Phi')
    
    subplot(3,1,2)
    plot(t, theta,'.')
    hold on
    grid on
    grid minor
    plot(t, body_theta,'.','linewidth',2)
    %     plot(t, body_theta,'.')
%     a = gca;
%     plot([idx, idx], [a.YLim], 'g')
%     ylim([-10 10])
    legend('IMU Theta', 'Reference theta')
    
    subplot(3,1,3)
    plot(t, psi,'.')
    hold on
    grid on
    grid minor
    plot(t, com_psi)
%     a = gca;
%     plot([idx, idx], [a.YLim], 'g')
%     ylim([-20 20])
    legend('IMU Psi')
    
end

if plot_phase == 1
    figure('units','normalized','outerposition',[-1 0 1 1],'name','Body angles')
    plot(t,phase,'.')
    grid on
    grid minor
end
% a  = length(comy_ref);
% plot(1:a, comy_ref(1:a))
% hold on
% plot(1:a, ZMP_y)
% ylim([0 0.3])


% figure
% plot(t, data(:,101),'-')
% grid on
% grid minor


if plot_yaw == 1
    figure('units','normalized','outerposition',[-1 0 1 1],'name','Reference foot trajectories')
    plot(lfootx, lfooty,'.','linewidth',2)
    hold on
    grid on
    grid minor
    axis equal
	plot(rfootx, rfooty,'-','linewidth',2)
    plot(prefx, prefy,'--')
    
    plot(rfootx, rfooty,'rx')
    for i  = 1:500:length(ref_psi_support)
        len = 0.1;
        
        xshort = len * cos(ref_psi_support(i));
        yshort = len * sin(ref_psi_support(i));
%         yshort = tan(ref_psi_support(i))*xshort;
        
        plot([lfootx(i) lfootx(i)+xshort], [lfooty(i) lfooty(i)+yshort],'r','linewidth',5)
 
        xshort = len * cos(ref_psi_swing(i));
        yshort = len * sin(ref_psi_swing(i));
%         yshort = tan(ref_psi_swing(i))*xshort;
        
        plot([rfootx(i) rfootx(i)+xshort], [rfooty(i) rfooty(i)+yshort],'r','linewidth',2)

%     plot([swing(i,1) swing(i,1)+xshort], [swing(i,2) swing(i,2)+yshort],'c','linewidth',2)
    end
    
    

foot_distance_x = diff(lfootx);
foot_distance_y = diff(lfooty);

foot_distance = sqrt(foot_distance_x.^2+foot_distance_y.^2);
foot_distance(foot_distance == 0) = [];

%     subplot(3,1,1)
%     plot(t, bodyx,'.')
%     hold on
%     grid on
%     plot(t, lfootx,'.')
%     plot(t, rfootx,'.')
% figure
% subplot(2,1,1)
%     plot(t,prefx,'.')
%     grid on
%     hold on
%     grid minor
% %     legend('body','lfoot','rfoot')
% %     title('x axis')
% %     subplot(3,1,2)
% %     plot(t, bodyy,'.')
% %     hold on
% %     grid on
% %     plot(t, lfooty,'.')
% %     plot(t, rfooty,'.')
%     plot(t, lfootx,'.')
% subplot(2,1,2)
%     plot(t, prefy,'.')
%     hold on
%     grid on 
%     grid minor
%     plot(t, lfooty,'.')
%     legend('body','lfoot','rfoot', 'reference zmp')
%     title('y axis')
%     subplot(3,1,3)
%     plot(t, lfootz,'.')
%     hold
%     grid on
%     grid minor
%     plot(t, rfootz,'.')
%     plot(t, swingz_fk,'.')
%     plot(t, supz_fk,'.')
%     %     a = gca;
%     %     plot([idx1, idx1], [a.YLim], 'g--')
%     %     plottools
%     legend('lfoot reference','rfoot reference','Swing fk','Sup fk')
%     title('z axis')
figure
plot(t, ref_psi_support*180/pi,'.')
hold on
 grid on
    grid minor
plot(t, ref_psi_swing*180/pi,'.')
plot(t, ref_psi_body*180/pi,'.')
legend('Support','Swing','Body')
% plot(t, leftSupport*10)
end

% figure 
% plot(t,psilref)
% hold on
% plot(t,psirref)
