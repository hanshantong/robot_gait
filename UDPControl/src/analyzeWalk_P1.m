clear
close all
load('data.txt')

plot_q          = 1;
plot_torque     = 0;
plot_zmp        = 0;
plot_est        = 0;
plot_est_state  = 0;
plot_fk         = 0;
plot_ref        = 1;
plot_f          = 0;
plot_body_angle = 0;
% cut = 1000;

scale  = 1;
begin = 1;
stop  = size(data,1);
t = data(begin:stop,1);

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
Fyl = data(begin:stop,39);
Fzl = data(begin:stop,40);
Txl = data(begin:stop,41);
Tyl = data(begin:stop,42);
Tzl = data(begin:stop,43);

ZMP_xl = (-Tyl-Fxl*d)./Fzl;
ZMP_yl = (Txl-Fyl*d)./Fzl-0.09;

Fxr = data(begin:stop,44);
Fyr = data(begin:stop,45);
Fzr = data(begin:stop,46);
Txr = data(begin:stop,47);
Tyr = data(begin:stop,48);
Tzr = data(begin:stop,49);

ZMP_xr = (-Tyr-Fxr*d)./Fzr;
ZMP_yr = (Txr-Fyr*d)./Fzr+0.09;

ZMP_x = (ZMP_xl.*Fzl+ZMP_xr.*Fzr)./(Fzl+Fzr);
ZMP_y = (ZMP_yl.*Fzl+ZMP_yr.*Fzr)./(Fzl+Fzr);

% ZMP_x = ZMP_x(1:2000);
% ZMP_y = ZMP_y(1:2000);

phi = data(begin:stop,50);
theta = data(begin:stop,51);
psi = data(begin:stop,52);

lfootx = data(begin:stop,53);
lfooty = data(begin:stop,54);
lfootz = data(begin:stop,55);

rfootx = data(begin:stop,56);
rfooty = data(begin:stop,57);
rfootz = data(begin:stop,58);

bodyx = data(begin:stop,59);
bodyy = data(begin:stop,60);
bodyz = data(begin:stop,61);

supx_fk = data(begin:stop,62);
supy_fk = data(begin:stop,63);
supz_fk = data(begin:stop,65);

prefx = data(begin:stop,70);
prefy = data(begin:stop,71);



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
    plot(t, ZMP_xl)
    plot(t, ZMP_xr)
    
    subplot(2,1,2)
    plot(t, ZMP_y)
    hold on
    grid on
    grid minor
    plot(t, ZMP_yl)
    plot(t, ZMP_yr)
    
end

if plot_est == 1
    figure('units','normalized','outerposition',[-1 0 1 1],'name','Estimation')
    %     subplot(3,1,1)
    %     plot(t, x_est,'-');
    %     hold on
    %     plot(t, comx_fk,'-')
    %     plot(t, zmpx_calc,'--')
    %     plot(t, x_output1,'-')
    %
    %     legend('x state (no offset) estimation', 'X fk', 'ZMP X', ...
    %         'X output (with offset) estimation')
    %
    %     subplot(3,1,2)
    %     yyaxis left
    
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
    a = gca;
    plot([idx, idx], [a.YLim], 'g')
    
    
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
    a = gca;
    plot([idx, idx], [a.YLim], 'g')
    legend('Fzl','Fzr')
    title('Fz')
    subplot(3,1,2)
    plot(t, Tyl)
    grid on
    grid minor
    hold on
    plot(t,Tyr)
    a = gca;
    plot([idx, idx], [a.YLim], 'g')
    title('Ty')
    legend('Tyl','Tyr')
    subplot(3,1,3)
    plot(t, Txl)
    grid on
    grid minor
    hold on
    plot(t,Txr)
    a = gca;
    plot([idx, idx], [a.YLim], 'g')
    legend('Txl','Txr')
    title('Tx')
    
end
if plot_fk == 1
    %     figure('units','normalized','outerposition',[-1 0 1 1],'name','FK')
    %     subplot(2,1,1)
    %     plot(t, supx_fk,'.')
    %     hold on
    %     grid on
    %     grid minor
    %     plot(t, swingx_fk,'.')
    %     plot(t, comx_fk,'.')
    %     plot(t, comx_ref,'.')
    %     plot(t, lfootx,'.')
    %     plot(t, rfootx,'.')
    %     legend('Sup x', 'Swing x','COM','Ref COM','Left foot', 'Right foot')
    %     subplot(2,1,2)
    %     plot(t, supy_fk,'.')
    %     hold on
    %     plot(t, swingy_fk,'.')
    %     plot(t, comy_fk,'.')
    %     plot(t,comy_ref,'.')
    %     plot(t, lfooty,'.')
    %     plot(t, rfooty,'.')
    %
    %     legend('Sup y', 'Swing y', 'COM', 'Ref COM','Left foot','Right foot')
    %
    figure('units','normalized','outerposition',[-1 0 1 1],'name','FK')
    %     plottools('on')
    
    subplot(2,1,1)
    plot(t, com_phi)
    hold on
    grid on
    grid minor
    plot(t, sup_phi,'.','linewidth',2)
    plot(t, swing_phi,'.','linewidth',2)
    plot(t,philref,'.')
    plot(t,phirref,'.')
    plot(t,offset_l_phi,'k-.','linewidth',2)
    plot(t,offset_r_phi,'k--','linewidth',2)
    % a = gca;
    %     plot([idx, idx], [a.YLim], 'g')
    legend('COM', 'SUP', 'SWING','Phiref_l','Phiref_r','Offset Phiref_l',' Offset Phiref_r')
    
    subplot(2,1,2)
    plot(t, com_psi)
    hold on
    grid on
    grid minor
    plot(t, sup_psi,'.','linewidth',2)
    plot(t, swing_psi,'.','linewidth',2)
    plot(t,psilref,'.')
    plot(t,psirref,'.')
    plot(t,offset_l_psi,'k-.','linewidth',2)
    plot(t,offset_r_psi,'k--','linewidth',2)
    %     a = gca;
    %     plot([idx, idx], [a.YLim], 'g')
    legend('COM', 'SUP', 'SWING','Psiref_l','Psiref_r','Offset Phiref_l',' Offset Phiref_r')
    
    figure('units','normalized','outerposition',[-1 0 1 1],'name','FK')
    
    %     subplot(3,1,2)
    plot(t, com_theta)
    hold on
    grid on
    grid minor
    plot(t, sup_theta,'.','linewidth',2)
    plot(t, swing_theta,'.','linewidth',2)
    plot(t,thetalref,'.')
    plot(t,thetaref,'.')
    plot(t,offset_l_theta,'-.')
    plot(t,offset_r_theta,'--')
    %     a = gca;
    %     plot([idx, idx], [a.YLim], 'g')
    legend('COM', 'SUP', 'SWING','Thetaref_l','Thetaref_r','Offset Phiref_l',' Offset Phiref_r')
    
end

if plot_ref == 1
    figure('units','normalized','outerposition',[-1 0 1 1],'name','ref')
    subplot(3,1,1)
    plot(t, bodyx,'.')
    hold on
    grid on
    plot(t, lfootx,'.')
    plot(t, rfootx,'.')
    legend('body','lfoot','rfoot')
    title('x axis')
    
    subplot(3,1,2)
    plot(t, bodyy,'.')
    hold on
    grid on
    plot(t, lfooty,'.')
    plot(t, rfooty,'.')
    plot(t, prefy)
    legend('body','lfoot','rfoot', 'reference zmp')
    title('y axis')
    
    subplot(3,1,3)
    plot(t, lfootz,'.')
    hold
    grid on
    grid minor
    plot(t, rfootz,'.')
%     plot(t, swingz_fk)
%     plot(t, supz_fk)
%     a = gca;
%     plot([idx1, idx1], [a.YLim], 'g--')
%     plottools
    legend('lfoot','rfoot')
    title('z axis')
end

if plot_body_angle == 1
    
    figure('units','normalized','outerposition',[-1 0 1 1],'name','BODY')
    subplot(3,1,1)
    plot(t, phi,'.')
    hold on
    grid on
    grid minor
    %     plot(t, body_phi,'.','linewidth',2)
    a = gca;
    plot([idx, idx], [a.YLim], 'g')
    legend('IMU Phi')
    
    subplot(3,1,2)
    plot(t, theta,'.')
    hold on
    grid on
    grid minor
    plot(t, body_theta,'.','linewidth',2)
    %     plot(t, body_theta,'.')
    a = gca;
    plot([idx, idx], [a.YLim], 'g')
    legend('IMU Theta', 'Reference theta')
    
    subplot(3,1,3)
    plot(t, psi,'.')
    hold on
    grid on
    grid minor
    plot(t, com_psi)
    a = gca;
    plot([idx, idx], [a.YLim], 'g')
    legend('IMU Psi')
    
end
% a  = length(comy_ref);
% plot(1:a, comy_ref(1:a))
% hold on
% plot(1:a, ZMP_y)
% ylim([0 0.3])