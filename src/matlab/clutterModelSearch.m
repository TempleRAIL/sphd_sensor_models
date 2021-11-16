clc
clear 
close all

%% ground truth:
BIN_NUM_B = 8;
BIN_NUM_R = 6;
% false negative items:
total_fp = load("../python/sphd_clutter_model/bin_fp.txt");
sum_fp = sum(total_fp,'all')
% true items:
total_tr = load("../python/sphd_clutter_model/bin_true.txt");
sum_tr = sum(total_tr,'all')
% clutter items:
total_cl = load("../python/sphd_clutter_model/clutter_cnt.txt");
sum_cl = sum(total_cl,'all')

% clutter probability:
N_B = length(total_fp(:,1));
N_R = length(total_fp(1,:));
p_c = zeros(N_B,N_R);
for i=1:N_B
    for j=1:N_R
        if(total_tr(i,j) + total_fp(i,j) == 0)
            p_c(i,j) = 0;
        else
            p_c(i,j) = total_fp(i,j)/(total_tr(i,j) + total_fp(i,j));
        end
    end
end

%% experiment data:
% range:
b_max = pi/4;
b_min = -pi/4;
r_max = 1.5;
r_min = 0.6;
% clutter cardinality parameter:
mu_opt = 0.28; 

% fitting:
c_z = zeros(N_B+1,N_R+1);
p_c_opt = zeros(N_B+1,N_R+1);
for i = 1:N_B+1
    for j = 1:N_R+1
        c_z(i,j) = mu_opt/(0.5*(b_max-b_min)*(r_max^2-r_min^2));
        p_c_opt(i,j) = c_z(i,j)/mu_opt;
    end
end

%% figure: 
% Define some angular and radial range vectors for example plots
t1 = 2*pi;
t2 = [b_min b_max];
r1 = 4;
r2 = [r_min r_max];%[.8 4];
t3 = fliplr(t2);
r3 = fliplr(r2);
t4 = [30 35 45 60 90 135 200 270]*pi/180;
r4 = [0.8:0.4:2.8 3:0.2:4];
% Axis property cell array
axprop = {'DataAspectRatio',[1 1 8],'View', [-12 38],...
          'Xlim', [-4.5 4.5],       'Ylim', [-4.5 4.5],...
          'XTick',[-4 -2 0 2 4],    'YTick',[-4 -2 0 2 4]};

figure('color','white');
polarplot3d(p_c,'plottype','surfcn','angularrange',t2,'radialrange',r2);
hold on;
polarplot3d(p_c_opt,'plottype','surfcn','angularrange',t2,'radialrange',r2);
% set(gca,axprop{:});
legend('Fit','Data', 'Location','northeast')
xlabel('Range [m]','FontSize',10);
ylabel('Bearing [rad]','FontSize',10);
zlabel('Probability of clutter','FontSize',10);

print('../../results/clutter_model_paper','-dpng');

