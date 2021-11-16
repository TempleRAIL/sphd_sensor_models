clc
clear 
close all

%% range sigma: 0.05 ~ 0.5 m, bearing sigma: 1 degree ~ 10 degree:
% true number:
sum_tr = load("../python/sphd_observation_model/tr_cnt.txt");
% false negative number:
sum_fn = load("../python/sphd_observation_model/fn_cnt.txt");
% cluster number:
sum_cl = load("../python/sphd_observation_model/cl_cnt.txt");
% sum square error:
square_error = load("../python/sphd_observation_model/square_error.txt");

%% calculate the fraction of true detection measurements and the sum square error:
N_R = length(sum_tr(:,1));
N_B = length(sum_tr(1,:));
frac_det = zeros(N_R,N_B);
sse = zeros(N_R,N_B);
for r = 1:N_R
    for b = 1:N_B
        frac_det(r,b) = sum_tr(r,b)/(sum_tr(r,b) + sum_fn(r,b)); %+ sum_cl(i));
        sse(r,b) = square_error(r,b)/sum_tr(r,b);
    end
end

%% figure:
r = linspace(1,N_R,N_R)*0.025;
b = linspace(1,N_B,N_B)*0.5;
figure;
subplot(2,1,1);
c_frac(:,:,1) = ones(20); % red
c_frac(:,:,2) = ones(20); % green
c_frac(:,:,3) = zeros(20);% blue
surf(r,b,frac_det, 'FaceAlpha','0.5');
hold on;
grid on;
xlabel('\sigma\_r [m]','FontSize',10);
ylabel('\sigma\_b [deg]','FontSize',10);
zlabel('Fraction of Detections','FontSize',10);

subplot(2,1,2);
c_sse(:,:,1) = zeros(20); % red
c_sse(:,:,2) = zeros(20); % green
c_sse(:,:,3) = ones(20);% blue
surf(r,b,sse, 'FaceAlpha','0.5');
hold on;
% grid on;
xlabel('\sigma\_r [m]','FontSize',10);
ylabel('\sigma\_b [deg]','FontSize',10);
zlabel('Detection Model SSE','FontSize',10);
print('../../results/observation_model_3d','-dpng');

% contour:
figure;
[X1, X2] = meshgrid(r, b);
contour(X1, X2, frac_det, 10, '--','ShowText','on'); 
hold on;
% grid on;
contour(X1, X2, sse, 10, 'ShowText','on'); 
xlabel('\sigma\_r [m]','FontSize',10);
ylabel('\sigma\_b [deg]','FontSize',10);
legend('Frac', 'SSE')

print('../../results/observation_model_contour_paper','-dpng');

