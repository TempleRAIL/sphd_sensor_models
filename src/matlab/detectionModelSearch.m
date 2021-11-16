%clc
%clear 
close all

%% ground truth:
% false negative items:
total_fn = load("../python/sphd_detection_model/bin_fn.txt");
sum_fn = sum(total_fn)
% true items:
total_tr = load("../python/sphd_detection_model/bin_true.txt");
sum_tr = sum(total_tr)

% detection probability:
N = length(total_tr);
p_d = zeros(1,N);
for i=1:N
    p_d(i) = total_tr(i)/(total_tr(i) + total_fn(i));
end

%% experiment data:
range = 0.3 + 0.2*(1:N);
theta_sep = (90*pi/180)/1280;
% detection model parameters search:
p_dp = (0:0.001:0.1); 
p_db = (0:0.01:1); 
d_t = 0.0667;
d_m = 0.5;
d_o = 1.0; 
square_error = zeros(length(p_db),length(p_dp));
for k = 1:length(p_db)
    for l = 1:length(p_dp)
      for r = 1:length(range)
          p_dt = max(min(p_db(k)*(range(r)- d_m)/d_o, p_dp(l)*d_t/(range(r)*theta_sep)), 0);
          square_error(k,l) = square_error(k,l) + (p_dt - p_d(r))^2;
      end
     end
end

% get the minimum element value and its index of a matrix:
[min_val, min_idx] = min(square_error(:));
[k,l]=ind2sub(size(square_error),min_idx);
% optimal parameters:
p_db_opt = p_db(k)
p_dp_opt = p_dp(l)
p_dt_opt = zeros(1,N);
for i = 1:N
    p_dt_opt(i) = max(min(p_db_opt*(range(i)- d_m)/d_o, p_dp_opt*d_t/(range(i)*theta_sep)), 0);
end

%% figure:
figure;
plot(range, p_d, 'kx', 'LineWidth', 2);
hold on;
% grid on;
plot(range, p_dt_opt, 'k-', 'LineWidth', 2);
xlabel('Range [m]','FontSize',10);
ylabel('Probability of detection','FontSize',10);
legend('Data', 'Fit')
axis([0 3 0 1])
% set(gca,'XTick',0:0.2:3);
set(gca,'YTick',0:0.2:1);

print('../../results/detection_model_paper','-dpng');

