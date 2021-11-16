clc
clear 
close all

%% ground truth:
BIN_NUM_B = 6;
BIN_NUM_R = 4;
bin_size = (pi/4 - (-pi/4))/BIN_NUM_B;
bin_size_r = (1.5-0.6)/BIN_NUM_R;
% clutter items:
total_cl = load("../python/sphd_clutter_model/clutter_cnt.txt");
sum_cl = sum(total_cl)

% frequency table:
frq_tab = tabulate(total_cl);

% clutter probability per measurement:
cl_cnt = frq_tab(:,1);
p_cl = frq_tab(:,3)/100;

%% experiment data:
% clutter cardinality parameter search:
mu = 0:0.005:4;
square_error = zeros(1,length(mu));
for i = 1:length(mu)
  p_cl_t = poisspdf(cl_cnt,mu(i));
  for k = 1:length(cl_cnt) 
      square_error(i) = square_error(i) + (p_cl_t(k) - p_cl(k))^2;
  end
end

% get the minimum element value and its index of a matrix:
[min_val, min_idx] = min(square_error);

% optimal parameters:
mu_opt = mu(min_idx)
p_cl_opt = poisspdf(cl_cnt, mu_opt);

%% figure:
figure;
stem(cl_cnt+0.1, p_cl, 'kx', 'LineWidth', 2);

hold on;
% grid on;
stem(cl_cnt-0.1, p_cl_opt, 'ko', 'LineWidth', 2,'MarkerFaceColor','k');
xlabel('# of false positives','FontSize',10);
ylabel('Probability of clutter','FontSize',10);
legend('Data', 'Fit')
axis([0-0.2 5 0 1])
set(gca,'YTick',0-0.2:1:5);
set(gca,'YTick',0:0.2:1);

print('../../results/clutter_cardinality_model_paper','-dpng');

