clc; clear;

% -------- 基本参数 --------
n1 = 4;             %乘客+司机人数       
m1 = 65;             %人的质量
m2 = 20;             %行李质量
m3 = 800;             %车的质量
m = n1 * m1 + (n1-1) * m2 + m3;           % 总质量 (kg)
g = 9.79;              % 重力加速度 (m/s^2)
rho = 1.18;            % 空气密度 (kg/m^3)
Cd = 0.3;              % 阻力系数
A = 2;                 % 迎风面积 (m^2)
eta = 0.8;             % 总效率
E_battery = 100;       % 电池能量 (kWh)
N = 8;                 % 旋翼数
D = 2;                 % 单旋翼直径 (m)
R = D/2;               % 单旋翼半径 (m)

% -------- 优化变量的上下界 --------
lb = [200,   120, 1000, 3];    % [h, v, N_rpm, v_climb]的下界
ub = [500,   160, 4000, 6];    % 上界

% -------- 目标函数（最小化负航程） --------
fun = @(x) -range_obj(x, m, g, rho, Cd, A, eta, E_battery, N, R);

% -------- 非线性约束（保证能耗不超过电池能量，且上升/下降速度为正） --------
nonlcon = @(x) range_con(x, m, g, rho, Cd, A, eta, E_battery, N, R);

% -------- fmincon求解 --------
x0 = [300, 140, 2000, 4];       % 初始猜测
options = optimset('Display', 'iter');
[xopt, fval] = fmincon(fun, x0, [], [], [], [], lb, ub, nonlcon, options);

% -------- 输出结果 --------
optimal_h = xopt(1);
optimal_v = xopt(2);
optimal_N_rpm = xopt(3);
optimal_v_climb = xopt(4);
max_S = -fval;

fprintf('最大航程: %.1f km\n', max_S);
fprintf('最优飞行高度: %.1f m\n', optimal_h);
fprintf('最优巡航速度: %.1f km/h\n', optimal_v);
fprintf('最优旋翼转速: %.1f RPM\n', optimal_N_rpm);
fprintf('最优爬升速度: %.2f m/s\n', optimal_v_climb);

% -------- 可视化：最优参数下航程-巡航速度折线 --------
v_cruise_values = 120:2:160;
S_vs_v = zeros(size(v_cruise_values));
for idx = 1:length(v_cruise_values)
    v = v_cruise_values(idx);
    x_now = [optimal_h, v, optimal_N_rpm, optimal_v_climb];
    S_vs_v(idx) = range_obj(x_now, m, g, rho, Cd, A, eta, E_battery, N, R);
end

figure;
plot(v_cruise_values, S_vs_v, '-o','LineWidth',2);
xlabel('巡航速度 (km/h)');
ylabel('最大航程 (km)');
title('最优参数下最大航程随巡航速度变化');
grid on;

% 标注最优点
[~, idx_max] = max(S_vs_v);
hold on;
plot(v_cruise_values(idx_max), S_vs_v(idx_max), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
text(v_cruise_values(idx_max), S_vs_v(idx_max), sprintf('%.1f km', S_vs_v(idx_max)), ...
    'VerticalAlignment', 'bottom', 'Color', 'r', 'FontSize', 12);
hold off;

% -------- 目标函数 --------
function S = range_obj(x, m, g, rho, Cd, A, eta, E_battery, N, R)
    h = x(1); v = x(2); N_rpm = x(3); v_climb = x(4);
    v_mps = v / 3.6;
    omega = (2 * pi * N_rpm) / 60;
    v_ih = sqrt(m * g / (2 * rho * N * pi * R^2));
    P_hover = (m*g)^1.5 / (sqrt(2*rho*N*pi*R^2) * eta) / 1000;
    t_climb = h / v_climb / 3600;
    E_up = P_hover * t_climb;
    E_down = 0.4 * E_up;
    v_if = -0.5 * v_mps + sqrt( (0.5 * v_mps)^2 + v_ih^2 );
    P_induced = m * g * v_if / eta / 1000;
    P_drag = 0.5 * rho * Cd * A * v_mps^3 / eta / 1000;
    P_cruise = P_induced + P_drag;
    E_available = E_battery - E_up - E_down;
    if E_available <= 0
        S = 0;
    else
        t_cruise = E_available / P_cruise;
        S = v * t_cruise;
    end
end

% -------- 非线性约束函数 --------
function [c, ceq] = range_con(x, m, g, rho, Cd, A, eta, E_battery, N, R)
    % 约束1：电池能耗不能超标（其实目标函数内部已保证，但加个约束更正规）
    h = x(1); v = x(2); N_rpm = x(3); v_climb = x(4);
    v_mps = v / 3.6;
    omega = (2 * pi * N_rpm) / 60;
    v_ih = sqrt(m * g / (2 * rho * N * pi * R^2));
    P_hover = (m*g)^1.5 / (sqrt(2*rho*N*pi*R^2) * eta) / 1000;
    t_climb = h / v_climb / 3600;
    E_up = P_hover * t_climb;
    E_down = 0.4 * E_up;
    v_if = -0.5 * v_mps + sqrt( (0.5 * v_mps)^2 + v_ih^2 );
    P_induced = m * g * v_if / eta / 1000;
    P_drag = 0.5 * rho * Cd * A * v_mps^3 / eta / 1000;
    P_cruise = P_induced + P_drag;
    E_available = E_battery - E_up - E_down;
    % 约束式：能耗需为正
    c(1) = -E_available;
    % 约束式：爬升速度需为正
    c(2) = -v_climb;
    ceq = [];
end