clc; clear;

% 基本参数
m = 1300; g = 9.79; rho = 1.18; Cd = 0.3; A = 2; eta = 0.8;
E_battery = 100; N = 8; D = 2; R = D/2;
noise_limit = 55;  % 可自行修改为任意目标噪声约束
v_climb = 5;       % 爬升速度

% 读取数据
T = readtable('附件3 巡航速度高度旋翼转速噪声统计数据.xlsx');
T.Properties.VariableNames = {'xhsd', 'xhgd', 'xyzs', 'dmzs'};

% 1. 训练噪声回归模型（这里用高斯过程回归GPR）
X = [T.xhsd, T.xhgd, T.xyzs];
Y = T.dmzs;
rng(1);
gpr_model = fitrgp(X, Y, ... 
    'Basis','constant', ...
    'KernelFunction','squaredexponential', ...
    'Standardize',true);

% 检查GPR模型对原始低噪声点拟合情况
idx_low = T.dmzs <= 45;
if sum(idx_low) > 0
    Y_pred_low = predict(gpr_model, X(idx_low,:));
    disp('原始低噪声点及其GPR预测值：');
    disp(table(T.xhsd(idx_low), T.xhgd(idx_low), T.xyzs(idx_low), T.dmzs(idx_low), Y_pred_low));
    fprintf('GPR模型对原始低噪声点的最小预测值: %.2f dB(A)\n', min(Y_pred_low));
end

% 2. 在参数空间生成密集网格
speed_list  = linspace(min(T.xhsd), max(T.xhsd), 80);    % 更密集更连续
height_list = linspace(min(T.xhgd), max(T.xhgd), 80);
rpm_list    = linspace(min(T.xyzs), max(T.xyzs), 80);

% 3. 仅在原始数据凸包内搜索
xyz = [T.xhsd, T.xhgd, T.xyzs];
DT = delaunayn(xyz);

best_S = -inf;
best_params = [];
hc = [];
cs = [];
zs = [];

for vi = 1:length(speed_list)
    for hi = 1:length(height_list)
        for ri = 1:length(rpm_list)
            xhsd = speed_list(vi);
            xhgd = height_list(hi);
            xyzs = rpm_list(ri);
            X_pred = [xhsd, xhgd, xyzs];
            if isnan(tsearchn(xyz, DT, X_pred))
                continue;
            end
            % GPR模型预测噪声
            pred_noise = predict(gpr_model, X_pred);
            if pred_noise > noise_limit
                continue;
            end
            % 能量与航程计算
            v_mps = xhsd / 3.6;
            v_ih = sqrt(m * g / (2 * rho * N * pi * R^2));
            P_hover = (m*g)^1.5 / (sqrt(2*rho*N*pi*R^2) * eta) / 1000; % kW
            t_climb = xhgd / v_climb / 3600; % h
            E_up = P_hover * t_climb;
            E_down = 0.4 * E_up;
            v_if = -0.5 * v_mps + sqrt((0.5 * v_mps)^2 + v_ih^2);
            P_induced = m * g * v_if / eta / 1000; % kW
            P_drag = 0.5 * rho * Cd * A * v_mps^3 / eta / 1000; % kW
            P_cruise = P_induced + P_drag;
            E_available = E_battery - E_up - E_down;
            if E_available > 0
                t_cruise = E_available / P_cruise; % h
                S = xhsd * t_cruise; % km
                hc(end+1) = S;
                cs(end+1,:) = [xhsd, xhgd, xyzs];
                zs(end+1) = pred_noise;
                if S > best_S
                    best_S = S;
                    best_params = [xhsd, xhgd, xyzs, pred_noise];
                end
            end
        end
    end
end

if isempty(best_params)
    disp('没有满足噪声约束和能量约束的点。');
else
    fprintf('最大航程: %.1f km\n', best_S);
    fprintf('最优巡航高度: %.1f m\n', best_params(2));
    fprintf('最优巡航速度: %.1f km/h\n', best_params(1));
    fprintf('最优旋翼转速: %.0f RPM\n', best_params(3));
    fprintf('对应地面噪声: %.1f dB(A)\n', best_params(4));
end

% 可视化所有可行点
if ~isempty(cs)
    figure;
    scatter(zs, hc, 40, cs(:,1), 'filled');
    colorbar; xlabel('地面噪声 dB(A)'); ylabel('航程 (km)');
    title('全参数空间可行解的航程-噪声分布（颜色为速度）');
    grid on;
end