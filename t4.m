clc; clear;
% ========== 1. 站点经纬度与距离矩阵 ==========
station_names = {
    '萧山机场站', '钱塘金沙湖站', '西湖文化广场站', '上城吴山广场站', ...
    '临安人民广场站', '富阳秦望广场站', '桐庐中心广场站', ...
    '建德新安江广场站', '淳安千岛湖广场站'
};
latlon = [
    30.2296, 120.4345;
    30.2742, 120.3016;
    30.2741, 120.1551;
    30.2376, 120.1697;
    30.2302, 119.7247;
    30.0482, 119.9605;
    29.7932, 119.6982;
    29.6086, 119.2819;
    29.6070, 119.0367;
];
R = 6371; % 地球半径km
n = size(latlon, 1);
dist_mat = zeros(n);
for i = 1:n
    for j = 1:n
        if i ~= j
            lat1 = deg2rad(latlon(i,1));
            lon1 = deg2rad(latlon(i,2));
            lat2 = deg2rad(latlon(j,1));
            lon2 = deg2rad(latlon(j,2));
            dlat = lat2 - lat1; dlon = lon2 - lon1;
            a = sin(dlat/2)^2 + cos(lat1)*cos(lat2)*sin(dlon/2)^2;
            c = 2 * atan2(sqrt(a), sqrt(1 - a));
            dist_mat(i,j) = R * c; % km
        end
    end
end

% ========== 2. 参数设置 ==========
max_battery = 100;      % kWh
min_remain = 0.3 * max_battery;  % 剩余电量不得低于30%
charge_rate = 0.1 * max_battery; % 每分钟充10%电量
n_station = n;

% ========== 3. 计算每段飞行的能耗与时间 ==========
energy_mat = zeros(n_station); % kWh
time_mat = zeros(n_station);   % min
speed = 140; % km/h, 用之前最优解
for i = 1:n_station
    for j = 1:n_station
        if i ~= j
            d = dist_mat(i, j);
            [e, t] = calc_energy_time(d, speed); % 用下方自定义函数
            energy_mat(i, j) = e; % kWh
            time_mat(i, j) = t;   % min
        end
    end
end

% ========== 4. 路径搜索（广度优先+能量约束） ==========
start_id = 1; end_id = 9; must_id = 4;
min_stops = 6; % 至少6站（首末+4个中间）
route_result = {};
queue = {struct('cur', start_id, 'E', max_battery, 'path', [start_id], ...
    'time', 0, 'charge', 0, 'sight', [], 'visit', zeros(1,n))};
queue{1}.visit(start_id) = 1;

while ~isempty(queue)
    state = queue{1};
    queue(1) = []; % 出队
    cur = state.cur; E = state.E; path = state.path;
    t = state.time; c = state.charge; sight = state.sight; visit = state.visit;
    % 到达终点且满足约束
    if cur == end_id && numel(path) >= min_stops && ismember(must_id, path)
        route_result{end+1} = state;
        continue;
    end
    % 枚举所有下一站
    for nxt = 1:n_station
        if nxt == cur || visit(nxt)==1, continue; end
        % 观光策略：若不是终点且未观光过，观光2小时消耗30%电量
        if nxt ~= end_id && ~ismember(nxt, sight)
            need_sight = 0.3 * max_battery; time_sight = 120;
            new_sight = [sight, nxt];
        else
            need_sight = 0; time_sight = 0;
            new_sight = sight;
        end
        % 飞行消耗
        need_fly = energy_mat(cur, nxt);
        time_fly = time_mat(cur, nxt);
        total_need = need_fly + need_sight;
        % 判断是否需要充电
        if E < total_need + min_remain
            charge_needed = total_need + min_remain - E;
            charge_time = ceil(charge_needed/charge_rate);
            new_E = E + charge_time*charge_rate - total_need;
            new_c = c + 1;
        else
            charge_time = 0;
            new_E = E - total_need;
            new_c = c;
        end
        if new_E < min_remain, continue; end % 电量不足不合法
        % 更新状态
        new_path = [path, nxt];
        new_time = t + time_fly + time_sight + charge_time;
        new_visit = visit; new_visit(nxt) = 1;
        queue{end+1} = struct('cur', nxt, 'E', new_E, ...
            'path', new_path, 'time', new_time, ...
            'charge', new_c, 'sight', new_sight, 'visit', new_visit);
    end
end

% ========== 5. 选最优路线 ==========
all_times = cellfun(@(x)x.time, route_result);
all_charges = cellfun(@(x)x.charge, route_result);
[~, idx1] = min(all_times);
time_min = all_times(idx1);
charge_min = all_charges(idx1);

idx2 = find(all_charges == min(all_charges), 1, 'first');
route1 = route_result{idx1};
route2 = route_result{idx2};

% ========== 6. 输出 ==========
disp('路线1（总时间最短）:');
disp(station_names(route1.path));
fprintf('总用时: %.1f min, 充电次数: %d\n', route1.time, route1.charge);

disp('路线2（总充电最少）:');
disp(station_names(route2.path));
fprintf('总用时: %.1f min, 充电次数: %d\n', route2.time, route2.charge);

% ========== 7. 更美观可视化 ==========
figure; hold on; box on; set(gcf, 'Color', 'w');

% 配色
col_all = [0.5 0.5 0.5];
col_start = [0 0.7 0];
col_end = [1 0 0];
col_must = [0 0.4 1];
col1 = [0.2 0.6 1];
col2 = [1 0.5 0.2];

% 散点+主标注
scatter(latlon(:,2), latlon(:,1), 80, 'k', 'LineWidth', 1.5, 'MarkerFaceColor', [1 1 1]);
for i = 1:n
    if i == start_id
        scatter(latlon(i,2), latlon(i,1), 150, 's', 'MarkerFaceColor', col_start, 'MarkerEdgeColor', 'k', 'LineWidth',2);
        text(latlon(i,2), latlon(i,1)+0.04, station_names{i}, 'FontSize',12,'FontWeight','bold','HorizontalAlignment','center','Color',col_start);
    elseif i == end_id
        scatter(latlon(i,2), latlon(i,1), 150, 'p', 'MarkerFaceColor', col_end, 'MarkerEdgeColor', 'k', 'LineWidth',2);
        text(latlon(i,2), latlon(i,1)-0.04, station_names{i}, 'FontSize',12,'FontWeight','bold','HorizontalAlignment','center','Color',col_end);
    elseif i == must_id
        scatter(latlon(i,2), latlon(i,1), 150, 'd', 'MarkerFaceColor', col_must, 'MarkerEdgeColor', 'k', 'LineWidth',2);
        text(latlon(i,2)+0.04, latlon(i,1), station_names{i}, 'FontSize',12,'FontWeight','bold','HorizontalAlignment','left','Color',col_must);
    else
        text(latlon(i,2)-0.04, latlon(i,1), station_names{i}, 'FontSize',11,'HorizontalAlignment','right','Color',[0.2 0.2 0.2]);
    end
end

% 路线1
plot(latlon(route1.path,2), latlon(route1.path,1), '-o', 'Color', col1, 'LineWidth',3, 'MarkerSize',10, ...
    'MarkerFaceColor', col1, 'DisplayName','总时间最短');
% 路线2
plot(latlon(route2.path,2), latlon(route2.path,1), '-s', 'Color', col2, 'LineWidth',3, 'MarkerSize',10, ...
    'MarkerFaceColor', col2, 'DisplayName','充电最少');

% 图例
legend({'所有站点','起点','终点','必经点','总时间最短','充电最少'}, 'Location','northeast','FontSize',11,'Box','off');

xlabel('经度','FontSize',13,'FontWeight','bold');
ylabel('纬度','FontSize',13,'FontWeight','bold');
title('杭州市低空航线最优路径规划','FontSize',15,'FontWeight','bold');
set(gca,'FontSize',12,'LineWidth',1.2);

grid on;
axis tight;
hold off;

% ========== 8. 能耗物理建模函数 ==========
function [E, t] = calc_energy_time(d, v_kmh)
% 基于问题1/3的eVTOL能耗模型
m = 1300; g = 9.79; rho = 1.18; Cd = 0.3; A = 2;
N = 8; D = 2; R = D/2; eta = 0.8;
v = v_kmh / 3.6;
v_ih = sqrt(m * g / (2 * rho * N * pi * R^2));
v_if = -0.5 * v + sqrt((0.5*v)^2 + v_ih^2);
P_hover = (m*g)^1.5 / (sqrt(2*rho*N*pi*R^2) * eta) / 1000;
P_induced = m * g * v_if / eta / 1000;
P_drag = 0.5 * rho * Cd * A * v^3 / eta / 1000;
P_cruise = P_induced + P_drag;
t = d / v * 60; % min
E = P_cruise * d / v; % kWh
end