%% C172 Simulation Setup and Doublet Test
clear; clc; close all;

% 1. Load the Master Model Struct
load('C172MasterModel.mat');

% 2. Define Simulation Time Parameters
startTime = 0;
stopTime = 60;    % 60 seconds gives enough time for 3 sequential maneuvers
stepTime = 0.01;
t = (startTime:stepTime:stopTime)'; % Time vector

% 3. Define Initial Conditions
% Note: Flat Earth implies a North-East-Down (NED) coordinate system.
% Therefore, an altitude of 5000 ft is a Z-position of -5000.
Pos0 = [0; 0; -5000];     % [North(ft); East(ft); Down(ft)]
Vel0 = [168; 0; 0];       % [u(ft/s); v(ft/s); w(ft/s)] (~100 knots forward)
Att0 = [0; 0; 0];         % [Roll(rad); Pitch(rad); Yaw(rad)]
Ome0 = [0; 0; 0];         % [p(rad/s); q(rad/s); r(rad/s)]

% 4. Define Control Inputs (Timeseries)
% Baseline trim values
RPM_val = 2300;         % Estimated engine RPM
df_val     = 0;           % Flaps up
da_val     = 0;
de_val     = 0;
dr_val     = -0.05;

% Initialize input arrays
RPM_array = RPM_val * ones(length(t), 1);
df_deg_array = df_val * ones(length(t), 1);
da_rad_array = da_val * ones(length(t), 1);
de_rad_array = de_val * ones(length(t), 1);
dr_rad_array = dr_val * ones(length(t), 1);

% -- Maneuver 1: Elevator Doublet (Pitch test) --
% 5 seconds to 7 seconds: Pitch Up, then Pitch Down
de_rad_array(t >= 5 & t < 6) = de_val - 0.05; % Negative elevator usually pitches UP
de_rad_array(t >= 6 & t < 7) = de_val + 0.05;


RPM_array(t >= 10 & t < 15) = 800;


% -- Maneuver 2: Aileron Doublet (Roll test) --
% 20 seconds to 22 seconds: Roll Right, then Roll Left
% da_rad_array(t >= 59.99) =  0.08; 
da_rad_array(t >= 20 & t < 21) = da_val + 0.08; 
da_rad_array(t >= 21 & t < 22) = da_val - 0.08;

% -- Maneuver 3: Rudder Doublet (Yaw test) --
% 40 seconds to 42 seconds: Yaw Right, then Yaw Left
dr_rad_array(t >= 40 & t < 41) = dr_val - 0.08; 
dr_rad_array(t >= 41 & t < 42) = dr_val + 0.08;

% Convert arrays to Simulink timeseries objects
RPM = timeseries(RPM_array, t);
df_deg = timeseries(df_deg_array, t);
da_rad = timeseries(da_rad_array, t);
de_rad = timeseries(de_rad_array, t);
dr_rad = timeseries(dr_rad_array, t);

% 5. Run the Simulation
disp('Running Simulation...');

% The correct parameter is 'ExternalInput', mapped to our 5 timeseries variables
out = sim('C172_Plant.slx', 'ExternalInput', 'RPM, df_deg, da_rad, de_rad, dr_rad');

disp('Simulation Complete.');

% % 6. Extract Output Data
% --- Extract Simulation Outputs ---
Xe_data  = out.yout.signals(2).values; % Position [North, East, Down]
Ve_data  = out.yout.signals(1).values;
Vb_data  = out.yout.signals(4).values; % Body Velocity [u, v, w]
Att_data = out.yout.signals(3).values; % Attitude [Roll, Pitch, Yaw]
time     = out.tout;                   % Simulation time vector

% --- Calculate Aerodynamic States from Body Velocities ---
u = Vb_data(:, 1);
v = Vb_data(:, 2);
w = Vb_data(:, 3);

V_inf = sqrt(u.^2 + v.^2 + w.^2);               % True Airspeed (ft/s)
alpha_deg = atan2(w, u) * (180/pi);             % Angle of Attack (degrees)

% Protect against division by zero for Sideslip at t=0
beta_deg = zeros(size(V_inf));
idx = V_inf > 0;
beta_deg(idx) = asin(v(idx) ./ V_inf(idx)) * (180/pi); % Sideslip (degrees)

% --- Extract Control Inputs from Timeseries ---
% Interpolate the timeseries data to match the simulation output time steps
de_data = interp1(de_rad.Time, de_rad.Data, time, 'linear', 'extrap');
da_data = interp1(da_rad.Time, da_rad.Data, time, 'linear', 'extrap');
dr_data = interp1(dr_rad.Time, dr_rad.Data, time, 'linear', 'extrap');

% --- Animation Setup ---
playback_speed = 1; 
dt = time(2) - time(1);
pause_time = dt / playback_speed;

fig = figure('Name', 'C172 6DOF Chase Cam with HUD', 'Color', 'white', 'Position', [100, 100, 800, 600]);
ax = axes('Parent', fig);
hold(ax, 'on');
grid(ax, 'on');
view(ax, 3);
axis(ax, 'equal');

% Standard Aerospace Coordinate System: Z is Down
set(ax, 'ZDir', 'reverse');
set(ax, 'YDir', 'reverse'); 

xlabel(ax, 'North (ft)');
ylabel(ax, 'East (ft)');
zlabel(ax, 'Down (ft)');
title(ax, 'Aircraft 3D Kinematics - Chase Cam');

% --- Create the HUD Text Box ---
% [x y width height] in normalized figure coordinates (0 to 1)
hud_box = annotation(fig, 'textbox', [0.02 0.65 0.25 0.3], ...
    'String', 'Initializing...', ...
    'EdgeColor', 'black', 'BackgroundColor', [1 1 1], 'FaceAlpha', 0.8, ...
    'FitBoxToText', 'on', 'FontName', 'Courier', 'FontSize', 10);

% --- Build the 3D Aircraft Geometry ---
craft_tform = hgtransform('Parent', ax);

% Draw a basic Cessna-scale wireframe 
plot3(craft_tform, [5, -15], [0, 0], [0, 0], 'b', 'LineWidth', 3); % Fuselage
plot3(craft_tform, [0, 0], [-18, 18], [0, 0], 'r', 'LineWidth', 3); % Main Wing
plot3(craft_tform, [-14, -14], [-5, 5], [0, 0], 'g', 'LineWidth', 3); % Horizontal Stab
plot3(craft_tform, [-14, -15], [0, 0], [0, -5], 'k', 'LineWidth', 3); % Vertical Stab

% Initialize Trajectory Line
traj_line = plot3(ax, Xe_data(1,1), Xe_data(1,2), Xe_data(1,3), 'k--', 'LineWidth', 1);

% Define the zoom window radius around the aircraft (in feet)
zoom_radius = 40; 

% --- Animation Loop ---
disp('Starting Chase Cam Animation...');

for i = 1:length(time)
    % Extract current state
    N = Xe_data(i, 1);
    E = Xe_data(i, 2);
    D = Xe_data(i, 3);
    
    phi   = Att_data(i, 1); % Roll
    theta = Att_data(i, 2); % Pitch
    psi   = Att_data(i, 3); % Yaw
    
    % Dynamically update the axis limits to center on the aircraft
    xlim(ax, [N - zoom_radius, N + zoom_radius]);
    ylim(ax, [E - zoom_radius, E + zoom_radius]);
    zlim(ax, [D - zoom_radius, D + zoom_radius]);
    
    % Build the Translation and Rotation Matrices
    T = makehgtform('translate', [N, E, D]);
    Rz = makehgtform('zrotate', psi);
    Ry = makehgtform('yrotate', theta);
    Rx = makehgtform('xrotate', phi);
    
    % Apply combined transform matrix to the 3D model
    set(craft_tform, 'Matrix', T * Rz * Ry * Rx);
    
    % Update the trailing trajectory line
    set(traj_line, 'XData', Xe_data(1:i, 1), ...
                   'YData', Xe_data(1:i, 2), ...
                   'ZData', Xe_data(1:i, 3));
               
    % --- Update HUD Text ---
    hud_text = sprintf(['Time: %.2f s\n\n' ...
                        '--- Aero States ---\n' ...
                        'Airspeed: %6.1f ft/s\n' ...
                        'Alpha:    %6.2f deg\n' ...
                        'Beta:     %6.2f deg\n\n' ...
                        '--- Controls ---\n' ...
                        'Elevator: %6.3f rad\n' ...
                        'Aileron:  %6.3f rad\n' ...
                        'Rudder:   %6.3f rad'], ...
                        time(i), V_inf(i), alpha_deg(i), beta_deg(i), ...
                        de_data(i), da_data(i), dr_data(i));
    set(hud_box, 'String', hud_text);
    
    drawnow;
    pause(pause_time);
end

disp('Animation Complete.');