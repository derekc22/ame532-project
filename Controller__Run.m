%% C172 Integrated Controller + Advanced Animation Loop
clear; clc; close all;

% 1. Timing & Model Config
startTime = 0;
stopTime  = 30;    
stepTime  = 0.01;
modelName = 'C172_Plant_Controller2';

%% 3. Controller Targets & Gains (FORCED UPDATE)
% Run this in the Command Window (the one with the >>)
% set_param('C172_Plant_Controller1/Slow Flight Controller', 'InputPortDimensions', '{[3], [1], [1], [1], [3], [1], [1], [1], [1], [1], [1], [1], [1], [1], [1]}')
%assignin('base', 'Ve_target', 90);
%assignin('base', 'alt_target', 5000);
%assignin('base', 'df_deg_in', 30);

% Control Gains
%assignin('base', 'kp_alt', 40.0);
%assignin('base', 'ki_alt', 0.2);
%assignin('base', 'kp_v', 0.08);
%assignin('base', 'ki_v', 0.005);
%assignin('base', 'kq', 0.15);
%assignin('base', 'kp_phi', 0.5);
%assignin('base', 'kp_p', 0.1);

% Verify they exist BEFORE running the sim
%fprintf('Verification: kp_alt is now %.2f in Base Workspace.\n', evalin('base', 'kp_alt'));

% 2. Load Parameters & Define Initial Conditions
load('C172MasterModel.mat');

% Initial conditions for the 6-DOF block (NED)
Pos0 = [0; 0; -5000];     % 5000 ft altitude 
Vel0 = [80; 0; 0];        % 90 ft/s forward velocity
Att0 = [0; 0.12; 0];      % Trimmed pitch to avoid sink
Ome0 = [0; 0; 0];         

%% 3. Controller Targets & Gains
Ve_target  = 90;       
alt_target = 5000;  
df_deg_in = 30;

% Gains to prevent "Spinning" and manage Slow Flight
kp_alt = 0.9;   ki_alt = 2;         % Alt -> Thrust
kp_v   = 0.9;   ki_v   = 0.9;       % Speed -> Elevator
kq     = 0.30;                      % Pitch Damper
kp_phi = 0.2;   kp_p   = 0.1;       % Roll Damping (Wing Leveler)

%% 4. Run Simulation
open_system(modelName);

% Force parameter updates to the model
set_param(modelName, 'StartTime', num2str(startTime), 'StopTime', num2str(stopTime));
set_param(modelName, 'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', num2str(stepTime));

disp('Running Controller Simulation...');

% Diagnostic: Print the values currently in the Base Workspace
fprintf('Current kp_alt in Workspace: %.2f\n', evalin('base', 'kp_alt'));
fprintf('Current Flap Setting: %.2f\n', evalin('base', 'df_deg_in'));

out = sim(modelName); 
disp('Simulation Complete.');

%% 5. Data Extraction 
time      = out.tout;
Vb_data   = out.yout.signals(1).values; % Body Velocity [u, v, w]
Ve_data   = out.yout.signals(2).values; % Airspeed (Scalar)
Xe_data   = out.yout.signals(3).values; % Position [North, East, Down]
Att_data  = out.yout.signals(4).values; % Attitude [Roll, Pitch, Yaw]

% Calculated States
alpha_deg = atan2(Vb_data(:,3), Vb_data(:,1)) * (180/pi);
beta_deg  = asin(Vb_data(:,2) ./ max(Vb_data, 0.1)) * (180/pi);

% --- Extract Control Inputs from Outports ---
% Assuming the new Outports are indices 5, 6, and 7
try
    de_data = out.yout.signals(5).values; 
    da_data = out.yout.signals(6).values; 
    dr_data = out.yout.signals(7).values;
    disp('Control surface data successfully extracted.');
catch
    warning('Control Outports not found. Check Simulink wiring.');
    de_data = zeros(size(time)); 
    da_data = zeros(size(time)); 
    dr_data = zeros(size(time));
end

%% 6. Animation Setup
playback_speed = 2; 
pause_time = (time(2) - time(1)) / playback_speed;
fig = figure('Name', 'C172 Chase Cam', 'Color', 'white', 'Position', [100 100 1000 600]);
ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); view(3); axis equal;
set(ax, 'ZDir', 'reverse', 'YDir', 'reverse'); % Aerospace NED
set(ax, 'ZDir', 'reverse');
set(ax, 'YDir', 'reverse'); 

xlabel(ax, 'North (ft)');
ylabel(ax, 'East (ft)');
zlabel(ax, 'Down (ft)');
title(ax, 'Aircraft 3D Kinematics - Chase Cam');

% Improved HUD Box from Doublet Script
hud_box = annotation(fig, 'textbox', [0.02 0.65 0.25 0.3], ...
    'String', 'Initializing HUD...', ...
    'EdgeColor', 'black', 'BackgroundColor', [1 1 1], 'FaceAlpha', 0.8, ...
    'FitBoxToText', 'on', 'FontName', 'Courier', 'FontSize', 10);

% Build Aircraft Transform (HGTransform)
craft_tform = hgtransform('Parent', ax);
plot3(craft_tform, [5, -15], [0, 0], [0, 0], 'b', 'LineWidth', 3); % Fuselage
plot3(craft_tform, [0, 0], [-18, 18], [0, 0], 'r', 'LineWidth', 3); % Main Wing
plot3(craft_tform, [-14, -14], [-5, 5], [0, 0], 'g', 'LineWidth', 3); % Horizontal Stab
plot3(craft_tform, [-14, -15], [0, 0], [0, -5], 'k', 'LineWidth', 3); % Vertical Stab

traj_line = plot3(ax, Xe_data(1,1), Xe_data(1,2), Xe_data(1,3), 'k--', 'LineWidth', 1);
zoom_radius = 60;

% --- Animation Loop ---
disp('Starting Chase Cam Animation...');
for i = 1:1:length(time) % Step by 5 for smooth/fast rendering
    N = Xe_data(i, 1); E = Xe_data(i, 2); D = Xe_data(i, 3);
    phi = Att_data(i, 1); theta = Att_data(i, 2); psi = Att_data(i, 3);
    
    % Dynamic Axis Updates (Chase Cam)
    xlim(ax, [N - zoom_radius, N + zoom_radius]);
    ylim(ax, [E - zoom_radius, E + zoom_radius]);
    zlim(ax, [D - zoom_radius, D + zoom_radius]);
    
    % Apply 6-DOF Rotation and Translation
    set(craft_tform, 'Matrix', makehgtform('translate', [N, E, D]) * ...
                               makehgtform('zrotate', psi) * ...
                               makehgtform('yrotate', theta) * ...
                               makehgtform('xrotate', phi));
    
    set(traj_line, 'XData', Xe_data(1:i, 1), 'YData', Xe_data(1:i, 2), 'ZData', Xe_data(1:i, 3));
               
    % Update HUD Text (Synchronized with Simulation Time)
hud_text = sprintf(['Time: %.2f s\n\n' ...
                    '--- Aero States ---\n' ...
                    'Airspeed: %6.1f ft/s\n' ...
                    'Altitude: %6.1f ft\n' ...
                    'Alpha:    %6.2f deg\n' ...
                    'Beta:     %6.2f deg\n\n' ...
                    '--- Controls ---\n' ...
                    'Elevator: %6.3f rad\n' ...
                    'Aileron:  %6.3f rad\n' ...
                    'Rudder:   %6.3f rad'], ...
                    time(i), Ve_data(i), -Xe_data(i,3), alpha_deg(i), beta_deg(i), ...
                    de_data(i), da_data(i), dr_data(i));
set(hud_box, 'String', hud_text);
    
    drawnow;
end
disp('Animation Complete.');