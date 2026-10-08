% Independent transient skidpad experiment. Edit settings here.
experiment_dir = fileparts(mfilename('fullpath'));
addpath(experiment_dir);
config = transient_skidpad_config();
% config.vehicle.Iz = ...; % measured yaw inertia [kg*m^2]
% config.startMode = "flying";
% config.speedScale = 1.0; % attempt the full equilibrium GGV speed target
transient_result = transient_skidpad(config);
fprintf('\nTransient skidpad: %s\n',transient_result.status);
fprintf('Yaw inertia: %.4f kg*m^2 (%s)\n',config.vehicle.Iz,config.yawInertiaSource);
fprintf('Planned lap: %.5f s; actual lap: %.5f s\n', ...
    transient_result.plan.lapTime,transient_result.lapTime);
fprintf('Maximum path error: %.4f m\n',max(abs(transient_result.e_y)));
fprintf('Sideslip range: %.3f to %.3f deg\n', ...
    min(rad2deg(transient_result.beta)),max(rad2deg(transient_result.beta)));
fprintf('Tire-fit range exceeded: %d\n',any(transient_result.tireFitRangeExceeded));
save(fullfile(experiment_dir,'transient_skidpad_results.mat'),'transient_result','config');
