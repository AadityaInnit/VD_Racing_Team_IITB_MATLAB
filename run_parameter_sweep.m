% Edit parameter and values below, then run this script.
sweep_dir = fileparts(mfilename('fullpath'));
addpath(sweep_dir);
base = transient_skidpad_config();
spec = struct('parameter',"vehicle.m",'values',100:50:500);
% Any numeric configuration setting, e.g. vehicle.m, vehicle.ClA,
% vehicle.rwb, vehicle.tire.P, radius, speedScale, steerTimeConstant,
% powerLimit, courseGain, or a matrix entry such as speedSchedule(2,2).
% For a Cartesian sweep, append another specification:
% spec(2) = struct('parameter',"steerTimeConstant",'values',[0.03 0.05 0.1]);
sweep_study = sweep_transient_skidpad(spec,base, ...
    OutputDirectory=fullfile(sweep_dir,'sweep_output'),MakePlots=true);
disp(sweep_study.results);
