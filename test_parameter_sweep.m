function tests = test_parameter_sweep()
%TEST_PARAMETER_SWEEP Verify sweep grids, coupling, errors, and exported data.
tests = functiontests(localfunctions);
end

function testCartesianGridAndGeometry(t)
c = transient_skidpad_config(); c.maxTime = 0.05;
spec(1) = struct('parameter',"vehicle.m",'values',[300 350]);
spec(2) = struct('parameter',"vehicle.rwb",'values',[0.5 0.6]);
s = sweep_transient_skidpad(spec,c,MakePlots=false,StoreTrajectories=true);
verifyEqual(t,height(s.results),4);
verifyEqual(t,s.results.vehicle_m,[300;350;300;350]);
verifyEqual(t,s.results.vehicle_rwb,[0.5;0.5;0.6;0.6]);
verifyTrue(t,all(s.results.UsesNominalGGV));
verifyEqual(t,numel(unique(s.results.PlannedTime_s)),1);
for j = 1:4
    p = s.trajectories{j}.config.vehicle;
    verifyEqual(t,p.a+p.b,p.wb,'AbsTol',1e-12);
    verifyEqual(t,p.Iz,p.m*p.a*p.b,'AbsTol',1e-10);
end
end

function testErrorsDoNotStopSweep(t)
c = transient_skidpad_config(); c.maxTime = 0.05;
s = sweep_transient_skidpad(struct('parameter',"vehicle.Iz",'values',[-1 150]),c,MakePlots=false);
verifyEqual(t,s.results.Status(1),"error");
verifyGreaterThan(t,strlength(s.results.Error(1)),0);
verifyNotEqual(t,s.results.Status(2),"error");
verifyTrue(t,all(isnan(s.results.LapTime_s)));
end

function testIndexedPathAndExports(t)
c = transient_skidpad_config(); c.maxTime = 0.05;
directory = string(tempname); mkdir(directory);
cleanup = onCleanup(@() rmdir(directory,'s'));
s = sweep_transient_skidpad(struct('parameter',"speedSchedule(2,2)",'values',[0.8 1]), ...
    c,MakePlots=false,StoreTrajectories=true,OutputDirectory=directory);
verifyEqual(t,s.trajectories{1}.config.speedSchedule(2,2),0.8);
verifyTrue(t,isfile(fullfile(directory,'sweep_results.csv')));
a = load(fullfile(directory,'sweep_results.mat'),'study');
verifyEqual(t,a.study.parameters,s.parameters);
end

function testPressureCapacityRebuild(t)
c = transient_skidpad_config(); c.maxTime = 0.05;
s = sweep_transient_skidpad(struct('parameter',"vehicle.tire.P",'values',65),c,MakePlots=false);
verifyNotEqual(t,s.results.Status,"error");
verifyTrue(t,s.results.UsesNominalGGV);
end

function testBadPathsAndRunLimit(t)
verifyError(t,@() sweep_transient_skidpad(struct('parameter',"vehicle.missing",'values',1), ...
    MakePlots=false),'sweep_transient_skidpad:Path');
verifyError(t,@() sweep_transient_skidpad(struct('parameter',"vehicle.Iz",'values',1:3), ...
    MaxRuns=2,MakePlots=false),'sweep_transient_skidpad:Size');
p = list_sweep_parameters();
verifyTrue(t,any(p.Parameter == "vehicle.Iz"));
verifyTrue(t,any(p.Parameter == "speedSchedule(2,2)"));
end
