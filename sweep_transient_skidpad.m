function study = sweep_transient_skidpad(spec,base,options)
%SWEEP_TRANSIENT_SKIDPAD Cartesian sweeps of any numeric configuration entry.
% spec = struct('parameter',"vehicle.Iz",'values',100:25:300);
% Multiple spec entries sweep every combination. Matrix entries use (row,col).
arguments
    spec (1,:) struct
    base (1,1) struct = transient_skidpad_config()
    options.OutputDirectory (1,1) string = ""
    options.MakePlots (1,1) logical = true
    options.StoreTrajectories (1,1) logical = false
    options.UpdateEstimatedInertia (1,1) logical = true
    options.MaxRuns (1,1) double {mustBePositive,mustBeInteger} = 1000
end
paths = strings(1,numel(spec)); counts = zeros(size(paths));
for j = 1:numel(spec)
    paths(j) = string(spec(j).parameter);
    getValue(base,paths(j)); % Validate paths before running anything.
    validateattributes(spec(j).values,{'numeric'},{'vector','nonempty','real','finite'});
    counts(j) = numel(spec(j).values);
end
if isempty(spec) || numel(unique(paths)) ~= numel(paths)
    error('sweep_transient_skidpad:Parameters','Supply unique parameter paths.');
end
if any(ismember(paths,["vehicle.wb" "vehicle.rwb"])) && any(ismember(paths,["vehicle.a" "vehicle.b"]))
    error('sweep_transient_skidpad:Geometry','Sweep wb/rwb or a/b, not both geometry representations together.');
end
n = prod(counts);
if n > options.MaxRuns
    error('sweep_transient_skidpad:Size','%g combinations exceed MaxRuns=%g.',n,options.MaxRuns);
end
values = zeros(n,numel(spec));
status = strings(n,1); message = strings(n,1); completed = false(n,1);
metrics = nan(n,10); fits = false(n,1); nominal = false(n,1);
trajectories = cell(n,1);
for run = 1:n
    c = base; remainder = run-1;
    for j = 1:numel(spec)
        index = mod(remainder,counts(j))+1;
        remainder = floor(remainder/counts(j));
        values(run,j) = spec(j).values(index);
        c = setValue(c,paths(j),values(run,j));
    end
    c = consistentGeometry(c,paths);
    if any(paths == "vehicle.Iz")
        c.yawInertiaSource = "Swept yaw inertia";
    elseif options.UpdateEstimatedInertia && contains(string(c.yawInertiaSource),"Estimate",'IgnoreCase',true)
        c.vehicle.Iz = c.vehicle.m*c.vehicle.a*c.vehicle.b;
    end
    c.makePlots = false; c.allowNominalGGV = true;
    fprintf('Sweep %d/%d:',run,n);
    for j = 1:numel(paths), fprintf(' %s=%g',paths(j),values(run,j)); end
    fprintf('\n');
    runTimer = tic;
    try
        o = transient_skidpad(c);
        metrics(run,:) = [o.lapTime o.plan.lapTime max(abs(o.e_y)) ...
            max(abs(rad2deg(o.beta))) max(abs(o.rDot)) ...
            max(o.tireUtilization) min(o.Fz,[],'all') max(o.Fz,[],'all') c.vehicle.Iz toc(runTimer)];
        status(run) = o.status; completed(run) = o.completed;
        fits(run) = any(o.tireFitRangeExceeded);
        nominal(run) = ~o.referenceGGVMatchesPlant;
        if options.StoreTrajectories, trajectories{run} = o; end
    catch err
        status(run) = "error"; message(run) = string(err.identifier)+": "+string(err.message);
        metrics(run,end) = toc(runTimer);
    end
end
names = matlab.lang.makeUniqueStrings(matlab.lang.makeValidName(cellstr(paths)));
results = array2table(values,'VariableNames',names);
results = [results array2table(metrics,'VariableNames', ...
    {'LapTime_s','PlannedTime_s','MaxPathError_m','MaxSideslip_deg', ...
    'MaxYawAcceleration_rad_s2','MaxTireUtilization','MinWheelLoad_N','MaxWheelLoad_N','YawInertia_kg_m2','Runtime_s'})];
results.Completed = completed; results.Status = status; results.Error = message;
results.TireFitRangeExceeded = fits; results.UsesNominalGGV = nominal;
study.parameters = paths; study.parameterColumns = string(names);
study.specification = spec; study.baseConfig = base; study.results = results;
study.referencePolicy = "Fixed package GGV; changed dynamic plant and regenerated tire capacities when needed";
study.trajectories = trajectories;
fig = [];
if options.MakePlots, fig = plotSweep(study,values); end
if strlength(options.OutputDirectory) > 0
    directory = char(options.OutputDirectory);
    if ~isfolder(directory), mkdir(directory); end
    writetable(results,fullfile(directory,'sweep_results.csv'));
    save(fullfile(directory,'sweep_results.mat'),'study');
    if ~isempty(fig), exportgraphics(fig,fullfile(directory,'sweep_summary.png'),'Resolution',160); end
end
end

function [fields,index] = parsePath(path)
pieces = strsplit(char(path),'(');
if numel(pieces) > 2 || isempty(regexp(pieces{1}, ...
        '^[A-Za-z][A-Za-z0-9_]*(\.[A-Za-z][A-Za-z0-9_]*)*$','once'))
    error('sweep_transient_skidpad:Path','Use field.subfield or field(row,col), without expressions.');
end
fields = strsplit(pieces{1},'.'); index = [];
if numel(pieces) == 2
    suffix = pieces{2};
    if isempty(suffix) || suffix(end) ~= ')'
        error('sweep_transient_skidpad:Path','Close the parameter index with a parenthesis.');
    end
    digits = suffix(1:end-1);
    if isempty(regexp(digits,'^[0-9]+(,[0-9]+)?$','once'))
        error('sweep_transient_skidpad:Index','Use one or two positive integer indices.');
    end
    index = str2double(strsplit(digits,','));
    if any(index < 1), error('sweep_transient_skidpad:Index','Indices must be positive.'); end
end
end

function value = getValue(c,path)
[fields,index] = parsePath(path);
value = c;
for j = 1:numel(fields)
    if ~isstruct(value) || ~isfield(value,fields{j})
        error('sweep_transient_skidpad:Path','Unknown parameter %s.',path);
    end
    value = value.(fields{j});
end
if ~isempty(index)
    if isscalar(index)
        if index > numel(value), error('sweep_transient_skidpad:Index','Index outside parameter array.'); end
        value = value(index);
    else
        if index(1) > size(value,1) || index(2) > size(value,2)
            error('sweep_transient_skidpad:Index','Index outside parameter array.');
        end
        value = value(index(1),index(2));
    end
end
validateattributes(value,{'numeric'},{'scalar','real','finite'});
end

function c = setValue(c,path,value)
[fields,index] = parsePath(path);
c = assignField(c,fields,index,value);
end

function c = assignField(c,fields,index,value)
name = fields{1};
if numel(fields) > 1
    c.(name) = assignField(c.(name),fields(2:end),index,value);
elseif isempty(index)
    c.(name) = value;
elseif isscalar(index)
    c.(name)(index) = value;
else
    c.(name)(index(1),index(2)) = value;
end
end

function c = consistentGeometry(c,paths)
if any(ismember(paths,["vehicle.wb" "vehicle.rwb"]))
    c.vehicle.a = c.vehicle.rwb*c.vehicle.wb;
    c.vehicle.b = (1-c.vehicle.rwb)*c.vehicle.wb;
elseif any(ismember(paths,["vehicle.a" "vehicle.b"]))
    c.vehicle.wb = c.vehicle.a+c.vehicle.b;
    c.vehicle.rwb = c.vehicle.a/c.vehicle.wb;
end
end

function fig = plotSweep(study,values)
r = study.results;
fig = figure('Name','Transient skidpad parameter sweep','Color','w'); tiledlayout(2,1);
if size(values,2) == 1
    [x,order] = sort(values(:,1));
    nexttile; plot(x,r.LapTime_s(order),'-o'); ylabel('Completed lap time [s]'); grid on;
    xlabel(study.parameters(1),'Interpreter','none');
    nexttile; plot(x,r.MaxPathError_m(order),'-o'); ylabel('Maximum path error [m]'); grid on;
    xlabel(study.parameters(1),'Interpreter','none');
elseif size(values,2) == 2
    nexttile; scatter(values(:,1),values(:,2),60,r.LapTime_s,'filled'); colorbar;
    xlabel(study.parameters(1),'Interpreter','none'); ylabel(study.parameters(2),'Interpreter','none');
    title('Completed lap time [s]; additional parameters in CSV'); grid on;
    nexttile; plot(r.MaxPathError_m,'o'); xlabel('Combination number'); ylabel('Maximum path error [m]'); grid on;
else
    nexttile; plot(r.LapTime_s,'o'); xlabel('Combination number'); ylabel('Completed lap time [s]'); grid on;
    nexttile; plot(r.MaxPathError_m,'o'); xlabel('Combination number'); ylabel('Maximum path error [m]'); grid on;
end
set(findall(fig,'Type','axes'),'Color','w','XColor',[0.1 0.1 0.1], ...
    'YColor',[0.1 0.1 0.1],'GridColor',[0.7 0.7 0.7]);
set(findall(fig,'Type','text'),'Color',[0.1 0.1 0.1]);
sgtitle(sprintf('%d/%d completed; failed lap times omitted',nnz(r.Completed),height(r)), ...
    'Color',[0.1 0.1 0.1]);
end
