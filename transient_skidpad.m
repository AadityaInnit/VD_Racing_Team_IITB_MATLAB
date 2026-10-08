function out = transient_skidpad(c)
%TRANSIENT_SKIDPAD Integrate a nonlinear bicycle following a GGV speed plan.
% out = transient_skidpad(transient_skidpad_config())
% All plant, driver, trim, and station-planner functions are local below.
% States: [Vx; Vy; r; s; ey; epsi; delta]. Positive y/yaw/steer is left.
% The Frenet chart evolves path errors; it does not constrain them to zero.
arguments
    c (1,1) struct
end
data = load(fullfile(fileparts(mfilename('fullpath')),'data','ggv_lts_lookup.mat'),'ggvLUT');
LUT = data.ggvLUT;
% Preserve compatibility with configurations saved before sweep support.
if ~isfield(c,'allowNominalGGV'), c.allowNominalGGV = false; end
if ~isfield(c,'powerLimit'), c.powerLimit = LUT.P_max; end
if ~isfield(c,'driveType'), c.driveType = string(LUT.drive_type); end
validateConfig(c);
if ~isfield(LUT,'speed_frame') || ~strcmp(LUT.speed_frame,'path') ...
        || ~strcmp(LUT.acceleration_frame,'path')
    error('transient_skidpad:Frame','The package requires a GGV expressed in CG path speed and acceleration.');
end
referenceMatches = isequaln(rmfield(c.vehicle,'Iz'),LUT.vehicle);
referenceMatchesPlant = referenceMatches && c.powerLimit == LUT.P_max ...
    && strcmpi(c.driveType,LUT.drive_type);
if ~referenceMatchesPlant && ~c.allowNominalGGV
    error('transient_skidpad:Configuration','Regenerate the package GGV, or explicitly enable allowNominalGGV, before changing vehicle or drivetrain parameters.');
end
LUT.P_max = c.powerLimit; LUT.drive_type = char(c.driveType);
q = capacityInterpolants(LUT,c);
plan = speedPlan(c,LUT);
trim = trimMap(c,plan,q,LUT);
if lower(c.startMode) == "flying"
    V0 = plan.speed(1);
    beta0 = interp1(trim.speed,trim.beta,V0,'linear','extrap');
    delta0 = interp1(trim.speed,trim.delta,V0,'linear','extrap');
else
    V0 = min(c.entrySpeed,plan.speed(1));
    beta0 = 0;
    delta0 = trim.delta(1);
end
beta0 = beta0+c.initialSideslipOffset;
x0 = [V0*cos(beta0);V0*sin(beta0);V0/c.radius+c.initialYawRateOffset; ...
    0;c.initialLateralError;-beta0;delta0];
options = odeset('RelTol',c.relTol,'AbsTol',c.absTol,'MaxStep',c.maxStep, ...
    'Events',@(t,x) stopEvents(t,x,c,plan));
[t,x,~,~,events] = ode45(@(t,x) dynamics(t,x,c,plan,trim,q,LUT),[0 c.maxTime],x0,options);
n = numel(t);
D = zeros(n,14); Fz = zeros(n,4); slips = zeros(n,2);
for j = 1:n
    [dx,d] = dynamics(t(j),x(j,:).',c,plan,trim,q,LUT);
    V = hypot(x(j,1),x(j,2));
    betaDot = 0;
    if V > 1e-6
        betaDot = (x(j,1)*dx(2)-x(j,2)*dx(1))/V^2;
    end
    actualCurvature = NaN;
    if V > 0.25, actualCurvature = (x(j,3)+betaDot)/V; end
    D(j,:) = [dx(1:3).' betaDot dx(4) actualCurvature d.FX d.FY d.Mz ...
        d.targetSpeed d.steerCommand d.forceCommand d.utilization d.loadResidual];
    Fz(j,:) = d.Fz.'; slips(j,:) = d.alpha.';
end
out.config = c;
out.referenceVehicle = data.ggvLUT.vehicle;
out.referenceMatchesVehicle = referenceMatches;
out.referenceGGVMatchesPlant = referenceMatchesPlant;
out.plan = plan;
out.trim = trim;
out.t = t;
out.Vx = x(:,1); out.Vy = x(:,2); out.r = x(:,3);
out.s = x(:,4); out.e_y = x(:,5); out.e_psi = x(:,6); out.delta = x(:,7);
out.speed = hypot(out.Vx,out.Vy);
out.beta = atan2(out.Vy,out.Vx);
out.betaDot = D(:,4); out.rDot = D(:,3); out.sDot = D(:,5);
out.curvature = D(:,6); out.FX = D(:,7); out.FY = D(:,8); out.Mz = D(:,9);
out.VxDot = D(:,1); out.VyDot = D(:,2);
out.axBody = out.FX/c.vehicle.m; out.ayBody = out.FY/c.vehicle.m;
out.targetSpeed = D(:,10); out.steerCommand = D(:,11); out.forceCommand = D(:,12);
out.tireUtilization = D(:,13); out.loadResidual = D(:,14);
out.Fz = Fz; out.slipAngles = slips;
out.wheelLiftPredicted = any(Fz < 1,2);
theta = out.s/c.radius;
out.X = (c.radius-out.e_y).*sin(theta);
out.Y = c.radius-(c.radius-out.e_y).*cos(theta);
out.psi = theta+out.e_psi;
out.course = out.psi+out.beta;
% Longitudinal tire fit specifies 200--1100 N; alpha lookup uses +/-30 deg.
out.tireFitRangeExceeded = any(Fz < 200 | Fz > 1100,2) ...
    | any(abs(slips) > deg2rad(30),2);
out.completed = any(events == 1);
out.lapTime = NaN;
if out.completed
    out.lapTime = t(end); out.status = "completed";
elseif any(events == 2)
    out.status = "failed: track boundary exceeded";
elseif any(events == 3)
    out.status = "failed: sideslip limit exceeded";
elseif any(events == 4)
    out.status = "failed: reversed body motion";
else
    out.status = "failed: time limit reached";
end
if c.makePlots, plotResult(out); end
end

function validateConfig(c)
validateattributes(c.vehicle.m,{'numeric'},{'scalar','positive','finite'});
validateattributes(c.vehicle.a,{'numeric'},{'scalar','positive','finite'});
validateattributes(c.vehicle.b,{'numeric'},{'scalar','positive','finite'});
validateattributes(c.vehicle.tire.P,{'numeric'},{'scalar','positive','finite'});
validateattributes(c.powerLimit,{'numeric'},{'scalar','positive','finite'});
if abs(c.vehicle.a+c.vehicle.b-c.vehicle.wb) > 1e-9 ...
        || abs(c.vehicle.a/c.vehicle.wb-c.vehicle.rwb) > 1e-9
    error('transient_skidpad:Geometry','Keep a, b, wheelbase, and rear weight fraction consistent.');
end
validateattributes(c.vehicle.Iz,{'numeric'},{'scalar','positive','finite'});
validateattributes(c.radius,{'numeric'},{'scalar','positive','finite'});
validateattributes(c.nStations,{'numeric'},{'scalar','integer','>=',10});
validateattributes(c.speedScale,{'numeric'},{'scalar','positive','<=',1});
validateattributes(c.maxStep,{'numeric'},{'scalar','positive','finite'});
validateattributes(c.maxTime,{'numeric'},{'scalar','positive','finite'});
validateattributes(c.entrySpeed,{'numeric'},{'scalar','nonnegative','finite'});
validateattributes(c.brakeBiasFront,{'numeric'},{'scalar','>=',0,'<=',1});
validateattributes(c.lowSpeedBlend,{'numeric'},{'scalar','positive','finite'});
validateattributes(c.trackHalfWidth,{'numeric'},{'scalar','positive','<',0.5*c.radius});
if abs(c.initialLateralError) >= c.trackHalfWidth
    error('transient_skidpad:InitialPosition','Initial lateral error must be inside the track boundary.');
end
validateattributes(c.steerTimeConstant,{'numeric'},{'scalar','positive','finite'});
if ~any(lower(c.startMode) == ["standing" "flying"])
    error('transient_skidpad:StartMode','Use standing or flying start mode.');
end
if size(c.speedSchedule,2) ~= 2 || c.speedSchedule(1,1) ~= 0 ...
        || c.speedSchedule(end,1) ~= 1 || any(diff(c.speedSchedule(:,1)) <= 0) ...
        || any(c.speedSchedule(:,2) <= 0 | c.speedSchedule(:,2) > 1)
    error('transient_skidpad:Schedule','Speed schedule must cover fractions 0 to 1 in increasing order, with multipliers in (0,1].');
end
end

function q = capacityInterpolants(L,c)
a = L.capacity;
if ~isequaln(c.vehicle.tire,L.vehicle.tire)
    % Rebuild capacities for pressure/camber changes, keeping reference GGV.
    z = a.Fz; angles = deg2rad(linspace(-30,30,401));
    kappas = linspace(-0.198,0.13,401);
    p = c.vehicle.tire;
    a.Fyf = max(abs(pacejka_MF_core(angles,z,p.gamma_f,p.P)),[],2);
    a.Fyr = max(abs(pacejka_MF_core(angles,z,p.gamma_r,p.P)),[],2);
    [~,~,fx] = pacejka_MF_core(0,z,p.gamma_f,p.P,kappas);
    a.Fxdf = max(fx,[],2); a.Fxbf = abs(min(fx,[],2));
    [~,~,fx] = pacejka_MF_core(0,z,p.gamma_r,p.P,kappas);
    a.Fxdr = max(fx,[],2); a.Fxbr = abs(min(fx,[],2));
end
q.FzMin = min(a.Fz); q.FzMax = max(a.Fz);
q.Fyf = griddedInterpolant(a.Fz,a.Fyf,'linear','nearest');
q.Fyr = griddedInterpolant(a.Fz,a.Fyr,'linear','nearest');
q.Fxdf = griddedInterpolant(a.Fz,a.Fxdf,'linear','nearest');
q.Fxdr = griddedInterpolant(a.Fz,a.Fxdr,'linear','nearest');
q.Fxbf = griddedInterpolant(a.Fz,a.Fxbf,'linear','nearest');
q.Fxbr = griddedInterpolant(a.Fz,a.Fxbr,'linear','nearest');
end

function plan = speedPlan(c,L)
% Same corner cap / forward / reverse / min / trapezoidal-time algorithm.
[vgrid,aygrid] = meshgrid(L.v,L.ay);
a = isfinite(L.ax_acc); b = isfinite(L.ax_brk);
Fa = scatteredInterpolant(vgrid(a),aygrid(a),L.ax_acc(a),'linear','nearest');
Fb = scatteredInterpolant(vgrid(b),aygrid(b),L.ax_brk(b),'linear','nearest');
valid = isfinite(L.ay_ss);
v = L.v(valid); ay = L.ay_ss(valid); f = v.^2/c.radius-ay;
i = find(f(1:end-1) <= 0 & f(2:end) >= 0,1);
if isempty(i), error('transient_skidpad:GGV','GGV does not bracket the corner speed.'); end
vc = v(i)-f(i)*(v(i+1)-v(i))/(f(i+1)-f(i));
s = linspace(0,2*pi*c.radius,c.nStations+1).'; ds = diff(s);
cap = vc*c.speedScale*interp1(c.speedSchedule(:,1),c.speedSchedule(:,2),s/s(end));
forward = cap; reverse = cap;
if lower(c.startMode) == "standing", forward(1) = min(c.entrySpeed,cap(1)); end
for j = 1:c.nStations
    at = Fa(max(forward(j),min(L.v)),forward(j)^2/c.radius);
    forward(j+1) = min(cap(j+1),sqrt(max(0,forward(j)^2+2*at*ds(j))));
end
for j = c.nStations:-1:1
    if cap(j) <= reverse(j+1), continue; end
    lo = reverse(j+1); hi = cap(j);
    for k = 1:60
        vm = (lo+hi)/2;
        at = min(0,Fb(max(vm,min(L.v)),vm^2/c.radius));
        residual = vm^2+2*at*ds(j)-reverse(j+1)^2;
        if residual > 0, hi = vm; else, lo = vm; end
    end
    reverse(j) = (lo+hi)/2;
end
speed = min([cap forward reverse],[],2);
at = diff(speed.^2)./(2*ds);
plan.s = s; plan.speed = speed; plan.acceleration = [at;at(end)];
plan.cornerSpeed = vc; plan.length = s(end);
plan.lapTime = sum(2*ds./(speed(1:end-1)+speed(2:end)));
end

function trim = trimMap(c,plan,q,L)
% Steady plant trim is steering feedforward, never a reset of dynamic states.
vs = linspace(0.25,max(0.5,max(plan.speed)),35).';
values = nan(numel(vs),3); residuals = inf(size(vs));
p = c.vehicle;
guess = [asin(p.b/c.radius);atan(p.wb/c.radius);0];
options = optimoptions('lsqnonlin','Display','off','MaxIterations',250, ...
    'FunctionTolerance',1e-12,'OptimalityTolerance',1e-9);
for j = 1:numel(vs)
    V = vs(j);
    fun = @(z) trimResidual(z,V,c,q,L);
    [z,~,res] = lsqnonlin(fun,guess,[-c.maxSideslip;-c.steerLimit;0], ...
        [c.maxSideslip;c.steerLimit;4*p.m*p.g],options);
    residuals(j) = max(abs(res));
    if residuals(j) < 1e-5
        values(j,:) = z.'; guess = z;
    end
end
ok = all(isfinite(values),2);
if nnz(ok) < 2, error('transient_skidpad:Trim','Insufficient steady steering trims.'); end
trim.speed = vs(ok); trim.beta = values(ok,1); trim.delta = values(ok,2);
trim.driveForce = values(ok,3); trim.residual = residuals(ok);
trim.requestedMaxSpeed = vs(end);
trim.coversTarget = trim.speed(end) >= max(plan.speed)-1e-6;
end

function e = trimResidual(z,V,c,q,L)
beta = z(1); delta = z(2); force = z(3);
u = V*cos(beta); w = V*sin(beta); r = V/c.radius;
d = plant(u,w,r,delta,force,c,q,L);
p = c.vehicle;
e = [(d.FX+p.m*r*w)/(p.m*p.g); ...
    (d.FY-p.m*r*u)/(p.m*p.g);d.Mz/(p.m*p.g*p.wb)];
end

function [dx,d] = dynamics(~,x,c,plan,trim,q,L)
u = x(1); w = x(2); r = x(3); s = x(4);
ey = x(5); epsi = x(6); delta = x(7);
p = c.vehicle; V = hypot(u,w); beta = atan2(w,u);
sref = min(max(s,0),plan.length);
vtarget = interp1(plan.s,plan.speed,sref);
atref = interp1(plan.s,plan.acceleration,sref);
ff = interp1(trim.speed,trim.delta,max(0.25,V),'linear','extrap');
courseError = atan2(sin(epsi+beta),cos(epsi+beta));
steer = ff-c.courseGain*courseError-c.lateralGain*ey-c.yawGain*(r-V/c.radius);
steer = min(max(steer,-c.steerLimit),c.steerLimit);
deltaDot = min(max((steer-delta)/c.steerTimeConstant,-c.steerRateLimit),c.steerRateLimit);
atCommand = atref+c.speedGain*(vtarget-V);
anApprox = V^2/c.radius;
% Approximate drag/scrub/centripetal compensation is driver feedforward.
% Actual acceleration always comes from the plant forces below.
force = p.m*(atCommand*cos(beta)-anApprox*sin(beta)) ...
    + 0.5*p.rho*p.CdA*V^2*cos(beta)+p.m*anApprox*p.b/p.wb*tan(delta);
d = plant(u,w,r,delta,force,c,q,L);
sDot = (u*cos(epsi)-w*sin(epsi))/max(0.1,1-ey/c.radius);
eyDot = u*sin(epsi)+w*cos(epsi);
dx = [d.FX/p.m+r*w;d.FY/p.m-r*u;d.Mz/p.Iz; ...
    sDot;eyDot;r-sDot/c.radius;deltaDot];
d.targetSpeed = vtarget; d.steerCommand = steer; d.forceCommand = force;
end

function d = plant(u,w,r,delta,force,c,q,L)
p = c.vehicle; V = hypot(u,w);
down = 0.5*p.rho*p.ClA*V^2;
axle = [p.m*p.g*(1-p.rwb)+p.aero_bal_f*down; ...
    p.m*p.g*p.rwb+(1-p.aero_bal_f)*down];
Fz = [axle(1)/2;axle(1)/2;axle(2)/2;axle(2)/2];
% A regularized rolling tire law is needed at zero speed. Body ODEs remain
% unregularized; no lateral/yaw equilibrium is imposed during integration.
alpha = [atan2(w+p.a*r,max(u,0.25))-delta;atan2(w-p.b*r,max(u,0.25))];
blend = V/sqrt(V^2+c.lowSpeedBlend^2);
gamma = [p.tire.gamma_f;p.tire.gamma_f;p.tire.gamma_r;p.tire.gamma_r];
drag = 0.5*p.rho*p.CdA*V*[u;w];
for j = 1:30
    [Fx,Fy,util] = tireForces(Fz,alpha,delta,force,u,w,r,blend,gamma,c,q,L);
    xb = Fx; yb = Fy;
    xb(1:2) = Fx(1:2)*cos(delta)-Fy(1:2)*sin(delta);
    yb(1:2) = Fx(1:2)*sin(delta)+Fy(1:2)*cos(delta);
    FX = sum(xb)-drag(1); FY = sum(yb)-drag(2);
    % Drag acts at the CG. Pitch load transfer therefore follows the
    % ground-level tire force, including force needed to overcome drag.
    longitudinalTransfer = sum(xb)*p.h_cg/p.wb;
    frontRoll = p.h_cg*sum(yb(1:2))/p.track_f;
    rearRoll = p.h_cg*sum(yb(3:4))/p.track_r;
    target = [(axle(1)-longitudinalTransfer)/2-frontRoll; ...
        (axle(1)-longitudinalTransfer)/2+frontRoll; ...
        (axle(2)+longitudinalTransfer)/2-rearRoll; ...
        (axle(2)+longitudinalTransfer)/2+rearRoll];
    % Conserve each axle's vertical load when a predicted inner wheel lifts.
    frontTotal = max(axle(1)-longitudinalTransfer,0);
    rearTotal = max(axle(2)+longitudinalTransfer,0);
    target(1) = min(max(target(1),0),frontTotal);
    target(2) = frontTotal-target(1);
    target(3) = min(max(target(3),0),rearTotal);
    target(4) = rearTotal-target(3);
    loadResidual = max(abs(target-Fz));
    if loadResidual < 1e-4, break; end
    Fz = 0.5*(Fz+target);
end
d.FX = FX; d.FY = FY; d.Mz = p.a*sum(yb(1:2))-p.b*sum(yb(3:4));
d.Fz = Fz; d.alpha = alpha; d.utilization = max(util);
d.loadResidual = loadResidual;
end

function [Fx,Fy,util] = tireForces(Fz,alpha,delta,force,u,w,r,blend,gamma,c,q,L)
z = min(max(Fz,q.FzMin),q.FzMax);
loadScale = min(1,Fz/q.FzMin); % unload to zero without finite force at lift
FyCap = [q.Fyf(z(1:2));q.Fyr(z(3:4))].*loadScale;
if force >= 0
    FxCap = [q.Fxdf(z(1:2));q.Fxdr(z(3:4))].*loadScale;
    switch upper(string(L.drive_type))
        case "RWD", weights = [0;0;0.5;0.5];
        case "FWD", weights = [0.5;0.5;0;0];
        case "AWD", weights = ones(4,1)/4;
        otherwise, error('transient_skidpad:Drive','Unknown drive layout.');
    end
    wheelU = [repmat(u*cos(delta)+(w+c.vehicle.a*r)*sin(delta),2,1);u;u];
    force = min(force,L.P_max/max(0.25,sum(weights.*wheelU)));
else
    FxCap = [q.Fxbf(z(1:2));q.Fxbr(z(3:4))].*loadScale;
    weights = [c.brakeBiasFront;c.brakeBiasFront;1-c.brakeBiasFront;1-c.brakeBiasFront]/2;
end
Fx = sign(force)*min(abs(force)*weights,FxCap);
pure = pacejka_MF_core([alpha(1);alpha(1);alpha(2);alpha(2)], ...
    max(Fz,1e-3),gamma,c.vehicle.tire.P)*blend;
pure = min(max(pure,-FyCap),FyCap);
Fy = pure.*sqrt(max(0,1-(Fx./max(FxCap,1e-6)).^2));
util = (Fx./max(FxCap,1e-6)).^2+(Fy./max(FyCap,1e-6)).^2;
end

function [value,terminal,direction] = stopEvents(~,x,c,plan)
betaMargin = c.maxSideslip-abs(atan2(x(2),x(1)));
if hypot(x(1),x(2)) < 0.5, betaMargin = 1; end
value = [plan.length-x(4);c.trackHalfWidth-abs(x(5));betaMargin;x(1)+0.1];
terminal = ones(4,1); direction = -ones(4,1);
end

function plotResult(o)
trajectoryFigure = figure('Name','Transient skidpad trajectory','Color','w'); tiledlayout(2,2);
nexttile; plot(o.X,o.Y,'LineWidth',1.5); hold on;
a = linspace(0,2*pi,400); R = o.config.radius;
plot(R*sin(a),R*(1-cos(a)),'--'); axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]'); legend('Actual CG trajectory','Reference circle');
nexttile; plot(o.t,[o.speed o.targetSpeed]); grid on;
xlabel('Time [s]'); ylabel('Path speed [m/s]'); legend('Actual','GGV target');
nexttile; plot(o.t,rad2deg([o.beta o.delta])); grid on;
xlabel('Time [s]'); ylabel('Angle [deg]'); legend('Sideslip','Actual steer');
nexttile; plot(o.t,o.e_y); grid on;
xlabel('Time [s]'); ylabel('Lateral path error [m]');
sgtitle("Transient skidpad: "+o.status,'Color',[0.1 0.1 0.1]);
formatFigure(trajectoryFigure);
dynamicsFigure = figure('Name','Transient skidpad dynamics','Color','w'); tiledlayout(2,2);
nexttile; plot(o.t,[o.r o.speed/R]); grid on;
xlabel('Time [s]'); ylabel('Yaw rate [rad/s]'); legend('Integrated r','V/R');
nexttile; plot(o.t,o.betaDot); grid on;
xlabel('Time [s]'); ylabel('Sideslip rate [rad/s]');
nexttile; plot(o.t,o.curvature); yline(1/R,'--'); grid on;
xlabel('Time [s]'); ylabel('Actual CG curvature [1/m]');
nexttile; plot(o.t,[o.axBody o.ayBody]/o.config.vehicle.g); grid on;
xlabel('Time [s]'); ylabel('Body acceleration [g]'); legend('Ax','Ay');
formatFigure(dynamicsFigure);
end

function formatFigure(fig)
set(findall(fig,'Type','axes'),'Color','w','XColor',[0.1 0.1 0.1], ...
    'YColor',[0.1 0.1 0.1],'GridColor',[0.7 0.7 0.7]);
set(findall(fig,'Type','text'),'Color',[0.1 0.1 0.1]);
set(findall(fig,'Type','legend'),'Color','w','TextColor',[0.1 0.1 0.1]);
end
