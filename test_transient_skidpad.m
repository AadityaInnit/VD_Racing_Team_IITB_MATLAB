function tests = test_transient_skidpad()
%TEST_TRANSIENT_SKIDPAD Integration, geometry, inertia, and failure checks.
tests = functiontests(localfunctions);
end

function setupOnce(t)
c = transient_skidpad_config(); c.makePlots = false;
t.TestData.config = c;
t.TestData.standing = transient_skidpad(c);
c.startMode = "flying";
t.TestData.flying = transient_skidpad(c);
c.speedScale = 0.9;
c.speedSchedule = [0 1;0.25 1;0.45 0.6;0.70 0.6;0.85 1;1 1];
t.TestData.braking = transient_skidpad(c);
end

function testStandingLaunch(t)
o = t.TestData.standing;
verifyTrue(t,o.completed);
verifyEqual(t,o.speed(1),0,'AbsTol',1e-12);
verifyTrue(t,all(isfinite([o.Vx;o.Vy;o.r;o.e_y])));
verifyGreaterThan(t,max(abs(o.Vy)),0.05);
verifyGreaterThan(t,max(abs(o.rDot)),0.1);
verifyLessThan(t,max(abs(o.e_y)),o.config.trackHalfWidth);
verifyEqual(t,o.s(end),o.plan.length,'AbsTol',1e-6);
verifyLessThanOrEqual(t,max(o.tireUtilization),1+1e-10);
verifyLessThan(t,max(o.loadResidual),1e-2);
end

function testSteadyFlyingCircle(t)
o = t.TestData.flying;
verifyTrue(t,o.completed);
verifyTrue(t,o.trim.coversTarget);
verifyLessThan(t,abs(o.lapTime-o.plan.lapTime),0.01);
verifyLessThan(t,max(abs(o.e_y)),0.01);
verifyLessThan(t,max(abs(o.rDot)),0.02);
end

function testIntegratedNewtonEquations(t)
o = t.TestData.standing; p = o.config.vehicle;
% Differentiate the integrated states independently of stored RHS outputs.
du = gradient(o.Vx,o.t); dw = gradient(o.Vy,o.t); dr = gradient(o.r,o.t);
mask = o.t > 0.3 & o.t < o.t(end)-0.1;
verifyLessThan(t,max(abs(du(mask)-o.FX(mask)/p.m-o.r(mask).*o.Vy(mask))),0.08);
verifyLessThan(t,max(abs(dw(mask)-o.FY(mask)/p.m+o.r(mask).*o.Vx(mask))),0.08);
verifyLessThan(t,max(abs(dr(mask)-o.Mz(mask)/p.Iz)),0.05);
end

function testActualTrajectoryCurvature(t)
o = t.TestData.standing;
xd = gradient(o.X,o.t); yd = gradient(o.Y,o.t);
worldXd = o.Vx.*cos(o.psi)-o.Vy.*sin(o.psi);
worldYd = o.Vx.*sin(o.psi)+o.Vy.*cos(o.psi);
% Verify free trajectory kinematics, then differentiate course once. A
% second position derivative amplifies roundoff on adaptive ODE output.
k = gradient(unwrap(o.course),o.t)./max(o.speed,1e-6);
mask = o.speed > 2 & o.t < o.t(end)-0.1;
verifyLessThan(t,max(abs(xd(mask)-worldXd(mask))),0.01);
verifyLessThan(t,max(abs(yd(mask)-worldYd(mask))),0.01);
verifyLessThan(t,max(abs(k(mask)-o.curvature(mask))),0.005);
% r is a state, not forcibly assigned the reference yaw rate.
verifyGreaterThan(t,max(abs(o.r-o.speed/o.config.radius)),0.01);
end

function testBrakingYawTransient(t)
o = t.TestData.braking;
verifyTrue(t,o.completed);
verifyLessThan(t,min(o.axBody),-1);
verifyLessThan(t,min(o.rDot),-0.1);
verifyGreaterThan(t,max(abs(o.betaDot)),0.01);
verifyLessThan(t,max(abs(o.e_y)),o.config.trackHalfWidth);
end

function testYawInertiaResponse(t)
c = t.TestData.config;
c.startMode = "flying"; c.speedScale = 0.8;
c.initialYawRateOffset = -0.2; c.maxTime = 0.3;
a = transient_skidpad(c);
c.vehicle.Iz = 2*c.vehicle.Iz;
b = transient_skidpad(c);
verifyGreaterThan(t,abs(a.Mz(1)),1);
verifyEqual(t,a.Mz(1),b.Mz(1),'AbsTol',1e-8);
verifyEqual(t,a.rDot(1),2*b.rDot(1),'AbsTol',1e-10);
end

function testTrackFailureIsNotLapTime(t)
c = t.TestData.config; c.trackHalfWidth = 0.001;
o = transient_skidpad(c);
verifyFalse(t,o.completed);
verifyTrue(t,isnan(o.lapTime));
verifyEqual(t,o.status,"failed: track boundary exceeded");
end

function testTimeStepConvergence(t)
c = t.TestData.config; c.maxStep = c.maxStep/2;
o = transient_skidpad(c);
a = t.TestData.standing;
verifyTrue(t,o.completed);
verifyLessThan(t,abs(o.lapTime-a.lapTime),0.002);
verifyLessThan(t,abs(o.e_y(end)-a.e_y(end)),0.001);
end
