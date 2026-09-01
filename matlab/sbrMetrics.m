function m = sbrMetrics(X, why, recordSecs)
%SBRMETRICS  Score one stretch of balancing. Shared by s03 and s04 so their
%   numbers mean the same thing and can be compared directly.
%
%   X is one row per sample: [t ang rate vel pos u] in s, rad, rad/s, rad/s,
%   rad, V. why is 'ok' if the stretch ran to completion. recordSecs is how
%   long it was meant to last, used only for m.frac.
%
%   pos    peak-to-peak travel        how far it toured the bench
%   sway   travel energy 0.2-1.5 Hz   rocking fore-aft in a rhythm
%   vel    RMS wheel speed            shuffling
%   angHf  tilt energy above 0.8 Hz   buzz
%   jerk   RMS d(command)/dt          actuator chatter
%   angP2P peak-to-peak tilt          the visible wobble
%   rate   RMS gyro rate              how fast it wobbles

m = struct('aborted',true,'why',why,'frac',0, ...
           'angP2P',NaN,'rate',NaN,'angHf',NaN,'jerk',NaN, ...
           'vel',NaN,'pos',NaN,'sway',NaN);
if isempty(X) || size(X,1) < 40
    return
end
m.frac = min(1, X(end,1) / max(recordSecs, eps));
t = X(:,1); ang = X(:,2); rate = X(:,3); vel = X(:,4); pos = X(:,5); u = X(:,6);

fs  = 1 / max(median(diff(t)), 1e-4);
win = max(3, round(fs/0.8));

% Band-pass from two moving averages, so this needs no Signal Processing
% Toolbox: a 1.5 Hz low-pass minus a 0.2 Hz one keeps the rocking band.
wFast = max(3, round(fs/1.5));
wSlow = max(wFast + 2, round(fs/0.2));

m.angP2P  = prctile(ang,99) - prctile(ang,1);
m.rate    = rms(rate);
m.angHf   = rms(ang - movmean(ang, win));
m.jerk    = rms(diff(u) ./ max(diff(t), 1e-4));
m.vel     = rms(vel);
m.pos     = prctile(pos,99) - prctile(pos,1);
m.sway    = rms(movmean(pos, wFast) - movmean(pos, wSlow));
m.aborted = ~strcmp(why, 'ok');
end
