function [t, stopFcn] = setupMotionCorrection(hSI, refStack, varargin)
%SETUPMOTIONCORRECTION  Initialise conservative XYZ online motion correction.
%
%  [t, stopFcn] = setupMotionCorrection(hSI, refStack)
%  [t, stopFcn] = setupMotionCorrection(hSI, refStack, 'Name', Value, ...)
%
%  Name-Value options            Default  Notes
%  ---------------------------   -------  ------------------------------------
%  'timerPeriod_s'               0.5      Frame collection interval (s)
%  'correctionInterval_s'        60       Apply correction every N s (1 min)
%  'bufferDuration_s'             5       Nominal rolling average window (s)
%  'minNCC_buffer'                0.3     Min NCC to admit frame into buffer
%  'minNCC_correction'            0.5     Min NCC to trust Z estimate
%  'minFramesForCorr'             5       Require ≥ N frames before first corr.
%  'deadband_z_um'                2       Z: ignore drift smaller than this
%  'deadband_xy_um'               1       XY: ignore drift smaller than this
%  'maxStep_z_um'                10       Z: max single correction (µm)
%  'maxStep_xy_um'                5       XY: max single correction per axis (µm)
%  'maxShift_pix'                50       XY search radius (pixels)
%  'zSign'                        1       Flip to −1 if Z corrections worsen drift
%  'xSign'                        1       Flip to −1 if X corrections worsen drift
%  'ySign'                        1       Flip to −1 if Y corrections worsen drift
%
%  RETURNS
%    t        — MATLAB timer object (stop with stopFcn or stop(t); delete(t))
%    stopFcn  — convenience handle: stopFcn() stops timer and prints summary

global MCORR_REF MCORR_STATE

p = inputParser;
addParameter(p, 'timerPeriod_s',        0.5,  @isnumeric);
addParameter(p, 'correctionInterval_s', 60,   @isnumeric);
addParameter(p, 'bufferDuration_s',      5,   @isnumeric);
addParameter(p, 'minNCC_buffer',         0.3, @isnumeric);
addParameter(p, 'minNCC_correction',     0.5, @isnumeric);
addParameter(p, 'minFramesForCorr',      5,   @isnumeric);
addParameter(p, 'deadband_z_um',         2,   @isnumeric);
addParameter(p, 'deadband_xy_um',        1,   @isnumeric);
addParameter(p, 'maxStep_z_um',         10,   @isnumeric);
addParameter(p, 'maxStep_xy_um',         5,   @isnumeric);
addParameter(p, 'maxShift_pix',         50,   @isnumeric);
addParameter(p, 'zSign',                 1,   @isnumeric);
addParameter(p, 'xSign',                 1,   @isnumeric);
addParameter(p, 'ySign',                 1,   @isnumeric);
parse(p, varargin{:});
o = p.Results;

MCORR_REF = refStack;

% ---- Frame ring buffer ------------------------------------------
%  Holds ceil(bufferDuration_s / timerPeriod_s) + 2 slots.
%  With ~50% NCC gate acceptance in 2-plane mode, the effective
%  wall-clock coverage ≈ 2 × bufferDuration_s — acceptable for
%  a 60-second correction interval.
bufSize = ceil(o.bufferDuration_s / o.timerPeriod_s) + 2;
imgSz   = refStack.imSize;    % [rows, cols]

MCORR_STATE = struct( ...
    'enabled',              true, ...
    'frameBuffer',          zeros([imgSz, bufSize], 'single'), ...
    'bufferSize',           bufSize, ...
    'bufIdx',               0, ...              % ring-buffer write head
    'bufCount',             0, ...              % frames written so far (≤ bufSize)
    'tRef',                 tic, ...
    'tLastCorrection',      -inf, ...
    'correctionInterval_s', o.correctionInterval_s, ...
    'minNCC_buffer',        o.minNCC_buffer, ...
    'minNCC_correction',    o.minNCC_correction, ...
    'minFramesForCorr',     o.minFramesForCorr, ...
    'deadband_z_um',        o.deadband_z_um, ...
    'deadband_xy_um',       o.deadband_xy_um, ...
    'maxStep_z_um',         o.maxStep_z_um, ...
    'maxStep_xy_um',        o.maxStep_xy_um, ...
    'maxShift_pix',         o.maxShift_pix, ...
    'zSign',                o.zSign, ...
    'xSign',                o.xSign, ...
    'ySign',                o.ySign, ...
    'cumCorr',              [0, 0, 0]);        % [X, Y, Z] µm

% ---- Cleanup old timer ------------------------------------------
old = timerfind('Name','MotionCorrTimer');
if ~isempty(old), stop(old); delete(old); end

% ---- Create and start timer -------------------------------------
t = timer( ...
    'Name',          'MotionCorrTimer', ...
    'ExecutionMode', 'fixedRate', ...
    'Period',        o.timerPeriod_s, ...
    'BusyMode',      'drop', ...       % silently skip if callback still running
    'TimerFcn',      @(~,~) motionCorrStep(hSI), ...
    'ErrorFcn',      @(~,e) warning('MotionCorrTimer: %s', e.Data.message));
start(t);

stopFcn = @() stopMotionCorr(t);

fprintf('=== XYZ Motion Correction ACTIVE ===\n');
fprintf('  Collect : every %.2f s  (%.1f Hz)\n', ...
    o.timerPeriod_s, 1/o.timerPeriod_s);
fprintf('  Correct : every %.0f s  (%.1f min)\n', ...
    o.correctionInterval_s, o.correctionInterval_s/60);
fprintf('  Buffer  : %d slots (~%.0f s nominal)\n', bufSize, o.bufferDuration_s);
fprintf('  Deadband: Z=%.1f µm | XY=%.1f µm\n', ...
    o.deadband_z_um, o.deadband_xy_um);
fprintf('  Max step: Z=%.1f µm | XY=%.1f µm\n', ...
    o.maxStep_z_um, o.maxStep_xy_um);
fprintf('  Signs   : X=%+d | Y=%+d | Z=%+d\n', ...
    o.xSign, o.ySign, o.zSign);
fprintf('  Stop with: stopFcn()\n');
fprintf('  First correction in %.0f s.\n\n', o.correctionInterval_s);
end


% =========================================================================
function stopMotionCorr(t)
global MCORR_STATE
if ~isempty(MCORR_STATE)
    MCORR_STATE.enabled = false;
    fprintf('\n=== Motion Correction STOPPED ===\n');
    fprintf('  Total corrections applied:\n');
    fprintf('    X = %+.1f µm\n', MCORR_STATE.cumCorr(1));
    fprintf('    Y = %+.1f µm\n', MCORR_STATE.cumCorr(2));
    fprintf('    Z = %+.1f µm\n', MCORR_STATE.cumCorr(3));
end
if isvalid(t), stop(t); delete(t); end
end