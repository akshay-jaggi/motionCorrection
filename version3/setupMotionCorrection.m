function setupMotionCorrection(hSI, refStack, varargin)
%SETUPMOTIONCORRECTION  Configure XYZ online motion correction.
%
%  Stores the reference stack and parameters in globals consumed by
%  motionCorrUserFcn. After calling this, attach motionCorrUserFcn as
%  a ScanImage User Function on events:
%        acqModeStart, acqModeDone, frameAcquired
%
%  setupMotionCorrection(hSI, refStack)
%  setupMotionCorrection(hSI, refStack, 'Name', Value, ...)
%
%  Name-Value options             Default   Notes
%  ----------------------------   --------  ------------------------------------
%  'targetPlaneZ_um'              []        Sample-Z value (from rd.zs) of the
%                                           plane to correct against. If empty,
%                                           auto-detect deeper plane.
%  'correctionInterval_s'         60        Apply correction every N s
%  'avgDuration_s'                5         Rolling average window (s)
%  'minNCC_buffer'                0.30      Frame admission NCC threshold
%  'minNCC_correction'            0.50      Minimum NCC to trust Z estimate
%  'minFramesForCorr'             10        Require ≥ N frames before first corr.
%  'deadband_z_um'                1.5       Z deadband (µm)
%  'deadband_xy_um'               1.0       XY deadband per axis (µm)
%  'maxStep_z_um'                 3         Z max single move (µm)
%  'maxStep_xy_um'                5         XY max single move per axis (µm)
%  'gainXY'                       0.7       Proportional gain for XY (0..1)
%  'gainZ'                        0.5       Proportional gain for Z  (0..1)
%  'maxShift_pix'                 50        XY phase-corr search radius (px)
%  'xSign','ySign','zSign'        +1        Flip to -1 if a correction worsens drift
%  'verbose'                      true      Print per-correction diagnostics

global MCORR_REF MCORR_STATE

p = inputParser;
addParameter(p,'targetPlaneZ_um',     [],   @(x) isempty(x)||isnumeric(x));
addParameter(p,'correctionInterval_s',60,   @isnumeric);
addParameter(p,'avgDuration_s',       5,    @isnumeric);
addParameter(p,'minNCC_buffer',       0.30, @isnumeric);
addParameter(p,'minNCC_correction',   0.50, @isnumeric);
addParameter(p,'minFramesForCorr',    10,   @isnumeric);
addParameter(p,'deadband_z_um',       1.5,  @isnumeric);
addParameter(p,'deadband_xy_um',      1.0,  @isnumeric);
addParameter(p,'maxStep_z_um',        3,    @isnumeric);
addParameter(p,'maxStep_xy_um',       5,    @isnumeric);
addParameter(p,'gainXY',              0.7,  @isnumeric);
addParameter(p,'gainZ',               0.5,  @isnumeric);
addParameter(p,'maxShift_pix',        50,   @isnumeric);
addParameter(p,'xSign',               1,    @isnumeric);
addParameter(p,'ySign',               1,    @isnumeric);
addParameter(p,'zSign',               1,    @isnumeric);
addParameter(p,'verbose',             true, @islogical);
parse(p, varargin{:});
o = p.Results;

% Sanity-check ref stack
assert(isstruct(refStack) && isfield(refStack,'refFlat'), ...
    'refStack appears invalid; pass output of acquireRefStack().');

MCORR_REF = refStack;

scanFR = hSI.hRoiManager.scanFrameRate;
% Buffer holds enough frames for ~2× the averaging window — generous
% so that 2-plane gating (only ~half of frames are layer 2) does not
% starve the average.
bufSize = max(20, ceil(2 * o.avgDuration_s * scanFR));

MCORR_STATE = struct( ...
    'enabled',              true, ...
    'targetPlaneZ_um',      o.targetPlaneZ_um, ...
    'zSeen',                [], ...     % for auto-detect
    'frameBuffer',          zeros([refStack.imSize, bufSize], 'single'), ...
    'frameTimes',           zeros(1, bufSize), ...
    'bufferSize',           bufSize, ...
    'bufIdx',               0, ...
    'bufCount',             0, ...
    'tRef',                 tic, ...
    'tLastCorrection',     -inf, ...
    'avgDuration_s',        o.avgDuration_s, ...
    'correctionInterval_s', o.correctionInterval_s, ...
    'minNCC_buffer',        o.minNCC_buffer, ...
    'minNCC_correction',    o.minNCC_correction, ...
    'minFramesForCorr',     o.minFramesForCorr, ...
    'deadband_z_um',        o.deadband_z_um, ...
    'deadband_xy_um',       o.deadband_xy_um, ...
    'maxStep_z_um',         o.maxStep_z_um, ...
    'maxStep_xy_um',        o.maxStep_xy_um, ...
    'gainXY',               o.gainXY, ...
    'gainZ',                o.gainZ, ...
    'maxShift_pix',         o.maxShift_pix, ...
    'xSign',                o.xSign, ...
    'ySign',                o.ySign, ...
    'zSign',                o.zSign, ...
    'verbose',              o.verbose, ...
    'cumCorr',              [0 0 0]);

fprintf('=== XYZ Motion Correction CONFIGURED ===\n');
fprintf('  Reference   : %d planes, ±%g µm, ch %d, pixel [%.3f %.3f] µm/px\n', ...
    refStack.nZ, max(refStack.zOffsets_um), refStack.channel, refStack.pixelSizeXY_um);
fprintf('  Target plane: %s\n', ternary(isempty(o.targetPlaneZ_um), ...
    'auto-detect (deeper of two FastZ planes)', sprintf('%.2f µm', o.targetPlaneZ_um)));
fprintf('  Correct     : every %.0f s, using last %.0f s avg\n', ...
    o.correctionInterval_s, o.avgDuration_s);
fprintf('  Deadbands   : Z=%.1f  XY=%.1f µm    Max step: Z=%.1f  XY=%.1f µm\n', ...
    o.deadband_z_um, o.deadband_xy_um, o.maxStep_z_um, o.maxStep_xy_um);
fprintf('  Gains       : XY=%.2f  Z=%.2f       Signs: X=%+d Y=%+d Z=%+d\n', ...
    o.gainXY, o.gainZ, o.xSign, o.ySign, o.zSign);
fprintf('  Buffer      : %d slots\n', bufSize);
fprintf('  Now attach motionCorrUserFcn to:\n');
fprintf('     acqModeStart, acqModeDone, frameAcquired\n');
fprintf('  To stop:       global MCORR_STATE; MCORR_STATE.enabled = false;\n');
fprintf('  To flip a sign: global MCORR_STATE; MCORR_STATE.xSign = -1; (etc.)\n\n');
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
