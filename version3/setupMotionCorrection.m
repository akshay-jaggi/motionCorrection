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
%  'deadband_z_um'                2         Z deadband (µm)
%  'deadband_xy_um'               1.0       XY deadband per axis (µm)
%  'maxStep_z_um'                 3         Z max single move (µm)
%  'maxStep_xy_um'                5         XY max single move per axis (µm)
%  'gainXY'                       0.7       Proportional gain for XY (0..1)
%  'gainZ'                        0.5       Proportional gain for Z  (0..1)
%  'maxShift_pix'                 50        XY phase-corr search radius (px)
%  'xSign','ySign','zSign'        +1        Flip to -1 if a correction worsens drift
%  'minMoveInterval_s'            30        Hard floor between motor commands (s).
%                                           Enforced in addition to
%                                           correctionInterval_s. Protects the
%                                           MP-285 from frequent-command lockup.
%  'postMoveQuiet_s'              0.5       Pause after each moveSample so the
%                                           controller has guaranteed settle time
%                                           before any other code path can talk
%                                           to it.
%  'verbose'                      true      Print per-correction diagnostics
%
%  ---------------------------------------------------------------------------
%  MP-285 SAFETY NOTES
%  ---------------------------------------------------------------------------
%  The Sutter MP-285 is documented as having an unstable serial connection.
%  ScanImage's docs warn that (a) frequent commands and (b) position queries
%  while the stage is moving can lock up the controller; SI has therefore
%  disabled its own live position updates for this device.
%
%  This implementation mitigates that as follows:
%    * samplePosition is queried EXACTLY ONCE per acquisition (at acqModeStart,
%      when the stage is guaranteed idle). The position is then cached in
%      MCORR_STATE.cachedMotorPos and incremented by the commanded delta after
%      each successful moveSample. No further position queries are issued.
%    * moveSample is rate-limited by BOTH correctionInterval_s AND a hard
%      floor (minMoveInterval_s) that cannot be defeated by mis-set params.
%    * Each moveSample is followed by postMoveQuiet_s of pause so the
%      controller settles before anything else can touch the serial port.
%
%  CAVEAT: If you move the stage via the MP-285 joystick or any non-SI path
%  while acquisition is running, the cached position will desync. The next
%  correction will then command an absolute target based on the stale cache.
%  Do not use the joystick during a corrected run; stop/restart acquisition
%  if you need to reposition manually.

global MCORR_REF MCORR_STATE MCORR_FRAMEBUF MCORR_FRAMETIMES MCORR_DIAG

p = inputParser;
addParameter(p,'targetPlaneZ_um',     [],   @(x) isempty(x)||isnumeric(x));
addParameter(p,'correctionInterval_s',60,   @isnumeric);
addParameter(p,'avgDuration_s',       5,    @isnumeric);
addParameter(p,'minNCC_buffer',       0.30, @isnumeric);
addParameter(p,'minNCC_correction',   0.50, @isnumeric);
addParameter(p,'minFramesForCorr',    10,   @isnumeric);
addParameter(p,'deadband_z_um',       2,    @isnumeric);
addParameter(p,'deadband_xy_um',      1.0,  @isnumeric);
addParameter(p,'maxStep_z_um',        3,    @isnumeric);
addParameter(p,'maxStep_xy_um',       5,    @isnumeric);
addParameter(p,'gainXY',              0.7,  @isnumeric);
addParameter(p,'gainZ',               0.5,  @isnumeric);
addParameter(p,'maxShift_pix',        50,   @isnumeric);
addParameter(p,'xSign',               1,    @isnumeric);
addParameter(p,'ySign',               1,    @isnumeric);
addParameter(p,'zSign',               1,    @isnumeric);
addParameter(p,'minMoveInterval_s',   30,   @(x) isnumeric(x) && x >= 0);
addParameter(p,'postMoveQuiet_s',     0.5,  @(x) isnumeric(x) && x >= 0);
addParameter(p,'collectMargin_s',     2,    @(x) isnumeric(x) && x >= 0);
addParameter(p,'dryRun',              false,@islogical);
addParameter(p,'diag',                true, @islogical);
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
    'minMoveInterval_s',    o.minMoveInterval_s, ...
    'postMoveQuiet_s',      o.postMoveQuiet_s, ...
    'collectMargin_s',      o.collectMargin_s, ...
    'dryRun',               o.dryRun, ...
    'diag',                 o.diag, ...
    'tLastMove',           -inf, ...   % timestamp of last moveSample;
                                       % Phase 2 is suppressed for
                                       % postMoveQuiet_s after each move
    'cachedMotorPos',       [], ...    % populated at acqModeStart, then
                                       % incremented by each commanded move
    'verbose',              o.verbose, ...
    'cumCorr',              [0 0 0]);

% Frame ring buffer lives in separate top-level globals so that per-frame
% indexed writes (MCORR_FRAMEBUF(:,:,idx) = feat) are in-place and never
% trigger MATLAB's copy-on-write on the main state struct. At typical
% frame rates (30 Hz, bufSize ~300), the frameBuffer alone is ~113 MB;
% copying it inside MCORR_STATE every frame was causing the
% "Data logging lags behind acquisition" frame-drop error.
MCORR_FRAMEBUF   = zeros([refStack.imSize, bufSize], 'single');
MCORR_FRAMETIMES = zeros(1, bufSize);

% Cache the initial sample position NOW (stage is idle, no SI pressure)
% rather than during acqModeStart. The MP-285 serial round-trip can take
% hundreds of milliseconds and inside acqModeStart this delays SI's own
% pipeline init, which can drop the first frame.
try
    MCORR_STATE.cachedMotorPos = hSI.hMotors.samplePosition;
    fprintf('  Cached initial pos: X=%.2f Y=%.2f Z=%.2f µm\n', MCORR_STATE.cachedMotorPos);
catch ME
    warning('setupMotionCorrection:posRead', ...
        'Failed to read samplePosition: %s\nMotion correction will be disabled until you retry setup.', ...
        ME.message);
    MCORR_STATE.enabled = false;
end

% Diagnostics ring-buffer for per-phase timing (filled by the callback).
MCORR_DIAG = struct( ...
    'nFrames',       0, ...
    'nProcessed',    0, ...    % frames that got past the collection-window gate
    'tGrab',         zeros(1,200,'single'), ...  % last 200 frames
    'tFeature',      zeros(1,200,'single'), ...
    'tBuffer',       zeros(1,200,'single'), ...
    'tTotal',        zeros(1,200,'single'), ...
    'idx',           0, ...
    'peakTotal_ms',  0, ...
    'peakWhen',      0);

% JIT pre-warm: call featureImage once now so MATLAB compiles the
% imgaussfilt/imgradient path before the first frameAcquired fires.
% Without this the very first callback invocation is ~10x slower and
% can cause ScanImage to drop a frame.
if ~isempty(refStack.rIdx) && ~isempty(refStack.cIdx)
    dummy = zeros(numel(refStack.rIdx), numel(refStack.cIdx), 'single');
    jitWarm_(dummy, refStack.useFeatureImage);
end

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
fprintf('  MP285 safety: min %g s between moves, %g s post-move quiet\n', ...
    o.minMoveInterval_s, o.postMoveQuiet_s);
fprintf('  Buffer      : %d slots   Collection window: last %.1f s before each correction\n', ...
    bufSize, o.avgDuration_s + o.collectMargin_s);
if o.dryRun
    fprintf('  *** DRY RUN MODE *** computes everything but never moves stage.\n');
end
fprintf('  Now attach motionCorrUserFcn to:\n');
fprintf('     acqModeStart, acqModeDone, frameAcquired\n');
fprintf('  To stop:       global MCORR_STATE; MCORR_STATE.enabled = false;\n');
fprintf('  To flip a sign: global MCORR_STATE; MCORR_STATE.xSign = -1; (etc.)\n\n');
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end

function jitWarm_(dummy, useFeatureImage)
% Force MATLAB to JIT-compile the feature image pipeline used in the
% frameAcquired callback so the first live frame doesn't stall.
if useFeatureImage
    try
        x = dummy;
        x = x - imgaussfilt(x, 6);
        x = imgaussfilt(x, 1);
        imgradient(x);
    catch
        [gx, gy] = gradient(double(dummy));
        sqrt(gx.^2 + gy.^2); %#ok<VUNUS>
    end
end
end
