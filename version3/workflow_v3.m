% workflow_v3.m  -  Quick reference of the v3 motion-correction sequence.
%                   Run interactively; do not execute as a script blindly.

%% 1) Single-plane FOCUS at your reference plane
%    - Disable FastZ (multi-plane).  In ScanImage GUI: FastZ Controls → uncheck.
%    - Pick the plane you want to keep stable (typically the deeper/structural one).
hSI = evalin('base','hSI');
hSI.hFastZ.enable = false;     % belt-and-braces
hSI.startFocus();

% Verify a frame is reachable on your reference channel (e.g. structural / red = 2)
[img, z, ok] = grabCurrentFrame(hSI, 2);
assert(ok, 'No frame returned — fix grabCurrentFrame or check focus state.');
figure; imagesc(img); axis image; colormap gray; colorbar;
title(sprintf('z = %.2f µm   min %.0f  max %.0f', z, min(img(:)), max(img(:))));

%% 2) Acquire reference Z stack (a few minutes)
%    Use the channel with the most invariant structural signal.
refStack = acquireRefStack(hSI, ...
    'channel',         2, ...          % structural / red
    'zRange_um',       12, ...         % ±12 µm around current plane
    'zStep_um',        1, ...
    'avgDuration_s',   5, ...
    'useFeatureImage', true, ...       % robust to activity
    'pixelSizeXY_um',  [1.3125 1.3125]);  % 672 µm FOV / 512 px

hSI.abort();                      % leave focus mode

%% 3) Re-enable 2-plane / 2-channel acquisition exactly as you normally use it
hSI.hFastZ.enable = true;
% (restore your piezo waveform / FastZ settings here)

%% 4) Configure the motion-correction state
setupMotionCorrection(hSI, refStack, ...
    'targetPlaneZ_um',      [], ...     % [] = auto-detect deeper plane
    'correctionInterval_s', 60, ...
    'avgDuration_s',        5, ...
    'deadband_z_um',        1.5, ...
    'deadband_xy_um',       1.0, ...
    'maxStep_z_um',         3, ...
    'maxStep_xy_um',        5, ...
    'gainXY',               0.7, ...
    'gainZ',                0.5, ...
    % --- scope-specific axis calibration ----------------------------
    % Image cols (X) map to motor Y; image rows (Y) map to motor X.
    % Both image axes are anti-parallel to their motor counterpart:
    %   +image_x drift → motor_y decreased  → correct with +motor_y  → ySign=-1
    %   +image_y drift → motor_x decreased  → correct with +motor_x  → xSign=-1
    %   +z-stack shift → motor_z increased  → correct with -motor_z  → zSign=+1
    'xSign',               -1, ...
    'ySign',               -1, ...
    'zSign',                1);

%% 5) Attach motionCorrUserFcn as a ScanImage USER FUNCTION on:
%      acqModeStart, acqModeDone, frameAcquired
%    Use the GUI:  ScanImage  →  USER  →  User Functions...
%    or programmatically:
%        hSI.hUserFunctions.userFunctionsCfg = ...
%            struct('EventName',{'acqModeStart','acqModeDone','frameAcquired'}, ...
%                   'UserFcnName',{'motionCorrUserFcn','motionCorrUserFcn','motionCorrUserFcn'}, ...
%                   'Arguments',{{} {} {}}, ...
%                   'Enable',{true,true,true});

%% 6) Start acquisition (LOOP / GRAB)
hSI.startLoop();   % or click GRAB / LOOP in the GUI

%% 7) Sign calibration — DO THIS IN THE FIRST 5 MINUTES
%    Watch the Command Window. After a correction, observe whether the
%    cumulative drift is shrinking (good) or growing (sign wrong).
%    Flip on the fly without restarting:
%        global MCORR_STATE
%        MCORR_STATE.xSign = -1;     % if X corrections worsen drift
%        MCORR_STATE.ySign = -1;
%        MCORR_STATE.zSign = -1;

%% 8) Stop / disable
%    To pause correction without ending acquisition:
%        global MCORR_STATE; MCORR_STATE.enabled = false;
%    To re-enable:  MCORR_STATE.enabled = true;
