% ═══ STEP 1: Navigate to target plane, single-plane mode ════════════════
hSI.startFocus();                    % disable FastZ before this
% hSI.hFastZ.enable = false;         % if needed via command line

% ═══ STEP 2: Verify frame access ════════════════════════════════════════
img = grabCurrentFrame(hSI, 1);
figure; imagesc(img); colormap gray; axis image; colorbar;
title(sprintf('Frame access OK  (min=%.0f  max=%.0f)', min(img(:)), max(img(:))));

% ═══ STEP 3: Acquire reference stack (~3–5 min) ══════════════════════════
% If ScanImage cannot auto-detect pixel size, specify it:
%   refStack = acquireRefStack(hSI, 'pixelSizeXY_um', [1.0, 1.0]);
refStack = acquireRefStack(hSI, ...
    'zRange_um',     30, ...     % ±30 µm around current plane
    'zStep_um',       2, ...     % 2 µm steps  →  31 reference planes
    'avgDuration_s',  5, ...     % 5 s average per plane (~150 frames @ 30 Hz)
    'channel',        1);        % PMT channel with best SNR

% ═══ STEP 4: Re-enable 2-plane imaging ══════════════════════════════════
% hSI.hFastZ.enable = true;
% (restore your piezo settings for 2-plane jump-and-settle)

% ═══ STEP 5: Start acquisition and motion correction ════════════════════
[t, stopFcn] = setupMotionCorrection(hSI, refStack);
                                     % all defaults: 60 s interval, 5 s buffer
hSI.startLoop();

% ═══ STEP 6: Calibrate signs (IMPORTANT — do this in first 5 minutes) ═══
%  See "Sign Calibration" section below.
%  Flip signs on the fly without restarting:
%    MCORR_STATE.zSign = -1;   % if Z corrections made drift worse
%    MCORR_STATE.xSign = -1;   % if X corrections made drift worse
%    MCORR_STATE.ySign = -1;   % if Y corrections made drift worse

% ═══ STEP 7: Monitor (watch Command Window output) ═══════════════════════
%
%  [MotionCorr @    60 s]  10/12 frames buffered  Computing drift...
%    Z : est=+3.45 µm  corr=+3.45 µm   NCC=0.784   ref plane 18/31
%    Y : est=−1.20 µm  corr=−1.20 µm   (−1.20 px)
%    X : est=+0.40 µm  corr=+0.00 µm   (0.40 px)   ← within XY deadband
%    Motor moved : X+0.00  Y+1.20  Z−3.45 µm
%    Cumulative  : X+0.0   Y+1.2   Z−3.5 µm

% ═══ STEP 8: Stop ════════════════════════════════════════════════════════
stopFcn();