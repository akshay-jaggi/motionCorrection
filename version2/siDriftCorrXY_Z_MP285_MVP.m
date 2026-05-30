function siDriftCorrXY_Z_MP285_MVP(src, evt, arguments)
% Conservative XY + Z drift correction (minutes-hours) using MP-285
% - Templates: average 5 seconds per Z offset (FOCUS)
% - Online: once per minute, compare last 5 seconds average (layer 2 only)
%
% Configure ScanImage User Functions to call this same file on:
%   focusStart, focusDone, acqModeStart, acqModeDone, frameAcquired

persistent S
hSI = src.hSI;

% ======================= USER SETTINGS =========================
% --- Which ScanImage channel to use as reference (your "most invariant") ---
REF_CHAN_NUM = 2;   % <-- set to your RED PMT channel number (1..4)

% --- Which plane to use for drift correction ---
% If you know the piezo Z for layer 2 exactly, you can hardcode it here and skip auto-detect:
LAYER2_Z_HARDCODED = NaN;  % e.g., 100; set NaN to auto-detect deeper plane

% --- Reference Z template stack (MP-285 offsets around current layer-2 focus) ---
TEMPLATE_Z_UM = -6:1:6;     % offsets relative to starting position (um)
TEMPLATE_AVG_S = 5.0;       % average this many seconds at each offset (conservative)

% --- Online averaging and correction schedule ---
RUN_AVG_S   = 5.0;          % average last 5 seconds (layer-2 frames)
CORR_PERIOD_S = 60.0;        % correct once per minute

% --- Conservative control / safety limits ---
Z_DEADBAND_UM  = 1.5;
XY_DEADBAND_UM = 2.0;

KZ  = 0.4;                  % proportional gain (0..1); conservative
KXY = 0.6;

Z_MAXSTEP_UM  = 2.0;         % per correction (once/min) max move
XY_MAXSTEP_UM = 5.0;

% --- Pixel size (you must calibrate/measure) ---
UM_PER_PX_X = 0.8;          % <-- set from measurement
UM_PER_PX_Y = 0.8;          % <-- set from measurement

% --- Stage sign conventions (empirically determine) ---
STAGE_SIGN_X = +1;          % flip to -1 if correction runs away
STAGE_SIGN_Y = +1;
STAGE_SIGN_Z = +1;

% --- MP-285 microstep scale (common: 0.04 um/microstep => 25 microsteps/um) ---
US_PER_UM = 25;             % verify your MP-285 resolution mode if unsure

% --- Serial defaults ---
DEFAULT_BAUD = 9600;

% --- Feature image crop (makes matching faster & more stable) ---
CROP_FRACTION = 0.6;        % use central 60% region
% ===============================================================

if isempty(S)
    S = struct();
    S.sp = [];
    S.mode = "idle";  % "idle" | "calibrating" | "running"

    % Calibration state
    S.templatesReady = false;
    S.kTemplate = 1;
    S.offsetTic = [];
    S.sumImg = [];
    S.nImg = 0;
    S.currentOffset_um = 0;
    S.templateImgs = [];     % [H x W x nZ]
    S.templateVec = [];      % [numPix x nZ]
    S.templateZ0Img = [];    % [H x W] at z offset nearest 0

    % Runtime buffer (ring)
    S.t0 = tic;
    S.bufInit = false;
    S.bufNmax = 1200;        % enough for >~2 minutes at ~10 Hz layer2
    S.bufWrite = 0;
    S.bufT = [];
    S.buf = [];              % [H x W x Nmax] single

    % Plane identification
    S.layer2ZLevel = LAYER2_Z_HARDCODED;
    S.zSeen = [];

    % Correction timing
    S.lastCorrT = -inf;

    fprintf('[DriftCorr MVP] Initialized.\n');
end

switch evt.EventName
    case 'focusStart'
        S = ensureMp285Connected(S, DEFAULT_BAUD);
        if isempty(S.sp), fprintf('[DriftCorr MVP] MP-285 not connected.\n'); return; end
        mp285_setRelativeMode(S.sp);

        % Start calibration at first offset
        S.mode = "calibrating";
        S.templatesReady = false;
        S.kTemplate = 1;
        S.templateImgs = [];
        S.templateVec = [];
        S.templateZ0Img = [];
        S.currentOffset_um = 0;

        % Move to first offset
        mp285_moveRelXYZ(S, 0, 0, TEMPLATE_Z_UM(1) - S.currentOffset_um, US_PER_UM);
        S.currentOffset_um = TEMPLATE_Z_UM(1);

        % Reset averaging accumulator
        S.offsetTic = tic;
        S.sumImg = [];
        S.nImg = 0;

        fprintf('[DriftCorr MVP] CALIBRATION: %d Z offsets, %.1f s each (FOCUS at layer 2!).\n', ...
            numel(TEMPLATE_Z_UM), TEMPLATE_AVG_S);

    case 'focusDone'
        if S.mode == "calibrating"
            % Return to 0 offset
            mp285_moveRelXYZ(S, 0, 0, 0 - S.currentOffset_um, US_PER_UM);
            S.currentOffset_um = 0;
            S.mode = "idle";
            fprintf('[DriftCorr MVP] Focus ended; calibration aborted/ended.\n');
        end

    case 'acqModeStart'
        S = ensureMp285Connected(S, DEFAULT_BAUD);
        if isempty(S.sp)
            fprintf('[DriftCorr MVP] MP-285 not connected; will not correct.\n');
            S.mode = "idle"; return;
        end
        mp285_setRelativeMode(S.sp);

        if ~S.templatesReady
            fprintf('[DriftCorr MVP] Templates not ready. Run FOCUS to calibrate first.\n');
            S.mode = "idle";
        else
            S.mode = "running";
            if isnan(LAYER2_Z_HARDCODED)
                S.layer2ZLevel = NaN; % re-detect per run
                S.zSeen = [];
            end
            S.lastCorrT = -inf;
            fprintf('[DriftCorr MVP] RUNNING: correcting once every %.0f s using last %.0f s average.\n', ...
                CORR_PERIOD_S, RUN_AVG_S);
        end

    case 'acqModeDone'
        S.mode = "idle";
        fprintf('[DriftCorr MVP] Acquisition ended.\n');

    case 'frameAcquired'
        % Drain any returned bytes (MP-285 returns CR on completion)
        if ~isempty(S.sp) && isvalid(S.sp) && S.sp.NumBytesAvailable > 0
            read(S.sp, S.sp.NumBytesAvailable, "uint8"); %#ok<NASGU>
        end

        [img, zNow_um, ok] = getLatestImage(hSI, REF_CHAN_NUM);
        if ~ok, return; end

        % Build feature image (cropped, high-pass/edge-like)
        f = featureImage(img, CROP_FRACTION);

        % Initialize runtime ring buffer size based on feature image size
        if ~S.bufInit
            [H,W] = size(f);
            S.buf = zeros(H,W,S.bufNmax,'single');
            S.bufT = zeros(1,S.bufNmax,'single');
            S.bufInit = true;
        end

        if S.mode == "calibrating"
            S = calibrationStep(S, f, TEMPLATE_Z_UM, TEMPLATE_AVG_S, US_PER_UM);
            return;
        end

        if S.mode ~= "running" || ~S.templatesReady
            return;
        end

        % Detect layer 2 z (deeper plane) unless hardcoded
        if isnan(S.layer2ZLevel)
            S = detectLayer2Z(S, zNow_um);
            if isnan(S.layer2ZLevel), return; end
        end

        % Only buffer layer-2 frames (so the "last 5 sec average" is layer 2 only)
        if abs(zNow_um - S.layer2ZLevel) > 1e-3
            return;
        end

        % Push into ring buffer
        tNow = toc(S.t0);
        S.bufWrite = S.bufWrite + 1;
        k = mod(S.bufWrite-1, S.bufNmax) + 1;
        S.buf(:,:,k) = f;
        S.bufT(k) = tNow;

        % Correct only once per minute
        if (tNow - S.lastCorrT) < CORR_PERIOD_S
            return;
        end

        % Compute average of last RUN_AVG_S
        [Fcur, nUsed] = averageLastSeconds(S.buf, S.bufT, tNow, RUN_AVG_S);
        if nUsed < 3
            fprintf('[DriftCorr MVP] Not enough frames in last %.1fs to correct.\n', RUN_AVG_S);
            return;
        end

        % 1) XY drift: register Fcur to z0 template (one registration per minute)
        [dxPix, dyPix, FcurXYaligned] = estimateXYtoTemplate(Fcur, S.templateZ0Img);

        dx_um = STAGE_SIGN_X * (-dxPix * UM_PER_PX_X);   % negative: move sample opposite image shift (may need sign flip)
        dy_um = STAGE_SIGN_Y * (-dyPix * UM_PER_PX_Y);

        % Apply deadband/gain/limits
        dx_cmd = applyConservativeCmd(dx_um, XY_DEADBAND_UM, KXY, XY_MAXSTEP_UM);
        dy_cmd = applyConservativeCmd(dy_um, XY_DEADBAND_UM, KXY, XY_MAXSTEP_UM);

        % 2) Z drift: match XY-aligned average to template stack
        zEst_um = estimateZfromTemplates(FcurXYaligned, S.templateVec, TEMPLATE_Z_UM);

        dz_um = STAGE_SIGN_Z * (-zEst_um);
        dz_cmd = applyConservativeCmd(dz_um, Z_DEADBAND_UM, KZ, Z_MAXSTEP_UM);

        % If nothing to do, skip
        if dx_cmd==0 && dy_cmd==0 && dz_cmd==0
            S.lastCorrT = tNow;  % still advance schedule to “once per minute”
            fprintf('[DriftCorr MVP] Correction check: no move needed (n=%d, xyPix=[%.2f %.2f], zEst=%.2f).\n', ...
                nUsed, dxPix, dyPix, zEst_um);
            return;
        end

        % Execute one combined move
        mp285_moveRelXYZ(S, dx_cmd, dy_cmd, dz_cmd, US_PER_UM);
        S.lastCorrT = tNow;

        fprintf(['[DriftCorr MVP] MOVE (n=%d avg): dx=%.2fum dy=%.2fum dz=%.2fum | ' ...
                 'xyPix=[%.2f %.2f] zEst=%.2fum\n'], ...
                 nUsed, dx_cmd, dy_cmd, dz_cmd, dxPix, dyPix, zEst_um);
end

end

% ======================= Helpers =======================

function cmd = applyConservativeCmd(x_um, deadband_um, K, maxstep_um)
if abs(x_um) < deadband_um
    cmd = 0;
else
    cmd = K * x_um;
    cmd = max(-maxstep_um, min(maxstep_um, cmd));
end
end

function S = ensureMp285Connected(S, baud)
if ~isempty(S.sp) && isvalid(S.sp), return; end

ports = serialportlist("available");
if isempty(ports)
    fprintf('[DriftCorr MVP] No COM ports found.\n'); S.sp = []; return;
end

for p = string(ports)
    % Try hardware flow control first, then none
    for flow = ["hardware","none"]
        try
            sp = serialport(p, baud, "DataBits", 8, "StopBits", 1, "Parity", "none", ...
                "FlowControl", flow, "ByteOrder", "little-endian", "Timeout", 0.2);
            configureTerminator(sp, "CR");
            % Nudge: 'n' refresh display (usually harmless)
            write(sp, uint8(['n' 13]), "uint8");
            pause(0.03);
            S.sp = sp;
            fprintf('[DriftCorr MVP] Connected MP-285 on %s (FlowControl=%s).\n', p, flow);
            return;
        catch
        end
    end
end

fprintf('[DriftCorr MVP] Failed to connect MP-285.\n');
S.sp = [];
end

function mp285_setRelativeMode(sp)
write(sp, uint8(['b' 13]), "uint8"); % relative mode
end

function mp285_moveRelXYZ(S, dx_um, dy_um, dz_um, usPerUm)
if isempty(S.sp), return; end
d_us = int32(round([dx_um dy_um dz_um] * usPerUm));
pkt = [uint8('m') typecast(d_us,'uint8') uint8(13)];
write(S.sp, pkt, "uint8");
end

function [img, zNow_um, ok] = getLatestImage(hSI, refChanNum)
ok = false; img = []; zNow_um = NaN;
try
    lastStripe = hSI.hDisplay.stripeDataBuffer{hSI.hDisplay.stripeDataBufferPointer};
    rd = lastStripe.roiData{1};

    chList = rd.channels;
    chIdx = find(chList == refChanNum, 1);
    if isempty(chIdx), return; end

    zs = rd.zs;
    if isempty(zs), return; end
    zNow_um = zs(1);

    img = rd.imageData{chIdx}{1};
    img = img.'; % common transpose
    ok = true;
catch
    ok = false;
end
end

function f = featureImage(img, cropFrac)
x = single(img);
x = x - mean(x(:));

% Crop to central region
[H,W] = size(x);
ch = round(H*cropFrac); cw = round(W*cropFrac);
r0 = floor((H-ch)/2)+1; c0 = floor((W-cw)/2)+1;
x = x(r0:r0+ch-1, c0:c0+cw-1);

% Make it more “structural” and less activity-dependent:
x = x - imgaussfilt(x, 6);        % remove low-freq background
x = imgaussfilt(x, 1);
f = abs(imgradient(x));           % edge magnitude tends to be stable
f = f / (std(f(:)) + 1e-6);
end

function S = calibrationStep(S, f, zGrid, avgSeconds, usPerUm)
if isempty(S.sumImg)
    S.sumImg = zeros(size(f), 'single');
    S.nImg = 0;
    S.offsetTic = tic;
end

S.sumImg = S.sumImg + f;
S.nImg = S.nImg + 1;

if toc(S.offsetTic) >= avgSeconds
    fAvg = S.sumImg / max(1,S.nImg);

    if isempty(S.templateImgs)
        S.templateImgs = zeros([size(fAvg) numel(zGrid)], 'single');
    end
    S.templateImgs(:,:,S.kTemplate) = fAvg;

    fprintf('[DriftCorr MVP] Template %d/%d collected (zOff=%.1f um, n=%d, %.1fs).\n', ...
        S.kTemplate, numel(zGrid), zGrid(S.kTemplate), S.nImg, avgSeconds);

    % Reset accumulator
    S.sumImg = zeros(size(f), 'single');
    S.nImg = 0;
    S.offsetTic = tic;

    % Next step or finalize
    S.kTemplate = S.kTemplate + 1;

    if S.kTemplate <= numel(zGrid)
        dz = zGrid(S.kTemplate) - S.currentOffset_um;
        mp285_moveRelXYZ(S, 0, 0, dz, usPerUm);
        S.currentOffset_um = zGrid(S.kTemplate);
    else
        % Finalize: build template vectors & pick z0 image
        S.templateVec = buildTemplateVectors(S.templateImgs);
        [~,k0] = min(abs(zGrid - 0));
        S.templateZ0Img = S.templateImgs(:,:,k0);

        S.templatesReady = true;
        S.mode = "idle";

        % Return to 0 offset
        mp285_moveRelXYZ(S, 0, 0, 0 - S.currentOffset_um, usPerUm);
        S.currentOffset_um = 0;

        fprintf('[DriftCorr MVP] Calibration complete. Templates ready.\n');
    end
end
end

function T = buildTemplateVectors(templateImgs)
[H,W,N] = size(templateImgs);
T = reshape(templateImgs, H*W, N);
T = T - mean(T,1);
T = T ./ (vecnorm(T,2,1) + 1e-9);
end

function zEst_um = estimateZfromTemplates(F, templateVec, zGrid)
v = single(F(:));
v = v - mean(v);
v = v / (norm(v) + 1e-9);
scores = templateVec.' * v;
[~,k] = max(scores);
zEst_um = zGrid(k);
end

function [Favg, nUsed] = averageLastSeconds(buf, bufT, tNow, winS)
idx = find(bufT > (tNow - winS) & bufT <= tNow);
nUsed = numel(idx);
if nUsed == 0
    Favg = [];
    return;
end
% Sum explicitly (avoids huge temporary cat(3,...) memory spikes)
Favg = zeros(size(buf(:,:,1)), 'single');
for i = 1:nUsed
    Favg = Favg + buf(:,:,idx(i));
end
Favg = Favg / nUsed;
end

function [dxPix, dyPix, movingAligned] = estimateXYtoTemplate(moving, fixed)
% Returns translation in pixels that aligns "moving" to "fixed"
% Uses imregcorr if available; otherwise uses FFT phase correlation.

try
    tform = imregcorr(moving, fixed, "translation");
    Rfixed = imref2d(size(fixed));
    movingAligned = imwarp(moving, tform, "OutputView", Rfixed);

    % For affine2d, translation is in tform.T(3,1:2) for the row-vector convention.
    % In practice, easiest is to *measure* translation by registering a delta,
    % but we’ll use T and keep sign configurable upstream.
    dxPix = tform.T(3,1);
    dyPix = tform.T(3,2);
catch
    % Fallback: basic phase correlation (integer-ish)
    [dxPix, dyPix] = phaseCorrShift(moving, fixed);
    movingAligned = imtranslate(moving, [dxPix dyPix], "OutputView","same", "FillValues",0);
end
end

function [dx, dy] = phaseCorrShift(moving, fixed)
A = fft2(moving);
B = fft2(fixed);
R = A .* conj(B);
R = R ./ (abs(R) + 1e-9);
c = real(ifft2(R));
[~, idx] = max(c(:));
[py, px] = ind2sub(size(c), idx);

% Convert peak location to shift
[H,W] = size(c);
dy = py - 1; dx = px - 1;
if dy > H/2, dy = dy - H; end
if dx > W/2, dx = dx - W; end
end

function S = detectLayer2Z(S, zNow_um)
% Collect samples; if two clusters appear, pick deeper as layer2.
if numel(S.zSeen) < 50
    S.zSeen(end+1) = zNow_um;
end
if numel(S.zSeen) >= 20
    z = sort(S.zSeen);
    z1 = median(z(1:floor(end/2)));
    z2 = median(z(floor(end/2)+1:end));
    if abs(z2 - z1) > 10
        S.layer2ZLevel = max(z1,z2);
        fprintf('[DriftCorr MVP] Detected layer2 z ~= %.1f um (deeper plane).\n', S.layer2ZLevel);
    end
end
end