function motionCorrUserFcn(src, evt, varargin)
%MOTIONCORRUSERFCN  ScanImage 2023 User Function for online XYZ motion
%                   correction during 2-plane / 2-channel imaging.
%
%  Attach to events: acqModeStart, acqModeDone, frameAcquired
%
%  This callback uses globals populated by setupMotionCorrection:
%      MCORR_REF    — reference stack (from acquireRefStack)
%      MCORR_STATE  — mutable buffer + parameters
%
%  Per frame ('frameAcquired'):
%    1. Read the latest frame and its piezo Z from the stripe buffer.
%    2. Keep only frames at the target plane (auto-detected on first
%       ~50 frames as the deeper of the two FastZ values, or hardcoded).
%    3. NCC-gate against the z=0 reference (rejects spurious off-plane).
%    4. Push into a ring buffer with timestamps.
%
%  Every correctionInterval_s (default 60 s):
%    5. Average buffered frames within the last avgDuration_s.
%    6. Z estimate: NCC over full reference Z stack with parabolic
%       sub-step interpolation.
%    7. XY estimate: phase correlation against the BEST-matching Z
%       reference image with parabolic subpixel refinement.
%    8. Apply sign × gain, deadband, and per-axis clamp.
%    9. Issue a single combined hSI.hMotors.moveSample command.
%
%  Axis mapping (scope-specific — MP-285 rotated 90° vs scan frame):
%      image columns (X, dx_um) → motor Y
%      image rows    (Y, dy_um) → motor X
%
%  Sign convention (after auto-/manually-set MCORR_STATE.[xyz]Sign):
%      dz       = -gainZ  * zEst  * zSign      (zSign =+1)
%      motorY   = -gainXY * dx_um * ySign      (ySign =-1, img-col → motor Y)
%      motorX   = -gainXY * dy_um * xSign      (xSign =-1, img-row → motor X)
%  With these signs, positive image drift produces a positive corrective
%  motor command on the anti-parallel axis (reverses the drift).

global MCORR_REF MCORR_STATE MCORR_FRAMEBUF MCORR_FRAMETIMES MCORR_DIAG
if isempty(MCORR_STATE) || ~MCORR_STATE.enabled || isempty(MCORR_REF), return; end

hSI = src.hSI;
s   = MCORR_STATE;
ref = MCORR_REF;

switch evt.EventName
    case 'acqModeStart'
        s.tRef            = tic;
        s.tLastCorrection = -inf;
        s.tLastMove       = -inf;
        s.lastDz_um       = 0;
        s.bufIdx          = 0;
        s.bufCount        = 0;
        s.zSeen           = [];
        s.cumCorr         = [0 0 0];
        % NOTE: samplePosition is NOT queried here. It was cached during
        % setupMotionCorrection (stage idle, no SI pressure). Querying the
        % MP-285 inside acqModeStart can take 100s of ms and delays SI's
        % own pipeline init, causing the first frame to drop.
        if isempty(s.cachedMotorPos)
            warning('motionCorrUserFcn:noBase', ...
                'No cached motor position. Re-run setupMotionCorrection while stage is idle.');
            s.enabled = false;
        end
        if s.diag
            MCORR_DIAG.nFrames     = 0;
            MCORR_DIAG.nProcessed  = 0;
            MCORR_DIAG.idx         = 0;
            MCORR_DIAG.peakTotal_ms= 0;
            MCORR_DIAG.peakWhen    = 0;
        end
        if s.verbose
            fprintf('[MotionCorr] Acquisition started%s\n', ...
                ternary_(s.dryRun, ' (DRY RUN — no stage moves)', ''));
            fprintf('[MotionCorr]   first correction in ~%.0f s; per-frame work only in last %.1f s of each interval.\n', ...
                s.correctionInterval_s, s.avgDuration_s + s.collectMargin_s);
        end
        MCORR_STATE = s; return;

    case 'acqModeDone'
        if s.verbose
            fprintf('[MotionCorr] Acquisition ended.  Cumulative correction:  X%+.1f  Y%+.1f  Z%+.1f µm\n', ...
                s.cumCorr);
        end
        if s.diag && ~isempty(MCORR_DIAG) && MCORR_DIAG.idx > 0
            n = min(MCORR_DIAG.idx, numel(MCORR_DIAG.tTotal));
            fprintf(['[MotionCorr]   diagnostics:  %d frames seen, %d processed past gate\n' ...
                     '                 last %d processed-frame timings (ms):\n' ...
                     '                   grab    : mean %.2f  max %.2f\n' ...
                     '                   feature : mean %.2f  max %.2f\n' ...
                     '                   buffer  : mean %.2f  max %.2f\n' ...
                     '                   TOTAL   : mean %.2f  max %.2f\n' ...
                     '                 peak callback ever: %.2f ms at t=%.1fs\n'], ...
                MCORR_DIAG.nFrames, MCORR_DIAG.nProcessed, n, ...
                1e3*mean(MCORR_DIAG.tGrab(1:n)),    1e3*max(MCORR_DIAG.tGrab(1:n)), ...
                1e3*mean(MCORR_DIAG.tFeature(1:n)), 1e3*max(MCORR_DIAG.tFeature(1:n)), ...
                1e3*mean(MCORR_DIAG.tBuffer(1:n)),  1e3*max(MCORR_DIAG.tBuffer(1:n)), ...
                1e3*mean(MCORR_DIAG.tTotal(1:n)),   1e3*max(MCORR_DIAG.tTotal(1:n)), ...
                MCORR_DIAG.peakTotal_ms, MCORR_DIAG.peakWhen);
        end
        MCORR_STATE = s; return;

    case 'frameAcquired'
        % fall through

    otherwise
        return;
end

% ================================================================
% FAST PATH — collection window gate
% ================================================================
% Skip ALL per-frame work outside the collection window leading up to
% the next correction. Default reduces processed frames by ~12×.
% This is intentionally placed BEFORE grabCurrentFrame so the callback
% returns in microseconds for ~92% of frames.
if s.diag
    tCallback0 = tic;
    MCORR_DIAG.nFrames = MCORR_DIAG.nFrames + 1;
    if MCORR_DIAG.nFrames <= 3
        % Skip the first 3 frames entirely — SI pipeline is still warming.
        MCORR_STATE = s; return;
    end
else
    if s.bufCount == 0 && s.bufIdx == 0
        % no-op (kept symmetric)
    end
end

tNow_approx       = toc(s.tRef);
timeUntilNextCorr = s.correctionInterval_s - (tNow_approx - s.tLastCorrection);
inCollectWindow   = timeUntilNextCorr <= (s.avgDuration_s + s.collectMargin_s);
if ~inCollectWindow
    MCORR_STATE = s; return;   % fast path — nothing else runs
end

% ================================================================
% PHASE 1 — frame intake (only runs inside collection window)
% ================================================================
if s.diag, tPhase = tic; end
[frame, zNow_um, ok] = grabCurrentFrame(hSI, ref.channel);
if s.diag
    tGrab = toc(tPhase);
end
if ~ok
    if s.diag, logDiag_(MCORR_DIAG, tGrab, 0, 0, toc(tCallback0), tNow_approx); end
    return;
end

% Auto-detect target plane Z if not specified
if isempty(s.targetPlaneZ_um)
    if numel(s.zSeen) < 60 && ~isnan(zNow_um)
        s.zSeen(end+1) = zNow_um;
    end
    if numel(s.zSeen) >= 20 && isempty(s.targetPlaneZ_um)
        zs = sort(s.zSeen(:));
        m  = floor(numel(zs)/2);
        z1 = median(zs(1:m));
        z2 = median(zs(m+1:end));
        if abs(z2 - z1) > 5      % two distinct planes seen
            s.targetPlaneZ_um = max(z1, z2);     % deeper one
            if s.verbose
                fprintf('[MotionCorr] Auto-detected target plane: z = %.2f µm (deeper of %.2f / %.2f)\n', ...
                    s.targetPlaneZ_um, z1, z2);
            end
        elseif numel(s.zSeen) >= 60
            % Only one plane ever seen — use it
            s.targetPlaneZ_um = median(zs);
            if s.verbose
                fprintf('[MotionCorr] Single plane detected: z = %.2f µm — using it.\n', ...
                    s.targetPlaneZ_um);
            end
        end
    end
end

% Gate by plane (skip non-reference plane in 2-plane mode)
if ~isempty(s.targetPlaneZ_um) && ~isnan(zNow_um)
    if abs(zNow_um - s.targetPlaneZ_um) > 1
        MCORR_STATE = s; return;
    end
end

% Crop + (optional) feature image for matching
if s.diag, tPhase = tic; end
crop = frame(ref.rIdx, ref.cIdx);
if ref.useFeatureImage
    feat = featureImage(crop);
else
    feat = crop;
end
if s.diag, tFeature = toc(tPhase); tPhase = tic; end
fv = single(feat(:));
fv = fv - mean(fv);
nv = norm(fv);
if nv < 1e-6
    if s.diag
        tBuffer = toc(tPhase);
        logDiag_(MCORR_DIAG, tGrab, tFeature, tBuffer, toc(tCallback0), tNow_approx);
    end
    MCORR_STATE = s; return;
end
fvN = fv / nv;

% NCC gate: is this frame really at the reference plane?
gateNCC = ref.refFlat(:, ref.zeroIdx)' * fvN;
if gateNCC < s.minNCC_buffer
    if s.diag
        tBuffer = toc(tPhase);
        logDiag_(MCORR_DIAG, tGrab, tFeature, tBuffer, toc(tCallback0), tNow_approx);
    end
    MCORR_STATE = s; return;
end

% Push to ring buffer (store the cropped feature, not the full raw frame)
tNow = tNow_approx;
s.bufIdx   = mod(s.bufIdx, s.bufferSize) + 1;
% Resize global buffer on first use if feature image is smaller than imSize
% (should only happen once per session after first valid frame)
if size(MCORR_FRAMEBUF,1) ~= size(feat,1) || size(MCORR_FRAMEBUF,2) ~= size(feat,2)
    MCORR_FRAMEBUF   = zeros([size(feat), s.bufferSize], 'single');
    MCORR_FRAMETIMES = zeros(1, s.bufferSize);
end
% In-place indexed write to top-level global — no struct COW, no 113 MB copy.
MCORR_FRAMEBUF(:,:,s.bufIdx) = feat;
MCORR_FRAMETIMES(s.bufIdx)   = tNow;
s.bufCount = min(s.bufCount + 1, s.bufferSize);
if s.diag
    tBuffer = toc(tPhase);
    logDiag_(MCORR_DIAG, tGrab, tFeature, tBuffer, toc(tCallback0), tNow);
end

% ================================================================
% PHASE 2 — correction cycle
% ================================================================
if (tNow - s.tLastCorrection) < s.correctionInterval_s, MCORR_STATE = s; return; end
% MP285 safety: hard floor on inter-command spacing, independent of
% correctionInterval_s (which a user could lower unsafely).
if (tNow - s.tLastCorrection) < s.minMoveInterval_s,    MCORR_STATE = s; return; end
% Non-blocking post-move quiet period: suppress the next correction
% for postMoveQuiet_s after a moveSample so the controller settles.
% (Never use pause() inside a frameAcquired callback — it blocks
% ScanImages acquisition pipeline and causes frame drops.)
if (tNow - s.tLastMove) < s.postMoveQuiet_s,             MCORR_STATE = s; return; end
if s.bufCount < s.minFramesForCorr,                      MCORR_STATE = s; return; end
% Cannot command moves if we never got a baseline position.
if isempty(s.cachedMotorPos),                            MCORR_STATE = s; return; end

% Average frames whose timestamps fall within the last avgDuration_s
mask = MCORR_FRAMETIMES(1:s.bufferSize) > (tNow - s.avgDuration_s) & ...
       MCORR_FRAMETIMES(1:s.bufferSize) <= tNow;
mask = mask(:);
% Restrict to slots actually written
written = false(s.bufferSize,1);
if s.bufCount < s.bufferSize
    written(1:s.bufCount) = true;
else
    written(:) = true;
end
mask = mask & written;
nUsed = nnz(mask);
if nUsed < max(3, round(s.minFramesForCorr/2))
    MCORR_STATE = s; return;
end
avgFrame = mean(MCORR_FRAMEBUF(:,:,mask), 3);

if s.verbose
    fprintf('\n[MotionCorr @ %5.0f s]  %d frames in last %.0f s — computing drift...\n', ...
        tNow, nUsed, s.avgDuration_s);
end

% --- Z estimate: NCC over full reference stack ------------------
% XY-first ordering (default): first phase-correlate against the z=0
% reference image to get an approximate XY shift, integer-shift the
% avg frame to remove gross XY drift, then run Z NCC on the aligned
% image. Phase correlation is far more tolerant of Z mismatch than NCC
% magnitude is to XY shift, so this single pre-alignment dramatically
% sharpens the Z NCC peak and eliminates the cross-axis ambiguity that
% causes alternating max-step Z oscillation and one-sided runaway.
if s.xyFirst
    [dy0, dx0] = phaseCorrShift(avgFrame, ref.featureImages{ref.zeroIdx}, s.maxShift_pix);
    zSearchFrame = circshift(avgFrame, [round(dy0), round(dx0)]);
else
    zSearchFrame = avgFrame;
end

v = zSearchFrame(:); v = v - mean(v); nv = norm(v);
if nv < 1e-6, s.tLastCorrection = tNow; MCORR_STATE = s; return; end
v = single(v / nv);
corrVec = ref.refFlat' * v;                 % [nZ × 1]
[peakNCC, bi] = max(corrVec);

% --- Z confidence checks ----------------------------------------
% (1) Stack-boundary rejection: argmax at index 1 or nZ means the true
%     Z is outside the calibrated range and the parabolic refinement is
%     ill-defined. Skip Z (but still allow XY) and warn.
zAtBoundary = (bi == 1 || bi == ref.nZ);

% (2) Peak unambiguity: require the global peak to exceed the best
%     runner-up OUTSIDE a ±2-step neighborhood by at least zPeakMargin.
%     Catches the bistable-stack case that drives alternating max-step
%     oscillation.
nbhd = false(ref.nZ,1);
nbhd(max(1,bi-2):min(ref.nZ,bi+2)) = true;
runner = max(corrVec(~nbhd));
if isempty(runner), runner = -inf; end
peakMargin = peakNCC - runner;
zAmbiguous = peakMargin < s.zPeakMargin;

if peakNCC < s.minNCC_correction
    if s.verbose
        fprintf('  peak NCC %.3f < %.3f — skipping cycle entirely.\n', peakNCC, s.minNCC_correction);
    end
    s.tLastCorrection = tNow; MCORR_STATE = s; return;
end

zEst = ref.zOffsets_um(bi);
% Parabolic sub-step Z refinement (skip at boundary)
if bi > 1 && bi < ref.nZ
    c1 = corrVec(bi-1); c2 = corrVec(bi); c3 = corrVec(bi+1);
    d  = 2*(c1 - 2*c2 + c3);
    if abs(d) > 1e-9
        sub  = ref.zStep_um * (c1 - c3) / d;
        zEst = zEst + max(-ref.zStep_um, min(ref.zStep_um, sub));
    end
end

trustZ = ~zAtBoundary && ~zAmbiguous;

% --- XY estimate (refine): phase correlation vs the z=0 reference -
% Use ref.zeroIdx (NOT ref.featureImages{bi}). The z=0 reference defines
% the operational rest position; every other Z slice in the stack carries
% its own small XY offset from sample tilt, piezo-axis misalignment, and
% optical aberrations during the z-sweep. Refining against ref{bi} bakes
% those per-slice offsets into the XY command and walks the stage away
% from the true rest XY whenever bi drifts off ref.zeroIdx. Phase
% correlation is robust to the structural difference between focal planes
% at small Z offsets, so refining against ref{zeroIdx} is both correct
% (preserves the XY zero) and reliable.
refImg = ref.featureImages{ref.zeroIdx};
[dy_pix, dx_pix] = phaseCorrShift(avgFrame, refImg, s.maxShift_pix);
dx_um = dx_pix * ref.pixelSizeXY_um(1);    % cols  → image X
dy_um = dy_pix * ref.pixelSizeXY_um(2);    % rows  → image Y

% --- Apply sign, gain, deadband, clamp --------------------------
% NOTE: image cols (dx_um) drive motor Y; image rows (dy_um) drive motor X.
% xSign/ySign are -1 on this scope (each image axis is anti-parallel to its
% motor axis); zSign is +1.
if trustZ
    dz = clampDB( -s.gainZ  * zEst  * s.zSign,  s.deadband_z_um,  s.maxStep_z_um  );
else
    dz = 0;
end
dy = clampDB( -s.gainXY * dx_um * s.ySign,  s.deadband_xy_um, s.maxStep_xy_um );  % img-col → motorY
dx = clampDB( -s.gainXY * dy_um * s.xSign,  s.deadband_xy_um, s.maxStep_xy_um );  % img-row → motorX

% --- Anti-oscillation guard (Z only) ---------------------------
% If the previous Z move was clamped at max AND this cycle wants to
% move at full clamp in the OPPOSITE direction, suppress this Z move.
% One stalled cycle is far better than an infinite ping-pong; if the
% reversal is genuine it will still command again next cycle.
zClamp = s.maxStep_z_um - 1e-6;
if dz ~= 0 && abs(s.lastDz_um) >= zClamp && abs(dz) >= zClamp && sign(dz) ~= sign(s.lastDz_um)
    if s.verbose
        fprintf('  *** Z oscillation guard: prev %+.2f µm, this %+.2f µm — suppressing.\n', ...
            s.lastDz_um, dz);
    end
    dz = 0;
end

if s.verbose
    zStatus = '';
    if zAtBoundary, zStatus = [zStatus ' [boundary]']; end
    if zAmbiguous,  zStatus = [zStatus sprintf(' [ambig margin=%.3f<%.3f]', peakMargin, s.zPeakMargin)]; end
    fprintf('  Z:      est %+.2f µm  cmd %+.2f  NCC=%.3f (margin %+.3f) ref %d/%d%s\n', ...
        zEst, dz, peakNCC, peakMargin, bi, ref.nZ, zStatus);
    fprintf('  motorY (imgX): est %+.2f µm  cmd %+.2f  (%+.2f px)\n', dx_um, dy, dx_pix);
    fprintf('  motorX (imgY): est %+.2f µm  cmd %+.2f  (%+.2f px)\n', dy_um, dx, dy_pix);
end

% --- Move stage --------------------------------------------------
if abs(dx)+abs(dy)+abs(dz) > 0
    if s.dryRun
        if s.verbose
            fprintf('  [DRY RUN] would move X%+.2f Y%+.2f Z%+.2f µm — not commanded.\n', dx, dy, dz);
        end
        s.tLastMove = tNow;   % still respect quiet-period gate
        s.lastDz_um = dz;     % track in dry run too for accurate diagnostics
    else
        try
            % MP285 safety: do NOT query samplePosition here. Use the cached
            % position (snapshot at setup, incremented by every commanded
            % delta). Querying the controller is exactly what the ScanImage
            % docs warn can lock it up.
            pos    = s.cachedMotorPos;
            newPos = pos + [dx dy dz];
            hSI.hMotors.moveSample(newPos);
            % Record move timestamp for the non-blocking quiet-period gate
            % at the top of Phase 2. Do NOT pause() here — blocking inside
            % a frameAcquired callback causes ScanImage frame drops.
            s.tLastMove      = tNow;
            s.cachedMotorPos = newPos;        % only update on success
            s.cumCorr        = s.cumCorr + [dx dy dz];
            s.lastDz_um      = dz;            % anti-oscillation tracker
            if s.verbose
                fprintf('  motor moved  X%+.2f Y%+.2f Z%+.2f µm  | cumulative X%+.1f Y%+.1f Z%+.1f\n', ...
                    dx, dy, dz, s.cumCorr);
            end
        catch ME
            warning('motionCorrUserFcn:move', '%s', ME.message);
        end
    end
elseif s.verbose
    fprintf('  all within deadband — no move.\n');
end

s.tLastCorrection = tNow;
MCORR_STATE = s;
end


% =========================================================================
function v = clampDB(v, db, maxV)
if abs(v) < db, v = 0; else, v = sign(v)*min(abs(v),maxV); end
end


% =========================================================================
function logDiag_(~, tGrab, tFeature, tBuffer, tTotal, tWhen)
% Append timings to MCORR_DIAG ring buffer (last 200 processed frames)
% and update the peak-callback tracker. Direct globals avoid COW.
global MCORR_DIAG
if isempty(MCORR_DIAG), return; end
N = numel(MCORR_DIAG.tTotal);
MCORR_DIAG.idx        = mod(MCORR_DIAG.idx, N) + 1;
MCORR_DIAG.nProcessed = MCORR_DIAG.nProcessed + 1;
MCORR_DIAG.tGrab(MCORR_DIAG.idx)    = single(tGrab);
MCORR_DIAG.tFeature(MCORR_DIAG.idx) = single(tFeature);
MCORR_DIAG.tBuffer(MCORR_DIAG.idx)  = single(tBuffer);
MCORR_DIAG.tTotal(MCORR_DIAG.idx)   = single(tTotal);
if 1e3*tTotal > MCORR_DIAG.peakTotal_ms
    MCORR_DIAG.peakTotal_ms = 1e3*tTotal;
    MCORR_DIAG.peakWhen     = tWhen;
end
end


% =========================================================================
function s = ternary_(c, a, b)
if c, s = a; else, s = b; end
end


% =========================================================================
function f = featureImage(x)
x = single(x);
try
    x = x - imgaussfilt(x, 6);
    x = imgaussfilt(x, 1);
    f = abs(imgradient(x));
catch
    x = x - mean(x(:));
    [gx, gy] = gradient(double(x));
    f = single(sqrt(gx.^2 + gy.^2));
end
sd = std(f(:));
if sd > 0, f = f / sd; end
end


% =========================================================================
function [dy_pix, dx_pix] = phaseCorrShift(curImg, refImg, maxShift_pix)
%PHASECORRSHIFT  Hanning-windowed phase correlation with parabolic subpixel.
%  Returns shift such that refImg ≈ circshift(curImg, [dy_pix, dx_pix]).

cC = double(curImg); rC = double(refImg);
[nr, nc] = size(rC);

cC = (cC - mean(cC(:))) / (std(cC(:)) + eps);
rC = (rC - mean(rC(:))) / (std(rC(:)) + eps);

wr  = 0.5*(1 - cos(2*pi*(0:nr-1)'/(nr-1)));
wc  = 0.5*(1 - cos(2*pi*(0:nc-1) /(nc-1)));
win = wr * wc;
cC = cC .* win; rC = rC .* win;

cc   = real(ifft2(fft2(rC) .* conj(fft2(cC))));
cc_s = fftshift(cc);
cr  = floor(nr/2)+1; cc_= floor(nc/2)+1;

rl = max(1, cr - maxShift_pix); rh = min(nr, cr + maxShift_pix);
cl = max(1, cc_- maxShift_pix); ch = min(nc, cc_+ maxShift_pix);
sub = cc_s(rl:rh, cl:ch);
[~, idx] = max(sub(:));
[sr, sc] = ind2sub(size(sub), idx);
pr = rl + sr - 1; pc = cl + sc - 1;
dy_pix = double(pr - cr); dx_pix = double(pc - cc_);

% Parabolic subpixel
if pr > 1 && pr < nr
    cy = cc_s(pr-1:pr+1, pc); d = 2*(cy(1)-2*cy(2)+cy(3));
    if abs(d) > 1e-12, dy_pix = dy_pix + max(-1,min(1,(cy(1)-cy(3))/d)); end
end
if pc > 1 && pc < nc
    cx = cc_s(pr, pc-1:pc+1); d = 2*(cx(1)-2*cx(2)+cx(3));
    if abs(d) > 1e-12, dx_pix = dx_pix + max(-1,min(1,(cx(1)-cx(3))/d)); end
end
end
