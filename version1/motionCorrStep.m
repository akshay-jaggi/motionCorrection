function motionCorrStep(hSI)
%MOTIONCORRSTEP  Conservative once-per-minute XYZ motion correction callback.
%
%  ─── Phase 1 — Frame accumulation (every timer tick, ~2 Hz) ──────────────
%    Grab latest frame → quick NCC gate against z=0 reference → if accepted,
%    write into a circular ring buffer.
%
%    The NCC gate rejects frames from the "wrong" imaging plane in 2-plane
%    mode (100 µm separation → NCC with z=0 reference is typically < 0.2,
%    well below the 0.3 default threshold).
%
%  ─── Phase 2 — Correction (every correctionInterval_s, default 60 s) ─────
%    Average the ring buffer (last ~bufferDuration_s s of accepted frames).
%    Z  : NCC search over all z-reference planes → argmax + parabolic interp.
%    XY : Phase correlation of average vs. best-matching z reference image.
%    Apply MP285 correction (with deadband and per-axis safety clamp).
%
%  ─── Sign convention ──────────────────────────────────────────────────────
%    dz = −zEst  * zSign     zEst  > 0 → brain drifted in +z
%    dy = −dy_um * ySign     dy_um > 0 → image features moved UP (+y = up)
%    dx = −dx_um * xSign     dx_um > 0 → image features moved RIGHT
%
%    All signs default to +1.  If a correction makes drift worse rather than
%    better, flip the relevant sign to −1 (see sign calibration in workflow).
%
%  Globals (set by setupMotionCorrection):
%    MCORR_REF    — reference stack struct (from acquireRefStack)
%    MCORR_STATE  — mutable state: buffer, counters, parameters

global MCORR_REF MCORR_STATE
if isempty(MCORR_STATE) || ~MCORR_STATE.enabled || isempty(MCORR_REF)
    return
end
s   = MCORR_STATE;
ref = MCORR_REF;

% ================================================================
% PHASE 1 — Frame accumulation
% ================================================================

% --- Grab latest frame ------------------------------------------
try
    frame = single(grabCurrentFrame(hSI, ref.channel));
catch ME
    warning('motionCorrStep:grab', '%s', ME.message);
    MCORR_STATE = s;
    return
end

% --- NCC gate: is this frame from the reference plane? ----------
crop = frame(ref.rIdx, ref.cIdx);
mu   = mean(crop(:));
nrm  = norm(crop(:) - mu);
if nrm < 1e-6
    MCORR_STATE = s;
    return   % blank or saturated frame
end
cropNorm = single((crop - mu) / nrm);
gateNCC  = ref.refFlat(:, ref.zeroIdx)' * cropNorm(:);   % single dot product

if gateNCC >= s.minNCC_buffer
    s.bufIdx   = mod(s.bufIdx, s.bufferSize) + 1;
    s.frameBuffer(:,:,s.bufIdx) = frame;
    s.bufCount = min(s.bufCount + 1, s.bufferSize);
end

% ================================================================
% PHASE 2 — Correction cycle
% ================================================================

t_now = toc(s.tRef);
if (t_now - s.tLastCorrection) < s.correctionInterval_s || ...
        s.bufCount < s.minFramesForCorr
    MCORR_STATE = s;
    return
end

fprintf('\n[MotionCorr @ %6.0f s]  %d/%d frames buffered  Computing drift...\n', ...
    t_now, s.bufCount, s.bufferSize);

% --- Build time-averaged frame ----------------------------------
%  The ring buffer holds the last ~bufferDuration_s s of accepted frames.
%  With 2-plane imaging and NCC gating, actual coverage ≈ 2× bufferDuration_s
%  in wall-clock time — acceptable for this conservative scheme.
if s.bufCount < s.bufferSize
    avgFrame = mean(s.frameBuffer(:,:,1:s.bufCount), 3);
else
    avgFrame = mean(s.frameBuffer, 3);   % all slots valid (ring buffer full)
end

% --- Z estimation : NCC over full reference z-stack -------------
crop = avgFrame(ref.rIdx, ref.cIdx);
mu   = mean(crop(:));
nrm  = norm(crop(:) - mu);
if nrm < 1e-6
    fprintf('  Averaged frame is blank — skipping this cycle.\n');
    s.tLastCorrection = t_now;
    MCORR_STATE = s;
    return
end
cropNorm = single((crop - mu) / nrm);

corrVec = ref.refFlat' * cropNorm(:);      % [nZ × 1] single
peakNCC = max(corrVec);

if peakNCC < s.minNCC_correction
    fprintf('  Peak NCC = %.3f < %.3f threshold — image quality insufficient, skipping.\n', ...
        peakNCC, s.minNCC_correction);
    s.tLastCorrection = t_now;
    MCORR_STATE = s;
    return
end

[~, bi] = max(corrVec);
zEst    = ref.zOffsets_um(bi);

% Parabolic sub-step interpolation for Z
if bi > 1 && bi < ref.nZ
    c1 = corrVec(bi-1); c2 = corrVec(bi); c3 = corrVec(bi+1);
    d  = 2*(c1 - 2*c2 + c3);
    if abs(d) > 1e-9
        sub  = ref.zStep_um * (c1 - c3) / d;
        zEst = zEst + max(-ref.zStep_um, min(ref.zStep_um, sub));
    end
end

% --- XY estimation : phase correlation vs. best-matching z ref --
[dy_pix, dx_pix] = estimateXYShift( ...
    avgFrame, ref.rawImages{bi}, ref.rIdx, ref.cIdx, s.maxShift_pix);

dy_um = dy_pix * ref.pixelSizeXY_um(2);   % rows  → Y in µm
dx_um = dx_pix * ref.pixelSizeXY_um(1);   % cols  → X in µm

% --- Apply sign convention, deadband, and per-axis clamp --------
dz = clampDB( -zEst  * s.zSign,  s.deadband_z_um,  s.maxStep_z_um  );
dy = clampDB( -dy_um * s.ySign,  s.deadband_xy_um, s.maxStep_xy_um );
dx = clampDB( -dx_um * s.xSign,  s.deadband_xy_um, s.maxStep_xy_um );

% --- Print diagnostics ------------------------------------------
fprintf('  Z : est=%+.2f µm  corr=%+.2f µm   NCC=%.3f   ref plane %d/%d\n', ...
    zEst, dz, peakNCC, bi, ref.nZ);
fprintf('  Y : est=%+.2f µm  corr=%+.2f µm   (%.2f px)\n', dy_um, dy, dy_pix);
fprintf('  X : est=%+.2f µm  corr=%+.2f µm   (%.2f px)\n', dx_um, dx, dx_pix);

% --- Move MP285 -------------------------------------------------
if abs(dx) + abs(dy) + abs(dz) > 0
    try
        pos = hSI.hMotors.motorPosition;
        pos(1) = pos(1) + dx;
        pos(2) = pos(2) + dy;
        pos(3) = pos(3) + dz;
        hSI.hMotors.moveSample(pos);

        s.cumCorr = s.cumCorr + [dx, dy, dz];
        fprintf('  Motor moved : X%+.2f  Y%+.2f  Z%+.2f µm\n', dx, dy, dz);
        fprintf('  Cumulative  : X%+.1f   Y%+.1f   Z%+.1f µm\n', s.cumCorr);
    catch ME
        warning('motionCorrStep:move', '%s', ME.message);
    end
else
    fprintf('  All within deadband — no motor move.\n');
end

% Note: ring buffer is NOT reset after correction; it keeps rolling.
s.tLastCorrection = t_now;
MCORR_STATE = s;
end   % ── end motionCorrStep ──


% =========================================================================
function v = clampDB(v, db, maxV)
%CLAMPDB  Zero out if |v| < deadband; clamp magnitude to maxV.
if abs(v) < db
    v = 0;
else
    v = sign(v) * min(abs(v), maxV);
end
end


% =========================================================================
function [dy_pix, dx_pix] = estimateXYShift(curImg, refImg, rIdx, cIdx, maxShift_pix)
%ESTIMATEXYSHIFT  Phase-correlation lateral shift with subpixel precision.
%
%  Returns [dy_pix, dx_pix] such that:
%
%      refImg  ≈  circshift(curImg, [dy_pix, dx_pix])
%
%  Interpretation:
%    dy_pix > 0  →  features in curImg appear ABOVE reference (shifted up)
%    dy_pix < 0  →  features in curImg appear BELOW reference (shifted down)
%    dx_pix > 0  →  features in curImg appear to the RIGHT of reference
%    dx_pix < 0  →  features in curImg appear to the LEFT of reference
%
%  Steps: extract centre crop → normalise → 2D Hanning window →
%         FFT cross-correlation → fftshift → peak in ±maxShift_pix window →
%         parabolic subpixel refinement.

% Extract centre crops (doubles for FFT)
cC = double(curImg(rIdx, cIdx));
rC = double(refImg(rIdx, cIdx));
[nr, nc] = size(rC);

% Zero-mean, unit-variance normalisation
cC = (cC - mean(cC(:))) / (std(cC(:)) + eps);
rC = (rC - mean(rC(:))) / (std(rC(:)) + eps);

% 2D Hanning window (reduces spectral leakage; no toolbox required)
wr  = 0.5*(1 - cos(2*pi*(0:nr-1)'/(nr-1)));
wc  = 0.5*(1 - cos(2*pi*(0:nc-1) /(nc-1)));
win = wr * wc;
cC  = cC .* win;
rC  = rC .* win;

% FFT circular cross-correlation
%   ifft2( FFT(ref) · conj(FFT(cur)) )  →  peak at (dy,dx) means
%   ref ≈ circshift(cur,[dy,dx])
cc   = real(ifft2(fft2(rC) .* conj(fft2(cC))));
cc_s = fftshift(cc);

% Centre of fftshift output = zero-shift position
cr  = floor(nr/2) + 1;
cc_ = floor(nc/2) + 1;

% Restrict peak search to ±maxShift_pix (avoids spurious large-shift peaks)
rl = max(1,  cr  - maxShift_pix);   rh = min(nr, cr  + maxShift_pix);
cl = max(1,  cc_ - maxShift_pix);   ch = min(nc, cc_ + maxShift_pix);
sub = cc_s(rl:rh, cl:ch);

[~, idx] = max(sub(:));
[sr, sc]  = ind2sub(size(sub), idx);

pr = rl + sr - 1;              % peak row in full cc_s
pc = cl + sc - 1;              % peak col in full cc_s
dy_pix = double(pr - cr);
dx_pix = double(pc - cc_);

% Parabolic subpixel refinement — row (Y)
if pr > 1 && pr < nr
    cy    = cc_s(pr-1:pr+1, pc);
    d     = 2*(cy(1) - 2*cy(2) + cy(3));
    if abs(d) > 1e-12
        dy_pix = dy_pix + max(-1, min(1, (cy(1)-cy(3))/d));
    end
end

% Parabolic subpixel refinement — col (X)
if pc > 1 && pc < nc
    cx    = cc_s(pr, pc-1:pc+1);
    d     = 2*(cx(1) - 2*cx(2) + cx(3));
    if abs(d) > 1e-12
        dx_pix = dx_pix + max(-1, min(1, (cx(1)-cx(3))/d));
    end
end
end