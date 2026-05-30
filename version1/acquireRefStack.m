function refStack = acquireRefStack(hSI, varargin)
%ACQUIREREFSTACK  Build a reference z-stack for online XYZ motion correction.
%
%  refStack = acquireRefStack(hSI)
%  refStack = acquireRefStack(hSI, 'Name', Value, ...)
%
%  Name-Value options          Default  Notes
%  -------------------------   -------  -------------------------------------------
%  'zRange_um'                  30       scan ±this from base position (µm)
%  'zStep_um'                    2       step size (µm)
%  'avgDuration_s'               5       seconds of frames averaged per position
%  'channel'                     1       PMT channel
%  'cropFrac'                    0.5     centre-crop fraction for NCC computation
%  'settleTime_s'                0.5     motor settle time per step (s)
%  'pixelSizeXY_um'             []       [µm/px_x, µm/px_y]; auto-detect if empty
%
%  PREREQUISITES
%    1. Set ScanImage to SINGLE-PLANE focus (disable FastZ / volume imaging).
%    2. hSI.startFocus() — navigate to your TARGET plane (this becomes z = 0).
%    3. Run grabCurrentFrame(hSI,1) to confirm frame access before starting.

p = inputParser;
addParameter(p, 'zRange_um',      30,   @isnumeric);
addParameter(p, 'zStep_um',        2,   @isnumeric);
addParameter(p, 'avgDuration_s',   5,   @isnumeric);
addParameter(p, 'channel',         1,   @isnumeric);
addParameter(p, 'cropFrac',       0.5,  @isnumeric);
addParameter(p, 'settleTime_s',   0.5,  @isnumeric);
addParameter(p, 'pixelSizeXY_um',  [],  @(x) isempty(x)||(isnumeric(x)&&numel(x)==2));
parse(p, varargin{:});
o = p.Results;

assert(strcmp(hSI.acqState,'focus'), ...
    'hSI must be in focus mode. Call hSI.startFocus() first.');

% ---- Pixel size --------------------------------------------------
if isempty(o.pixelSizeXY_um)
    pixSizeXY = autoPixelSize(hSI);
else
    pixSizeXY = o.pixelSizeXY_um(:)';
    fprintf('  Pixel size (manual): [%.3f, %.3f] µm/pixel\n', pixSizeXY);
end

% ---- Setup -------------------------------------------------------
zOffsets   = -o.zRange_um : o.zStep_um : o.zRange_um;
nZ         = numel(zOffsets);
basePosXYZ = hSI.hMotors.motorPosition;        % [X, Y, Z] in µm
scanFR     = hSI.hRoiManager.scanFrameRate;
framePer   = 1 / scanFR;

fprintf('\nAcquiring reference z-stack:\n');
fprintf('  ±%.0f µm | step %.0f µm | %d planes | channel %d\n', ...
    o.zRange_um, o.zStep_um, nZ, o.channel);
fprintf('  Averaging %.1f s (~%.0f frames) per position | settle %.2f s\n', ...
    o.avgDuration_s, o.avgDuration_s * scanFR, o.settleTime_s);
fprintf('  Estimated time: ~%.1f min\n', ...
    nZ * (o.avgDuration_s + o.settleTime_s) / 60);
fprintf('  Base motor: X=%.2f  Y=%.2f  Z=%.2f µm\n\n', basePosXYZ);

rawImages = cell(nZ, 1);

% ---- Step through z positions ------------------------------------
for iz = 1:nZ
    % Move motor to z offset
    tgt    = basePosXYZ;
    tgt(3) = basePosXYZ(3) + zOffsets(iz);
    hSI.hMotors.moveSample(tgt);
    pause(o.settleTime_s + 3*framePer);   % settle + flush scan pipeline

    % Accumulate frames for avgDuration_s seconds
    acc   = 0;
    nAcc  = 0;
    t0    = tic;
    while toc(t0) < o.avgDuration_s
        f = double(grabCurrentFrame(hSI, o.channel));
        if any(f(:) ~= 0)          % skip blank frames during flush
            acc  = acc + f;
            nAcc = nAcc + 1;
        end
        pause(framePer * 1.05);    % wait just over one frame period
    end

    rawImages{iz} = single(acc / max(nAcc, 1));

    fprintf('  Plane %2d/%d  dz=%+5.1f µm  n=%3d frames  mean=%6.0f cts\n', ...
        iz, nZ, zOffsets(iz), nAcc, mean(rawImages{iz}(:)));
end

% Return to base
hSI.hMotors.moveSample(basePosXYZ);
pause(o.settleTime_s);
fprintf('\nMotor returned to base. Post-processing...\n');

% ---- Image dimensions and centre-crop indices --------------------
imgSz = size(rawImages{1});                     % [rows, cols]
dr    = round(imgSz(1) * (1 - o.cropFrac) / 2);
dc    = round(imgSz(2) * (1 - o.cropFrac) / 2);
rIdx  = (1+dr):(imgSz(1)-dr);
cIdx  = (1+dc):(imgSz(2)-dc);
nPx   = numel(rIdx) * numel(cIdx);

% ---- Pre-compute normalised crop vectors for Z NCC search --------
%  refFlat: [nPx × nZ] — each column is zero-mean, unit-norm
refFlat = zeros(nPx, nZ, 'single');
for iz = 1:nZ
    p  = single(rawImages{iz}(rIdx, cIdx));
    p  = p - mean(p(:));
    n  = norm(p(:));
    if n > 0, p = p / n; end
    refFlat(:, iz) = p(:);
end

% ---- Index of z=0 plane ------------------------------------------
[~, zeroIdx] = min(abs(zOffsets));

% ---- Pack output -------------------------------------------------
refStack.rawImages      = rawImages;       % {nZ × 1} cell of single images — for XY corr
refStack.refFlat        = refFlat;         % [nPx × nZ] — for Z NCC search
refStack.zOffsets_um    = zOffsets;
refStack.zeroIdx        = zeroIdx;
refStack.baseMotorPos   = basePosXYZ;
refStack.rIdx           = rIdx;
refStack.cIdx           = cIdx;
refStack.channel        = o.channel;
refStack.zStep_um       = o.zStep_um;
refStack.nZ             = nZ;
refStack.imSize         = imgSz;
refStack.pixelSizeXY_um = pixSizeXY;      % [µm/px_x, µm/px_y]

save('refStack.mat', 'refStack');
fprintf('Saved → refStack.mat\n');
fprintf('Next: [t, stopFcn] = setupMotionCorrection(hSI, refStack)\n');
end


% =========================================================================
function psz = autoPixelSize(hSI)
%AUTOPIXELSIZE  Attempt to read pixel size from ScanImage 2023 ROI manager.
try
    sf   = hSI.hRoiManager.currentRoiGroup.rois(1).scanfields(1);
    szXY = sf.sizeXY;                  % [x, y] in µm (ScanImage reference space)
    res  = sf.pixelResolutionXY;       % [nx, ny] pixels
    psz  = szXY ./ res;
    fprintf('  Pixel size (auto): [%.3f, %.3f] µm/pixel | FOV [%.0f × %.0f] µm | %d×%d px\n', ...
        psz(1), psz(2), szXY(1), szXY(2), res(1), res(2));
catch
    psz = [1, 1];
    warning(['autoPixelSize: could not read pixel size from ScanImage.\n', ...
             'Defaulting to 1 µm/pixel — XY corrections will be inaccurate.\n', ...
             'Specify manually: acquireRefStack(hSI, ''pixelSizeXY_um'', [dx_um, dy_um])']);
end
end