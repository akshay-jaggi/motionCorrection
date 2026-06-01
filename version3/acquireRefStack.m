function refStack = acquireRefStack(hSI, varargin)
%ACQUIREREFSTACK  Build a single-plane, single-channel Z reference stack
%                 for online XYZ motion correction.
%
%  refStack = acquireRefStack(hSI, 'Name', Value, ...)
%
%  Name-Value options          Default  Notes
%  -------------------------   -------  -------------------------------------------
%  'channel'                    1        ScanImage channel to use as reference
%  'zRange_um'                 12        scan ±this around current Z (µm)
%  'zStep_um'                   1        Z step (µm); ~axial PSF / 2 is good
%  'avgDuration_s'              5        seconds averaged per Z position
%  'settleTime_s'              0.5       motor settle time after each move (s)
%  'cropFrac'                  0.6       central-crop fraction used for matching
%  'pixelSizeXY_um'            []        [µm/px_x, µm/px_y]; auto-detect if empty
%  'useFeatureImage'           true      preprocess with high-pass + gradient
%                                        magnitude (robust to activity)
%  'savePath'                  'refStack.mat'   set '' to skip saving
%
%  PREREQUISITES
%   - ScanImage in single-plane FOCUS mode at the chosen reference plane.
%   - FastZ / multi-plane imaging DISABLED while this runs.
%   - Frames available on requested channel (verify with grabCurrentFrame).
%
%  MP-285 SAFETY
%   - motorPosition is queried EXACTLY ONCE (at the start, while the stage
%     is idle). All subsequent stage targets are computed as basePos+offset
%     — we never poll the controller during the stack acquisition.
%   - moveSample commands are spaced by at least settleTime_s +
%     avgDuration_s (≥ ~5 s by default), well above any "frequent command"
%     concern. settleTime_s is floored at 0.3 s for additional safety.
%
%  The returned struct is the input to setupMotionCorrection().

p = inputParser;
addParameter(p,'channel',          1,          @isnumeric);
addParameter(p,'zRange_um',        12,         @isnumeric);
addParameter(p,'zStep_um',         1,          @isnumeric);
addParameter(p,'avgDuration_s',    5,          @isnumeric);
addParameter(p,'settleTime_s',     0.5,        @isnumeric);
addParameter(p,'cropFrac',         0.6,        @isnumeric);
addParameter(p,'pixelSizeXY_um',   [],         @(x) isempty(x)||(isnumeric(x)&&numel(x)==2));
addParameter(p,'useFeatureImage',  true,       @islogical);
addParameter(p,'savePath',         'refStack.mat', @(x) ischar(x)||isstring(x));
parse(p, varargin{:});
o = p.Results;

% MP-285 safety floor on settle time
if o.settleTime_s < 0.3
    warning('acquireRefStack:settle', ...
        'settleTime_s=%g is below the 0.3 s MP-285 safety floor; clamping to 0.3 s.', ...
        o.settleTime_s);
    o.settleTime_s = 0.3;
end

assert(strcmp(hSI.acqState,'focus'), ...
    'hSI must be in FOCUS mode at your reference plane. Run hSI.startFocus() first.');

% ---- Pixel size --------------------------------------------------
if isempty(o.pixelSizeXY_um)
    pixSizeXY = autoPixelSize(hSI);
else
    pixSizeXY = o.pixelSizeXY_um(:)';
    fprintf('  Pixel size (manual): [%.3f, %.3f] µm/px\n', pixSizeXY);
end

zOffsets = -o.zRange_um : o.zStep_um : o.zRange_um;
nZ       = numel(zOffsets);
basePos  = hSI.hMotors.motorPosition;            % [X Y Z] µm
scanFR   = hSI.hRoiManager.scanFrameRate;
framePer = 1/scanFR;

fprintf('\nReference stack:  ±%g µm | step %g µm | %d planes | ch %d\n', ...
    o.zRange_um, o.zStep_um, nZ, o.channel);
fprintf('  Avg %.1f s/plane (~%.0f frames) | settle %.2f s | est. %.1f min\n', ...
    o.avgDuration_s, o.avgDuration_s*scanFR, o.settleTime_s, ...
    nZ*(o.avgDuration_s+o.settleTime_s)/60);
fprintf('  Base motor: X=%.2f Y=%.2f Z=%.2f µm\n\n', basePos);

rawImages = cell(nZ,1);

for iz = 1:nZ
    tgt = basePos; tgt(3) = basePos(3) + zOffsets(iz);
    hSI.hMotors.moveSample(tgt);
    pause(o.settleTime_s + 3*framePer);

    acc = 0; n = 0; t0 = tic;
    while toc(t0) < o.avgDuration_s
        [f, ~, ok] = grabCurrentFrame(hSI, o.channel);
        if ok && any(f(:) ~= 0)
            acc = acc + double(f); n = n + 1;
        end
        pause(framePer*1.05);
    end
    rawImages{iz} = single(acc / max(n,1));
    fprintf('  plane %2d/%d  dz=%+5.1f µm  n=%3d  mean=%6.0f\n', ...
        iz, nZ, zOffsets(iz), n, mean(rawImages{iz}(:)));
end

hSI.hMotors.moveSample(basePos);
pause(o.settleTime_s);
fprintf('\nMotor returned to base. Building NCC vectors...\n');

% ---- Centre-crop indices ----------------------------------------
imgSz = size(rawImages{1});
dr = round(imgSz(1)*(1-o.cropFrac)/2);
dc = round(imgSz(2)*(1-o.cropFrac)/2);
rIdx = (1+dr):(imgSz(1)-dr);
cIdx = (1+dc):(imgSz(2)-dc);

% ---- Optional feature-image preprocessing (robust to activity) ---
featureImages = cell(nZ,1);
for iz = 1:nZ
    if o.useFeatureImage
        featureImages{iz} = featureImage(rawImages{iz}(rIdx,cIdx));
    else
        x = rawImages{iz}(rIdx,cIdx);
        featureImages{iz} = x - mean(x(:));
    end
end

% ---- Pre-compute normalised vectors for fast Z NCC ---------------
nPx = numel(rIdx)*numel(cIdx);
refFlat = zeros(nPx, nZ, 'single');
for iz = 1:nZ
    v = featureImages{iz}(:);
    v = v - mean(v);
    nv = norm(v); if nv > 0, v = v/nv; end
    refFlat(:,iz) = single(v);
end

[~, zeroIdx] = min(abs(zOffsets));

refStack = struct( ...
    'rawImages',      {rawImages}, ...
    'featureImages',  {featureImages}, ...
    'refFlat',        refFlat, ...
    'zOffsets_um',    zOffsets, ...
    'zeroIdx',        zeroIdx, ...
    'baseMotorPos',   basePos, ...
    'rIdx',           rIdx, ...
    'cIdx',           cIdx, ...
    'channel',        o.channel, ...
    'zStep_um',       o.zStep_um, ...
    'nZ',             nZ, ...
    'imSize',         imgSz, ...
    'pixelSizeXY_um', pixSizeXY, ...
    'useFeatureImage',o.useFeatureImage);

if ~isempty(o.savePath) && strlength(o.savePath) > 0
    save(char(o.savePath), 'refStack');
    fprintf('Saved → %s\n', o.savePath);
end
fprintf('Next: setupMotionCorrection(hSI, refStack);  then start acquisition.\n');
end


% =========================================================================
function psz = autoPixelSize(hSI)
try
    sf  = hSI.hRoiManager.currentRoiGroup.rois(1).scanfields(1);
    szXY = sf.sizeXY;
    res  = sf.pixelResolutionXY;
    psz  = szXY ./ res;
    fprintf('  Pixel size (auto): [%.3f, %.3f] µm/px | FOV [%.0f x %.0f] µm | %dx%d px\n', ...
        psz(1), psz(2), szXY(1), szXY(2), res(1), res(2));
catch
    psz = [1 1];
    warning(['autoPixelSize failed; defaulting to 1 µm/px.\n' ...
             'Pass pixelSizeXY_um manually for accurate XY correction.']);
end
end


% =========================================================================
function f = featureImage(x)
%FEATUREIMAGE  High-pass + gradient magnitude. Reduces sensitivity to
%              fluorescence activity, emphasises invariant structure.
x = single(x);
try
    x = x - imgaussfilt(x, 6);
    x = imgaussfilt(x, 1);
    f = abs(imgradient(x));
catch
    % Fallback if Image Processing Toolbox unavailable
    x = x - mean(x(:));
    [gx, gy] = gradient(double(x));
    f = single(sqrt(gx.^2 + gy.^2));
end
sd = std(f(:));
if sd > 0, f = f / sd; end
end
