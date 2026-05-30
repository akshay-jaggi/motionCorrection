function img = grabCurrentFrame(hSI, channel)
%GRABCURRENTFRAME  Return the latest acquired frame from ScanImage 2023.
%
%  img = grabCurrentFrame(hSI)          % channel 1 (default)
%  img = grabCurrentFrame(hSI, 2)       % PMT channel 2
%
%  Three fallback methods are attempted in order.
%  If all fail, inspect hSI.hScan2D.hAcq and hSI.hDisplay in the
%  workspace to locate the correct path for your build.

if nargin < 2, channel = 1; end

% --- Method 1: Directly from the acquisition engine (preferred) -------
try
    sd  = hSI.hScan2D.hAcq.lastAcquiredStripe;
    img = sd.roiData{1}.imageData{channel}{1};
    if ~isempty(img), return; end
catch
end

% --- Method 2: Rolling display stripe buffer --------------------------
try
    buf = hSI.hDisplay.stripeDataBuffer;
    for k = numel(buf):-1:1
        if ~isempty(buf(k)) && ~isempty(buf(k).roiData)
            img = buf(k).roiData{1}.imageData{channel}{1};
            if ~isempty(img), return; end
        end
    end
catch
end

% --- Method 3: Processed channel display image (last resort) ----------
try
    img = hSI.hDisplay.channelImage{channel};
    if ~isempty(img), return; end
catch
end

error(['grabCurrentFrame: all access paths failed.\n', ...
       'While in focus mode, inspect:\n', ...
       '  hSI.hScan2D.hAcq.lastAcquiredStripe.roiData{1}.imageData\n', ...
       '  hSI.hDisplay\n', ...
       'and edit grabCurrentFrame.m to match your SI 2023 build.']);
end