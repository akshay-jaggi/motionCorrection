function [img, zNow_um, ok] = grabCurrentFrame(hSI, channel)
%GRABCURRENTFRAME  Return latest acquired frame (and its z) from ScanImage 2023.
%
%  [img, zNow_um, ok] = grabCurrentFrame(hSI, channel)
%
%  channel : ScanImage channel number (e.g. 1 or 2). Required.
%  img     : 2-D single image (empty if ok==false)
%  zNow_um : piezo/FastZ value reported for that frame (NaN if unknown)
%  ok      : true if a usable frame was returned
%
%  Three fallback methods are attempted in order. The first method is
%  preferred because it includes the per-stripe z metadata needed to
%  distinguish the two imaging planes during 2-plane FastZ imaging.

if nargin < 2, channel = 1; end
img = []; zNow_um = NaN; ok = false;

% --- Method 1: hDisplay stripeDataBuffer (gives z metadata) -----------
try
    p   = hSI.hDisplay.stripeDataBufferPointer;
    sd  = hSI.hDisplay.stripeDataBuffer{p};
    rd  = sd.roiData{1};
    chs = rd.channels;
    ci  = find(chs == channel, 1);
    if ~isempty(ci)
        im = rd.imageData{ci}{1};
        if ~isempty(im)
            img = single(im);
            if ~isempty(rd.zs), zNow_um = double(rd.zs(1)); end
            ok  = true; return;
        end
    end
catch
end

% --- Method 2: lastAcquiredStripe (no z metadata) ---------------------
try
    sd = hSI.hScan2D.hAcq.lastAcquiredStripe;
    im = sd.roiData{1}.imageData{channel}{1};
    if ~isempty(im)
        img = single(im);
        if isprop(sd.roiData{1},'zs') && ~isempty(sd.roiData{1}.zs)
            zNow_um = double(sd.roiData{1}.zs(1));
        end
        ok = true; return;
    end
catch
end

% --- Method 3: processed channel display image ------------------------
try
    im = hSI.hDisplay.channelImage{channel};
    if ~isempty(im), img = single(im); ok = true; return; end
catch
end
end
