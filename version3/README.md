# Version 3 — Online XYZ Motion Correction for ScanImage 2023 + MP-285

This folder contains a single, combined implementation that supersedes
[version1/](../version1) and [version2/](../version2). It performs once-per-minute
XYZ drift correction during 2-plane / 2-channel two-photon imaging, using **one**
plane and **one** channel as the reference.

---

## 1. Why a v3?

Both prior versions implement the same core scheme — a Z reference stack of
locally-averaged templates, NCC-based Z search, XY phase correlation, and a
conservative once-per-minute corrective move with deadband + clamp safety.
They differ along several axes; v3 takes the better choice from each:

| Decision | v1 | v2 | **v3 (chosen)** |
|---|---|---|---|
| Acquisition driver | MATLAB `timer` polling frames | ScanImage **User Function** (`frameAcquired`) | **User Function** — synchronised with imaging, no race conditions |
| Stage commands | `hSI.hMotors.moveSample` (ScanImage abstraction) | Direct serial bytes to MP-285 | **`hSI.hMotors.moveSample`** — no hardcoded MP-285 protocol/microstep scale |
| Plane gating (2-plane mode) | NCC-only against z=0 reference | Uses per-frame `rd.zs` z metadata | **z metadata + NCC double-gate** — explicit plane id, NCC catches occasional noise |
| XY image | Raw cropped frame | High-pass + gradient magnitude "feature image" | **Feature image** — robust to fluorescence activity changes |
| XY reference for matching | Best-matching Z plane | z=0 plane only | **Best-matching Z plane** — XY is correct even when sample drifted axially |
| Subpixel refinement | Parabolic in both X, Y, Z | Only via `imregcorr` | **Parabolic everywhere** — no toolbox dependency, well-conditioned |
| Pixel size | Auto-detect from ROI manager + override | Manual constant | **Auto-detect + override** |
| Reference acquisition | Standalone single-plane focus session | Triggered inside `focusStart` callback | **Standalone** — clearer separation, easier to repeat / verify |
| Proportional gain | None (full correction) | `KZ`, `KXY` | **Tunable per axis** (default 0.5/0.7) — soft step-toward-target avoids oscillation |
| Persistence | Saves `refStack.mat` | In-memory only | **Saves `refStack.mat`** |
| Frame access | Three-method fallback | One method | **Three-method fallback** (with z metadata first) |

The result is a single user-function callback (`motionCorrUserFcn.m`) plus a
small calibration helper (`acquireRefStack.m`) and a setup function
(`setupMotionCorrection.m`).

---

## 2. Files

- [`acquireRefStack.m`](acquireRefStack.m) — collect Z-stack templates (single-plane, single-channel).
- [`setupMotionCorrection.m`](setupMotionCorrection.m) — configure parameters and globals.
- [`motionCorrUserFcn.m`](motionCorrUserFcn.m) — ScanImage User Function callback.
- [`grabCurrentFrame.m`](grabCurrentFrame.m) — robust frame + Z-metadata accessor.
- [`workflow_v3.m`](workflow_v3.m) — interactive walkthrough script.

---

## 3. One-time setup steps

### 3.1 Install
Place the five `.m` files on your MATLAB path (or `addpath` the folder).

### 3.2 Verify frame access
With ScanImage running, in single-plane FOCUS:

```matlab
[img, z, ok] = grabCurrentFrame(hSI, 2);    % 2 = your reference channel
```

If `ok == false`, inspect `hSI.hDisplay.stripeDataBuffer` and edit
`grabCurrentFrame.m` to match your build.

### 3.3 Determine pixel size (µm/pixel) for X and Y
Two options:
- **Auto-detect** (default in `acquireRefStack`): reads
  `hSI.hRoiManager.currentRoiGroup.rois(1).scanfields(1).sizeXY` and
  `pixelResolutionXY`. Verify that the printed values look right.
- **Manual**: image a calibrated graticule or 10/15 µm beads on a slide,
  measure the pixel separation between two features at known physical
  separation, and pass `'pixelSizeXY_um', [dx, dy]` to `acquireRefStack`.

These values can differ between X and Y if your scan is non-square.

### 3.4 Determine motor sign conventions (X, Y, Z)
This must be done **once per scope** because it depends on how the MP-285
is mounted relative to the scan/objective coordinate frame. Two methods:

**A. Static deliberate offset (preferred — 5 min)**
1. In single-plane FOCUS, image a recognisable feature near image centre.
2. Move the stage by a small known amount along **X** only:
   ```matlab
   p = hSI.hMotors.motorPosition;
   hSI.hMotors.moveSample(p + [+5 0 0]);    % +5 µm in motor X
   ```
3. Observe the image: did features shift left, right, up, or down?
   - If +X motor → features move **left** in image (this is the common case),
     then the algorithm's default `xSign = +1` is correct.
   - If +X motor → features move **right**, set `xSign = -1`.
   - If +X motor → features move along the **Y axis** of the image, your
     motor is rotated 90° relative to the scan; you'll need to swap which
     correction goes to which motor axis (edit `motionCorrUserFcn.m`
     accordingly, or rotate the calibrated pixel-size axes).
4. Repeat for Y, then for Z (use `zRange_um` + or - 5 µm).

**B. Live calibration during operation**
Start acquisition with all signs at `+1`. After the first correction or two,
watch the cumulative sum:
- If `cumCorr` is shrinking the absolute value of natural drift → signs OK.
- If it is growing (or oscillating with growing amplitude) → flip the bad
  sign on the fly:
  ```matlab
  global MCORR_STATE
  MCORR_STATE.xSign = -1;     % or ySign / zSign
  ```

The proportional gain (`gainXY`, `gainZ` ≤ 1) makes wrong-sign runaway
much less violent, which is why v3 uses it by default.

### 3.5 Choose the reference plane and channel
- **Channel**: the one with the most invariant **structural** signal (red /
  morphological label, e.g. tdTomato, mRuby, blood-vessel autofluorescence).
  Avoid using a sparse activity channel (e.g. GCaMP) as reference.
- **Plane**: typically the **deeper** of your two FastZ planes (more axially
  contained PSF blur → better Z localisation), but really pick whichever
  has more landmark structure.

---

## 4. Per-experiment workflow

See [`workflow_v3.m`](workflow_v3.m) for runnable code. Summary:

1. **FOCUS at the reference plane**, FastZ disabled.
2. `refStack = acquireRefStack(hSI, 'channel', 2, ...);`  (3–5 min)
3. **Re-enable** FastZ (2-plane imaging).
4. `setupMotionCorrection(hSI, refStack, ...);`
5. **Attach `motionCorrUserFcn`** as a ScanImage User Function on events
   `acqModeStart`, `acqModeDone`, `frameAcquired`.
6. `hSI.startLoop()` (or click GRAB / LOOP).
7. Watch the Command Window during the first 1–2 corrections; flip
   `xSign/ySign/zSign` if a correction makes drift worse.
8. To pause: `MCORR_STATE.enabled = false;`. To stop entirely: stop
   acquisition; the `acqModeDone` event prints a summary.

---

## 5. Constants & parameters to determine

These are listed in priority order: **must measure** first, **should tune**
next, **rarely-touched** last.

### Must measure (scope-specific, do once)

| Parameter | Where set | How to determine |
|---|---|---|
| `pixelSizeXY_um` | `acquireRefStack` | Auto-detect (verify printout) **or** image a graticule / 10 µm bead and measure pixel offset between two features at known separation. |
| `xSign`, `ySign`, `zSign` | `setupMotionCorrection` | Move the stage by a known amount along each axis in FOCUS and observe image-feature direction (see §3.4). |
| Reference channel number | `acquireRefStack('channel', N)` | Whichever PMT carries the structural label. Verify with `grabCurrentFrame(hSI, N)` showing recognisable morphology. |
| Reference plane Z (sample-Z value) | `setupMotionCorrection('targetPlaneZ_um', ...)` | Read from `rd.zs` printed during a brief acquisition, or leave `[]` for auto-detect (deeper of the two planes). |

### Should tune (sample- / experiment-specific)

| Parameter | Default | How to choose |
|---|---|---|
| `zRange_um` | 12 | Cover ±2× your expected worst-case axial drift over the session. Larger range = longer calibration time. |
| `zStep_um` | 1 | Roughly half the axial PSF FWHM (≈1 µm at NA 1.0; 2 µm at NA 0.6). Smaller = better sub-step interpolation; longer calibration. |
| `avgDuration_s` | 5 | Long enough to suppress shot noise *and* activity flicker on the reference channel. Lower bound: ~10 frames. |
| `correctionInterval_s` | 60 | Conservative once-per-minute. Reduce if drift is faster than ~1 µm/min, but verify stability first. |
| `deadband_z_um` | 1.5 | Roughly 1× your Z estimation noise floor (RMS of zEst when sample is known stable). |
| `deadband_xy_um` | 1.0 | ~1× pixel size, or 1× XY estimation noise. |
| `maxStep_z_um` | 3 | Largest single Z move you trust per minute. Should be ≥ typical drift over `correctionInterval_s`. |
| `maxStep_xy_um` | 5 | Ditto for X/Y. |
| `gainXY` | 0.7 | < 1 prevents oscillation around the deadband edge; lower = safer / slower convergence. |
| `gainZ` | 0.5 | Lower than XY because Z errors are more catastrophic (loses imaging plane). |

### Rarely changed

| Parameter | Default | Notes |
|---|---|---|
| `cropFrac` | 0.6 | Central crop for matching. Larger = more signal but more sensitive to FOV-edge artifacts. |
| `useFeatureImage` | `true` | Set `false` if your reference label is dim / sparse and high-pass filtering destroys SNR. |
| `minNCC_buffer` | 0.30 | Frame admission gate. Lower if your reference plane NCC is intrinsically low. |
| `minNCC_correction` | 0.50 | If real corrections are being skipped with "peak NCC < threshold", lower it. |
| `maxShift_pix` | 50 | XY phase-correlation search radius. Increase if drift can exceed 50 px between corrections. |

### Things you do **not** need to set in v3 (gone vs. v2)

- MP-285 baud rate, microsteps-per-µm (`US_PER_UM`), serial port, flow
  control: handled by ScanImage's motor abstraction.
- Two separate "reference for XY" vs "reference for Z" images: v3 uses the
  best-matching Z plane for XY automatically.

---

## 6. What gets printed

```
[MotionCorr @   60 s]  138 frames in last 5 s — computing drift...
  Z: est +3.45 µm  cmd +1.72  NCC=0.784  ref 19/25
  Y: est -1.20 µm  cmd -0.84  (-1.50 px)
  X: est +0.40 µm  cmd +0.00  (+0.50 px)
  motor moved  X+0.00 Y-0.84 Z+1.72 µm  | cumulative X+0.0 Y-0.8 Z+1.7
```

`cmd` reflects deadband × gain × clamp applied to `est`. A line reading
"all within deadband — no move" is normal and indicates stability.

---

## 7. Failure modes and fixes

| Symptom | Likely cause | Fix |
|---|---|---|
| `peak NCC < 0.50 — skipping` repeatedly | Reference plane lost; channel saturated; bleaching | Re-acquire ref stack; reduce `minNCC_correction` cautiously |
| Drift grows after each correction | Wrong sign | `MCORR_STATE.xSign = -1` (etc.) live |
| Correction oscillates ±1 µm forever | Gain too high or deadband too tight | Lower `gainXY`/`gainZ`; raise `deadband_*` |
| Z always saturates `maxStep_z_um` | Drift faster than scheme; or wrong sign | Check sign first; then raise `maxStep_z_um` and shorten `correctionInterval_s` |
| "Auto-detected target plane" never prints | Only one z value seen (FastZ disabled?) | Set `targetPlaneZ_um` explicitly, or confirm FastZ on |
| `grabCurrentFrame` returns `ok=false` | SI build differs in stripe buffer location | Inspect `hSI.hDisplay` / `hSI.hScan2D.hAcq`, edit method 1 in `grabCurrentFrame.m` |
