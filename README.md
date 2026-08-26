# Windsurfing Track Inspector — Method Specification

This document specifies exactly how the inspector derives velocity, heading,
acceleration, and curvature from raw `(lat, lon, time)` GPS fixes, and the
run-detection, wind-estimation, and transition-classification logic built on
top of them. It reflects the current state of `wtrack.html` — a single
self-contained HTML/JS file, no dependencies, fully offline.

This is a specification of current behavior, not a history of how it was
built — for the design rationale, the approaches that were tried and
replaced, and the validation behind each decision, see `diary.md`.

**What's here:** coordinate projection, sampling-interval detection
(including variable-rate devices), kinematics (velocity, acceleration,
curvature, jerk), duplicate-point cleanup, run detection (finding stretches
of consistent point-of-sail), wind direction estimation, and classification
of jibes, tacks, and crash/recovery zones in the transitions between runs.

**What's not here (yet):** flagging/data-quality review and general
sub-run straight/turn classification are not currently implemented.
Transition types beyond jibes, tacks, and crashes aren't classified yet —
notably, a same-side heading swing (heading up then bearing away on one
tack) never crosses a pole, so it's currently invisible to the classifier
even when tactically meaningful (§6).

---

## 1. Coordinate projection

Raw points are in geographic coordinates (lat/lon, degrees). Kinematics are
computed in a local **East-North-Up (ENU)** plane, in metres, using an
**azimuthal equidistant** projection centered on the track's own origin:

```
lat0 = (min(lat) + max(lat)) / 2      // bounding-box midpoint, in degrees
lon0 = (min(lon) + max(lon)) / 2

for each point:
  (distance, bearing) = vincentyInverse(lat0, lon0, point.lat, point.lon)
  x = distance * sin(bearing)   // east, metres
  y = distance * cos(bearing)   // north, metres
```

`vincentyInverse` computes the true WGS84 ellipsoidal geodesic distance and
initial bearing between two lat/lon points (Vincenty's formulas), converging
in a handful of iterations per point. Every point gets its own exact
distance and bearing from the origin, so heading is the true compass
bearing at every point, and accuracy holds at any track scale — not just a
few-km session, but equally a 1,000 km point-to-point track.

The origin is the **lat/lon bounding-box midpoint** of the track, computed
once from the raw loaded file — not any single trackpoint. This keeps the
coordinate system stable even if points are later deleted.

## 2. Sampling interval detection

Both computed once, at load time, from the **untouched original file** —
never recalculated after points are later deleted.

### `baseDtMs` — the device's base rate

The minimum positive gap between consecutive raw timestamps, in ms:

```
baseDtMs = min( time[i+1] - time[i] )   over all i, where the gap > 0
```

### `dtMaxVar` — tolerance for variable-rate devices

Some devices (adaptive/smart recording) legitimately log at more than one
rate. At load time, every positive `dt` is collected and sorted; `dtMaxVar`
is the value at the **95th percentile** of that sorted list. If there are
no positive `dt` values at all, `dtMaxVar` simply equals `baseDtMs`.

```
sorted = sort( dt : dt > 0, over all consecutive raw timestamp pairs )
dtMaxVar = sorted[ floor(0.95 * length(sorted)) ]
         = baseDtMs   if sorted is empty
```

This feeds directly into run detection (§4) as the boundary between "normal
sampling" and a "true gap": an edge fails if its own `dt > dtMaxVar +
baseDtMs` (the `+ baseDtMs` allows one base interval of slack, so a single
transiently dropped sample on an otherwise fixed-rate track isn't treated
as a gap).

## 3. Kinematics: cause-and-effect attribution

For points **A, B, C** (three consecutive points), with edges `AB` and `BC`:

### Raw edge kinematics — attributed to the edge's endpoint

```
velocity(B) = (B - A) / (t_B - t_A)     — "the speed getting here"
velocity(C) = (C - B) / (t_C - t_B)
heading(i)  = atan2(vx, vy)              — compass convention, 0°=N, 90°=E
```

Needs only the **one point before it** — valid for every point except the
track's true first point. No blending between segments: each point's
velocity is exactly its own trailing edge.

`uniformSampling(i)` = does the single interval feeding this point's
velocity equal `baseDtMs`.

**Jerk** — the rate of change of *acceleration* — follows this same
edge-endpoint attribution, not acceleration/curvature's vertex attribution
below: for edge `BC` specifically, jerk describes how much acceleration
changed *across* that edge, so it's stored at `derived[i+1]` (the edge's
own ending index) — the same convention `speed(i)` uses for `edge(i-1,i)`.

```
jerk(BC) = ( accel(C) - accel(B) ) / dt(BC)
```

Computed as the raw `(ax, ay)` **vector** difference first, then projected
once onto edge `BC`'s own velocity — not a scalar difference of the two
already-projected `tanAccel` values, since those are each projected onto a
*different* basis (accel(B) onto the average of velocity(B)/velocity(C);
accel(C) onto the average of velocity(C)/velocity(D)).

Not read by any run-detection check — kept purely for its own
kinematics-chart plots, alongside a smoothed, fit-based version (§7).

### Acceleration/curvature — attributed to the point *between* the samples

Acceleration is the **cause** of the change from `velocity(B)` to
`velocity(C)` — that change happened *at B*, between the two velocity
samples, not at C:

```
h1 = t_B - t_A          h2 = t_C - t_B
h1_eff = min(h1, baseDtMs/1000)         h2_eff = min(h2, baseDtMs/1000)
dt = (h1_eff + h2_eff) / 2

accel(B) = (velocity(C) - velocity(B)) / dt
```

**No gap-invalidation.** Acceleration is computed unconditionally (never
suppressed to `null` because of a gap) for every point that has a full
3-point window. If `AB` spans a real gap, the resulting acceleration at B
is a genuine, well-defined, usually large discontinuity — the signal run
detection uses to break there naturally (§4).

**The `h1`/`h2` cap** exists specifically for the gap case: under a
"sliced into constant-velocity sub-segments" model, velocity stays flat
through almost the entire gap, with the real transition concentrated at
the boundary, on the scale of one base interval — not the gap's full
duration. Capping each side at `baseDtMs` before averaging stops a long gap
edge from artificially diluting the computed acceleration. It's a no-op
for any normal (non-gap) window, where `h1`/`h2` already equal `baseDtMs`.

**Projection reference.** Tangential/centripetal acceleration and curvature
are projected onto the **average** of `velocity(B)` and `velocity(C)`, not
`velocity(B)` alone:

```
projV = (velocity(B) + velocity(C)) / 2
cross = projV.x * accel.y − projV.y * accel.x

tangentialAccel  = (accel · projV) / |projV|
centripetalAccel = cross / |projV|
curvature        = cross / |projV|³
```

`derived[i].speed` (the point's own displayed/charted speed) is unaffected
by any of this — it's still exactly `velocity(B)`, unchanged.

### Coverage

| Quantity | Valid range | Reason |
|---|---|---|
| `velocity`, `heading` | index 1 … n−1 | needs one point before |
| `acceleration`, `curvature` | index 1 … n−2 | needs a point on **each** side |
| `jerk` | index 2 … n−2 | needs acceleration on **each side** of the edge it's attributed to |

The true track start (index 0) never has any derived data. The true track
end (index n−1) has velocity/heading but no acceleration/curvature.

## 4. Run detection

A **run** is a stretch of track where the general point of sail stays
consistent — tolerating real-world wiggle (waves, hand-steering, GPS
jitter) without fragmenting, while still breaking on a genuine turn, stall,
bad-data edge, or gap.

**Edge-based**, matching the kinematics attribution above: "edge `i`" is
the segment from point `i-1` to point `i`; its velocity/heading is
`derived[i]`. `curvature(i)` describes the *transition* at point `i`,
between the edge ending there and the edge starting there.

**Mixed edge-based and fit-based, deliberately.** Heading reads `derived`'s
own edge-based value throughout. Speed and curvature read `fitDerived`'s
smoothed value (§7).

### The shape of the criteria: two axes, plus warm-up-only additions

The checks fall into a small grid, split along two independent axes:

|  | **local** (fixed threshold) | **contextual** (compares against run state) |
|---|---|---|
| **edge-level** (pass-1 kinematics) | speed, gap | heading (narrow / wide) |
| **vertex-level** (pass-2 kinematics) | curvature, fit-vs-edge tangential-accel deviation | *(empty)* |

The edge/vertex split tracks which kinematics pass each check reads from
(§3): edge-level checks use the backward-difference velocity attributed to
an edge; vertex-level checks use the centered-difference acceleration/
curvature attributed to a vertex. The local/contextual split is about what
a check compares against — a fixed constant, or something the run's own
accumulated state produced (how far heading has swung across *this* window
so far). Heading is the one check that's edge-level but contextual, which
is why it alone needs two separate thresholds (narrow/wide) where nothing
else in the pipeline does.

Two further checks don't fit cleanly into this grid: **fit centripetal
acceleration** and **device-speed deviation** are both vertex/edge-level
and local, but scoped differently — checked only during warm-up and
trailing straightness, never during extension (see "The warm-up/warm-down-
only checks" below). A third, **gap debt**, is warm-up-only as well, but
tracks a sequential accumulator rather than a per-point reading (see its
own subsection below).

Two angle thresholds do different jobs:
- **narrow angle** (`runNarrowHeadingThresh`, default `10°`) — validates a
  candidate warm-up window as a genuinely stable direction, and separately
  validates a run's own trailing edge (see step 6).
- **wide angle** (`runBreakHeadingThresh`, default `90°`) — how far
  heading can swing, in total, anywhere across the run so far (tracked as
  a running `max − min` range, not distance from one fixed reference — see
  step 5) before that's a real point-of-sail change, not just wiggle. Set
  intentionally generous — see the walk-back mechanism in step 5, which is
  what makes a large tolerance here safe rather than just permissive.


### State machine

1. **Break point.** Start at the track's first point, or the end of a
   previously found run.

2. **Warm-up.** From the break point, accumulate edges until *both*
   `runMinSec` (default `2s`) *and* `runMinEdges` (default `2`) are
   covered.

3. **Warm-up validation — heading and edge check.** Every edge in the
   window, including the one feeding *into* the break point itself, must
   pass the *edge* check (see below: low speed, or a true gap). Any
   failure aborts the whole window — the break point slides forward by
   **one point**, and warm-up is retried from scratch.

   Heading is bounded by tracking the window's own `headingUnwrapped`
   range (`max − min`) directly, incrementally as each point is scanned,
   and failing as soon as that range reaches the narrow angle — rather
   than computing an average heading for the window and checking each
   point's distance from it. Uses `headingUnwrapped` throughout (both for
   the range being tracked and each point being compared) rather than raw
   heading compared via wraparound-aware `headingDiff`.

4. **Warm-up validation — vertex checks (curvature, accel-fit deviation,
   fit centripetal acceleration, device-speed deviation).** Checked at
   *every* vertex in the window — including its own start (the break
   point) and its own end. No exemptions, no special cases: any failure
   anywhere is a hard failure of the whole window.

5. **Extension, with walk-back.** One point at a time:
   - the new point's own `headingUnwrapped` is checked against the run's
     own range **so far** (seeded from the warm-up window's own range,
     then extended incrementally as each new point joins) — the new
     point must not push that range's `max − min` to or past the **wide**
     angle — and pass the edge check.
   - if the point passes, its endpoint **joins** the run (the tracked
     range updates to include it), and *then* gets its own vertex checks
     run: curvature first, then the accel-fit deviation — fit centripetal
     acceleration is *not* checked here (see "The warm-up/warm-down-only
     checks" below). Passing both means keep extending (tracking the
     point of highest `|fit centripetal acceleration|` seen so far along
     the way); either one failing means the point **stays included**, but
     extension stops there — a vertex check describes the *transition* at
     that point, so a bad reading implicates the edge that would come
     *next*, not the one that just landed the point in the run.
   - if the wide-angle (range) check specifically fails: instead of
     ending the run at the point right before the failure, **walk back**
     to wherever fit centripetal acceleration peaked during this
     extension (if anywhere) and end the run there instead — not
     curvature. Ties in the peak search favor the **latest** candidate.
     If the wide-angle check fails on the very first candidate point
     (nothing ever joined the run during extension, no walk-back
     candidate exists), the run simply ends at the warm-up window's own
     end. Either way, once walk-back happens the termination is
     attributed to `'curvature'` internally (not `'heading'`), for the
     purposes of step 7's resume logic.
   - if the edge fails on speed/gap instead, the new point is simply
     **excluded** — no walk-back, the run ends cleanly at the edge's start
     point.

   Walk-back is the *first* of two separate mechanisms that can pull a
   run's end backward — see step 6 for the second.

6. **Trailing straightness.** Before accepting the result, the run's own
   *tail* — built exactly like warm-up's own accumulation (`runMinEdges`/
   `runMinSec`), just backward from `runEnd` instead of forward from a
   break point — must itself be narrow-angle self-consistent: its own
   `headingUnwrapped` range (`max − min`, tracked the same way step 3
   tracks warm-up's own range) must stay under the narrow angle, and it
   must pass the fit-centripetal-acceleration check at every vertex.

   Checking the tail's own range, computed fresh from just the tail,
   asks "is the tail internally consistent with *itself*," regardless of
   how far it's drifted from where the run started. If the tail isn't
   self-consistent, shrink the run back by one point and retry — down to,
   never below, the warm-up window's own end.

   Curvature and accel-fit deviation don't get re-checked here — every
   point in the tail already passed them once during extension (step 5).
   Fit centripetal acceleration *is* newly checked here, since it was
   never checked during extension in the first place.

7. **Minimum duration.** The finished run (its full, exact extent) is
   checked against `runMinDuration` (default `15s`).
   - **Kept:** push the run, and resume the next search at its own end —
     runs can legitimately **share a boundary point**.
   - **Abandoned** (too short): where to resume depends on *why* it
     ended. A **heading**-caused ending is *contextual* — retrying just
     **one point later** gives a genuinely different starting range a
     chance. Every other reason (**velocity, gap, curvature, accel-fit
     deviation**) is *local* — resuming straight at the run's own end is
     both correct and cheaper. (A walked-back wide-angle ending counts as
     `'curvature'` here.) Warm-up failures (steps 3–4) always retry at
     `breakPoint + 1` regardless of which check failed.

8. Track end can be reached either mid-run or between runs — no special
   handling needed; the loop just stops.

### The warm-up/warm-down-only checks

Three checks are scoped to warm-up (step 4) and trailing straightness
(step 6) only — never checked during extension (step 5), unlike the other
vertex/edge checks.

**Fit centripetal acceleration** (`runWarmupCentripetalAccelThresh`,
default `1.2 m/s²`) — `|fitDerived[i].centripetalAccel| ≥` threshold fails
the vertex.

**Device-speed deviation** (`runWarmupDeviceSpeedDeviationThresh`,
default `2 m/s`, toggleable via `runDeviceSpeedDeviationEnabled`) —
`|fitDerived[i].speed − points[i].rawSpeed| ≥` threshold, checked only
where a device speed reading is actually present in the file (a no-op
when it's missing).

**Gap debt** (`runGapDebtEnabled`, `runGapDebtPaybackWeight`, default
`0.67`) — a sequential running accumulator, precomputed once for the
whole track:

```
debt[0] = 0
for i = 1 .. n-1:
  edgeDist = |points[i] - points[i-1]|              — straight-line distance
  if debt[i-1] <= 0 and edgeGapFails(i):  debt[i] = debt[i-1] + edgeDist
  else:                                   debt[i] = max(0, debt[i-1] - paybackWeight * edgeDist)
```

A gap edge encountered while the debt is already clear **adds** its own
straight-line jump distance, starting a new debt cycle. Every *other*
edge while a debt is outstanding — gap or not — **pays back** against
that debt, but only at `paybackWeight` (default `0.67`) of its own
distance, so it takes traveling roughly `1/paybackWeight` times (~1.5× at
the current default) a gap's own jump distance to net it back out. The
debt clamps at zero rather than going negative. `warmupOk` requires every
vertex in the window to have already netted its own debt to zero.

Its own checkbox defaults on or off per track: set once, right after
`computeDerived` runs at load time, to `!hasDeviceSpeed` — device-speed
deviation is the more direct check whenever a device speed reading
exists, so gap debt only defaults on where that more direct check has
nothing to read. Either checkbox can be flipped by hand afterward.

### Reconnection: revisiting walk-back breaks once the full picture is known

After the main search loop finishes and the full run list exists: revisit
every run flagged `wasWalkedBack`, and reconnect it with the next run if —

- `headingUnwrapped`'s own range (`max − min`) across the *whole candidate
  merged range* (from the first run's own start to the second run's own
  end) stays under the wide angle, and
- nothing else in the transition zone between them would have
  disqualified it anyway — re-run the same edge and vertex checks
  `computeRuns` already uses (excluding fit centripetal acceleration and
  device-speed deviation, both warm-up/warm-down-scoped) across every
  point from one run's end to the other's start.

If both hold, the two runs merge into one spanning the full original
range, inheriting the second run's own `wasWalkedBack` flag and
termination reason. This runs to a fixed point, not just a single pass —
a reconnected run can itself have inherited a walk-back flag from its new
right-hand neighbor, enabling a further chained merge.

A `reconnectEnabled` checkbox (default on) gates the entire pass —
unchecking it means `computeRuns`'s reconnection loop body never executes,
leaving every run exactly as the state machine's main search found it.

### The edge check

Beyond the vertex checks below and heading, an edge also fails (breaking
the run) if:

- **Low speed** (`runLowSpeedThresh`, default `0.3 m/s`) — `speed ≤`
  threshold, reading `fitDerived`'s own smoothed speed (§7), not
  `derived`'s raw one.
- **True gap** — the edge's own interval exceeds `dtMaxVar + baseDtMs`
  (§2). At 1Hz with no other variable rate (`dtMaxVar == baseDtMs ==
  1000ms`), the effective threshold is `2000ms` — a 2s edge is tolerated,
  a 3s edge isn't.

Both checked the same way heading is: across every edge during warm-up,
and against the new edge during extension.

### The vertex check

Checked at every vertex — the transition between the edge ending there and
the edge starting there (§3) — rather than at either edge specifically:

- **Curvature** (`runCurvThresh`, default `0.15`) — `|curvature| ≥`
  threshold, reading `fitDerived`'s own smoothed curvature (§7).
- **Tangential-accel deviation** (`runAccelDeviationThresh`, default
  `5 m/s²`) — `|edge-based tanAccel − fit-based tanAccel| ≥` threshold
  (§7) — the one check that's intentionally *both* sources at once, since
  it's measuring disagreement between them directly, not preferring one.

Checked at *every* vertex in the warm-up window uniformly — including its
own start and end, no exemptions (see step 4). During extension, the
point stays included when a check fails; only further extension is
blocked. Vertex and edge failures differ from heading in resume behavior
on abandonment (step 7): both skip straight to the run's end, where only
heading retries one point later.

### No gap-invalidation gate

A point right after a gap is judged on its own merits — if the
gap-spanning average velocity implies a real discontinuity, the ordinary
vertex/heading/edge checks above catch it and break the run there
naturally, the same way any other real turn would. The true-gap edge
check (above) exists as a backstop specifically for gaps *without* an
accompanying heading/curvature/accel-deviation signal (e.g. a stall
during a dropout).

## 5. Wind direction estimation

A sailing session typically covers both tacks roughly evenly, against one
roughly-steady wind. The estimator leans on that: find where the compass
splits most evenly into two halves of sailing distance, subject to a
physical sanity check (a genuine gap near at least one pole of the
split), analyze each half, refine the split against what that analysis
found, and combine what the refined halves say. All of this uses **run
edges only** — turns and transitions are deliberately excluded.

### Step 1 — find a rough balanced-split axis

Build a distance-weighted heading histogram (360 bins of 1° each, plus the
max speed seen in each bin) — straight-line distance per edge accumulated
into the bin its heading falls in. Then rotate a single 180°/180° divider
around the compass, in 1° steps, looking for the rotation where the two
halves' total distance is most equal.

A candidate split only qualifies if there's a genuine wide gap — every bin
across a real `SPLIT_GAP_MIN_WIDTH_DEG` (`15°`) window at or below 2% of
the histogram's peak bin — at **at least one** of the divider's two poles
(the axis itself or its antipode), not necessarily both.

If no rotation has a wide gap at either pole, there's no estimate.

### Step 1b — skewness check

Even at the best (most-balanced) rotation found, if the smaller side is
still under `WIND_EST_MIN_BUCKET_RATIO` (`0.4`, i.e. `40%` — an internal
constant, not UI-exposed) of the larger side's distance, there's no
estimate. Checked against the rough split from step 1, before any
refinement.

### Step 2 — analyze each bucket, from the histogram

The split axis divides the compass into two buckets (half-circles). For
each bucket, computed from the two histograms built in step 1:

- **Top-speed vector**: vector average of the bucket's own
  `WIND_EST_TOP_SPEED_COUNT` (`2`, an internal constant) highest-max-speed
  *bins* — scoped to just this bucket, not a global top-N.
- **Median heading**: the distance-weighted **median**, not mean.
  Computed by sorting the bucket's bins by position relative to the
  bucket's own axis (linearizing the half-circle without wraparound
  ambiguity, since a bucket never spans more than 180°) and walking the
  sorted list until cumulative distance reaches half the bucket's total.

If either bucket has no valid bins, no estimate.

### Step 3 — refine the split against the wind axis

The two buckets from step 2 give a first estimate of the wind axis:
bisect their median headings (circular mean, short arc). Then: re-split
using this wind axis as the new boundary, redo step 2's bucket analysis
against it (new top-speed vectors, new medians), and bisect those
*refined* medians for the wind axis actually used from here on. If either
refined bucket ends up with no valid bins, no estimate.

This is one refinement pass, not a full iterate-to-convergence loop.

### Step 4 — pick a downwind reference, no agreement required

For each (refined) bucket, compute the signed shortest rotation from its
top-speed heading to its own median heading (`headingDiff(avgHeading,
topSpeedAngle)`).

Trust whichever bucket shows the **larger** separation between its
top-speed heading and its median heading (`Math.abs(rotA) >=
Math.abs(rotB)`) — that's the bucket with the clearer signal, and its
relationship to the wind axis (closer to the axis, or to the axis's
antipode) determines which side is downwind.

Wind direction is the antipode of whichever side this reference bucket's
top-speed vector sits closer to — unless step 5 below overrides it.

### Step 5 — transition-distance cross-check, and a weak-speed override

Everything above leans on speed: top-speed sailing is presumed more
downwind than the median. That assumption breaks on **overpowered
tracks**, where a boat can point upwind just as fast as it reaches.

`computeTransitionDisplacementIndicator` is a second, independent signal
that doesn't touch speed at all: for **every** transition (any gap
between consecutive runs, all treated the same), draw a vector from the
transition's start (the preceding run's own end) to its end (the next
run's own start), and project it onto the wind axis being tested. A jibe
typically carries speed smoothly through a downwind-crossing turn; a tack
often involves slowing through or near head-to-wind — so transitions that
net-displace toward downwind should, on average, cover **more distance**
than ones that net-displace toward upwind. Whichever direction shows the
larger average is what this indicator calls downwind.

Two safeguards before trusting it:

- **Sample counts in both directions.** Requires at least
  `WIND_INDICATOR_MIN_SAMPLES` (`1`) transitions *and* at least
  `WIND_INDICATOR_MIN_DIFF` (`2m`) difference between the two directions'
  averages before calling itself reliable.
- **Excluding long transitions.** Any transition over
  `WIND_INDICATOR_MAX_DURATION` (`60s`) is skipped entirely before it can
  contribute to either side's sum.

**Weak-speed override.** If step 4's own signal is weak in *both* buckets
— `Math.abs(rotA)` and `Math.abs(rotB)` both under
`WIND_OVERRIDE_WEAK_SPEED_THRESH` (`15°`) — the transition-distance
indicator gets tried (testing the wind axis from step 3 as the downwind
hypothesis) before falling back to step 4's reference-bucket logic. If
it's reliable by the criteria above, its answer is used instead;
otherwise step 4 proceeds exactly as described there. Every value the
override used is preserved on the returned estimate
(`usedTransitionOverride`, `overrideIndicator`).

This indicator is *also* available independent of whether it triggered an
override, via `computeTransitionDisplacementIndicator`.

### Manual controls: wind direction and tack split

Two separate number textboxes, wind direction and tack split, sit side by
side in the wind panel — deliberately independent of each other. Wind
direction says which way is upwind; tack split says where the boundary
between the two tacks sits.

Both auto-initialize once per freshly-loaded track (`resetWindDirToCalculated`
/ `resetTackSplitToCalculated`, called from `loadFromText` after dedup
settles) to whatever the calculated pipeline found — wind direction from
`est.windDirection`, tack split from `est.splitAxis`. Tack split's own
reset checks `splitAxis` directly rather than `est.ok`, since `splitAxis`
can be valid even when the overall estimate isn't fully `ok`. Both are
left blank if their respective calculated value is entirely unavailable.
Blank wind direction means no wind direction — transitions go
unclassified.

Each has its own "recalculate" button. Wind direction's recalculate is
aware of an active tack-split override: if the split has been manually
overridden, recalculating wind direction derives it from *that* split's
own buckets (via `computeWindDirectionFromBuckets`, the same step 3/4
logic factored out) rather than silently reverting to the fully-automatic
`est.windDirection`. Wind direction also has a "flip 180°" button.

**Split override triggers a live bucket recompute.** Whenever the tack
split textbox meaningfully differs from `est.splitAxis` (or stands in for
an undetermined one), `redrawAll` redoes the bucket analysis against the
manual split, using the same fast histogram-based `computeBucketStats`
from step 2. Keyed off the split control specifically, not wind
direction — overriding wind direction alone just changes which side is
called downwind, it doesn't mean the split boundary itself needs to move.
These override buckets (`currentOverrideBuckets`), when active, are what
the split axis, median, and top-speed lines in both rose diagrams
actually draw from.

**The wind axis itself follows wind direction, not tack split.**
Independent of any split override, the purple wind-axis line in both rose
diagrams is `manualDir != null ? manualDir : est.refinedAxis`.

### Off-wind angle labels

In the full rose diagram, each median line, each top-speed line, and the
wind-in-use ray are labeled with a small angle:

- **Medians and wind-in-use**: `angleDiff180(heading, windAxis)` — the
  direct angle to the wind axis pole, 0°–180°.
- **Top-speed**: also `angleDiff180`, not the smaller axis-relative
  measure — top-speed sailing is presumed near-downwind, so this reads
  close to 180° (nearly opposite the wind).

### What's left undetected

A **radical wind shift mid-session** (the whole model assumes one steady
wind axis) and a **tow** (injects non-sailing motion directly into the
histogram and into transition displacement alike) are both left
undetected by the current one-shot-per-track model. The manual
wind-direction and tack-split controls are the workaround.

### Diagnostics for undetermined tracks

`computeWindEstimate` never returns a bare failure — every early exit
returns `{ ok:false, reason: '...', ...whatever was already computed }`,
so the rose diagram can render as much of the pipeline as got completed
before the actual failure point. The split axis, both buckets' own
top-speed and median headings, and the wind axis are all drawn (grey,
gold, green dashed, and purple dashed respectively) whenever they exist,
with the specific failure reason surfaced in the "undetermined — please
set manually" message.

## 6. Jibe and tack classification

Between two consecutive runs sits a transition zone — whatever the run
detector didn't claim as belonging to either run. Currently classified:
successful jibes and tacks, one failure mode for each (an aborted
attempt), and crash/recovery zones.

The transition zone's own boundary points — `runA.endIdx` and
`runB.startIdx`, each run's own full, validated extent — serve directly
as lead-in/lead-out.

Uses the effective wind direction from §5's manual textbox — the actual
source of truth from redraw to redraw, not `computeWindEstimate`'s return
value directly.

### Crash detection comes first, and doesn't need wind direction at all

Before attempting any jibe/tack classification, `scanForCrash` scans the
**whole** transition zone for the first edge spanning a true gap (the
same `edgeGapFails` check §4 uses). If no crash is found, jibe/tack
classification proceeds normally across the whole zone. If one is found:

- Whatever's strictly before the crash edge may still qualify as a
  **failed** attempt (evaluated with success disallowed).
- The crash edge itself onward is always `crash_recovery`, regardless of
  what came before it. The marking starts one edge before the crash's own
  endpoint — at the edge's *own* start.

A single transition can therefore produce **two** results: a failed
attempt for the good part, and a separate crash-recovery entry for the
rest.

### Sign convention

A **right** turn (heading increasing) produces **negative** curvature; a
**left** turn (heading decreasing) produces **positive** curvature.
`expectedCurvatureSign = -sign(headingChange)`.

### A jibe and a tack are the same test, aimed at opposite poles

A jibe crosses through **downwind**; a tack crosses through **upwind**.
`classifyPoleInRange(rangeStart, rangeEnd, poleDirection, successType,
failType, allowSuccess)` holds the entire shared logic below,
parameterized by which pole to aim for, what type strings to return, and
whether a success classification is even possible for this call (`false`
when the range was truncated by an upcoming crash). `classifyTransition`
tries the jibe pole first, and if that returns nothing, tries the tack
pole.

### Expected turn direction — from lead-in alone

Whichever rotation reaches the target pole first (the shorter arc, from
the lead-in heading) is what this maneuver requires — a strict left/right
test relative to the wind axis, computed from lead-in **only**, not from
any relationship between lead-in and lead-out.

Since turning toward one pole and turning toward the other are always
opposite rotations (except from the single degenerate beam-reach heading,
exactly 90° from both poles), a jibe-shaped transition can never
simultaneously satisfy the tack test, or vice versa.

### The scan: curvature consistency, and a low-speed check

Two conditions are checked at every point across the range, marked
differently when they break it:

- **Curvature** must match `expectedCurvatureSign`, with a small
  tolerance (`CURVATURE_BREAK_TOLERANCE = 0.02`): a point still counts as
  consistent if its curvature matches the expected sign, **or** if it's
  within `0.02` of zero even on the wrong side. `null` (undefined)
  curvature always breaks the scan.
- **Speed** must exceed the same low-speed threshold run detection uses
  (`runLowSpeedThresh`).

The two break kinds are marked with different *inclusion* semantics for
the attempt's own reported extent (`attemptEndIdx`):

- A **curvature** break is **inclusive**: `attemptEndIdx = breakIdx`.
- A **speed** break is **exclusive**: `attemptEndIdx = breakIdx - 1`.

Either way, *qualification* is judged against the last reliably-consistent
point (`preBreakIdx = breakIdx - 1`) regardless of break kind.

- **No break at all**: lead-in to lead-out must differ by at least
  **60°**, or this is left unclassified (or, if `allowSuccess` is `false`
  because a crash truncated the range, judged as an attempt using the
  same 2s/10° bar below). If it clears the 60° bar: **jibe** / **tack**.
- **A break is found**: check `preBreakIdx`. If at least **2 seconds**
  elapsed since lead-in *and* at least **10°** of heading change was
  achieved by that point, this is a **failed attempt** — lead-out heading
  is not checked at all for this case. Otherwise: left unclassified.

None of the thresholds above (60° net change, 2s, 10°, 0.02 curvature
tolerance) are currently exposed in the UI — they're fixed constants in
`classifyPoleInRange`.

### Loosened success path, tacks only

If the strict scan doesn't already call a transition a `tack` success,
`checkLooseTackSuccess` gets a chance to upgrade the result (from either
`null` or `failed_tack`) if **all** of:

- Net heading change (lead-in to lead-out) still clears the same **60°**
  bar.
- Total time spent at or below the low-speed threshold, summed across the
  whole transition, is under `looseTackMaxStallTime` (default `5s`).
- The whole thing took under `looseTackMaxDuration` (default `20s`).
- **Entry** heading is itself upwind-ish (`|headingDiff(entry,
  windDirection)| < 90°`).
- Entry and exit sit on **opposite** sides of the wind axis.
- The boat's actual **net displacement** over the whole maneuver — a
  vector drawn from the entry point straight to the exit point — has a
  positive component toward the wind (dot product with the upwind unit
  vector `> 0`).

No requirement on **exit heading** at all. This path is tack-only —
jibes always go through the strict scan alone.

### Marking the crossing point, and the speed extremes

Two further things are tracked for every jibe, tack, and failed attempt,
as a post-processing pass over the already-classified transitions:

- **The crossing vertex ("through").** The last vertex still on the
  original side, just before heading actually crosses the relevant pole
  — found by walking the transition's edges and checking where the
  signed angular difference to that pole (`headingDiff(heading, pole)`)
  changes sign between consecutive edges. `null` for a failed attempt
  that broke off before the heading ever actually swung past its pole.
- **Min and max speed vertices**, found independently of wind direction
  by a straightforward scan of the transition's own edges.

The selection info panel (§11) surfaces the actual speed and off-wind
angle at each of these points — entry, through, exit, min, max, in that
order — for whichever transition the currently selected point falls
within.

### A known, accepted limitation

A same-side heading swing — heading up to build speed, then bearing away
hard, all on one tack — is **not** treated as a maneuver at all by this
classifier: it never crosses either pole, so neither the jibe nor tack
test can fire. §4's run detector keeps this as a single run. Genuinely
interesting same-side moments (the bear-away point in a speed-run
buildup, for instance) aren't marked as anything by this layer. A real
jibe that ends up briefly clew-first before the sailor switches feet,
versus a heading-only wiggle that dips across the pole and back without
any real maneuver, can in principle produce very similar-looking
kinematic signatures — GPS position and time alone can't always
distinguish the two. Treated as an accepted limit rather than something a
purely kinematic rule should be expected to solve.

## 7. Local polynomial path fit and artifact detection

A second, independent kinematics pipeline, alongside the finite-difference
one in §3 — not a replacement for it. This pipeline fits a small
polynomial through a window of *several* points and reads velocity/
acceleration off the fitted curve's own derivatives.

### The fit itself

`fitPolynomialLS(ts, ys, order)` — ordinary least-squares polynomial
regression via the normal equations, solved by Gaussian elimination with
partial pivoting. Returns `null` for a degenerate system (fewer distinct
`t` values than parameters) rather than throwing.

Window size and polynomial degree are both user-configurable (`window` /
`degree` inputs, defaulting to 8 points / degree 5).

- **Degree 5.** A degree-4 fit already lets acceleration vary
  continuously across the window, but its own acceleration curve —
  `x''(t) = 2a2 + 6a3·t + 12a4·t²` — is only quadratic in `t`, which has
  constant curvature and can never itself change concavity. Degree 5
  makes acceleration cubic in `t` (`x''(t) = 2a2 + 6a3·t + 12a4·t² +
  20a5·t³`), which *can* inflect within the window.
- **8 points.** With 6 coefficients (degree 5) and 8 points, the fit
  stays only mildly over-determined (2 "extra" points beyond exact
  interpolation) — enough to smooth out one or two genuinely bad points
  without drifting far from what was actually recorded, while still snug
  enough to capture a real, legitimate maneuver.

### Evaluation point: centered

For `N` window points there are `N-1` edges; the point evaluated is
whichever point *ends* the middle-most edge — `numBefore = floor(N/2)`
points before it, `numAfter = (N-1) - numBefore` after. For odd `N` this
lands exactly centered. For even `N` (the default, 8: `numBefore=4`,
`numAfter=3`), there are two equally-central vertices, and one of the two
gets picked, landing one point off true-center.

### Kinematics from the fit's own coefficients

No finite differences, and no averaged-adjacent-edge-velocity projection
trick. A polynomial's derivatives are exact at any point: with `t`
re-centered so the evaluation point sits at `t=0`, `x'(0) = a1` and
`x''(0) = 2·a2` directly from the fitted coefficients, regardless of the
polynomial's degree. Tangential/centripetal/curvature use the same
formulas as §3's pass 2, just with this velocity as the projection
direction directly.

**Does not skip windows spanning a true gap.** The map may show a strange
loop near a crash (the curve reconciling two disconnected local trends
across a time span with no constraining data), but the *kinematics*
derived from it at the evaluation point still come out as reasonable
values — a known, cosmetic-only quirk.

### Two derived diagnostics

**Tangential-acceleration deviation** (`findSpeedLagArtifacts`): flags any
point where `|derived[i].tanAccel - fitDerived[i].tanAccel|` exceeds a
configurable threshold (default `1 m/s²`). Compares *acceleration* rather
than speed itself: a position glitch shows up as a sharp, transient spike
in how quickly speed is changing, rather than as an offset in the speed
value itself, which the fit's own smoothing already partially absorbs.

Marked on both the map (bright red X) and the kinematics charts. A
"delete speed-lag points" button removes them and recomputes, mirroring
`removeDuplicateCoordPoints`'s pattern of validating the result before
committing to the `points` array. As of this writing, deletion is the
only corrective action offered — see §12 for a more targeted correction
approach that was designed but not yet implemented.

**Arc-length deviation**: the fit's own path length (finely sampled — 30
points, summed as a piecewise-linear approximation) minus the actual
edge-based path length (straight-line sum between the real recorded
points), over the same window. The *sign* carries real information: a
gap-induced loop overshoots (positive), while a sharp, isolated spatial
spike gets smoothed over/cut short by the polynomial (negative).
Available as its own kinematics-chart plot ("path length deviation").

### On the map: a live curve around the current selection

`computeLocalCubicFit()` — the same fit, same window/degree settings,
built around whichever point is currently selected, sampled at 3× the
window's own point count and drawn as a dashed curve on the track map.
Purely a visualization; the swept `computeFitKinematics()` above is the
one that actually powers the artifact detectors and chart overlays.

## 8. Duplicate cleanup: timestamps and coordinates

Two distinct data artifacts, each handled separately, since neither
cleanup catches the other's failure mode.

### Duplicate timestamps

Consecutive points sharing an identical timestamp mean `dt = 0` for that
edge, which means `speed = distance / 0 = Infinity` downstream.

Runs automatically and unconditionally, positioned after `project()` but
before `computeDerived()`.

For a run of points sharing a timestamp, keeps whichever *one* point
minimizes the difference between the **speed** implied immediately before
it (distance to the nearest already-kept point, divided by the time to
it) and the speed implied immediately after it — not raw distance. Every
candidate in the run shares the identical timestamp, so the time split on
either side (`dtBefore`, `dtAfter`) is *fixed* regardless of which
candidate gets kept — only the resulting distances (and therefore speeds)
vary by candidate.

### Duplicate coordinates

Consecutive points sharing an *exact* identical `(lat, lon)` are a device
artifact (re-logging the same fix across several timestamps) rather than
real stationary motion, which still has tiny position noise.

Same balance principle as duplicate timestamps, transposed: here, time
varies within a run sharing one position, so distance is fixed instead —
but the thing actually worth balancing is still **speed**. Picks
whichever timestamp minimizes the difference between the speed implied
immediately before it and the speed implied immediately after.

This runs **automatically, silently**, immediately after a fresh file
load. "Restore original track" deliberately does **not** re-run it. The
"remove duplicate-coord points" button remains available for a manual
re-run at any time, with a summary alert.

## 9. GPX export

Saves the current (post-editing) points as standard GPX 1.1, preserving
`<speed>` and `<ele>` where present. Filename defaults to
`<original-basename>_t.gpx`.

## 10. UI controls reference

| Control | Default | Meaning |
|---|---|---|
| run breaks if \|curvature\| ≥ | `0.15` /m | hard vertex-check threshold (§4 step 4/5), fit-based |
| heading swings ≥ (wide angle) anywhere in the run | `90°` | range-based extension tolerance, edge-based |
| speed ≤ | `0.3` m/s | low-speed edge failure, fit-based |
| tangential accel deviates (fit vs edge) ≥ | `5` m/s² | the other hard vertex-check threshold (§4 step 4/5, §7) |
| warm-up/trailing vertices need fit centripetal accel < | `1.2` m/s² | §4's warm-up/warm-down-only check — never checked during extension |
| warm-up/trailing edges need \|fit − device speed\| < | `2` m/s | §4's warm-up/warm-down-only check — toggleable (`runDeviceSpeedDeviationEnabled`), only when device speed is present in the file |
| gap debt payback weight | `0.67` | §4's warm-up-only check — toggleable (`runGapDebtEnabled`); doesn't depend on device speed being present |
| warm-up validated at ≥ (narrow angle) | `10°` | warm-up + trailing-straightness tolerance, edge-based |
| warm-up/trailing window needs ≥ | `2` edges | minimum edge count — both must be satisfied, not either alone |
| warm-up/trailing window needs ≥ | `2` s | minimum duration — same accumulation logic used forward (warm-up) and backward (trailing) |
| min run duration | `15` s | runs shorter than this are abandoned |
| reconnect walk-back breaks | on (checkbox) | gates §4's entire reconnection pass; unchecking leaves every run exactly as the main search found it |
| loose tack success: under | `20` s | §6 loosened tack path — total transition duration cap |
| loose tack success: ≤ | `5` s | §6 loosened tack path — total time below low-speed threshold, summed |
| local path fit window | `8` points | §7 |
| local path fit degree | `5` (quintic) | §7 |
| flag if tangential accel deviates ≥ | `1` m/s² | §7's artifact detector — edge-based vs. fit-based tangential acceleration |

**Wind panel** (§5) — title row has a "show/hide details" toggle
(default hidden) covering everything below the two textboxes:

| Control | Default | Meaning |
|---|---|---|
| wind direction (textbox + recalculate + flip 180° buttons) | calculated, or blank | single source of truth for wind direction everywhere it's used; auto-set once per fresh track load, holds through threshold tweaks otherwise |
| tack split (textbox + recalculate button) | calculated, or blank | independent split-boundary override; drives a live bucket recompute when it meaningfully differs from the calculated split — see §5 |

**Track panel** — title row has a "show legend" toggle (default hidden),
a "compass: corner / follow" toggle (default corner — §11), a "color: by
type / by speed" toggle (default type — §11), and a "show/hide map
background" toggle (default hidden, needs network — §11).

**Global**: a "speed unit: m/s / km/h / knots" selector and a "length
unit: km / nm" selector in the controls row — the former affects every
displayed speed throughout the tool (§11), the latter currently affects
only the session-length stat (§11's header stats bar). 1 nautical mile
is treated as exactly 1852m, not an approximation.

A handful of wind-estimation values are internal constants, not
UI-exposed (search for the name to find and tweak):

| Constant | Default | Meaning |
|---|---|---|
| `WIND_EST_TOP_SPEED_COUNT` | `2` | §5 step 2 — highest-max-speed bins averaged for each bucket's top-speed vector |
| `WIND_EST_MIN_BUCKET_RATIO` | `0.4` | §5 step 1b — smaller bucket's distance as a fraction of the larger, below which the split is too skewed to trust |
| `SPLIT_GAP_MIN_WIDTH_DEG` | `15°` | §5 step 1 — angular width of the required gap window at at least one pole of a candidate split |
| `WIND_INDICATOR_MIN_SAMPLES` | `1` | §5 step 5 — minimum sample count needed in *each* direction |
| `WIND_INDICATOR_MIN_DIFF` | `2` m | §5 step 5 — minimum average-distance gap between the two directions |
| `WIND_INDICATOR_MAX_DURATION` | `60` s | §5 step 5 — likely a recording pause, not a real maneuver |
| `WIND_OVERRIDE_WEAK_SPEED_THRESH` | `15°` | §5 step 5 — both buckets' fast-vs-median deviation must be under this to trigger the override |

## 11. UI layout and mobile support

### Header stats bar

Two groups, wrapped onto its own line via a `flex-basis:100%` spacer. A
raw group — point count, duration, sampling interval — reflects the file
as loaded, unaffected by any analysis. A "Processed:" group reflects the
run-detection/classification results: session duration and session length
(first run's start to last run's end, and the summed edge distance over
that same span), run count as a percentage *of session duration*
specifically, jibe/tack counts with their failed counterparts,
crash/recovery count alongside its own summed total duration, and max
speed — derived, local-path-fit (§7), and device, each when available —
restricted to edges that are part of a run, a jibe, or a tack
specifically.

Session length (and only session length, currently) respects the
separate length-unit selector (km / nm — §10); every speed figure
respects the speed-unit selector as usual.

Recomputed on every `redrawAll()`, not just once at file load.

### Panel order

Everything is a single vertical stack (`display:flex; flex-direction:
column`): header, wind estimate, run detection, track, then the
kinematic plot, then the threshold/action controls.

Run detection's own panel follows the exact same collapsed-by-default,
"show/hide details" pattern wind estimate already established
(`setupToggle`), split into four grouped rows (vertex checks, edge
checks, warm-up/trailing-only checks, run acceptance — matching §4's own
conceptual grouping). Loose-tack success timing (§6) and local-path-fit/
speed-lag settings (§7) stay in the threshold/action controls panel.

The kinematic plots (speed, tangential acceleration, centripetal
acceleration, heading, curvature, path length deviation, tangential jerk,
centripetal jerk — eight total) live in one canvas plus a `<select>`
dropdown (`kinematicSelect`) choosing which metric renders into it. A
`KINEMATIC_PLOTS` config maps each dropdown option to the `key`/`opts`
`drawTimeSeries` needs. Pan/zoom/tap interaction (`attachTimeInteraction`)
is attached once, to the single shared canvas.

### Sticky navbar

Prev/next, the position slider, and reset-zoom live in a `position:fixed`
bar anchored to the viewport bottom (`#navBar`). A spacer div
(`#navbarSpacer`) at the end of the scrollable content reserves space so
the fixed bar doesn't cover the last panel; its height is measured from
the navbar's actual rendered height (`syncNavbarSpacer`, called on load
and on resize) rather than a guessed constant, since the bar can wrap to
two rows on a narrow screen.

Prev/next support press-and-hold: `pointerdown` fires the step once
immediately, then repeats every `120ms` after an initial `450ms` hold,
until `pointerup`/`pointercancel`/`pointerleave`. A `click` listener is
kept alongside the pointer handlers specifically for keyboard activation,
guarded by a flag so an ordinary mouse/touch tap doesn't step twice. The
buttons also suppress the mobile long-press context-menu/text-selection
callout (`-webkit-touch-callout:none`, `user-select:none`,
`touch-action:manipulation`, plus a `contextmenu` preventDefault
backstop).

### Touch and gesture handling

The map and the kinematic-plot canvas each get their own `touchstart`/
`touchmove`/`touchend` handlers, entirely separate from their existing
`mousedown`/`wheel` handlers. Single-finger drag pans (same 3px move
threshold before treating it as a drag rather than a tap, matching the
mouse path); two-finger pinch zooms, using full 2D finger distance on the
map (2D pan) and horizontal-only distance on the charts (1D time axis).
A tap with no drag selects the nearest point, same as a mouse click,
suppressed correctly through a pinch → single-finger-lift transition.

At the page level: a `<meta name=viewport>` with `user-scalable=no`
disables native pinch/double-tap page zoom; `overscroll-behavior:none` on
`html`/`body` disables pull-to-refresh and scroll-chaining bounce; and
`touch-action:none` is applied specifically to the two canvases with
custom drag/wheel handling (map plus the kinematic-plot canvas), left off
canvases with no such handling (the wind rose).

### Mini compass overlay

A small compass (`#miniRoseCanvas`, ~90×90) sits over the top-left corner
of the track map: a fixed "N" line + dot (the map is always drawn
north-up with no rotation — see §1), the wind axis and its beam-reach
perpendicular, and the wind-in-use ray. `pointer-events:none` on the
whole overlay keeps it from intercepting the map's own drag/pinch/tap
gestures underneath.

A "compass: corner / follow" toggle switches between a fixed corner
position and following the selected point's own screen position through
pan and zoom — recentered every time `drawMap()` runs, since `drawMap()`
is also called directly from the drag/wheel/touch handlers, which don't
go through the main redraw loop. In follow mode, panning or zooming the
followed point out of the visible map area hides the overlay entirely.

Below the canvas, a text readout shows the selected point's speed and its
heading relative to wind (`angleDiff180`, 0°–180°). Speed prefers
`fitDerived`'s own smoothed value (`preferFitSpeed`, shared with the
selection info box below and the speed-gradient map coloring — see
"Track color modes"), falling back to `derived`'s raw edge speed only in
the rare case `fitDerived` isn't available at that point; heading stays
edge-based regardless (§4). The two are grouped in one positioned wrapper
(`#miniRoseWrapper`), but the wrapper's own bounding box is *not* what
gets centered on the followed point — positioning instead uses the
canvas's own measured size (`offsetWidth`/`offsetHeight`) directly.

### OpenStreetMap background tiles

Optional, off by default — the one feature in the whole file that needs
live network access. Standard Web Mercator slippy-map tiles from
`tile.openstreetmap.org`.

The track itself is projected with azimuthal equidistant (§1), a
different projection than Web Mercator — but each tile draws as a single
axis-aligned image rather than needing a full quad warp. `vincentyDirect`
(the inverse of `vincentyInverse`) converts the canvas's own screen
corners back to lat/lon to figure out which tiles are needed;
`vincentyInverse` again converts each tile's own corners into the
track's ENU space to position it.

Zoom level is picked once per redraw to roughly match the current view's
own resolution (`156543.03392 · cos(lat) / metersPerPixel`, the standard
Web Mercator resolution formula, solved for zoom and rounded). Tiles are
fetched as plain `Image` objects and cached by `z/x/y` key. An
`OSM_MAX_TILES_PER_DRAW` cap (144) skips drawing entirely rather than
requesting an unreasonable number of tiles. An attribution link (required
by OSM's usage policy) appears only while the background is shown.

### Track color modes: type vs. speed

A "color: by type / by speed" toggle on the track panel. Type mode
(default) is everything described in the run detection and jibe/tack
sections above. Speed mode recolors every *eligible* edge along a blue
(slow) → green → red (fast) gradient instead, using an HSL hue sweep from
240° down to 0°.

"Eligible" excludes, regardless of mode — `edgeGradientExcluded`, which
inherits its criteria from the classification pipeline itself (§4/§6):

- A true dt gap (`edgeGapFails`).
- Anything not part of a run, and not part of a classified transition
  segment other than `crash_recovery`.

The gradient's min/max range, and every individual edge's own color
within it, both read the **local path fit's** speed (§7), not the raw
edge speed. The track legend (toggleable, same panel) rebuilds itself per
mode — type mode shows the usual swatches; speed mode shows a single
gradient bar sampled directly from the same `speedToColor` function used
for the actual edges, labeled with the range's min/max in the current
display unit.

`preferFitSpeed(i, edgeSpeed)` — the same fit-speed-with-edge-fallback
helper this gradient uses — is shared by two other UI surfaces: the mini
compass overlay's own selected-point readout, and every speed figure in
the selection info box (run max speed; jibe/tack entry, exit,
through-the-wind, min, and max speed; crash/recovery's pre-crash speed).
Heading stays edge-based at every one of these same call sites (§4).

### Speed display unit

A global m/s / km/h / knots selector (`speedUnit`, default `ms`) affects
every displayed speed *readout* via a single `formatSpeed()` /
`dispSpeed()` pair. Deliberately does **not** touch threshold inputs in
the controls panel — those are tuning parameters with a specific internal
meaning in m/s. Internally, everything stays computed in m/s throughout —
only the presentation layer converts, at the point of formatting.

### Kinematics panel

The collapsed single-canvas-plus-dropdown chart (§3's metrics) has its
own title, a toggleable legend (dynamically rebuilt per metric), and a
single "unit: X" label next to the dropdown rather than repeating the
unit on every axis label.

Horizontal gridlines subdivide the vertical range into quarters (25% /
50% / 75%), each with its own value label. The selected point's own value
is drawn directly on the canvas, at the base of its white marker line —
no separate text readout.

### Selection info panel

A second overlay on the track map (`#selectionInfoBox`), positioned on
the left, stacked below the mini compass when it's docked in its default
corner position. Shows details about whichever run, transition, or
unclassified gap the currently selected point falls within.

`findSelectedSegment(idx)` determines this: checks run membership first,
then transition membership, and if neither, treats the point as
unclassified — bounded by the nearest occupied range ending before it and
the nearest one starting after it.

Fields shown depend on category:

- **Run**: duration, average speed (distance ÷ time over the segment, not
  a simple mean of per-edge speeds), max speed with its off-wind angle,
  the off-wind angle range observed across the run, upwind distance.
- **Jibe / tack / failed attempt**: entry, through, exit, min, max — each
  as speed plus off-wind angle at that specific vertex — then duration,
  average speed, upwind distance.
- **Crash / recovery**: speed and off-wind angle on the edge just before
  the crash, then duration, average speed, upwind distance.
- **Unclassified**: duration, average speed, upwind distance.

**Upwind distance**, common to every category: the segment's net
displacement (start point to end point) projected onto the wind axis,
positive for net progress toward upwind and negative toward downwind.

## 12. Track artifacts: a taxonomy and a design not yet built

**Nothing in this section is implemented.** §7's tangential-acceleration
and arc-length deviation detectors exist and work; everything here is
design discussion toward a *correction* mechanism, which does not exist
yet — the only corrective action currently offered for a flagged point is
deletion (§7). See `diary.md` for the full taxonomy of what the detectors
are actually catching and the detailed correction design under
consideration (separating a segment's route "shape" from its "pacing"
along that route, fit independently and recombined) — summarized briefly
here: flagged points fall into at least three distinct causes (GPS
jitter, gapless stalls/in-water activity, and crash gaps already caught
by `dt`), and only the jitter case is currently believed to be a good
target for position correction.

## License

MIT — see [LICENSE](./LICENSE).
