# Windsurfing Track Inspector — Method Reference

This document specifies exactly how the inspector derives velocity, heading,
acceleration, and curvature from raw `(lat, lon, time)` GPS fixes, and the
run-detection logic built on top of them. It reflects the current state of
`gps_track_inspector.html` — a single self-contained HTML/JS file, no
dependencies, fully offline.

**What's here:** coordinate projection, sampling-interval detection
(including variable-rate devices), kinematics (velocity, acceleration,
curvature), duplicate-point cleanup, run detection (finding stretches of
consistent point-of-sail), wind direction estimation, and classification of
jibes, tacks, and crash/recovery zones in the transitions between runs.

**What's not here (yet):** flagging/data-quality review and general
sub-run straight/turn classification were both built earlier in this
project and then deliberately removed to make room for reworking run
detection from scratch on top of the current kinematics. Interpolation
(synthesizing points to force uniform sampling) was also removed — it's no
longer needed, since the kinematics now work directly on variable-dt edges.
Transition types beyond jibes, tacks, and crashes aren't classified yet —
notably, a same-side heading swing (heading up then bearing away on one
tack) never crosses a pole, so it's currently invisible to the classifier
even when tactically meaningful (§6 has more on this). All of these are
candidates for future work, informed by everything below.

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
initial bearing between two lat/lon points (Vincenty's formulas — the same
standard method behind most GPS-distance libraries), converging in a
handful of iterations per point. This was picked over the more common
approach of a single fixed metres-per-degree scale factor (an
equirectangular-style projection) for two reasons:

- **High local accuracy.** Every point gets its own exact distance and
  bearing from the origin — there's no shared scale factor that's only
  exactly right at one particular latitude and increasingly wrong moving
  away from it. A fixed-scale approach is fine at typical GPS noise levels
  for a session-sized track, but degrades in a real, systematic,
  direction-dependent way as a track gets larger or moves toward higher
  latitudes.
- **Correct headings.** Bearing comes directly from the same geodesic
  calculation as distance, so the heading used throughout the rest of this
  file (run detection, wind estimation, jibe/tack classification) is the
  true compass bearing at every point, not a flat-plane approximation that
  quietly drifts as a track moves away from the projection center.

Both properties hold at any scale — this projection would still be exact
for a 1,000 km point-to-point marathon track, not just a few-km session,
since it's built from real geodesic distance and bearing rather than a
single reference-point approximation that degrades with distance.

The origin is the **lat/lon bounding-box midpoint** of the track, computed
once from the raw loaded file — not any single trackpoint. This keeps the
coordinate system stable even if points are later deleted.

## 2. Sampling interval detection

Both computed once, at load time, from the **untouched original file** —
never recalculated after points are later deleted, since deletions only
ever widen gaps and re-deriving would erode the reference rather than
reflect the device's actual behavior.

### `baseDtMs` — the device's base rate

The minimum positive gap between consecutive raw timestamps, in ms:

```
baseDtMs = min( time[i+1] - time[i] )   over all i, where the gap > 0
```

### `dtMaxVar` — tolerance for variable-rate devices

Some devices (adaptive/smart recording) legitimately log at more than one
rate — `baseDtMs` alone can't distinguish "this device normally logs at
several different intervals" from "this specific gap is a real dropout."

At load time, every distinct positive `dt` value is counted. `dtMaxVar` is
the **highest** `dt` that still appears in **at least 10% of all
intervals** — the upper edge of what this device's normal variable sampling
looks like. If no other `dt` value qualifies (a normal fixed-rate track),
`dtMaxVar` simply equals `baseDtMs`.

```
dtMaxVar = max( dt : count(dt) >= 0.10 * totalIntervals )
         = baseDtMs   if nothing else qualifies
```

This feeds directly into run detection (§4) as the boundary between "normal
sampling" and a "true gap."

## 3. Kinematics: cause-and-effect attribution

For points **A, B, C** (three consecutive points), with edges `AB` and `BC`:

### Velocity — attributed to the edge's endpoint

```
velocity(B) = (B - A) / (t_B - t_A)     — "the speed getting here"
velocity(C) = (C - B) / (t_C - t_B)
heading(i)  = atan2(vx, vy)              — compass convention, 0°=N, 90°=E
```

Needs only the **one point before it** — valid for every point except the
track's true first point. No blending between segments: each point's
velocity is exactly its own trailing edge, so there's no smoothing lag.

`uniformSampling(i)` = does the single interval feeding this point's
velocity equal `baseDtMs`. (Used elsewhere for diagnostics; not gated in
run detection — see §4.)

### Acceleration/curvature — attributed to the point *between* the samples

Acceleration is the **cause** of the change from `velocity(B)` to
`velocity(C)` — that change happened *at B*, between the two velocity
samples, not at C. Attributing both a point's own velocity **and** the
acceleration that produced the *next* velocity to the same point (the
naive backward-difference scheme) conflates cause and effect at a single
index. So:

```
h1 = t_B - t_A          h2 = t_C - t_B
h1_eff = min(h1, baseDtMs/1000)         h2_eff = min(h2, baseDtMs/1000)
dt = (h1_eff + h2_eff) / 2

accel(B) = (velocity(C) - velocity(B)) / dt
```

**No gap-invalidation.** A segment spanning a wide gap still has a
legitimate *average* velocity — equivalent to imagining the gap sliced into
equal-velocity sub-segments we just didn't sample. So acceleration is
computed unconditionally (never suppressed to `null` because of a gap) for
every point that has a full 3-point window. If `AB` spans a real gap, the
resulting acceleration at B is a genuine, well-defined, usually large
discontinuity — exactly the signal run detection uses to break there
naturally (§4), rather than a special-cased "this is undefined" rule.

**The `h1`/`h2` cap** exists specifically for that gap case: under the
"sliced into constant-velocity sub-segments" model, velocity stays flat
through almost the entire gap, and the real transition is concentrated at
the boundary, on the scale of *one base interval* — not the gap's full
duration. Capping each side at `baseDtMs` before averaging stops a long gap
edge from artificially diluting the computed acceleration by roughly
`gap_duration / baseDtMs`. It's a no-op for any normal (non-gap) window,
where `h1`/`h2` already equal `baseDtMs`.

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

This was checked against a case with a known-exact answer (uniform circular
motion, then a spiral with changing speed added). For constant-speed
turning, projecting onto `velocity(B)` or `velocity(C)` gives *algebraically
identical* results (`cross(AB, accel) = cross(BC, accel)` always holds), so
either works. But once speed is also changing — the realistic case for a
tack or jibe — the average matched a precise reference curvature roughly an
order of magnitude better than `velocity(B)` alone.

`derived[i].speed` (the point's own displayed/charted speed) is unaffected
by any of this — it's still exactly `velocity(B)`, unchanged.

### Coverage

| Quantity | Valid range | Reason |
|---|---|---|
| `velocity`, `heading` | index 1 … n−1 | needs one point before |
| `acceleration`, `curvature` | index 1 … n−2 | needs a point on **each** side |

The true track start (index 0) never has any derived data. The true track
end (index n−1) has velocity/heading but no acceleration/curvature (no
point after it to complete the window).

## 4. Run detection

A **run** is a stretch of track where the general point of sail stays
consistent — tolerating real-world wiggle (waves, hand-steering, GPS
jitter) without fragmenting, while still breaking on a genuine turn, stall,
bad-data edge, or gap.

**Edge-based**, matching the kinematics attribution above: "edge `i`" is
the segment from point `i-1` to point `i`; its velocity/heading is
`derived[i]`. `curvature(i)` describes the *transition* at point `i`,
between the edge ending there and the edge starting there.

Two angle thresholds do different jobs:
- **narrow angle** (`runNarrowHeadingThresh`, default `10°`) — validates a
  candidate warm-up window as a genuinely stable direction, and separately
  validates a run's own trailing edge (see step 6).
- **wide angle** (`runBreakHeadingThresh`, default `90°`) — once a run's
  reference direction is fixed, how far heading can wander from it before
  that's a real point-of-sail change, not just wiggle. Set intentionally
  generous — see the walk-back mechanism in step 5, which is what makes a
  large tolerance here safe rather than just permissive.

### State machine

1. **Break point.** Start at the track's first point, or the end of a
   previously found run.

2. **Warm-up.** From the break point, accumulate edges until `runMinSec`
   (default `5s`) of duration is covered.

3. **Warm-up validation — heading & edge checks.** Every edge in the
   window (**not** including the edge feeding *into* the break point
   itself — that belongs to whatever came before) must:
   - agree with the window's own narrow-angle average heading, and
   - pass the *edge* check (see below: low speed, device disagreement, or
     a true gap).

   Any failure aborts the whole window — the break point slides forward by
   **one point**, and warm-up is retried from scratch.

4. **Warm-up validation — curvature.** Checked at every vertex in the
   window **except its own start**: the start is already a break point, so
   high or undefined curvature there is expected — that's what makes it a
   break in the first place. An **inner** vertex failing curvature aborts
   warm-up (same as step 3). But a failure exactly at the window's own
   **end** is *not* a failure — it means this is a minimal candidate run,
   terminating right there; skip extension (step 5) entirely.

5. **Extension, with walk-back.** One point at a time, against the
   now-**fixed** warm-up reference direction:
   - the new edge must agree within the **wide** angle, and pass the edge
     check;
   - if the edge passes, its endpoint **joins** the run, and *then* gets
     its own curvature checked: passing means keep extending (tracking
     the point of highest `|curvature|` seen so far along the way);
     failing means the point **stays included**, but extension stops
     there.
   - if the wide-angle check specifically fails: instead of ending the run
     at the point right before the failure, **walk back** to wherever
     curvature peaked during this extension (if anywhere) and end the run
     there instead. A gradual, real course change can take a long time to
     accumulate past the wide-angle threshold — by the time it does, the
     actual turning may have already happened and finished well earlier,
     with the boat sailing close to straight again by the time the
     cumulative deviation alone finally trips the threshold. Walking back
     recovers the true turn location regardless of how long that took.
     Ties in the peak search favor the **latest** candidate — critical for
     a uniform, constant-rate drift with no distinguishable peak at all:
     without that tie-break, walking back would collapse the run to almost
     nothing at the very first point of the drift, defeating the entire
     purpose of a generous wide-angle tolerance. With it, undifferentiated
     drift is left exactly where it would have stopped anyway, and only a
     genuine, distinguishable spike gets preferentially selected. If the
     wide-angle check fails on the very first candidate point (nothing
     ever joined the run during extension, no walk-back candidate exists),
     the run simply ends at the warm-up window's own end, same as before.
     Either way, once walk-back happens the termination is attributed to
     `'curvature'`, not `'heading'`, for the purposes of step 8's resume
     logic — it now represents a located, real turning point rather than
     bare cumulative drift.
   - if the edge fails on speed/device-disagreement/gap instead, the new
     point is simply **excluded** — no walk-back, the run ends cleanly at
     the edge's start point (this failure mode already pinpoints a
     specific problem edge, nothing to relocate).

6. **Trailing straightness.** Before accepting the result, the run's own
   *tail* — its last `runTailSampleEdges` (default `5`) edges — must
   itself be narrow-angle self-consistent (checked against its own local
   mean, not the run's original reference). Even after step 5's walk-back,
   the tail can still drift slightly within the narrow band without
   tripping anything upstream — this catches that and shrinks the run back
   until the tail is clean again (down to, never below, the warm-up
   window's own end).

7. **Boundary shrinking (reporting only).** The run's *reported*
   `startIdx`/`endIdx` are pulled back by one edge on each side from what
   was actually validated (`breakPoint+1` and `runEnd-1`, falling back to
   the full unshrunk extent if that would invert the range). Two
   independent reasons: the break point itself is completely **exempt**
   from every check (that exemption is exactly what allows it to *be* a
   break point) — its own edge may not represent settled sailing at all.
   And a run's last edge, if reached via extension, was only held to the
   *wide* angle, not the narrow one. This is purely a reporting-layer
   decision — the internal search (including where the *next* run's break
   point starts, step 8) still uses the full, unshrunk extent, so nothing
   about the underlying validation changes. The practical payoff: the
   transition zone between two runs now naturally includes both of these
   edges automatically, which is exactly what §6's jibe/tack classifier
   needs as lead-in/lead-out, without any separate lookup into the
   adjacent run's own boundary. It also means two runs that internally
   shared a breakpoint (validation found them touching with zero gap) now
   get reported with a small one-point gap between them, rather than
   appearing to touch.

8. **Minimum duration.** The finished run (full, unshrunk extent) is
   checked against `runMinDuration` (default `12s`).
   - **Kept:** push the shrunk report (step 7), and resume the next
     search at the run's own **unshrunk** end — runs can legitimately
     **share a boundary point** internally (the point itself didn't fail
     anything; only the edge *leaving* it did), even though step 7 means
     that shared point won't appear in either run's own *reported* range.
   - **Abandoned** (too short): where to resume depends on *why* it ended.
     A **heading**-caused ending means the window's reference direction may
     have been a poor fit — retry from just **one point later**, giving a
     nearby start a chance at a better, longer-lived interpretation. A
     **velocity/curvature/gap**-caused ending is a genuine physical event —
     retrying nearby would hit the same wall, so skip straight to the run's
     own end instead. (A walked-back wide-angle ending counts as
     `'curvature'` here, per step 5.)

9. Track end can be reached either mid-run or between runs — no special
   handling needed; the loop just stops.

### Reconnection: revisiting walk-back breaks once the full picture is known

The walk-back in step 5 is doing its job correctly when it locates a real,
sharp turning point — but sometimes that "peak" is itself just a mild,
unremarkable deviation (a gentle jibe entry, say), only distinguishable
from noise in hindsight, once both sides of the break are known. So, after
the main search loop finishes and the full run list exists: revisit every
run flagged `wasWalkedBack`, and reconnect it with the next run if —

- lead-in and lead-out (the two runs' own reported boundary points) differ
  by less than `reconnectAngle` (default `30°`) — its **own** threshold,
  deliberately looser than the warm-up narrow angle: the two sides of a
  real gentle jibe entry can differ more than warm-up's strict
  self-consistency standard while still clearly being "basically the same
  run," and
- nothing else in the transition zone between them would have
  disqualified it anyway — re-run the same `edgeFails`/`curvatureFails`
  checks `computeRuns` already uses across every point from one run's end
  to the other's start (a genuine gap, stall, device disagreement, or a
  curvature spike past `runCurvThresh` all still block reconnection).

If both hold, the two runs merge into one spanning the full original
range (including whatever sat between them), inheriting the second run's
own `wasWalkedBack` flag and termination reason. This runs to a fixed
point, not just a single pass — a reconnected run can itself have
inherited a walk-back flag from its new right-hand neighbor, enabling a
further chained merge (three or more short, gentle breaks in a row all
collapsing back into one run).

### The edge check

Beyond curvature and heading, an edge also fails (breaking the run) if:

- **Low speed** (`runLowSpeedThresh`, default `0.3 m/s`) — `speed ≤`
  threshold. Too slow to still be "sailing."
- **Device disagreement** (`runDeviceDisagreeThresh`, default `5 m/s`) —
  `fit_speed − device_speed ≥` threshold, only when device speed is present
  on the track at all.
- **True gap** — the edge's own interval exceeds `dtMaxVar + baseDtMs`
  (§2). The `+ baseDtMs` is deliberate slack: even a well-behaved
  fixed-rate track can transiently drop a single sample (a brief GPS
  quirk) without that being a real gap. At 1Hz with no other variable rate
  (`dtMaxVar == baseDtMs == 1000ms`), the effective threshold is `2000ms`
  — a 2s edge is tolerated, a 3s edge isn't.

All three are checked the same way heading is: across every edge during
warm-up, and against the new edge during extension. A curvature failure and
an edge-check failure differ in *when* they're checked (curvature also
applies to the warm-up window's own start/end vertices, per step 4) and in
resume behavior (curvature/edge failures skip to the run's end on
abandonment; only heading failures retry one point later).

### No gap-invalidation gate, deliberately

Earlier in this project, points near a gap were specially excluded from run
membership. That's gone: a point right after a gap is judged on its own
merits — if the gap-spanning average velocity implies a real discontinuity,
the ordinary curvature/heading/edge checks above catch it and break the run
there naturally, the same way any other real turn would. The true-gap edge
check (above) exists as a backstop specifically for gaps *without* an
accompanying heading/curvature signal (e.g. a stall during a dropout).

## 5. Wind direction estimation

A sailing session typically covers both tacks roughly evenly, against one
roughly-steady wind. The estimator leans on that: find where the compass
splits most evenly into two halves of sailing distance, subject to a
physical sanity check (a genuine gap near at least one pole of the split),
analyze each half, refine the split against what that analysis found, and
combine what the refined halves say. All of this uses **run edges only**
— turns and transitions are deliberately excluded, since they'd smear
heading across the whole compass and obscure everything below.

This replaced an earlier approach (finding two separate cluster *peaks* via
a sliding-window search) that turned out to be more machinery than the
problem needed.

### Step 1 — find a rough balanced-split axis

Build a distance-weighted heading histogram (360 bins of 1° each, plus the
max speed seen in each bin — see §5's histogram note below) — straight-
line distance per edge accumulated into the bin its heading falls in. Then
rotate a single 180°/180° divider around the compass, in 1° steps, looking
for the rotation where the two halves' total distance is most equal.

Balance alone isn't sufficient — the search is otherwise free to place the
boundary *through* a single real cluster, splitting one direction of
sailing roughly in half and looking artificially balanced even though
there's really only one tack's worth of data, not two symmetric ones
(confirmed on real tracks, not just a theoretical worry). The fix: a
candidate split only qualifies if there's a genuine wide gap — every bin
across a real `SPLIT_GAP_MIN_WIDTH_DEG` (`15°`) window at or below 2% of
the histogram's peak bin — at **at least one** of the divider's two poles
(the axis itself or its antipode), not necessarily both. A real no-go
zone is reliably present (a boat physically can't point there), but a
clean gap on the downwind side isn't guaranteed the same way — plenty of
real sessions include some genuinely dead-downwind sailing, which fills in
what would otherwise be a gap there. Requiring a wide gap at only one pole
still rules out the actual failure mode this check exists for (cutting
through the middle of one continuous cluster, which never has a real gap
near *either* pole) just as well as requiring both would, without
rejecting valid splits that simply don't happen to have a clean downwind
gap. Checking a real angular window, not a single bin, matters for its
own reason: at 1° resolution a single-bin check would be far more exposed
to one stray edge's noise than a real window is.

If no rotation has a wide gap at either pole, there's no estimate.

### Step 1b — skewness check

Even at the best (most-balanced) rotation found, if the smaller side is
still under `WIND_EST_MIN_BUCKET_RATIO` (`0.4`, i.e. `40%` — an internal
constant, not UI-exposed; see §9) of the larger side's distance, the whole
premise — a session covering both tacks roughly evenly — doesn't hold well
enough to trust the split. No estimate. Checked against the rough split
from step 1, before any refinement — this is a question about whether the
session itself has the right shape at all, not about exactly where the
boundary ends up sitting.

### Step 2 — analyze each bucket, from the histogram

The split axis divides the compass into two buckets (half-circles). For
each bucket, computed from the two histograms built in step 1 — not by
re-scanning every edge:

- **Top-speed vector**: vector average of the bucket's own
  `WIND_EST_TOP_SPEED_COUNT` (`2`, an internal constant — see §9)
  highest-max-speed *bins* — scoped to just this bucket, not a global
  top-N. A broad reach is typically a boat's fastest point of sail on
  either tack, so this is a proxy for "which way is more downwind, within
  this bucket."
- **Median heading**: the distance-weighted **median**, not mean. A
  vector-sum mean is sensitive to the total mass of scattered small-
  distance edges at unusual headings (warm-up wobble, transition-zone
  leakage, noise) — individually negligible, but a long tail of them can
  still drag a resultant vector's angle away from where the real bulk of
  sailing actually is. A weighted median only cares about where cumulative
  distance crosses 50%, which a scattered tail barely moves unless it's a
  large fraction of the total. Computed by sorting the bucket's bins by
  position relative to the bucket's own axis (linearizing the half-circle
  without wraparound ambiguity, since a bucket never spans more than 180°)
  and walking the sorted list until cumulative distance reaches half the
  bucket's total.

If either bucket has no valid bins, no estimate.

**Why bins, not edges, and why 1° specifically.** Computing straight from
every edge was the original approach; it's now done from the histogram
instead, at 1° resolution, purely to make bucket analysis cheap enough to
re-run repeatedly (the manual tack-split override below redoes this same
computation live, every redraw, for whatever axis the person is currently
trying). A 1°-wide bin is finer than real GPS heading noise between
consecutive points, so this shouldn't lose meaningful accuracy versus the
edge-based version — verified against synthetic tracks with known cluster
centers and top-speed directions, both recovered closely by the
bin-based version.

### Step 3 — refine the split against the wind axis

The two buckets from step 2 give a first estimate of the wind axis:
bisect their median headings (circular mean, short arc). But the rough
split from step 1 only had to be *balanced and gapped* — nothing requires
it to actually sit on the true boundary between the two tacks. A real
session's two tacks are symmetric about the wind, not about wherever the
balanced-split search happened to land, so the wind axis just computed is
very likely a better dividing line than the rough one.

So: re-split using this wind axis as the new boundary, redo step 2's
bucket analysis against it (new top-speed vectors, new medians — the
medians can genuinely change here, even if only by a few degrees, since
some bins near the boundary can end up reassigned to the other bucket),
and bisect those *refined* medians for the wind axis actually used from
here on. If either refined bucket ends up with no valid bins, no
estimate.

This one refinement pass — not a full iterate-to-convergence loop — is
what actually corrects the case this whole step exists for: a tack with a
dense dominant heading cluster plus a smaller, gap-separated "deep"
sub-cluster (moments sailed noticeably more downwind than that tack's
typical heading). The rough split can end up on the wrong side of that
internal gap, pulling the deep samples into the *other* bucket and biasing
both medians. Re-splitting against the wind axis — derived from the
overall bulk of both sides, not an arbitrary balance point — tends to land
back on the correct side of that internal gap instead. Confirmed against
a real track that showed exactly this pattern: the refined estimate
correctly reabsorbed the deep samples into their own tack.

### Step 4 — pick a downwind reference, no agreement required

For each (refined) bucket, compute the signed shortest rotation from its
top-speed heading to its own median heading (`headingDiff(avgHeading,
topSpeedAngle)` — positive/negative indicates rotation direction, not
magnitude). Since top-speed sailing is presumed more downwind than the
median (broad reach outpaces close-hauled), this rotation points "away
from downwind, toward upwind" within that bucket — and since the two
buckets are mirror images of each other about the true wind axis, that
same upwind-ward move should rotate in *opposite* compass directions in
the two buckets, on a clear session.

An earlier version required exactly that (opposite signs) as a hard
pass/fail gate, and rejected sessions it shouldn't have — a session with
lots of struggles and breaks, or just more time spent on one tack than the
other, can leave one bucket's fast-vs-median signal noisy (small,
unreliable, occasionally even pointing the "wrong" way) even when the
other bucket's signal is perfectly clear on its own. Requiring both to
independently agree threw away a clean signal because a noisy one
disagreed with it.

Default approach (when the weak-speed override in step 5 doesn't apply):
no agreement check at all. Instead, trust whichever bucket shows the
**larger** separation between its top-speed heading and its median
heading (`Math.abs(rotA) >= Math.abs(rotB)`) — that's the bucket with the
clearer signal, and its relationship to the wind axis (closer to the
axis, or to the axis's antipode) determines which side is downwind. For a
balanced session both buckets point to the same answer anyway, so this
changes nothing there; for a lopsided one, the weaker side simply no
longer gets an equal, noisier vote.

Wind direction is the antipode of whichever side this reference bucket's
top-speed vector sits closer to — unless step 5 below overrides it.

### Step 5 — transition-distance cross-check, and a weak-speed override

Everything above leans on speed: top-speed sailing is presumed more
downwind than the median. That assumption breaks on **overpowered
tracks**, where a boat can point upwind just as fast as it reaches — the
whole premise step 4 relies on stops holding, even though the session
itself is otherwise completely normal.

`computeTransitionDisplacementIndicator` is a second, genuinely
independent signal that doesn't touch speed at all: for **every**
transition (any gap between consecutive runs — jibe, tack, failed
attempt, crash, or unclassified, all treated the same, since the point is
to be classification-independent), draw a vector from the transition's
start (the preceding run's own end) to its end (the next run's own
start), and project it onto the wind axis being tested. A jibe typically
carries speed smoothly through a downwind-crossing turn; a tack often
involves slowing through or near head-to-wind — so transitions that
net-displace toward downwind should, on average, cover **more distance**
than ones that net-displace toward upwind, regardless of how fast the
boat can point upwind while actually sailing. Whichever direction shows
the larger average is what this indicator calls downwind.

Two safeguards before trusting it:

- **Sample counts in both directions.** A track that's nothing but tacks
  (or nothing but jibes) would have zero or near-zero samples on one
  side, and comparing "average of many" to "average of one or two" isn't
  meaningful. Requires at least `WIND_INDICATOR_MIN_SAMPLES` (`1`)
  transitions *and* at least `WIND_INDICATOR_MIN_DIFF` (`2m`) difference
  between the two directions' averages before calling itself reliable —
  the same single pair of thresholds used everywhere this indicator is
  consulted, not two separate criteria for what's fundamentally the same
  reliability question. Both are internal constants now, not UI-exposed —
  see §9.
- **Excluding long transitions.** Any transition over
  `WIND_INDICATOR_MAX_DURATION` (`60s`) is skipped entirely before it can
  contribute to either side's sum — a long gap between runs is more
  likely an intentional recording pause (stopping the watch to rest,
  adjust gear) than a real maneuver, and the boat can end up anywhere
  after resuming for reasons that have nothing to do with jibe/tack
  dynamics. One such transition can badly distort an average built from
  only a handful of samples.

**Weak-speed override.** If step 4's own signal is weak in *both* buckets
— `Math.abs(rotA)` and `Math.abs(rotB)` both under
`WIND_OVERRIDE_WEAK_SPEED_THRESH` (`15°`, an internal constant) — the
transition-distance indicator gets tried (testing the wind axis from step
3 as the downwind hypothesis) before falling back to step 4's
reference-bucket logic. If it's reliable by the criteria above, its
answer is used instead; otherwise step 4 proceeds exactly as described
there. Every value the override used is preserved on the returned
estimate (`usedTransitionOverride`, `overrideIndicator`).

This indicator is *also* available independent of whether it triggered an
override, via `computeTransitionDisplacementIndicator` — useful even when
the override never fires: a track can have a perfectly strong speed
signal and still show a disagreeing transition-distance signal, worth
knowing about even if nothing acts on it.

### Manual controls: wind direction and tack split

Two separate number textboxes, wind direction and tack split, sit side by
side in the wind panel — deliberately independent of each other, since
they answer different questions. Wind direction says which way is
upwind; tack split says where the boundary between the two tacks sits.
Trusting the calculated split but disagreeing with which side is upwind
is common (flip the answer without moving the boundary); distrusting
exactly where the split landed while leaving the up/down call alone is
also common. Coupling them into one control would force overriding one
to silently override the other.

Both auto-initialize once per freshly-loaded track (`resetWindDirToCalculated`
/ `resetTackSplitToCalculated`, called from `loadFromText` after dedup
settles) to whatever the calculated pipeline found — wind direction from
`est.windDirection`, tack split from `est.splitAxis`. Note `splitAxis` can
be valid even when the overall estimate isn't fully `ok` (several failure
paths — skewed split, an empty bucket, a degenerate bisector — still carry
a real `splitAxis` forward), so tack split's own reset checks `splitAxis`
directly rather than `est.ok`. Both are left blank if their respective
calculated value is entirely unavailable. From then on each holds
whatever the person has typed, including through ordinary threshold
tweaks on the same track, which touch neither. Blank wind direction means
no wind direction, exactly like an undetermined calculated estimate —
transitions go unclassified.

Each has its own "recalculate" button (`resetWindDirToCalculated` /
`resetTackSplitToCalculated`). Wind direction's recalculate is aware of
an active tack-split override: if the split has been manually overridden,
recalculating wind direction derives it from *that* split's own buckets
(via `computeWindDirectionFromBuckets`, the same step 3/4 logic factored
out so it can run against any bucket pair) rather than silently reverting
to the fully-automatic `est.windDirection` — otherwise "recalculate"
would discard the split override the moment wind direction was touched.
Wind direction also has a "flip 180°" button, rotating whatever's
currently in the box (a no-op when blank).

**Split override triggers a live bucket recompute.** Whenever the tack
split textbox meaningfully differs from `est.splitAxis` (or stands in for
an undetermined one), `redrawAll` redoes the bucket analysis against the
manual split — using the same fast histogram-based `computeBucketStats`
from step 2, which is exactly why that step moved off raw edges in the
first place: this recompute has to happen on every redraw, including
live as the textbox is edited, not just once per full estimate.
Deliberately keyed off the split control specifically, not wind
direction — overriding wind direction alone just changes which side is
called downwind, it doesn't mean the split boundary itself needs to move.
These override buckets (`currentOverrideBuckets`), when active, are what
the split axis, median, and top-speed lines in both rose diagrams
actually draw from, in place of `est.bucketA`/`est.bucketB`.

**The wind axis itself follows wind direction, not tack split.**
Independent of any split override, the purple wind-axis line in both rose
diagrams is `manualDir != null ? manualDir : est.refinedAxis` — it
rotates with a manual wind-direction edit, falling back to the calculated
axis only when the textbox is blank. This is different from the split/
median/top-speed lines above, which follow the *split* control instead.

### Off-wind angle labels

In the full rose diagram, each median line, each top-speed line, and the
wind-in-use ray are labeled with a small angle:

- **Medians and wind-in-use**: `angleDiff180(heading, windAxis)` — the
  direct angle to the wind axis pole, 0°–180°. Wind-in-use reads 0° in
  the ordinary case, since the axis is itself derived from wind direction
  whenever one is set — that's not a bug, it's confirmation the ray
  actually sits on the axis, and would show something other than 0° if
  that ever stopped being true.
- **Top-speed**: also `angleDiff180`, not the smaller axis-relative
  measure — top-speed sailing is presumed near-downwind by the model's
  own assumption, so this reads close to 180° (nearly opposite the wind),
  which a 0°–90° axis-relative number would have folded away.

### What's left undetected

The weak-speed override in step 5 directly addresses the overpowered-track
case (upwind as fast as reaching) that used to be a real gap here. What's
left, and isn't fixable by refining the per-track logic further: a
**radical wind shift mid-session** (the whole model assumes one steady
wind axis; a shift corrupts both buckets' own internal statistics, and
the transition-distance indicator along with them, not just their
agreement with each other) and a **tow** (injects non-sailing motion
directly into the histogram *and* into transition displacement alike).
Fixing the former would mean windowing the estimate over time — rolling
buckets instead of one global pass, so a shift shows up as the axis
visibly moving rather than as noise in one global answer — rather than
anything that fits within the current one-shot-per-track model. The
manual wind-direction and tack-split controls are a blunt, immediate
workaround for whatever remains, rather than a fix for either.

### Diagnostics for undetermined tracks

`computeWindEstimate` never returns a bare failure — every early exit
returns `{ ok:false, reason: '...', ...whatever was already computed }`,
so the rose diagram can render as much of the pipeline as got completed
before the actual failure point, rather than showing nothing. The split
axis, both buckets' own top-speed and median headings, and the wind axis
are all drawn (grey, gold, green dashed, and purple dashed respectively —
the split and wind axes both as full lines across the whole diagram,
since an axis has no inherent direction, unlike the directional red ray
for whichever wind direction is actually in use) whenever they exist,
with the specific failure reason surfaced in the "undetermined — please
set manually" message — useful for telling a "not enough data yet" case
apart from a "found a split but it's too skewed" case apart from a
degenerate bisector.

## 6. Jibe and tack classification

Between two consecutive runs sits a transition zone — whatever the run
detector didn't claim as belonging to either run. Given the lead-in heading
(run A's own last edge), lead-out heading (run B's own first edge), and
every point in between, a lot can be inferred about what happened there.
Currently classified: successful jibes and tacks, one failure mode for
each (an aborted attempt), and crash/recovery zones.

Because run boundaries are already pulled back by one edge on each side
(§4 step 7), the transition zone's own boundary points — `runA.endIdx` and
`runB.startIdx` — serve directly as lead-in/lead-out. No separate lookup
into the adjacent run's own content is needed.

Uses the effective wind direction from §5's manual textbox — auto-initialized from the calculated estimate on each fresh track load, but the actual source of truth from then on, since it's what the user sees and can override. Not `computeWindEstimate`'s return value directly.

### Crash detection comes first, and doesn't need wind direction at all

Before attempting any jibe/tack classification, `scanForCrash` scans the
**whole** transition zone for the first edge spanning a true gap (the same
`edgeGapFails` check §4 uses, exposed at top-level scope specifically so
this can reuse it). This isn't a lookup of *why* the preceding run ended —
a run can end for any reason (a clean heading break, low speed, whatever)
and a crash can still occur later, mid-transition (losing control mid-jibe,
say). Checking the reason a run ended would miss that entirely.

If no crash is found, jibe/tack classification proceeds normally across
the whole zone (below). If one is found:

- Whatever's strictly before the crash edge may still qualify as a
  **failed** attempt (see below — evaluated with success disallowed,
  since the crash cuts off any chance of observing a real lead-out to
  confirm completion).
- The crash edge itself onward is always `crash_recovery`, regardless of
  what came before it. The marking starts one edge before the crash's own
  endpoint — at the edge's *own* start — so the anomalous-dt edge itself
  is included in the marking, not just its endpoint onward.

A single transition can therefore produce **two** results: a failed
attempt for the good part, and a separate crash-recovery entry for the
rest. Deliberately checked before, and independent of, jibe/tack — a
transition that starts with a genuine data dropout shouldn't be trusted to
represent a clean, continuous maneuver, since curvature computed across a
real gap (deliberately not null-invalidated — see §3) could coincidentally
look consistent enough to pass the jibe/tack scan by chance.

### Sign convention

Verified numerically against this file's actual curvature formula: a
**right** turn (heading increasing) produces **negative** curvature; a
**left** turn (heading decreasing) produces **positive** curvature —
matching the existing chart labels ("+left turn / −right turn"). So
`expectedCurvatureSign = -sign(headingChange)`.

### A jibe and a tack are the same test, aimed at opposite poles

A jibe crosses through **downwind**; a tack crosses through **upwind**.
`classifyPoleInRange(rangeStart, rangeEnd, poleDirection, successType,
failType, allowSuccess)` holds the entire shared logic below,
parameterized by which pole to aim for, what type strings to return, and
whether a success classification is even possible for this call (`false`
when the range was truncated by an upcoming crash). `classifyTransition`
tries the jibe pole first, and if that returns nothing, tries the tack
pole — the two are mutually exclusive by construction (see next section),
so at most one can ever match a given transition.

### Expected turn direction — from lead-in alone

Whichever rotation reaches the target pole first (the shorter arc, from
the lead-in heading) is what this maneuver requires — a strict left/right
test relative to the wind axis, not a proximity test against either pole.

This is deliberately computed from lead-in **only**, not from any
relationship between lead-in and lead-out. An earlier version required both
to sit on opposite sides of the target pole's axis, and that was fragile in
two ways: a genuine maneuver's captured lead-out edge can already be a few
degrees into settling on the new course, and a *failed* attempt may recover
onto nearly its original heading rather than completing the turn — both
cases could put lead-out on the "wrong" side even though the maneuver
itself was real.

Since turning toward one pole and turning toward the other are always
opposite rotations (except from the single degenerate beam-reach heading,
exactly 90° from both poles), this construction also means a jibe-shaped
transition can never simultaneously satisfy the tack test, or vice versa —
they require opposite curvature signs from the same lead-in.

### The scan: curvature consistency, AND a low-speed check

Two conditions are checked at every point across the range, and they're
marked differently when they break it:

- **Curvature** must match `expectedCurvatureSign`, with a small
  tolerance (`CURVATURE_BREAK_TOLERANCE = 0.02`): a point still counts as
  consistent if its curvature matches the expected sign (no magnitude
  limit, as always), **or** if it's within `0.02` of zero even on the
  wrong side — absorbing a brief near-zero or slightly-wrong-sign blip
  (GPS noise, a momentary flat spot mid-turn) without treating it as a
  genuine loss of control. `null` (undefined) curvature always breaks the
  scan regardless — the tolerance is about magnitude near zero, not a way
  to paper over missing data.
- **Speed** must exceed the same low-speed threshold run detection uses
  (`runLowSpeedThresh`). A stalled edge means the boat isn't turning
  anymore, it's stopped — a different kind of "this attempt ended" than a
  curvature deviation.

The two break kinds are marked with different *inclusion* semantics for
the attempt's own reported extent (`attemptEndIdx`):

- A **curvature** break is **inclusive** — the high-curvature point itself
  is part of the maneuver's story (it's the moment things went wrong
  *during* the turn), so `attemptEndIdx = breakIdx`.
- A **speed** break is **exclusive** — the stalled edge isn't part of the
  attempt at all, so `attemptEndIdx = breakIdx - 1` (its own start — the
  last point still actually moving).

Either way, *qualification* (did enough happen to call this an attempt)
is judged against the last reliably-consistent point (`preBreakIdx =
breakIdx - 1`) regardless of break kind — only the reported/rendered
extent differs.

- **No break at all**: curvature held and speed stayed up the whole way
  through. But a genuine maneuver also needs a substantial **net** heading
  change, not just a brief consistent wiggle that happened to stay
  unbroken — so lead-in to lead-out must differ by at least **60°**, or
  this is left unclassified (or, if `allowSuccess` is `false` because a
  crash truncated the range, judged as an attempt using the same 2s/10°
  bar below instead of ever being called a success). If it clears the 60°
  bar: **jibe** / **tack**.
- **A break is found**: check the point just before it (`preBreakIdx`).
  If at least **2 seconds** elapsed since lead-in *and* at least **10°**
  of heading change was achieved by that point, this is a **failed
  attempt** — the boat committed to the turn for a real stretch before
  losing it. Lead-out heading is deliberately not checked at all for this
  case (aborted attempts may just resume the original course). Otherwise:
  too little happened before the break to call it an attempt at all, left
  unclassified.

None of the thresholds above (60° net change, 2s, 10°, 0.02 curvature
tolerance) are currently exposed in the UI — they're fixed constants in
`classifyPoleInRange`.

### Loosened success path, tacks only

Real tacks are often fast, and the strict curvature-consistency scan above
can be thrown off by momentary noise (hanging near head-to-wind, a brief
stall or sail-drop) even when the maneuver clearly succeeded by every
other measure. If the strict scan doesn't already call a transition a
`tack` success, `checkLooseTackSuccess` gets a chance to upgrade the
result (from either `null` or `failed_tack`) if **all** of:

- Net heading change (lead-in to lead-out) still clears the same **60°**
  bar.
- Total time spent at or below the low-speed threshold, summed across the
  whole transition, is under `looseTackMaxStallTime` (default `5s`) — a
  cap, not a free pass: a brief stall or sail-drop mid-tack is normal and
  shouldn't by itself disqualify an otherwise-real tack, but extended low
  speed still should.
- The whole thing took under `looseTackMaxDuration` (default `20s`).
- **Entry** heading is itself upwind-ish (`|headingDiff(entry,
  windDirection)| < 90°`) — that's what makes this tack-shaped to begin
  with.
- Entry and exit sit on **opposite** sides of the wind axis — a genuine
  crossing.
- The boat's actual **net displacement** over the whole maneuver — a
  vector drawn from the entry point straight to the exit point — has a
  positive component toward the wind (dot product with the upwind unit
  vector `> 0`).

No requirement on **exit heading** at all, deliberately — a single
instantaneous heading reading can be noisy, especially coming out of a
stall or sail-drop. The net-displacement check is a direct physical
measurement of "did the boat actually make upwind progress," unaffected by
how noisy any single heading sample was — a genuinely more robust signal
than checking where the boat happened to be pointed at the last sample.

This path is tack-only. Jibes always go through the strict scan alone.

### Marking the crossing point, and the speed extremes

Beyond classifying a transition's type, two further things are tracked
for every jibe, tack, and failed attempt, each as a post-processing pass
over the already-classified transitions (kept separate from the delicate
success/failure logic above, rather than folded into it):

- **The crossing vertex ("through").** The last vertex still on the
  original side, just before heading actually crosses the relevant pole
  (the downwind pole for a jibe, the upwind pole for a tack) — found by
  walking the transition's edges and checking where the signed angular
  difference to that pole (`headingDiff(heading, pole)`) changes sign
  between consecutive edges. `null` for a failed attempt that broke off
  before the heading ever actually swung past its pole — there's nothing
  to mark in that case. Marked on the map as a small filled dot in the
  maneuver's own color, at whichever vertex `headingDiff`'s sign change
  identifies.
- **Min and max speed vertices**, found independently of wind direction
  (unlike the crossing point, which needs a pole to test against) by a
  straightforward scan of the transition's own edges. Marked as thin
  stroked circles (not filled, to stay visually distinct from the solid
  through-dot) in the same color, sized slightly larger than the
  through-dot so the two remain distinguishable on the rare edge where a
  speed extreme and the crossing point coincide.

The selection info panel (§10) surfaces the actual speed and off-wind
angle at each of these points — entry, through, exit, min, max, in that
order — for whichever transition the currently selected point falls
within.

### A known, accepted limitation

A same-side heading swing — heading up to build speed, then bearing away
hard, all on one tack — is deliberately **not** treated as a maneuver at
all by this classifier, no matter how large in raw degrees: it never
crosses either pole, so neither the jibe nor tack test can fire. That's
intentional (§4's run detector already keeps this as a single run, and
that's the right call — see the walk-back discussion in §4 step 5). But it
means genuinely interesting same-side moments (the bear-away point in a
speed-run buildup, for instance) aren't marked as anything by this layer
yet — they're just wherever curvature happens to look like inside one
continuous run. Separately: a real jibe that ends up briefly clew-first
before the sailor switches feet, versus a heading-only wiggle that dips
across the pole and back without any real maneuver, can in principle
produce very similar-looking kinematic signatures — GPS position and time
alone can't always distinguish "a human changed stance" from "the boat's
heading briefly touched the other side." Treated as an accepted limit
rather than something a purely kinematic rule should be expected to solve.

## 7. Duplicate cleanup: timestamps and coordinates

Two distinct data artifacts, each handled separately, since neither
cleanup catches the other's failure mode.

### Duplicate timestamps

Consecutive points sharing an identical timestamp — regardless of whether
their coordinates also match — mean `dt = 0` for that edge, which means
`speed = distance / 0 = Infinity` downstream in the kinematics, corrupting
run detection, wind estimation, and anything else that reads speed from
there on. This is a distinct failure mode from duplicate coordinates
(below): a logging glitch stamping two genuinely different fixes with the
same timestamp is a different bug than a device re-logging one fix
repeatedly, and the coordinate-based cleanup wouldn't catch it, since the
colliding points' positions can differ.

Runs automatically and unconditionally — not a togglable preference like
the coordinate cleanup below, since this is a correctness fix for
corrupted kinematics, not a cosmetic option. Positioned deliberately in
the load pipeline: after `project()` (needs each point's own `x`/`y`) but
before `computeDerived()` (so the corrupted `dt=0` edges never reach the
kinematics at all). Points surviving removal keep their existing
projected position unchanged — `project()` computes each point
independently with no dependency on its neighbors, so removing other
points never invalidates a kept point's own projection.

For a run of points sharing a timestamp, rather than arbitrarily keeping
whichever the device happened to log first, keeps whichever *one* point
minimizes the difference between the edge distance immediately before it
(from the nearest already-kept point) and the edge distance immediately
after it (to the next distinct-timestamp point) — the point that best
preserves a smooth, consistent step size through this moment, rather than
risking an arbitrary pick that leaves a large jump on one side and a tiny
one on the other.

### Duplicate coordinates

Consecutive points sharing an *exact* identical `(lat, lon)` are a device
artifact (re-logging the same fix across several timestamps — a
"stutter") rather than real stationary motion, which still has tiny
position noise; an exact repeat is a data artifact.

Same balance principle as duplicate timestamps, transposed: there,
position varied within a run sharing one timestamp, so the balance metric
was distance; here, time varies within a run sharing one position, so the
balance metric is time. Rather than arbitrarily keeping the run's first
(earliest) point, keeps whichever *one* point minimizes the difference
between the time gap immediately before it (from the nearest already-kept
point) and the time gap immediately after it (to the next
distinct-position point) — letting the resulting wider gap be handled by
the ordinary gap/true-gap machinery instead of producing a fake
zero-velocity segment.

This runs **automatically, silently**, immediately after a fresh file
load. "Restore original track" deliberately does **not** re-run it — that
gives back the true, unfiltered file if you want to inspect what the
automatic pass found. The "remove duplicate-coord points" button remains
available for a manual re-run at any time, with a summary alert.

## 8. GPX export

Saves the current (post-editing) points as standard GPX 1.1, preserving
`<speed>` and `<ele>` where present. Filename defaults to
`<original-basename>_t.gpx`.

## 9. UI controls reference

| Control | Default | Meaning |
|---|---|---|
| run breaks if \|curvature\| ≥ | `1` /m | hard curvature threshold (§4 step 4/5) |
| heading deviates ≥ (wide angle) | `90°` | extension tolerance from the fixed reference |
| speed ≤ | `0.3` m/s | low-speed edge failure |
| derived−device speed ≥ | `5` m/s | device-disagreement edge failure (if device speed present) |
| warm-up validated at ≥ (narrow angle) | `10°` | warm-up + trailing-straightness tolerance |
| warm-up length | `5` s | minimum warm-up window duration |
| min run duration | `12` s | runs shorter than this are abandoned |
| trailing straightness sample | `5` edges | tail window size for step 6 |
| reconnect walk-back breaks if lead-in/out differ < | `30°` | §4 reconnection pass — its own, looser threshold |
| loose tack success: under | `20` s | §6 loosened tack path — total transition duration cap |
| loose tack success: ≤ | `5` s | §6 loosened tack path — total time below low-speed threshold, summed |

**Wind panel** (§5) — title row has a "show/hide details" toggle
(default hidden) covering everything below the two textboxes:

| Control | Default | Meaning |
|---|---|---|
| wind direction (textbox + recalculate + flip 180° buttons) | calculated, or blank | single source of truth for wind direction everywhere it's used; auto-set once per fresh track load, holds through threshold tweaks otherwise |
| tack split (textbox + recalculate button) | calculated, or blank | independent split-boundary override; drives a live bucket recompute when it meaningfully differs from the calculated split — see §5 |

**Track panel** — title row has a "show legend" toggle (default hidden),
a "compass: corner / follow" toggle (default corner — §10), a "color: by
type / by speed" toggle (default type — §10), and a "show/hide map
background" toggle (default hidden, needs network — §10).

**Global**: a "speed unit: m/s / km/h / knots" selector in the controls
row affects every displayed speed throughout the tool (§10).

A handful of wind-estimation values were UI-exposed while the algorithm
was under active iteration; now settled, they're plain constants in code
instead (search for the name to find and tweak):

| Constant | Default | Meaning |
|---|---|---|
| `WIND_EST_TOP_SPEED_COUNT` | `2` | §5 step 2 — highest-max-speed bins averaged for each bucket's top-speed vector |
| `WIND_EST_MIN_BUCKET_RATIO` | `0.4` | §5 step 1b — smaller bucket's distance as a fraction of the larger, below which the split is too skewed to trust |
| `SPLIT_GAP_MIN_WIDTH_DEG` | `15°` | §5 step 1 — angular width of the required gap window at at least one pole of a candidate split |
| `WIND_INDICATOR_MIN_SAMPLES` | `1` | §5 step 5 — minimum sample count needed in *each* direction |
| `WIND_INDICATOR_MIN_DIFF` | `2` m | §5 step 5 — minimum average-distance gap between the two directions |
| `WIND_INDICATOR_MAX_DURATION` | `60` s | §5 step 5 — likely a recording pause, not a real maneuver |
| `WIND_OVERRIDE_WEAK_SPEED_THRESH` | `15°` | §5 step 5 — both buckets' fast-vs-median deviation must be under this to trigger the override |

## 10. UI layout and mobile support

### Header stats bar

Two groups, wrapped onto its own line via a `flex-basis:100%` spacer
(plain `<br>` doesn't force a line break in a `flex-wrap` container). A
raw group — point count, duration, sampling interval — reflects the file
as loaded, unaffected by any analysis. A "Processed:" group reflects the
run-detection/classification results: session duration (first run's
start to last run's end — deliberately narrower than the track's own
total duration, trimmed of any warm-up, breaks, or dead time before/after
the session itself), run count as a percentage *of that session
duration* specifically (not the full track — diluting by dead time the
session itself doesn't include would understate how much of the actual
session was spent running), jibe/tack counts with their failed
counterparts, crash/recovery count, and max speed — both derived and
device, when present — restricted to edges that are part of a run, a
jibe, or a tack specifically, so a stall or a gap-triggered crash edge
can't inflate the reported maximum.

Recomputed on every `redrawAll()`, not just once at file load — folded in
after fixing a real staleness bug: it used to run once, early, before the
wind-direction textbox had been initialized from the calculated estimate,
so its own transition classification saw no wind direction and reported
zero jibes/tacks, and nothing ever refreshed it afterward.

### Panel order

Everything is a single vertical stack (`display:flex; flex-direction:
column`, not the two-column grid this file used earlier): header, track,
wind estimate, then the kinematic plot, then the threshold/action
controls. Wind estimate comes before track deliberately — it's the
smaller, usually-collapsed panel, and reading order roughly matches "set
up wind, then look at the track" rather than the reverse.

The five kinematic plots (speed, tangential acceleration, centripetal
acceleration, heading, curvature) collapsed from five separate always-
visible canvases into one canvas plus a `<select>` dropdown
(`kinematicSelect`) choosing which metric renders into it. A
`KINEMATIC_PLOTS` config maps each dropdown option to the `key`/`opts`
`drawTimeSeries` needs — the same values that used to be hardcoded per
canvas. Pan/zoom/tap interaction (`attachTimeInteraction`) is attached
once, to the single shared canvas, instead of five times.

### Sticky navbar

Prev/next, the position slider, and reset-zoom live in a `position:fixed`
bar anchored to the viewport bottom (`#navBar`), not in the ordinary
document flow — so they stay reachable after scrolling down a long
vertical page, which matters most on a phone where the whole layout above
can run several screens tall. A spacer div (`#navbarSpacer`) at the end of
the scrollable content reserves space so the fixed bar doesn't cover the
last panel; its height is measured from the navbar's actual rendered
height (`syncNavbarSpacer`, called on load and on resize) rather than a
guessed constant, since the bar can wrap to two rows on a narrow screen
and a fixed guess would then leave content covered.

Prev/next support press-and-hold: `pointerdown` fires the step once
immediately, then repeats every `120ms` after an initial `450ms` hold,
until `pointerup`/`pointercancel`/`pointerleave`. A `click` listener is
kept alongside the pointer handlers specifically for keyboard activation
(Tab + Enter/Space on a focused button fires `click` without any
preceding `pointerdown`, since pointer events are tied to pointing
devices, not keyboards) — guarded by a flag so an ordinary mouse/touch tap
(which fires both `pointerdown` and `click`) doesn't step twice. The
buttons also suppress the mobile long-press context-menu/text-selection
callout (`-webkit-touch-callout:none`, `user-select:none`,
`touch-action:manipulation`, plus a `contextmenu` preventDefault
backstop) — without that, a held press-and-repeat interaction would fight
the browser's own long-press gesture.

### Touch and gesture handling

The map and the kinematic-plot canvas each get their own `touchstart`/
`touchmove`/`touchend` handlers, entirely separate from (not replacing)
their existing `mousedown`/`wheel` handlers — lower risk of regressing
already-working desktop behavior, and pinch-zoom needed dedicated
multi-touch logic regardless of how single-pointer drag was handled.
Single-finger drag pans (same 3px move threshold before treating it as a
drag rather than a tap, matching the mouse path); two-finger pinch zooms,
using full 2D finger distance on the map (2D pan) and horizontal-only
distance on the charts (1D time axis) — each matching its own existing
wheel-zoom direction convention. A tap with no drag selects the nearest
point, same as a mouse click, suppressed correctly through a pinch → 
single-finger-lift transition so lifting the second finger after a pinch
doesn't register as a spurious tap.

At the page level: a `<meta name=viewport>` with `user-scalable=no`
disables native pinch/double-tap page zoom (which would otherwise compete
with the in-canvas gestures above); `overscroll-behavior:none` on
`html`/`body` disables pull-to-refresh and scroll-chaining bounce; and
`touch-action:none` is applied specifically to the two canvases with
custom drag/wheel handling (map plus the kinematic-plot canvas), left off
canvases with no such handling (the wind rose) so those stay normally
scrollable.

### Mini compass overlay

A small compass (`#miniRoseCanvas`, ~90×90) sits over the top-left corner
of the track map, showing just enough to orient the plotted track against
the wind without opening the full wind panel: a fixed "N" line + dot
(the map is always drawn north-up with no rotation — see §1 — so this
needs no recomputation), the wind axis and its beam-reach perpendicular
(both derived from the effective wind direction, mirroring the full
rose's own purple lines), and the wind-in-use ray. `pointer-events:none`
on the whole overlay keeps it from intercepting the map's own drag/pinch/
tap gestures underneath.

A "compass: corner / follow" toggle switches between a fixed corner
position and following the selected point's own screen position through
pan and zoom — recentered every time `drawMap()` runs (not just on
selection change), since `drawMap()` is also called directly from the
drag/wheel/touch handlers, which don't go through the main redraw loop.
In follow mode, panning or zooming the followed point out of the visible
map area hides the overlay entirely rather than leaving it floating at a
clamped or otherwise confusing position.

Below the canvas, a text readout shows the selected point's speed and its
heading relative to wind (`angleDiff180`, 0°–180°, same convention as the
full rose's own off-wind labels — see §5). The two are grouped in one
positioned wrapper (`#miniRoseWrapper`) so they move together, but the
wrapper's own bounding box is *not* what gets centered on the followed
point — that would include the readout's height below the canvas and
pull the compass circle itself off-target. Positioning instead uses the
canvas's own measured size (`offsetWidth`/`offsetHeight`) directly, since
the canvas sits flush with the wrapper's top-left corner regardless of
whatever the readout ends up sized at.

### OpenStreetMap background tiles

Optional, off by default — this is the one feature in the whole file that
needs live network access, so it stays opt-in rather than compromising
the rest of the tool's offline-first design (see §1). Standard Web
Mercator slippy-map tiles from `tile.openstreetmap.org`.

The track itself is projected with azimuthal equidistant (§1), a
different projection than Web Mercator — checked whether this actually
matters before relying on it: projected a real tile's four corners into
the track's own ENU space at a representative mid-latitude origin a few
km away, and found opposite edge lengths matched to within 0.02% and
edge angles to within 0.001°, well under anything visible. So each tile
draws as a single axis-aligned image rather than needing a full quad
warp — `vincentyDirect` (the inverse of the `vincentyInverse` the
projection itself already uses) converts the canvas's own screen corners
back to lat/lon to figure out which tiles are needed, `vincentyInverse`
again converts each tile's own corners into the track's ENU space to
position it.

Zoom level is picked once per redraw to roughly match the current view's
own resolution (`156543.03392 · cos(lat) / metersPerPixel`, the standard
Web Mercator resolution formula, solved for zoom and rounded). Tiles are
fetched as plain `Image` objects and cached by `z/x/y` key, redrawing the
map once each finishes loading; a `OSM_MAX_TILES_PER_DRAW` cap (144) skips
drawing entirely rather than requesting an unreasonable number of tiles
if something computes an unexpectedly large range. An attribution link
(required by OSM's usage policy) appears only while the background is
shown.

### Track color modes: type vs. speed

A "color: by type / by speed" toggle on the track panel. Type mode
(default) is everything described in the run detection and jibe/tack
sections above — run edges, maneuver colors, crash/recovery. Speed mode
recolors every *eligible* edge along a blue (slow) → green → red (fast)
gradient instead, using an HSL hue sweep from 240° down to 0° — a genuine
mode switch, not an overlay, so the maneuver-type distinction is
deliberately traded away in favor of a speed-focused view for as long as
the mode is active.

"Eligible" excludes, regardless of mode:

- A true dt gap (`edgeGapFails`) — the derived speed there is a
  gap-spanning average, not a real instantaneous reading.
- A derived/device speed disagreement (the same check `computeRuns` uses
  to break runs on this basis — if the two don't trust each other,
  neither does this).
- A `crash_recovery` zone specifically — real motion, but not ordinary
  sailing speed.
- Anything outside any run or transition entirely (unclassified) —
  handled for free, since excluded edges simply aren't drawn over,
  leaving the grey base path visible underneath rather than needing an
  explicit grey re-draw.

The gradient's min/max range is computed **only** from eligible edges, so
a gap or disagreement edge with an anomalous speed value can't skew the
color scale for the legitimate data. The track legend (toggleable, same
panel) rebuilds itself per mode — type mode shows the usual swatches;
speed mode shows a single gradient bar sampled directly from the same
`speedToColor` function used for the actual edges (not a separately
duplicated color formula, so the two can't drift apart), labeled with the
range's min/max in the current display unit.

### Speed display unit

A global m/s / km/h / knots selector (`speedUnit`, default `ms`) affects
every displayed speed *readout* — charts, info grid, legends, the
selection panel below — via a single `formatSpeed()` / `dispSpeed()` pair
used everywhere a speed value reaches the screen. Deliberately does
**not** touch the low-speed or device-disagreement *threshold* inputs in
the controls panel: those are tuning parameters with a specific internal
meaning in m/s, tied to the algorithm, and converting their displayed
unit without also rescaling the underlying value would be misleading —
out of scope for what "display" means here.

Internally, everything stays computed in m/s throughout — only the
presentation layer converts, at the point of formatting, never the
underlying `derived[].speed` values themselves.

### Kinematics panel

The collapsed single-canvas-plus-dropdown chart (§3's five metrics) has
its own title, a toggleable legend (dynamically rebuilt per metric — the
plotted line's actual name, "device speed" only shown for the speed plot
specifically, "gap" instead of a longer description), and a single "unit:
X" label next to the dropdown rather than repeating the unit on every
axis label — speed's unit label updates with the global speed-unit
selector above, the other four metrics show their fixed unit.

Horizontal gridlines subdivide the vertical range into quarters (25% /
50% / 75%), each with its own value label, styled identically to the
existing min/max labels for visual consistency. The selected point's own
value is drawn directly on the canvas, at the base of its white marker
line — no separate text readout — sized and colored to match, with no
unit suffix (shown once near the dropdown instead of repeated per-point).

### Selection info panel

A second overlay on the track map (`#selectionInfoBox`), positioned on
the left, stacked below the mini compass when it's docked in its default
corner position — reflowing up to fill that space when the compass
switches to "follow" mode instead and moves away. Shows details about
whichever run, transition, or unclassified gap the currently selected
point falls within.

`findSelectedSegment(idx)` determines this: checks run membership first,
then transition membership (using the same full-length-vs-attempt-cut-
short distinction the map rendering itself uses), and if neither, treats
the point as unclassified — bounded by the nearest occupied range ending
before it and the nearest one starting after it (track start/end if
there's no such neighbor on one side).

Fields shown depend on category, matching the map's own colors:

- **Run**: duration, average speed (distance ÷ time over the segment,
  not a simple mean of per-edge speeds), max speed with its off-wind
  angle, the off-wind angle range observed across the run, upwind
  distance.
- **Jibe / tack / failed attempt**: entry, through, exit, min, max — each
  as speed plus off-wind angle at that specific vertex — then duration,
  average speed, upwind distance. "Through" and the min/max vertices are
  the same ones marked on the map itself (§6) — computed once in
  `computeTransitions` and threaded through rather than re-derived here.
- **Crash / recovery**: speed and off-wind angle on the edge just before
  the crash (the last normal edge before the gap begins), then duration,
  average speed, upwind distance.
- **Unclassified**: duration, average speed, upwind distance.

**Upwind distance**, common to every category: the segment's net
displacement (start point to end point — not total distance traveled)
projected onto the wind axis, positive for net progress toward upwind and
negative toward downwind — computed the same way regardless of category,
using whatever the effective (manual textbox) wind direction currently
is.

## License

MIT — see [LICENSE](./LICENSE).
