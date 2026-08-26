# Windsurfing Track Inspector — Method Reference

This document specifies exactly how the inspector derives velocity, heading,
acceleration, and curvature from raw `(lat, lon, time)` GPS fixes, and the
run-detection logic built on top of them. It reflects the current state of
`wtrack.html` — a single self-contained HTML/JS file, no dependencies,
fully offline.

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

At load time, every positive `dt` is collected and sorted; `dtMaxVar` is
the value at the **95th percentile** of that sorted list — the upper edge
of what this device's normal variable sampling looks like. If there are
no positive `dt` values at all, `dtMaxVar` simply equals `baseDtMs`.

```
sorted = sort( dt : dt > 0, over all consecutive raw timestamp pairs )
dtMaxVar = sorted[ floor(0.95 * length(sorted)) ]
         = baseDtMs   if sorted is empty
```

Percentile-based, not mode-based — replaced an earlier version that took
the highest `dt` value appearing in at least 10% of all intervals. That
approach depended on some specific `dt` value repeating often enough to
form a clear mode, which a highly variable device might never produce (no
single interval common enough to clear the 10% bar) — leaving `dtMaxVar`
stuck at `baseDtMs` and flagging most of the track as gaps. A percentile
doesn't need any value to repeat at all; it only needs the distribution's
overall shape, so it degrades gracefully regardless of how scattered the
device's own variable rate turns out to be.

This feeds directly into run detection (§4) as the boundary between "normal
sampling" and a "true gap."

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
velocity is exactly its own trailing edge, so there's no smoothing lag.

`uniformSampling(i)` = does the single interval feeding this point's
velocity equal `baseDtMs`. (Used elsewhere for diagnostics; not gated in
run detection — see §4.)

**Jerk** — the rate of change of *acceleration* — follows this same
edge-endpoint attribution, not acceleration/curvature's vertex attribution
below, even though it's derived from acceleration: for edge `BC`
specifically, jerk describes how much acceleration changed *across* that
edge, a property of the edge itself, so it's stored at `derived[i+1]` (the
edge's own ending index) — exactly the same convention `speed(i)` already
uses for `edge(i-1,i)`.

```
jerk(BC) = ( accel(C) - accel(B) ) / dt(BC)
```

Computed as the raw `(ax, ay)` **vector** difference first, then projected
once onto edge `BC`'s own velocity — not a scalar difference of the two
already-projected `tanAccel` values, since those were each projected onto
a *different* basis (accel(B) onto the average of velocity(B)/velocity(C);
accel(C) onto the average of velocity(C)/velocity(D)), so a plain scalar
subtraction would compare two numbers computed against two slightly
different tangent directions. Confirmed the difference is real, not
academic: a synthetic turning-and-accelerating case gave `-0.50` via the
naive scalar-difference shortcut versus `-1.50` via projecting the
difference vector once — the naive version was silently cancelling real
signal against the basis mismatch.

Not read by any run-detection check anymore — tried there in two
different forms and removed both times (too noisy in practice, and
separately unable to reliably locate its original motivating target, a
jibe's true exit — see §4's edge-check section for the full story). Kept
purely for its own kinematics-chart plots, alongside a smoothed, fit-based
version (§7) — genuinely useful to look at even though neither drives an
automatic decision.

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
| `jerk` | index 2 … n−2 | needs acceleration on **each side** of the edge it's attributed to |

The true track start (index 0) never has any derived data. The true track
end (index n−1) has velocity/heading but no acceleration/curvature (no
point after it to complete the window) — and jerk's own range is narrower
still, since it's a difference of two already-computed accelerations,
each of which already needs a point on either side.

## 4. Run detection

A **run** is a stretch of track where the general point of sail stays
consistent — tolerating real-world wiggle (waves, hand-steering, GPS
jitter) without fragmenting, while still breaking on a genuine turn, stall,
bad-data edge, or gap.

**Edge-based**, matching the kinematics attribution above: "edge `i`" is
the segment from point `i-1` to point `i`; its velocity/heading is
`derived[i]`. `curvature(i)` describes the *transition* at point `i`,
between the edge ending there and the edge starting there.

**Mixed edge-based and fit-based now, deliberately, not uniformly one or
the other.** Heading reads `derived`'s own edge-based value throughout.
Speed and curvature read `fitDerived`'s smoothed value (§7). This wasn't
the original design — everything read `derived` at first — and the split
itself is the result of two rounds of trying "just use the fit
everywhere" and walking part of it back after real failures on real
tracks:

- Switching curvature and speed to the fit held up and stayed.
- Switching heading to the fit did not, and was reverted. The fit's own
  path can genuinely loop or overshoot near a true `dt` gap (§7 —
  originally accepted there as a harmless cosmetic quirk of the map
  display, since nothing consumed the fit's heading for a real decision
  at the time). Once run detection actually started reading fit-based
  heading for its own decisions, that same loop meant a genuinely wrong
  *direction* reading right where a run boundary was being decided — the
  one place accuracy matters most. Curvature doesn't carry the same risk
  in practice: it's a magnitude, not a direction, so a gap-induced loop
  inflates it rather than silently pointing it the wrong way — and an
  inflated curvature at a gap tends to correctly trigger a break there
  anyway, which is what should happen at a gap regardless.
- A separate scare, initially blamed on the same cause, turned out to be
  a false alarm: an early attempt to also guard the *fit's own*
  heading/curvature computation against low-speed noise (nulling them
  out below a speed threshold, rather than letting through the small
  noisy values a near-stationary fit produces) broke run detection
  outright — confirmed directly on a real track, 133 runs before the
  guard, 0 after. The reasoning for why the guard was unnecessary in the
  first place: `headingUnwrapped` is only ever consumed as a *local
  difference* within an already-validated window (never as an absolute
  value needing a clean global baseline), and low-speed points are
  already excluded from ever being part of such a window by the ordinary
  low-speed edge check below — so the noise was never actually reaching
  a decision unfiltered. The guard didn't fix a real problem; it was net
  new complexity.

### The shape of the criteria: two axes, plus one scope-limited addition

Before the state machine itself, it's worth seeing the checks as a small
grid rather than a flat list — every failure mode besides one falls into
exactly one of four cells, split along two independent axes:

|  | **local** (fixed threshold) | **contextual** (compares against run state) |
|---|---|---|
| **edge-level** (pass-1 kinematics) | speed, gap | heading (narrow / wide) |
| **vertex-level** (pass-2 kinematics) | curvature, fit-vs-edge tangential-accel deviation | *(empty)* |

The edge/vertex split tracks exactly which kinematics pass each check
reads from (§3): edge-level checks use the backward-difference velocity
attributed to an edge; vertex-level checks use the centered-difference
acceleration/curvature attributed to a vertex. The local/contextual split
is about what a check compares against — a fixed constant (a threshold in
the controls panel), or something the run's own accumulated state
produced (how far heading has swung across *this* window so far). Heading is
the one check that's edge-level but contextual, which is why it alone
needed two separate thresholds (narrow/wide — see below) where nothing
else in the pipeline does. The vertex-level/contextual cell is empty:
curvature and the fit-deviation check are both inherently about whether a
single point's own reading is extreme, which doesn't have an obvious
run-relative analogue the way "has heading drifted from where this run
started" does — not an obvious gap to fill, just a structural asymmetry
worth noticing.

A fifth check, **fit centripetal acceleration**, doesn't fit cleanly into
this grid at all — it's vertex-level and local like curvature, but scoped
differently: checked only during warm-up and trailing straightness, never
during extension. See "The warm-up/warm-down-only check" below for why.

Two angle thresholds do different jobs:
- **narrow angle** (`runNarrowHeadingThresh`, default `10°`) — validates a
  candidate warm-up window as a genuinely stable direction, and separately
  validates a run's own trailing edge (see step 6).
- **wide angle** (`runBreakHeadingThresh`, default `120°`) — how far
  heading can swing, in total, anywhere across the run so far (tracked as
  a running `max − min` range, not distance from one fixed reference —
  see step 5) before that's a real point-of-sail change, not just
  wiggle. Set intentionally generous — see the walk-back mechanism in
  step 5, which is what makes a large tolerance here safe rather than
  just permissive.

### State machine

1. **Break point.** Start at the track's first point, or the end of a
   previously found run.

2. **Warm-up.** From the break point, accumulate edges until *both*
   `runMinSec` (default `2s`) *and* `runMinEdges` (default `2`) are
   covered, not duration alone — a stretch of slower variable-rate
   sampling could satisfy the duration bar with too few actual points to
   trust a heading range computed from them.

3. **Warm-up validation — heading and edge check.** Every edge in the
   window, including the one feeding *into* the break point itself, must
   pass the *edge* check (see below: low speed, or a true gap). Any
   failure aborts the whole window — the break point slides forward by
   **one point**, and warm-up is retried from scratch.

   Heading is bounded by tracking the window's own `headingUnwrapped`
   range (`max − min`) directly, incrementally as each point is scanned,
   and failing as soon as that range reaches the narrow angle — rather
   than computing an average heading for the window and checking each
   point's distance from it. An average is one number summarizing the
   window; it can't itself detect the window failing to be internally
   consistent the way a direct range bound can — a window whose points
   drift steadily in one direction (say -5°, 0°, +5°, +10°) has an
   average close to the middle of that drift, so points near either end
   can each individually sit comfortably within the narrow angle of the
   average while the window's own total spread already exceeds it.
   Tracking the range directly instead catches exactly that case, and
   removes the need to compute anything before scanning starts — the
   check can run in a single incremental pass.

   Uses `headingUnwrapped` throughout (both for the range being tracked
   and each point being compared) rather than raw heading compared via
   wraparound-aware `headingDiff` — not just for consistency: a wrapped
   comparison can't distinguish a genuine multi-hundred-degree rotation
   from a small one once it crosses 180°, since `headingDiff` always
   returns the *shortest* angular distance. Confirmed directly: a
   synthetic case stepping cumulative rotation from 90° up to 350° showed
   a wrapped comparison correctly blocking every step through 270°, then
   incorrectly *allowing* 350° (wrapped distance only 10°) despite it
   being the largest rotation in the whole sequence — meaning a run could
   have silently extended through most of a full-circle spin without ever
   tripping the check.

   A straight-line corridor check — fitting a line through the window's
   own points and failing if any point strayed too far perpendicular to
   it — also lived here for a while, meant to catch a path shape the
   heading check alone can miss: a small, consistent zigzag where every
   individual edge stays within the narrow angle, but the path itself
   visibly wanders. Removed once fit centripetal acceleration (step 4)
   existed alongside it: for a warm-up-sized window at ordinary sailing
   speed, bounding centripetal acceleration at every vertex already
   implies a bound on how far the path can stray from straight —
   confirmed directly, a synthetic zigzag with each individual bend held
   just under the centripetal threshold produced a corridor deviation of
   only ~0.5m, well under the corridor check's own former default (2m).
   The two checks were catching largely the same thing by the end, and
   the corridor check's own threshold was the harder one to tune well —
   set too tight, it started excluding genuinely reasonable warm-up/
   warm-down windows and visibly widening transition zones, while the
   visible difference between a loose and a tight setting was otherwise
   small.

4. **Warm-up validation — vertex checks (curvature, accel-fit deviation,
   fit centripetal acceleration).** Checked at *every* vertex in the
   window — including its own start (the break point) and its own end.
   No exemptions, no special cases: any failure anywhere is a hard
   failure of the whole window.

   This is a deliberate simplification from an earlier version, where the
   break point's own vertex was exempt ("it's already a break point, so
   high or undefined curvature there is expected — that's what makes it a
   break in the first place"), and a failure exactly at the window's own
   end was treated as a valid, minimal run rather than a failure. Both
   removed: if the break point's own vertex is still carrying real
   curvature, tangential-accel deviation, or centripetal acceleration,
   warm-up is starting from a point that hasn't genuinely settled either
   — the same reasoning that already drives pulling warm-down's own
   boundary back past its own tail-end curvature peak (step 6), just
   applied symmetrically to the other end. With the minimal-run
   possibility gone, warm-up always either fully passes or fully fails —
   there's no longer a path that skips extension outright.

5. **Extension, with walk-back.** One point at a time:
   - the new point's own `headingUnwrapped` is checked against the run's
     own range **so far** (seeded from the warm-up window's own range,
     then extended incrementally as each new point joins) — the new
     point must not push that range's `max − min` to or past the **wide**
     angle — and pass the edge check. This is a genuinely stricter
     standard than comparing each new point against one fixed average
     the way step 3 used to: an average is one number summarizing the
     window, so a candidate could sit within the wide angle of that
     average while the window's own internal spread, plus that
     candidate's own distance, together already exceed it. Confirmed
     directly: a warm-up window with heading -5°/0°/+5° (average 0°, its
     own range already 10°) and a candidate at 85° passes an average-only
     comparison (`|85−0|=85 < 90`) but fails a direct range comparison
     over the whole span (`90 ≥ 90`) — the same range-based principle the
     reconnection pass (below) already used before extension's own check
     was unified onto it.
   - if the point passes, its endpoint **joins** the run (the tracked
     range updates to include it), and *then* gets its own vertex checks
     run: curvature first, then the accel-fit deviation — fit centripetal
     acceleration is *not* checked here (see "The warm-up/warm-down-only
     check" below). Passing both means keep extending (tracking the point
     of highest `|`fit centripetal acceleration`|` seen so far along the
     way — see below for why acceleration rather than curvature);
     either one failing means the point **stays included**, but extension
     stops there — a vertex check describes the *transition* at that
     point, between the edge ending there and the edge starting there, so
     a bad reading implicates the edge that would come *next*, not the
     one that just landed the point in the run.
   - if the wide-angle (range) check specifically fails: instead of
     ending the run at the point right before the failure, **walk back**
     to wherever fit centripetal acceleration peaked during this
     extension (if anywhere) and end the run there instead — not
     curvature, a deliberate, if subtle, choice: centripetal acceleration
     (curvature × speed²) is the more physically representative signal
     for "was this an intentional turn." The same geometric bend taken at
     low speed barely registers as a felt turn at all, while the same
     bend at speed is a real, forceful direction change — curvature alone
     can't distinguish those two. A gradual, real course change can take
     a long time to accumulate past the wide-angle threshold — by the
     time it does, the actual turning may have already happened and
     finished well earlier, with the boat sailing close to straight again
     by the time the cumulative deviation alone finally trips the
     threshold. Walking back recovers the true turn location regardless
     of how long that took. Ties in the peak search favor the **latest**
     candidate — critical for a uniform, constant-rate drift with no
     distinguishable peak at all: without that tie-break, walking back
     would collapse the run to almost nothing at the very first point of
     the drift, defeating the entire purpose of a generous wide-angle
     tolerance. With it, undifferentiated drift is left exactly where it
     would have stopped anyway, and only a genuine, distinguishable spike
     gets preferentially selected. If the wide-angle check fails on the
     very first candidate point (nothing ever joined the run during
     extension, no walk-back candidate exists), the run simply ends at
     the warm-up window's own end, same as before. Either way, once
     walk-back happens the termination is attributed to `'curvature'`,
     not `'heading'`, for the purposes of step 7's resume logic — it now
     represents a located, real turning point rather than bare cumulative
     drift (the label stays `'curvature'` even though the peak search now
     uses acceleration — purely internal, never shown to the user, and
     the semantic meaning, "a locatable turning peak was found," stays
     accurate regardless of which metric located it).
   - if the edge fails on speed/gap instead, the new point is simply
     **excluded** — no walk-back, the run ends cleanly at the edge's start
     point (this failure mode already pinpoints a specific problem edge,
     nothing to relocate).

   Walk-back is the *first* of two separate mechanisms that can pull a
   run's end backward — see step 6 for the second, which does a
   genuinely different job despite the superficial resemblance.

6. **Trailing straightness.** Before accepting the result, the run's own
   *tail* — built exactly like warm-up's own accumulation (`runMinEdges`/
   `runMinSec`), just backward from `runEnd` instead of forward from a
   break point — must itself be narrow-angle self-consistent: its own
   `headingUnwrapped` range (`max − min`, tracked the same way step 3
   tracks warm-up's own range — not an average, see that step's own
   reasoning) must stay under the narrow angle, and it must pass the
   fit-centripetal-acceleration check at every vertex.

   Checking the tail's own range, computed fresh from just the tail
   rather than reusing anything from the run's own extension history,
   asks a different question than the wide-angle check in step 5 already
   answered — "is the tail internally consistent with *itself*,"
   regardless of how far it's drifted from where the run started. That's
   exactly the failure mode the wide-angle tolerance can miss on its own:
   a run whose last few points are already gradually sliding into what
   will become the next turn, without that drift ever being large enough
   (yet) to trip the wide check. If the tail isn't self-consistent,
   shrink the run back by one point and retry — down to, never below, the
   warm-up window's own end, which was already separately validated.

   Curvature and accel-fit deviation don't get re-checked here — every
   point in the tail already passed them once, at the moment it was
   originally added during extension (step 5), and this step only ever
   *removes* points from the end, never adds new ones. Fit centripetal
   acceleration *is* newly checked here, though, since it was never
   checked during extension in the first place (see below).

7. **Minimum duration.** The finished run (its full, exact extent — there
   is no separate reporting-layer boundary adjustment; an earlier version
   pulled the reported `startIdx`/`endIdx` back by one edge on each side,
   reasoning that the break point was exempt from validation and a
   wide-angle-only final edge might not represent settled sailing —
   removed once the break point stopped being exempt (step 4) and the
   reported extent could just *be* the validated one) is checked against
   `runMinDuration` (default `15s`).
   - **Kept:** push the run, and resume the next search at its own end —
     runs can legitimately **share a boundary point**: the point itself
     didn't fail anything, only the edge *leaving* it did.
   - **Abandoned** (too short): where to resume depends on *why* it ended,
     and the split tracks the same local/contextual distinction from
     above. A **heading**-caused ending is *contextual* — tied
     specifically to the range this particular break point's own warm-up
     window produced — so retrying just **one point later** gives a
     genuinely different starting range a chance. Every other reason
     (**velocity, gap, curvature, accel-fit deviation**) is *local* — a
     real gap, a real slow stretch,
     a real sharp turn, a real bad point — nothing about shifting the
     start by one changes whether that specific edge or vertex still
     fails, so resuming straight at the run's own end is both correct and
     cheaper. (A walked-back wide-angle ending counts as `'curvature'`
     here, per step 5.) Warm-up failures (steps 3–4) don't get this
     choice at all — they always retry at `breakPoint + 1` regardless of
     which check failed, since warm-up never produces a `runEnd` worth
     skipping to in the first place.

8. Track end can be reached either mid-run or between runs — no special
   handling needed; the loop just stops.

### The warm-up/warm-down-only checks

**Fit centripetal acceleration** (`runWarmupCentripetalAccelThresh`,
default `1.2 m/s²`) is checked only during warm-up (step 4) and trailing
straightness (step 6) — never during extension (step 5), unlike every
other vertex check.

Centripetal acceleration alone was tried much earlier in this project as
a *general* transition-detection signal and abandoned — it flagged
ordinary tactical heading changes too, since a deliberate, gentle course
adjustment mid-run has real centripetal acceleration too, just not enough
to be a maneuver. Scoping it to warm-up/warm-down only sidesteps that
failure mode entirely: extension still tolerates it freely (a run can
have a mild tactical wiggle in its own interior without breaking), and
this only ever governs where a run's *start* and *end* get positioned
once something else has already decided a break belongs there.

That distinction matters for exactly the case this is meant to catch: a
walk-back-terminated run's own last vertex is, by construction, wherever
fit centripetal acceleration peaked — so it's often still carrying real
centripetal acceleration even though it already passed the narrow-angle/
curvature checks. Confirmed against a real track: many walk-back run
endpoints measured 2–4 m/s² of fit centripetal acceleration right at
`runEnd`, well over this threshold, even though everything else about
that vertex already looked acceptable. This check exists to catch that
specifically and pull the boundary back a little further, past the peak
itself, to wherever the fit's own curve has genuinely settled — not to
decide whether a break happens at all. Confirmed on the same real track
that this genuinely narrows both ends symmetrically, not just the tail:
before this check existed, warm-up-side vertices commonly measured
similarly elevated values (some over 1.5 m/s²); after, both a run's
`startIdx` and `endIdx` consistently read well under the threshold.

**Device-speed deviation** (`runWarmupDeviceSpeedDeviationThresh`,
default `5 m/s`) is scoped identically — warm-up and trailing straightness
only, never extension — but exists for an entirely different reason: it's
the one check in this whole state machine that reads something *outside*
this app's own kinematics entirely. `|fitDerived[i].speed −
points[i].rawSpeed| ≥` threshold, checked only where a device speed
reading is actually present in the file (a no-op, not a failure, when
it's missing — most tracks don't have one at all).

Motivated by a specific, real failure mode none of the kinematics-based
checks can see: a gap that jumps the recorded position far away, then the
GPS "corrects" itself by gradually drifting the reported position back
toward the true one over the following points. That drift is a synthetic
path — nothing about it happened in the real world — but because the
drift itself is gradual by construction, it can look entirely legitimate
under every check above: smooth heading, reasonable curvature, no sharp
spike anywhere for the vertex checks to catch, no gap for the edge check
to catch (the *drift* isn't a gap, only the initial jump was, and that
jump already happened before this stretch begins). Confirmed on a real
track with exactly this failure mode: the fake "path home" after a gap
passed as a normal, valid run under every existing check, and only
comparing against the device's own independently-measured speed (from its
own GPS chipset, not derived from this app's position-to-position
calculation) exposed it — the device wasn't fooled by a synthetic
position drift the way a purely position-derived speed estimate is.

This is a narrower return of an idea tried and removed once already:
device-speed disagreement used to be a general edge-level run-detection
check (§4's edge-check section has that history) and was dropped once the
low-speed and curvature/heading checks already covered what it used to
catch. That removal still holds for the general case — this is a
deliberately different, much narrower reintroduction, scoped specifically
to warm-up/warm-down (not every edge in a run) and motivated by a
concrete failure case the other checks provably can't see, not a return
to the original general-purpose version.

### Reconnection: revisiting walk-back breaks once the full picture is known

The walk-back in step 5 is doing its job correctly when it locates a real,
sharp turning point — but sometimes that "peak" is itself just a mild,
unremarkable deviation (a gentle jibe entry, say), only distinguishable
from noise in hindsight, once both sides of the break are known. So, after
the main search loop finishes and the full run list exists: revisit every
run flagged `wasWalkedBack`, and reconnect it with the next run if —

- `headingUnwrapped`'s own range (`max − min`) across the *whole candidate
  merged range* (from the first run's own start to the second run's own
  end) stays under the wide angle, and
- nothing else in the transition zone between them would have
  disqualified it anyway — re-run the same edge and vertex checks
  `computeRuns` already uses (excluding fit centripetal acceleration,
  which is warm-up/warm-down-scoped and not part of this) across every
  point from one run's end to the other's start.

If both hold, the two runs merge into one spanning the full original
range, inheriting the second run's own `wasWalkedBack` flag and
termination reason. This runs to a fixed point, not just a single pass —
a reconnected run can itself have inherited a walk-back flag from its new
right-hand neighbor, enabling a further chained merge.

This criterion went through two earlier, less robust versions before
landing here, each replaced after a real failure surfaced its specific
blind spot — worth keeping on record, since each fix looked complete
until the next real track disproved it:

1. **Originally**: checked lead-in/lead-out heading (the two runs' own
   boundary points) against a separate, looser `reconnectAngle`
   threshold. Failed on a jibe gradual enough that no single edge-to-edge
   step exceeds much, but the *cumulative* course change across the whole
   transition is a genuine, large reversal — confirmed with a synthetic
   2.5°/point gradual jibe: boundary difference measured only 12.5°
   (would have silently reconnected), against a genuine 137.5° overall
   change.
2. **Replaced with**: comparing the candidate range's own warm-up average
   (a circular mean over its first few edges, bounded to the *owning*
   run's own extent) against its own warm-down average (same, over its
   last few), plus a peak-deviation "overshoot" check scanning the whole
   candidate range for double-backs. Failed on a real track: three
   consecutive walked-back runs, each independently a genuine ~90°+ turn,
   chain-merged in two pairwise steps that each looked individually fine.
   The deeper problem, once traced: a circular mean of several edges can
   itself *understate* a turn — three edges turning 60° at each vertex
   (0°→60°→120°, a genuine 120° turn) average to a warm-up of 30° and a
   warm-down of 90°, a circular-mean "difference" of only 60°, quietly
   losing a third of the real rotation before the comparison even runs.
3. **Current**: `headingUnwrapped`'s own range across the whole candidate
   merged range, described above. Sidesteps both prior failure modes at
   once: it's a continuous, non-wrapping, sequentially-accumulated
   quantity — never averaged, so it can't understate a turn the way a
   circular mean can (case 2's failure), and it directly measures "how
   far did heading ever swing, in total, anywhere in this range" rather
   than relying on two single boundary points (case 1's). Verified
   against the real track that broke case 2: the 232° range measured for
   that candidate merge — comfortably over the 90° wide angle — is
   already apparent from just the *first* of the two chained merges
   alone, so this catches the problem at its earliest point, not
   eventually. Also reuses the wide angle itself rather than a separate,
   bespoke reconnection-only threshold, matching the same "would this
   qualify as one run on its own merits" standard used everywhere else in
   this state machine.

A `reconnectEnabled` checkbox (default on) gates the entire pass — added
specifically as an investigation tool, after case 2 above turned out to
still be producing incorrect merges the checkbox helped isolate: setting
`merged` to the checkbox's own state instead of always `true` means the
`while(merged)` loop's body never executes at all when unchecked, leaving
every run exactly as the state machine's main search found it, nothing
reconnected.

### The edge check

Beyond the vertex checks below and heading, an edge also fails (breaking
the run) if:

- **Low speed** (`runLowSpeedThresh`, default `0.3 m/s`) — `speed ≤`
  threshold, reading `fitDerived`'s own smoothed speed (§7), not
  `derived`'s raw one. A genuine judgment call, unlike the heading/
  curvature split above: a smoothed reading could in principle blur over
  a brief, real stall, but a noisy raw reading can just as easily produce
  a false one from a single bad position fix.
- **True gap** — the edge's own interval exceeds `dtMaxVar + baseDtMs`
  (§2). The `+ baseDtMs` is deliberate slack: even a well-behaved
  fixed-rate track can transiently drop a single sample (a brief GPS
  quirk) without that being a real gap. At 1Hz with no other variable rate
  (`dtMaxVar == baseDtMs == 1000ms`), the effective threshold is `2000ms`
  — a 2s edge is tolerated, a 3s edge isn't.

Device-speed disagreement used to be a third edge-check criterion here,
but turned out unnecessary for run detection specifically once the two
checks above already existed — dropped from this list entirely. It also
used to be the exclusion criterion for the speed-gradient map coloring, a
separate, purely cosmetic concern — also dropped from there once a
second, independent speed estimate existed for every edge regardless (the
local path fit — §7), and superseded again since: gradient exclusion now
inherits directly from the classification pipeline itself (which edges
are part of a run or a classified transition) rather than maintaining any
device-speed logic of its own — see §11's track color modes discussion.
A fit-based `|tangentialJerk|` edge check (replacing an even earlier
vertex-level fit-accel-magnitude attempt) was also tried here and
removed — not a data-quality problem this time, a real-world
discrimination failure: too noisy in practice even on genuinely
straight-ish segments to set a useful threshold, and separately
unhelpful for its original motivating purpose (locating a jibe's true
exit) since exit jerk tends to be low anyway.
Both jerk kinematics (§3's edge-based version, §7's smoothed fit-based
one) and their kinematics-chart plots stay — genuinely useful to look at,
just not usable as an automatic pass/fail gate here.

Both checked the same way heading is: across every edge during warm-up,
and against the new edge during extension.

### The vertex check

Checked at every vertex — the transition between the edge ending there and
the edge starting there (§3) — rather than at either edge specifically:

- **Curvature** (`runCurvThresh`, default `0.15`) — `|curvature| ≥`
  threshold, reading `fitDerived`'s own smoothed curvature (§7), not
  `derived`'s raw one. A sharp, localized direction change concentrated
  at a single point, as opposed to heading's own gradual, accumulated
  drift.
- **Tangential-accel deviation** (`runAccelDeviationThresh`, default
  `5 m/s²`) — `|edge-based tanAccel − fit-based tanAccel| ≥` threshold
  (§7) — the one check that's intentionally *both*, since it's measuring
  disagreement between the two sources directly, not preferring one. A
  large disagreement between the ordinary finite-difference acceleration
  and the same quantity read off the independent local polynomial fit
  means the edge-based reading at this vertex probably isn't
  trustworthy — most often a GPS position glitch, not a genuine sailing
  event. Because the reading is attributed to the vertex itself, not to
  either adjacent edge, a bad reading here implicates the edge that would
  come *next*: the vertex and its incoming edge stay part of the run
  regardless, and only extension past this point is what stops.

Checked at *every* vertex in the warm-up window uniformly now — including
its own start and end, no exemptions (see step 4). During extension, the
point stays included when a check fails; only further extension is
blocked, for the same "the reading implicates what comes next" reasoning.
Vertex and edge failures differ from heading in resume behavior on
abandonment (step 7): both skip straight to the run's end, where only
heading retries one point later.

### No gap-invalidation gate, deliberately

Earlier in this project, points near a gap were specially excluded from run
membership. That's gone: a point right after a gap is judged on its own
merits — if the gap-spanning average velocity implies a real discontinuity,
the ordinary vertex/heading/edge checks above catch it and break the run
there naturally, the same way any other real turn would. The true-gap edge
check (above) exists as a backstop specifically for gaps *without* an
accompanying heading/curvature/accel-deviation signal (e.g. a stall during
a dropout).

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
constant, not UI-exposed; see §10) of the larger side's distance, the whole
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
  `WIND_EST_TOP_SPEED_COUNT` (`2`, an internal constant — see §10)
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
  see §10.
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

The transition zone's own boundary points — `runA.endIdx` and
`runB.startIdx`, each run's own full, validated extent (§4 no longer
shrinks these — see §4's step 7) — serve directly as lead-in/lead-out. No
separate lookup into the adjacent run's own content is needed: the
transition zone is exactly whatever sits between two consecutive runs'
own reported endpoints, so those endpoints already *are* the boundary
being asked about.

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

The selection info panel (§11) surfaces the actual speed and off-wind
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

## 7. Local polynomial path fit and artifact detection

A second, independent kinematics pipeline, alongside the finite-difference
one in §3 — not a replacement for it. Where §3's kinematics come from
2-point and 3-point differences between consecutive recorded positions,
this pipeline fits a small polynomial through a window of *several*
points and reads velocity/acceleration off the fitted curve's own
derivatives. The two disagreeing at a given point is itself a useful
signal — see the artifact detector below.

### The fit itself

`fitPolynomialLS(ts, ys, order)` — ordinary least-squares polynomial
regression via the normal equations, solved by Gaussian elimination with
partial pivoting. Small, fixed-size systems only (order 4 by default), so
a direct solve is simple and fast enough without a general linear-algebra
library. Returns `null` for a degenerate system (fewer distinct `t`
values than parameters) rather than throwing — in practice this can't
actually happen on real track data, since duplicate-timestamp cleanup
(§8, below) already guarantees strictly increasing timestamps, but the
guard costs nothing and stays defensive regardless.

Window size and polynomial degree are both user-configurable (`window` /
`degree` inputs, defaulting to 8 points / degree 5) — arrived at through
iteration, not chosen a priori. A few things worth knowing about *why*
these specific defaults:

- **Degree 5, not lower.** A degree-4 fit already lets acceleration vary
  continuously across the window (see below), but its own acceleration
  curve — `x''(t) = 2a2 + 6a3·t + 12a4·t²` — is only quadratic in `t`,
  which has constant curvature and can never itself change concavity.
  Degree 5 makes acceleration cubic in `t`
  (`x''(t) = 2a2 + 6a3·t + 12a4·t² + 20a5·t³`), which *can* — i.e. the
  acceleration curve itself can inflect within the window, not just rise
  or fall monotonically. Real sailing forces (a gust building then
  easing, a sheet correction overshooting slightly before settling) can
  genuinely have that shape; degree 4 structurally can't represent it no
  matter how the coefficients land.

  Worth being explicit that this is the *opposite* situation from the
  earlier degree-3-vs-4 finding, not an extension of it: that finding
  showed degree 3 changed nothing at a *symmetric* window's exact center,
  because odd-order terms provably vanish there. The window here is
  asymmetric by construction (`numBefore=4`, `numAfter=3` for the N=8
  default — see the next section), so that provable-vanishing property
  doesn't apply, and adding the degree-5 term genuinely changes the
  fitted result — confirmed numerically against a synthetic
  exponential-transient trajectory, where the fitted `a1` (velocity) and
  `a2` (half of acceleration) both measurably shifted between degree 4
  and degree 5 for the same window and same data, rather than matching
  to several decimal places the way the degree-3-vs-4 case did.
- **8 points, not fewer or more.** With 6 coefficients (degree 5) and 8
  points, the fit stays only mildly over-determined (2 "extra" points
  beyond exact interpolation — the same margin as the earlier 7-point/
  degree-4 configuration, so the smoothing-vs-snugness balance doesn't
  shift just because the degree went up) — enough to smooth out one or
  two genuinely bad points without drifting far from what was actually
  recorded, while still snug enough to capture a real, legitimate
  maneuver rather than averaging it away.

### Evaluation point: centered

For `N` window points there are `N-1` edges; the point evaluated is
whichever point *ends* the middle-most edge — `numBefore = floor(N/2)`
points before it, `numAfter = (N-1) - numBefore` after. For odd `N` this
lands exactly centered. For even `N` (the default, 8: `numBefore=4`,
`numAfter=3`), there are two equally-central vertices (the middle edge's
two endpoints), and the evaluation point has to be one actual recorded
vertex, not an interpolated halfway point between them — so one of the
two gets picked, landing one point off true-center. Not a deliberate
choice between them; just an unavoidable consequence of needing an
integer point count on each side of a real vertex when the edge count is
even.

A trailing window (evaluate at the window's own last point, using only
preceding history) was tried first and reverted. The appeal was
avoiding a centered window's own limitation — needing points from both
sides means a point sitting just before a gap/crash needs to look past
it too, losing kinematics there along with everything actually inside
the gap. But **leverage** (how much a fitted curve's value at a point
depends on that point's *own* observed value, vs. everyone else's in
the window; formally the diagonal of the regression hat matrix,
`H = X(XᵀX)⁻¹Xᵀ`) at a window's true trailing endpoint measured at 0.996
for the current default (8 points, degree 5) — essentially 1, meaning the
fit was nearly forced through that one point's own value, giving away
almost all the smoothing the wider window was supposed to provide.
Reverted back to centered, where leverage at the evaluation point is
meaningfully lower (0.55 for the same 8-point, degree-5 configuration) —
real smoothing, at the cost of needing forward history and therefore
losing kinematics for the last few points before a gap (the gap-handling
discussion below touches the practical effect of this).

Leverage is fundamentally tied to how informative a point is about *how
the curve is changing*, and points at a window's extremes are
unavoidably more informative about that than central ones, for any model
expressive enough to represent motion at all — the only way to get
uniform leverage is a model with no positional sensitivity whatsoever,
literally just fitting the mean, which can't represent a path. So this
isn't a shortcoming specific to this implementation; it's what any
windowed polynomial fit trades against, and centered is simply the
better side of that trade for this tool's purposes.

### Kinematics from the fit's own coefficients

No finite differences, and no averaged-adjacent-edge-velocity projection
trick (the trick §3's `tanAccel`/`centripetalAccel` need, since
finite-difference velocity is only ever an edge average, never truly
instantaneous). A polynomial's derivatives are exact at any point: with
`t` re-centered so the evaluation point sits at `t=0`,
`x'(0) = a1` and `x''(0) = 2·a2` directly from the fitted coefficients,
regardless of the polynomial's degree — terms beyond `a2` shape the
curve away from `t=0` but don't affect these two derivatives *at* `t=0`
at all. The fit already provides a genuinely instantaneous velocity at
the evaluation point, so tangential/centripetal/curvature use the same
formulas as §3's pass 2, just with this velocity as the projection
direction directly, nothing to average.

**Does not skip windows spanning a true gap.** Tried and reverted: fitting
across a genuine dt gap (device dropout — a crash, most commonly) asks
the curve to smoothly reconcile two disconnected local trends across a
time span with no data to constrain it in between, which reliably
produces a loop or a large excursion on the map (verified: a synthetic
crash scenario showed the fit overshooting to more than double the
plausible position range in the middle of the gap). But the *kinematics*
derived from it — velocity/acceleration at the evaluation point
specifically — still came out as reasonable values in practice, and
skipping the fit entirely near every gap lost real data for the several
points right before a crash, which is often the most useful part to see.
Accepted as a known, cosmetic-only quirk: the map may show a strange loop
near a crash, but the numbers stay usable.

### Two derived diagnostics

**Tangential-acceleration deviation** (`findSpeedLagArtifacts`): flags any
point where `|derived[i].tanAccel - fitDerived[i].tanAccel|` exceeds a
configurable threshold (default 1 m/s²). Went through several earlier,
more elaborate versions before landing here — a 4-edge windowed jump/dip
comparison with a heading-consistency filter, then a simpler EWMA-based
deviation check — each needing progressively more special cases to catch
real artifacts without also flagging genuine turns or genuine
acceleration. The fit is a better local trend line than either
predecessor specifically because it doesn't lag during real acceleration
the way a trailing average does (a backward-difference edge speed is
structurally an average velocity over the *past* interval, so it
systematically undershoots during real acceleration and overshoots during
real deceleration — confirmed against synthetic exponential-approach
acceleration, where the fit tracked the true instantaneous speed far
more closely than the edge speed did). Compares *acceleration* rather
than speed itself: a position glitch shows up as a sharp, transient spike
in how quickly speed is changing, which acceleration is directly built to
detect, rather than as an offset in the speed value itself, which the
fit's own smoothing already partially absorbs.

Marked on both the map (bright red X, distinct shape/color from every
other marker) and the kinematics charts (same marker, positioned at
whichever metric is currently plotted). A "delete speed-lag points"
button removes them and recomputes, mirroring `removeDuplicateCoordPoints`'s
pattern of validating the result before committing to the `points` array.
As of this writing, deletion is the only corrective action offered — see
the note at the end of this document for a more targeted correction
approach that was designed but not yet implemented.

**Arc-length deviation**: the fit's own path length (finely sampled — 30
points, summed as a piecewise-linear approximation) minus the actual
edge-based path length (straight-line sum between the real recorded
points), over the same window. A snugly-fitting segment has these nearly
equal; a problematic one has them diverge — but the *sign* carries real
information, not just the magnitude: a gap-induced loop overshoots
(positive — the fit travels further than the real points did), while a
sharp, isolated spatial spike gets smoothed over/cut short by the
polynomial (negative — confirmed against a synthetic spike test, -11.5 at
the glitch itself and an even larger -21.9 at its immediate neighbor,
whose own window still contains the glitch without it being the
evaluation point). Available as its own kinematics-chart plot
("path length deviation"); has no edge-based counterpart, since it's
inherently a fit-vs-edges comparison rather than a standalone measurable
quantity.

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

For a run of points sharing a timestamp, keeps whichever *one* point
minimizes the difference between the **speed** implied immediately before
it (distance to the nearest already-kept point, divided by the time to
it) and the speed implied immediately after it (same, to the next
distinct-timestamp point) — not raw distance. The distinction matters:
every candidate in the run shares the identical timestamp, so the time
split on either side (`dtBefore`, `dtAfter`) is *fixed* regardless of
which candidate gets kept — only the resulting distances (and therefore
speeds) vary by candidate. An earlier version balanced distance directly
(`|distBefore - distAfter|`), which seemed reasonable but has a real
flaw: the typical real cause of a duplicate timestamp is a point with a
genuinely correct position that got snapped to the wrong time, leaving
`dtBefore` and `dtAfter` asymmetric (one side effectively swallowed the
sample that should have been there). Balancing distance while ignoring
that asymmetry lets the longer-duration side's computed speed come out
silently lower (or the shorter side's silently higher) even when the true
physical speed through here was roughly steady — confirmed with a
synthetic case (`dtBefore=1s`, `dtAfter=2s`): distance-balancing picked a
point implying a jump from 15 m/s to 7.5 m/s across it, while
speed-balancing picked the point implying a flat 10 m/s on both sides,
the actual constant-speed answer the scenario was built from.

### Duplicate coordinates

Consecutive points sharing an *exact* identical `(lat, lon)` are a device
artifact (re-logging the same fix across several timestamps — a
"stutter") rather than real stationary motion, which still has tiny
position noise; an exact repeat is a data artifact.

Same balance principle as duplicate timestamps, transposed: there,
position varied within a run sharing one timestamp, so distance was the
free variable and speed got balanced by choosing *where* to put the
shared time; here, time varies within a run sharing one position, so
distance is fixed instead (every candidate shares the same position) —
but the thing actually worth balancing is still **speed**, not raw time.
An earlier version balanced time directly (`|dtBefore - dtAfter|`), which
turns out to have the same flaw the timestamp cleanup's own earlier
version did, just with distance and time swapped: a synthetic case with
`distBefore=10m`, `distAfter=20m` (points 10m apart before the stutter,
20m apart after) showed time-balancing picking the timestamp splitting
the interval exactly in half, which implies a flat speed *doubling*
across it (5 m/s to 10 m/s) — because equal time over unequal distances
is unequal speed, not equal. Speed-balancing instead picks whichever
timestamp minimizes the difference between the speed implied immediately
before it and the speed implied immediately after — letting the
resulting wider gap be handled by the ordinary gap/true-gap machinery
instead of producing a fake zero-velocity segment.

This runs **automatically, silently**, immediately after a fresh file
load. "Restore original track" deliberately does **not** re-run it — that
gives back the true, unfiltered file if you want to inspect what the
automatic pass found. The "remove duplicate-coord points" button remains
available for a manual re-run at any time, with a summary alert.

## 9. GPX export

Saves the current (post-editing) points as standard GPX 1.1, preserving
`<speed>` and `<ele>` where present. Filename defaults to
`<original-basename>_t.gpx`.

## 10. UI controls reference

| Control | Default | Meaning |
|---|---|---|
| run breaks if \|curvature\| ≥ | `0.15` /m | hard vertex-check threshold (§4 step 4/5), fit-based |
| heading swings ≥ (wide angle) anywhere in the run | `120°` | range-based extension tolerance, edge-based |
| speed ≤ | `0.3` m/s | low-speed edge failure, fit-based |
| tangential accel deviates (fit vs edge) ≥ | `5` m/s² | the other hard vertex-check threshold (§4 step 4/5, §7) |
| warm-up/trailing vertices need fit centripetal accel < | `1.2` m/s² | §4's warm-up/warm-down-only check — never checked during extension |
| warm-up/trailing edges need \|fit − device speed\| < | `5` m/s | §4's warm-up/warm-down-only check — only when device speed is present in the file |
| warm-up validated at ≥ (narrow angle) | `10°` | warm-up + trailing-straightness tolerance, edge-based |
| warm-up/trailing window needs ≥ | `2` edges | minimum edge count — both must be satisfied, not either alone |
| warm-up/trailing window needs ≥ | `2` s | minimum duration — same accumulation logic used forward (warm-up) and backward (trailing) |
| min run duration | `15` s | runs shorter than this are abandoned |
| reconnect walk-back breaks | on (checkbox) | gates §4's entire reconnection pass — added as an investigation tool; unchecking leaves every run exactly as the main search found it |
| loose tack success: under | `20` s | §6 loosened tack path — total transition duration cap |
| loose tack success: ≤ | `5` s | §6 loosened tack path — total time below low-speed threshold, summed |
| local path fit window | `8` points | §7 — arrived at through iteration; see there for the leverage/smoothing reasoning |
| local path fit degree | `5` (quintic) | §7 — see there for why degree 5 lets the fitted acceleration curve itself inflect, which degree 4 structurally cannot |
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

## 11. UI layout and mobile support

### Header stats bar

Two groups, wrapped onto its own line via a `flex-basis:100%` spacer
(plain `<br>` doesn't force a line break in a `flex-wrap` container). A
raw group — point count, duration, sampling interval — reflects the file
as loaded, unaffected by any analysis. A "Processed:" group reflects the
run-detection/classification results: session duration and session length
(first run's start to last run's end, and the summed edge distance over
that same span — deliberately narrower than the track's own total
duration/length, trimmed of any warm-up, breaks, or dead time
before/after the session itself; session length sums *every* edge in the
span, not just run-classified ones, matching session duration's own "the
whole trimmed session" scope), run count as a percentage *of session
duration* specifically (not the full track — diluting by dead time the
session itself doesn't include would understate how much of the actual
session was spent running), jibe/tack counts with their failed
counterparts, crash/recovery count alongside its own summed total
duration (so a session with several short dropouts reads differently
from one with a single long one, not just by count), and max speed —
derived, local-path-fit (§7), and device, each when available —
restricted to edges that are part of a run, a jibe, or a tack
specifically, so a stall or a gap-triggered crash edge can't inflate the
reported maximum.

Session length (and only session length, currently) respects the
separate length-unit selector (km / nm — §10); every speed figure
respects the speed-unit selector as usual.

Recomputed on every `redrawAll()`, not just once at file load — folded in
after fixing a real staleness bug: it used to run once, early, before the
wind-direction textbox had been initialized from the calculated estimate,
so its own transition classification saw no wind direction and reported
zero jibes/tacks, and nothing ever refreshed it afterward.

### Panel order

Everything is a single vertical stack (`display:flex; flex-direction:
column`, not the two-column grid this file used earlier): header, wind
estimate, run detection, track, then the kinematic plot, then the
threshold/action controls. Wind estimate and run detection both come
before track deliberately — they're the smaller, usually-collapsed
panels, and reading order roughly matches "set up wind and tune
detection, then look at the track" rather than the reverse.

Run detection's own panel follows the exact same collapsed-by-default,
"show/hide details" pattern wind estimate already established
(`setupToggle`, reused directly rather than duplicated) — added once the
run-detection controls, originally a single flat row buried in the
threshold/action controls panel at the bottom of the page, grew past the
point a flat row could present clearly. Split into four grouped rows
inside it (vertex checks, edge checks, warm-up/trailing-only checks, run
acceptance — matching §4's own conceptual grouping, not just visual
convenience) rather than moving the flat row over unchanged. Loose-tack
success timing (§6, not run detection itself despite living in the same
original row) and local-path-fit/speed-lag settings (§7, shared by more
than just run detection) stayed in the threshold/action controls panel
rather than moving along with it.

The kinematic plots (speed, tangential acceleration, centripetal
acceleration, heading, curvature, path length deviation, tangential jerk,
centripetal jerk — eight now, path length deviation and the two jerk
plots added after the original five-canvas collapse below) live in one
canvas plus a `<select>` dropdown (`kinematicSelect`) choosing which
metric renders into it, rather than five (now eight) separate always-
visible canvases. A `KINEMATIC_PLOTS` config maps each dropdown option to
the `key`/`opts` `drawTimeSeries` needs — the same values that used to be
hardcoded per canvas. Pan/zoom/tap interaction (`attachTimeInteraction`)
is attached once, to the single shared canvas, instead of once per
canvas.

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
full rose's own off-wind labels — see §5). Speed prefers `fitDerived`'s
own smoothed value (`preferFitSpeed`, shared with the selection info box
below and the speed-gradient map coloring — see "Track color modes"),
falling back to `derived`'s raw edge speed only in the rare case
`fitDerived` isn't available at that point (a narrower valid range than
`derived`'s — right at the very edges of the track, see §7); heading
stays edge-based regardless (§4's own heading-reversion story). The two
are grouped in one
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

"Eligible" excludes, regardless of mode — `edgeGradientExcluded`, which
now inherits its criteria from the classification pipeline itself (§4/§6)
rather than maintaining its own separate notion of "good enough to
color":

- A true dt gap (`edgeGapFails`) — the derived speed there is a
  gap-spanning average, not a real instantaneous reading.
- Anything not part of a run, and not part of a classified transition
  segment other than `crash_recovery` — a successful or failed jibe/tack
  still reflects genuine sailing and stays eligible, but `crash_recovery`
  is known-bad data by definition, and anything that fell through
  classification entirely (no wind direction set yet, or the transition
  zone matched no jibe/tack/crash pattern at all) has no basis for trust
  either. Handled for free either way, since excluded edges simply aren't
  drawn over, leaving the grey base path visible underneath rather than
  needing an explicit grey re-draw.

No separate device-speed check here anymore, even where device speed is
present in the file — an earlier version had its own, gradient-only
device-speed-disagreement threshold, dropped once a second, independent
speed estimate already existed for every edge regardless (the local path
fit — §7), and superseded again since: inheriting from run detection
picks up its own device-speed check (§4) automatically wherever that
governs, without this needing any device-speed logic of its own.

The gradient's min/max range, and now every individual edge's own color
within it, both read the **local path fit's** speed (§7), not the raw
edge speed — a deliberate change from an earlier version where only the
range came from the fit and each edge's own color still came from the
raw, edge-based reading. The fit is a second, independent speed estimate
that naturally smooths out an isolated glitched edge, so one bad point
can't drag the whole color scale's bounds along with it, without needing
an external device reading (which isn't always present anyway) to
provide that protection — and reading it consistently for the edge color
too, not just the range, keeps a single glitched point from painting
itself with a wildly different color than its genuinely similar-speed
neighbors. The track legend (toggleable, same
panel) rebuilds itself per mode — type mode shows the usual swatches;
speed mode shows a single gradient bar sampled directly from the same
`speedToColor` function used for the actual edges (not a separately
duplicated color formula, so the two can't drift apart), labeled with the
range's min/max in the current display unit.

`preferFitSpeed(i, edgeSpeed)` — the same fit-speed-with-edge-fallback
helper this gradient uses — is shared by two other UI surfaces too: the
mini compass overlay's own selected-point readout (above), and every
speed figure in the selection info box (run max speed; jibe/tack entry,
exit, through-the-wind, min, and max speed; crash/recovery's pre-crash
speed). All three switched together, for the same reason: a second,
independent speed estimate exists for every edge regardless of whether
device speed happens to be present, and prefers it consistently rather
than mixing sources depending on which UI element happens to be showing
it. Heading stays edge-based at every one of these same call sites
(§4's own heading-reversion story) — this only ever touches the speed
side of a lookup, never heading.

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

## 12. Track artifacts: a taxonomy and a design not yet built

**Nothing in this section is implemented.** §7's tangential-acceleration
and arc-length deviation detectors exist and work; everything below is
design discussion toward a *correction* mechanism, which does not exist
yet — the only corrective action currently offered for a flagged point is
deletion (§7). Recorded here so the reasoning isn't lost before it's
built.

### What the detectors are actually catching

Working through real flagged segments surfaced (at least) three distinct
underlying causes, not one:

- **GPS jitter.** A point's *timestamp* is trustworthy but its *position*
  is slightly off — usually on an otherwise straight-ish stretch. The
  point still sits close to the true route; it's placed at the wrong
  point *along* that route for its timestamp. This warps the two edges
  touching it (shrinks one, stretches the other, or vice versa),
  producing jagged tangential acceleration without any real path-shape
  problem. This is the case the deviation detectors were originally
  built for, and the case any future correction mechanism should target.
- **Gapless stalls, holds, and in-water activity** (uphauling, climbing
  back on the board, setting up a waterstart, actual swimming). Distinct
  from jitter, and — per direct observation — distinct from *each other*
  in a way that matters:
  - A **controlled, deliberate stall** (e.g. luffing to let another
    vessel pass) reads *quiet* to the detector — in one tested example,
    only a single flagged point, and only at a lowered threshold (0.5
    m/s² rather than the 1 m/s² default). The mechanical explanation:
    the recording device (wrist-worn) stays rigidly coupled to the boom
    through the rig, so even near-stationary motion is still a faithful
    proxy for the board's own motion. There's a real, if small, coherent
    signal the whole time.
  - **Uphauling, climbing back on, or a waterstart** likely reads
    differently — the hand moves independently of the board (pulling
    line, swimming strokes, repositioning relative to the rig), so the
    device is no longer measuring "where is the board" for that
    stretch. Not degraded signal about the thing of interest; not a
    signal about that thing at all for the duration. Untested as of this
    writing — a natural follow-up whenever a track with a genuine
    swim/uphaul segment is available, to see whether it produces the
    dense, sustained flagging the controlled-stall case didn't.
- **Crash gaps** (device dropout). Already and separately caught by `dt`
  (§4's `edgeGapFails`, reused throughout — §6, §7). The deviation
  detectors *can* also flag these (a large `dt` gap can produce a fit
  loop or excursion — §7), but this is redundant detection of something
  already known via a cleaner signal, not a new capability. Useful as a
  cross-check (both signals agreeing is reassuring; `dt` saying gap but
  deviation being unremarkable nearby would be odd) but `dt` should keep
  priority — a flagged point that's already part of a known
  `crash_recovery` zone (already excluded from speed stats — §11's
  stats-bar note) shouldn't be routed into whatever correction workflow
  eventually exists for jitter.

The practical implication: only the jitter case is a good target for
position *correction*. The stall/swim case, if the controlled-vs-active
distinction above holds up, may not need new discrimination logic at
all — the *existing* deviation detector, applied to a low-speed segment,
may already read differently for "held still" vs. "actively moving in
the water" without anything new being built, simply because the latter
genuinely has no coherent board-path to recover. Worth testing directly
before building anything to handle it.

### A correction design: separate shape from pacing

The first design considered — replace a flagged point's position with
wherever §7's own fit predicts for that timestamp — has a real flaw: it
would discard the point's own recorded position, which (per the jitter
diagnosis above) is usually *not* the actual problem; the position is
close to correct, only its pacing along the route is off. Overwriting it
with a fresh fit evaluation throws away good information and — since the
fit can itself loop or overshoot under some conditions (§7) — risks
moving a fine point to a *worse* spot.

The alternative worked out instead: treat "what the route looks like"
and "how far along it a point should be at a given time" as two separate
fits, not one.

1. **Shape.** A curve through the segment's own recorded positions,
   parameterized by something neutral like cumulative chord length —
   *not* time. Answers "what does the route look like here," independent
   of pacing. Includes the flagged points' own positions, since (for the
   jitter case) those are trusted.
2. **Pacing.** A separate fit of arc-length-along-the-shape-curve as a
   function of time, built using *only* the segment's non-flagged
   (trusted) time/arc-length pairs.
3. **Correction.** For each flagged point: look up the pacing fit's
   predicted arc-length position at that point's own (trusted) timestamp,
   then map that arc-length back onto the shape curve to get a corrected
   `(x, y)`.

The result is guaranteed to lie on essentially the same curve the
original flagged point helped define in step 1 — nothing is invented off
the visible route — just redistributed along that curve according to
pacing derived only from trusted data. This is a meaningfully different
guarantee than "evaluate the existing single fit at this timestamp,"
which doesn't separate the two questions and remains partly
self-referential.

**Known open issue, not resolved:** step 1's shape curve, when several
consecutive points are flagged, still uses those points' own mutual
chord-length spacing to build its parameterization — and that spacing is
itself computed from the distances between points whose pacing (not
position, but the two aren't perfectly separable at the margin) is in
question. A milder, second-order version of the self-reference problem
this design was meant to solve outright, not fully escaped. Possibly
fine in practice; possibly wants the shape curve's parameterization
anchored only to the segment's trusted points instead. Unresolved as of
this writing.

**Segment-length question, also unresolved:** a short, isolated glitch
(a point or two) seems like a safe target for this. A long stretch of
consecutive flagged points — a genuine multi-second dropout, especially
one where a real maneuver might have happened inside it — may not be
well-represented by a single polynomial at all. Whether there's a length
past which correction should refuse and fall back to exclusion (§7's
existing option) instead of attempting a fit, and if so where that
length sits, hasn't been decided.

**A cheap validation step worth keeping in mind:** once a segment is
corrected, re-running §7's own tangential-acceleration detector against
the corrected result is a direct, nearly-free check of whether the
correction actually worked — a still-flagged result after "fixing" it is
a clear signal something's wrong (segment too long, anchors too sparse,
wrong degree) rather than something to trust blindly. Device speed,
where present, is a second independent check for the same purpose — it's
not derived from position at all, so it can't be fooled by anything
wrong with position, and would be expected to agree with a genuinely
successful correction.

**Whatever gets built here should look different from every other action
this tool takes on a track.** Deletion (§7) is currently the most
invasive existing action; position correction goes further, since it
asserts a *specific* corrected value rather than just admitting
distrust. Any implementation should be opt-in, visually distinct on both
the map and the kinematics charts (not blended into the existing
"flagged" or "normal" appearance), and reversible the same way "restore
original track" already reverses the duplicate-cleanup passes (§8).

## License

MIT — see [LICENSE](./LICENSE).

