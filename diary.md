# Windsurfing Track Inspector — Development Diary

This is the companion document to `README.md`. Where the README specifies
current behavior, this records *why* — the approaches tried and replaced,
the real bugs found and how they were traced, the validation behind each
tuning decision, and the reasoning that didn't survive into the final
design but is still worth keeping on record. Organized by the same section
numbers as the README, not strictly chronologically.

## 1. Coordinate projection

Vincenty's geodesic method was picked over the more common approach of a
single fixed metres-per-degree scale factor (an equirectangular-style
projection) for two reasons: a fixed-scale approach is only exactly right
at one particular latitude, degrading in a real, direction-dependent way as
a track gets larger or moves toward higher latitudes; and bearing coming
from the same geodesic calculation as distance means heading throughout the
rest of the app (run detection, wind estimation, jibe/tack classification)
is the true compass bearing, not a flat-plane approximation that quietly
drifts away from the projection center. This was the original design
choice, not something arrived at by iteration.

## 2. Sampling interval detection

`dtMaxVar`'s percentile-based calculation replaced an earlier, mode-based
version that took the highest `dt` value appearing in at least 10% of all
intervals. That approach depended on some specific `dt` value repeating
often enough to form a clear mode, which a highly variable device might
never produce (no single interval common enough to clear the 10% bar) —
leaving `dtMaxVar` stuck at `baseDtMs` and flagging most of the track as
gaps. A percentile doesn't need any value to repeat at all; it only needs
the distribution's overall shape, so it degrades gracefully regardless of
how scattered the device's own variable rate turns out to be.

## 3. Kinematics: cause-and-effect attribution

**Jerk's vector-vs-scalar computation.** Computing jerk as the raw `(ax,
ay)` vector difference first, then projecting once onto edge `BC`'s own
velocity, rather than a scalar difference of the two already-projected
`tanAccel` values, matters because those two values are each projected
onto a *different* basis (accel(B) onto the average of velocity(B)/
velocity(C); accel(C) onto the average of velocity(C)/velocity(D)), so a
plain scalar subtraction compares two numbers computed against two
slightly different tangent directions. Confirmed the difference is real,
not academic: a synthetic turning-and-accelerating case gave `-0.50` via
the naive scalar-difference shortcut versus `-1.50` via projecting the
difference vector once — the naive version was silently cancelling real
signal against the basis mismatch.

**Jerk as a run-detection check.** Tried twice, in two different forms,
and removed both times. First attempt: a vertex-level fit-based
acceleration-magnitude check. Second attempt: an edge-level check on
fit-based `|tangentialJerk|`. Neither survived — not a data-quality
problem, a real-world discrimination failure: too noisy in practice even
on genuinely straight-ish segments to set a useful threshold, and
separately unhelpful for its original motivating purpose (locating a
jibe's true exit) since exit jerk tends to be low anyway. The narrow-angle
heading check turned out to be the practical, working constraint for
straightness instead. Both jerk kinematics (edge-based and the smoothed
fit-based version, §7) stayed in as chart-only diagnostics — genuinely
useful to look at, just not usable as an automatic pass/fail gate.

**Why the average of `velocity(B)`/`velocity(C)`, not `velocity(B)` alone,
for the tangential/centripetal/curvature projection reference.** Checked
against a case with a known-exact answer (uniform circular motion, then a
spiral with changing speed added). For constant-speed turning, projecting
onto `velocity(B)` or `velocity(C)` gives *algebraically identical*
results (`cross(AB, accel) = cross(BC, accel)` always holds), so either
works. But once speed is also changing — the realistic case for a tack or
jibe — the average matched a precise reference curvature roughly an order
of magnitude better than `velocity(B)` alone.

## 4. Run detection

**Mixed edge-based and fit-based, not uniformly one or the other.** This
wasn't the original design — everything read `derived` at first — and the
current split is the result of two rounds of trying "just use the fit
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
  heading/curvature computation against low-speed noise (nulling them out
  below a speed threshold, rather than letting through the small noisy
  values a near-stationary fit produces) broke run detection outright —
  confirmed directly on a real track, 133 runs before the guard, 0 after.
  The reasoning for why the guard was unnecessary in the first place:
  `headingUnwrapped` is only ever consumed as a *local difference* within
  an already-validated window (never as an absolute value needing a clean
  global baseline), and low-speed points are already excluded from ever
  being part of such a window by the ordinary low-speed edge check — so
  the noise was never actually reaching a decision unfiltered. The guard
  didn't fix a real problem; it was net new complexity.

**The vertex-level/contextual grid cell is empty.** Curvature and the
fit-deviation check are both inherently about whether a single point's own
reading is extreme, which doesn't have an obvious run-relative analogue
the way "has heading drifted from where this run started" does — not an
obvious gap to fill, just a structural asymmetry worth noticing.

**Wide angle tuning history:** started at `90°`. Briefly loosened to
`120°` — too generous in practice, since the very start of a genuine
failed jibe attempt could still fall within that wider range, letting
extension absorb it as if it were still part of the steady-state run
rather than a break — so it was reverted back to `90°`.


**Step 2 (warm-up accumulation) — why both `runMinSec` and `runMinEdges`,
not duration alone:** a stretch of slower variable-rate sampling could
satisfy the duration bar with too few actual points to trust a heading
range computed from them.

**Step 3 (heading range vs. average) — why range-tracking, not an
average:** an average is one number summarizing the window; it can't
itself detect the window failing to be internally consistent the way a
direct range bound can. A window whose points drift steadily in one
direction (say -5°, 0°, +5°, +10°) has an average close to the middle of
that drift, so points near either end can each individually sit
comfortably within the narrow angle of the average while the window's own
total spread already exceeds it. Tracking the range directly catches
exactly that case, and removes the need to compute anything before
scanning starts — the check runs in a single incremental pass.

**Step 3 (`headingUnwrapped` vs. `headingDiff`) — why not the wrapped
comparison:** a wrapped comparison can't distinguish a genuine
multi-hundred-degree rotation from a small one once it crosses 180°,
since `headingDiff` always returns the *shortest* angular distance.
Confirmed directly: a synthetic case stepping cumulative rotation from
90° up to 350° showed a wrapped comparison correctly blocking every step
through 270°, then incorrectly *allowing* 350° (wrapped distance only
10°) despite it being the largest rotation in the whole sequence —
meaning a run could have silently extended through most of a full-circle
spin without ever tripping the check.

**Step 3's removed corridor check.** A straight-line corridor check —
fitting a line through the window's own points and failing if any point
strayed too far perpendicular to it — lived here for a while, meant to
catch a path shape the heading check alone can miss: a small, consistent
zigzag where every individual edge stays within the narrow angle, but the
path itself visibly wanders. Removed once fit centripetal acceleration
(step 4) existed alongside it: for a warm-up-sized window at ordinary
sailing speed, bounding centripetal acceleration at every vertex already
implies a bound on how far the path can stray from straight — confirmed
directly, a synthetic zigzag with each individual bend held just under
the centripetal threshold produced a corridor deviation of only ~0.5m,
well under the corridor check's own former default (2m). The two checks
were catching largely the same thing by the end, and the corridor check's
own threshold was the harder one to tune well — set too tight, it started
excluding genuinely reasonable warm-up/warm-down windows and visibly
widening transition zones, while the visible difference between a loose
and a tight setting was otherwise small.

**Step 4's removed exemptions.** This uniform vertex-check treatment is a
deliberate simplification from an earlier version, where the break
point's own vertex was exempt ("it's already a break point, so high or
undefined curvature there is expected — that's what makes it a break in
the first place"), and a failure exactly at the window's own end was
treated as a valid, minimal run rather than a failure. Both removed: if
the break point's own vertex is still carrying real curvature,
tangential-accel deviation, or centripetal acceleration, warm-up is
starting from a point that hasn't genuinely settled either — the same
reasoning that already drives pulling warm-down's own boundary back past
its own tail-end curvature peak (step 6), just applied symmetrically to
the other end. With the minimal-run possibility gone, warm-up always
either fully passes or fully fails — there's no longer a path that skips
extension outright.

**Step 5 (extension's range check vs. one fixed average) — genuinely
stricter, not just a rewrite.** Comparing each new point against one fixed
average (the way step 3 used to work) is weaker than the current range
check: a candidate could sit within the wide angle of that average while
the window's own internal spread, plus that candidate's own distance,
together already exceed it. Confirmed directly: a warm-up window with
heading -5°/0°/+5° (average 0°, its own range already 10°) and a candidate
at 85° passes an average-only comparison (`|85−0|=85 < 90`) but fails a
direct range comparison over the whole span (`90 ≥ 90`) — the same
range-based principle the reconnection pass (below) already used before
extension's own check was unified onto it.

**Step 5 (walk-back peak: acceleration, not curvature) — the reasoning.**
Centripetal acceleration (curvature × speed²) is the more physically
representative signal for "was this an intentional turn." The same
geometric bend taken at low speed barely registers as a felt turn at all,
while the same bend at speed is a real, forceful direction change —
curvature alone can't distinguish those two. A gradual, real course change
can take a long time to accumulate past the wide-angle threshold — by the
time it does, the actual turning may have already happened and finished
well earlier, with the boat sailing close to straight again by the time
the cumulative deviation alone finally trips the threshold. Walking back
recovers the true turn location regardless of how long that took.

**Step 5 (walk-back tie-break: latest candidate) — why.** Critical for a
uniform, constant-rate drift with no distinguishable peak at all: without
that tie-break, walking back would collapse the run to almost nothing at
the very first point of the drift, defeating the entire purpose of a
generous wide-angle tolerance. With it, undifferentiated drift is left
exactly where it would have stopped anyway, and only a genuine,
distinguishable spike gets preferentially selected.

**Step 5's `'curvature'` label persisting after the switch to
acceleration.** The termination-reason string stays `'curvature'` even
though the peak search now uses acceleration — purely internal, never
shown to the user, and the semantic meaning ("a locatable turning peak was
found") stays accurate regardless of which metric located it, so it
wasn't worth renaming and touching every downstream consumer of the
string.

**Step 6 (why check the tail's own fresh range, not reuse extension's
history).** This asks a different question than the wide-angle check in
step 5 already answered — "is the tail internally consistent with
*itself*," regardless of how far it's drifted from where the run started.
That's exactly the failure mode the wide-angle tolerance can miss on its
own: a run whose last few points are already gradually sliding into what
will become the next turn, without that drift ever being large enough
(yet) to trip the wide check.

**Step 7's removed boundary-shrinking.** An earlier version pulled the
reported `startIdx`/`endIdx` back by one edge on each side, reasoning that
the break point was exempt from validation and a wide-angle-only final
edge might not represent settled sailing. Removed once the break point
stopped being exempt (step 4) and the reported extent could just *be* the
validated one — there is no separate reporting-layer boundary adjustment
anymore.

### The warm-up/warm-down-only checks

**Fit centripetal acceleration — why scoped to warm-up/warm-down only,
not extension.** Centripetal acceleration alone was tried much earlier in
this project as a *general* transition-detection signal and abandoned —
it flagged ordinary tactical heading changes too, since a deliberate,
gentle course adjustment mid-run has real centripetal acceleration too,
just not enough to be a maneuver. Scoping it to warm-up/warm-down only
sidesteps that failure mode entirely: extension still tolerates it freely,
and this only ever governs where a run's *start* and *end* get positioned
once something else has already decided a break belongs there. That
distinction matters for exactly the case this is meant to catch: a
walk-back-terminated run's own last vertex is, by construction, wherever
fit centripetal acceleration peaked — so it's often still carrying real
centripetal acceleration even though it already passed the narrow-angle/
curvature checks. Confirmed against a real track: many walk-back run
endpoints measured 2–4 m/s² of fit centripetal acceleration right at
`runEnd`, well over this threshold, even though everything else about
that vertex already looked acceptable. Confirmed on the same real track
that this genuinely narrows both ends symmetrically, not just the tail:
before this check existed, warm-up-side vertices commonly measured
similarly elevated values (some over 1.5 m/s²); after, both a run's
`startIdx` and `endIdx` consistently read well under the threshold.

**Device-speed deviation — motivation and history.** One of two checks in
this whole state machine (the other is gap debt) that reads something
*outside* this app's own kinematics entirely. Motivated by a specific,
real failure mode none of the kinematics-based checks can see: a gap that
jumps the recorded position far away, then the GPS "corrects" itself by
gradually drifting the reported position back toward the true one over
the following points. That drift is a synthetic path, but because the
drift itself is gradual by construction, it can look entirely legitimate
under every other check: smooth heading, reasonable curvature, no sharp
spike anywhere for the vertex checks to catch, no gap for the edge check
to catch (only the initial jump was a gap; the drift itself isn't).
Confirmed on a real track with exactly this failure mode: the fake "path
home" after a gap passed as a normal, valid run under every existing
check, and only comparing against the device's own independently-measured
speed exposed it. This is a narrower return of an idea tried and removed
once already — device-speed disagreement used to be a general edge-level
run-detection check (see "The edge check" below for that history) and was
dropped once the low-speed and curvature/heading checks already covered
what it used to catch. That removal still holds for the general case;
this reintroduction is deliberately narrower, scoped specifically to
warm-up/warm-down and motivated by a concrete failure case the other
checks provably can't see.

**Gap debt — full tuning history.** The payback weight went through three
values. Original default `0.5` (needing 2× the gap's own distance) turned
out too aggressive in general — it kept legitimate track excluded from
warm-up for longer than the artifact itself justified — so it was loosened
to `0.8` (1.25×) once real tracks showed the stricter setting eating into
genuine sailing. But `0.8` then proved too loose for at least one specific
real track, where the fake "path home" segment still qualified — `0.67`
(~1.5×, `2/3` rounded) is the current middle ground, not a value derived
from any particular formula.

**Gap debt — the "any edge after the first gap is payback" fix.**
Counting *any* edge after the first gap as payback — not just non-gap
edges — is itself a correction, not the original design. Earlier, only
non-gap edges paid back, and every gap edge added its own distance
unconditionally, regardless of direction. That broke on a real track
where two gaps happened close together and the second one's own jump
happened to land back toward the true position — a genuine correction,
not a further departure. The original rule added both distances together
anyway, since it only ever asked "is this edge a gap" to decide whether
to add, never which direction it moved — a correcting second gap
ballooned the debt instead of reducing it, and cost two entire genuine
runs before the inflated debt could clear. Confirmed directly with a
synthetic two-gap case (a 90m jump, then a second gap landing almost
exactly back on the true track): the debt swelled to over 150m under the
original rule, but dropped to around 30m under the corrected one, after
the identical two edges.

**Gap debt — full verification story.** Verified against two synthetic
cases built specifically to match each failure mode in turn. The original
single-gap "path home" case: a normal run, a gap that jumps 80m, an
8-point "drift back" covering roughly 40m with entirely smooth,
legitimate-looking kinematics, then genuine sailing resuming for well
over a minute. With the check enabled, the drift-back correctly fails
warm-up throughout, and the next accepted run only starts once the debt
is actually paid off — with the check disabled, the drift-back gets
silently absorbed into one continuous run. Re-verified after each
retuning: index 84 at `0.5` (2×), 76 at `0.8` (1.25×), 78 at `0.67`
(~1.5×) — monotonic with strictness. The double-gap correction case:
comparing the original and corrected payback rules directly against the
same synthetic double-gap track, the corrected rule accepted the next
genuine run starting at index 68, while the uncorrected rule delayed it
to index 95.

### Reconnection: the three-version history

The walk-back in step 5 is doing its job correctly when it locates a real,
sharp turning point — but sometimes that "peak" is itself just a mild,
unremarkable deviation (a gentle jibe entry, say), only distinguishable
from noise in hindsight, once both sides of the break are known. That's
the whole motivation for reconnection. The current criterion
(`headingUnwrapped`'s own range across the whole candidate merged range —
see README) went through two earlier, less robust versions before landing
there, each replaced after a real failure surfaced its specific blind
spot — worth keeping on record, since each fix looked complete until the
next real track disproved it:

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
   merged range. Sidesteps both prior failure modes at once: it's a
   continuous, non-wrapping, sequentially-accumulated quantity — never
   averaged, so it can't understate a turn the way a circular mean can
   (case 2's failure), and it directly measures "how far did heading ever
   swing, in total, anywhere in this range" rather than relying on two
   single boundary points (case 1's). Verified against the real track
   that broke case 2: the 232° range measured for that candidate merge —
   comfortably over the 90° wide angle — is already apparent from just
   the *first* of the two chained merges alone, so this catches the
   problem at its earliest point, not eventually. Also reuses the wide
   angle itself rather than a separate, bespoke reconnection-only
   threshold, matching the same "would this qualify as one run on its own
   merits" standard used everywhere else in this state machine.

**Why `reconnectEnabled` exists.** Added specifically as an investigation
tool, after case 2 above turned out to still be producing incorrect
merges the checkbox helped isolate.

### The edge check

**Low speed: fit vs. derived — a genuine judgment call**, unlike the
heading/curvature split elsewhere: a smoothed reading could in principle
blur over a brief, real stall, but a noisy raw reading can just as easily
produce a false one from a single bad position fix. Decided in favor of
the fit, for consistency with curvature.

**True gap's `+ baseDtMs` slack** is deliberate: even a well-behaved
fixed-rate track can transiently drop a single sample (a brief GPS quirk)
without that being a real gap.

**Removed checks.** Device-speed disagreement used to be a third
edge-check criterion here, but turned out unnecessary for run detection
specifically once low-speed and true-gap already existed — dropped from
this list entirely. (It also used to be the exclusion criterion for the
speed-gradient map coloring, a separate, purely cosmetic concern — also
dropped from there once a second, independent speed estimate existed for
every edge regardless, and superseded again since: gradient exclusion now
inherits directly from the classification pipeline itself — see §11's
track color modes discussion.) A fit-based `|tangentialJerk|` edge check
(replacing an even earlier vertex-level fit-accel-magnitude attempt) was
also tried here and removed — not a data-quality problem this time, a
real-world discrimination failure: too noisy in practice even on
genuinely straight-ish segments to set a useful threshold, and separately
unhelpful for its original motivating purpose (locating a jibe's true
exit) since exit jerk tends to be low anyway. Both jerk kinematics
(edge-based and fit-based) and their kinematics-chart plots stay —
genuinely useful to look at, just not usable as an automatic pass/fail
gate here.

### The vertex check

**Tangential-accel deviation — why the vertex, not either edge.** A large
disagreement between the ordinary finite-difference acceleration and the
same quantity read off the independent local polynomial fit means the
edge-based reading at this vertex probably isn't trustworthy — most often
a GPS position glitch, not a genuine sailing event. Because the reading is
attributed to the vertex itself, not to either adjacent edge, a bad
reading here implicates the edge that would come *next*: the vertex and
its incoming edge stay part of the run regardless, and only extension past
this point is what stops.

### No gap-invalidation gate

Earlier in this project, points near a gap were specially excluded from
run membership. That's gone — replaced by trusting the ordinary
vertex/heading/edge checks to catch a genuine discontinuity on their own
merits, the same way they'd catch any other real turn.

## 5. Wind direction estimation

This whole balanced-split approach replaced an earlier one (finding two
separate cluster *peaks* via a sliding-window search) that turned out to
be more machinery than the problem needed.

**Step 1's gap requirement — why balance alone isn't enough.** The search
is otherwise free to place the boundary *through* a single real cluster,
splitting one direction of sailing roughly in half and looking
artificially balanced even though there's really only one tack's worth of
data, not two symmetric ones — confirmed on real tracks, not just a
theoretical worry. A real no-go zone is reliably present (a boat
physically can't point there), but a clean gap on the downwind side isn't
guaranteed the same way — plenty of real sessions include some genuinely
dead-downwind sailing, which fills in what would otherwise be a gap
there. Requiring a wide gap at only one pole still rules out the actual
failure mode this check exists for (cutting through the middle of one
continuous cluster, which never has a real gap near *either* pole) just
as well as requiring both would, without rejecting valid splits that
simply don't happen to have a clean downwind gap. Checking a real angular
window, not a single bin, matters for its own reason: at 1° resolution a
single-bin check would be far more exposed to one stray edge's noise than
a real window is.

**Step 2's median, not mean.** A vector-sum mean is sensitive to the
total mass of scattered small-distance edges at unusual headings (warm-up
wobble, transition-zone leakage, noise) — individually negligible, but a
long tail of them can still drag a resultant vector's angle away from
where the real bulk of sailing actually is. A weighted median only cares
about where cumulative distance crosses 50%, which a scattered tail
barely moves unless it's a large fraction of the total.

**Why bins, not edges, and why 1° specifically.** Computing straight from
every edge was the original approach; it's now done from the histogram
instead, purely to make bucket analysis cheap enough to re-run repeatedly
(the manual tack-split override redoes this same computation live, every
redraw, for whatever axis the person is currently trying). A 1°-wide bin
is finer than real GPS heading noise between consecutive points, so this
shouldn't lose meaningful accuracy versus the edge-based version —
verified against synthetic tracks with known cluster centers and
top-speed directions, both recovered closely by the bin-based version.

**Step 3 — why one refinement pass corrects a real, specific failure.**
The rough split from step 1 only had to be *balanced and gapped* —
nothing requires it to actually sit on the true boundary between the two
tacks. This step corrects the case where a tack has a dense dominant
heading cluster plus a smaller, gap-separated "deep" sub-cluster (moments
sailed noticeably more downwind than that tack's typical heading). The
rough split can end up on the wrong side of that internal gap, pulling
the deep samples into the *other* bucket and biasing both medians.
Re-splitting against the wind axis — derived from the overall bulk of
both sides, not an arbitrary balance point — tends to land back on the
correct side of that internal gap instead. Confirmed against a real track
that showed exactly this pattern: the refined estimate correctly
reabsorbed the deep samples into their own tack.

**Step 4 — the removed hard agreement gate.** An earlier version required
the two buckets' rotation signs to agree (both pointing the same way
relative to the wind axis) as a hard pass/fail gate, and rejected sessions
it shouldn't have — a session with lots of struggles and breaks, or just
more time spent on one tack than the other, can leave one bucket's
fast-vs-median signal noisy (small, unreliable, occasionally even
pointing the "wrong" way) even when the other bucket's signal is
perfectly clear on its own. Requiring both to independently agree threw
away a clean signal because a noisy one disagreed with it. The current
"trust the larger separation" approach: for a balanced session both
buckets point to the same answer anyway, so this changes nothing there;
for a lopsided one, the weaker side simply no longer gets an equal,
noisier vote.

**Step 5 — why it exists.** Everything in steps 1-4 leans on the
assumption that top-speed sailing is more downwind than the median. That
assumption breaks on overpowered tracks, where a boat can point upwind
just as fast as it reaches — the whole premise step 4 relies on stops
holding, even though the session itself is otherwise completely normal.
This is the second, independent signal that doesn't touch speed at all.

**Manual controls — why wind direction and tack split are separate,
independent controls, not one.** Trusting the calculated split but
disagreeing with which side is upwind is common (flip the answer without
moving the boundary); distrusting exactly where the split landed while
leaving the up/down call alone is also common. Coupling them into one
control would force overriding one to silently override the other.

**What's left undetected — why these two specifically resist fixing
further.** A radical wind shift mid-session corrupts both buckets' own
internal statistics, and the transition-distance indicator along with
them, not just their agreement with each other — fixing it would mean
windowing the estimate over time (rolling buckets instead of one global
pass, so a shift shows up as the axis visibly moving) rather than
anything that fits within the current one-shot-per-track model. The
weak-speed override in step 5 already directly addressed what used to be
a real gap here (the overpowered-track case, upwind as fast as reaching)
before these two remained.

## 6. Jibe and tack classification

**Transition boundaries follow directly from §4's own history.** The
transition zone's own boundary points serving directly as lead-in/
lead-out only works cleanly because §4 no longer shrinks run boundaries
by one edge on each side (see §4's step 7 history) — no separate lookup
into the adjacent run's own content is needed, since the transition zone
is exactly whatever sits between two consecutive runs' own reported
endpoints.

**Crash detection — why it's not just "did the preceding run end in a
gap."** This isn't a lookup of *why* the preceding run ended — a run can
end for any reason (a clean heading break, low speed, whatever) and a
crash can still occur later, mid-transition (losing control mid-jibe,
say). Checking the reason a run ended would miss that entirely. Crash
detection is deliberately checked before, and independent of, jibe/tack —
a transition that starts with a genuine data dropout shouldn't be trusted
to represent a clean, continuous maneuver, since curvature computed
across a real gap (deliberately not null-invalidated — see §3) could
coincidentally look consistent enough to pass the jibe/tack scan by
chance.

**Sign convention** was verified numerically against this file's actual
curvature formula, matching the existing chart labels ("+left turn /
−right turn").

**Expected turn direction — the removed lead-out agreement requirement.**
An earlier version required lead-in and lead-out to sit on opposite sides
of the target pole's axis, and that was fragile in two ways: a genuine
maneuver's captured lead-out edge can already be a few degrees into
settling on the new course, and a *failed* attempt may recover onto
nearly its original heading rather than completing the turn — both cases
could put lead-out on the "wrong" side even though the maneuver itself
was real. Computing expected direction from lead-in alone sidesteps both.

**The loosened tack success path — why it exists, and why exit heading
was deliberately dropped.** Real tacks are often fast, and the strict
curvature-consistency scan can be thrown off by momentary noise (hanging
near head-to-wind, a brief stall or sail-drop) even when the maneuver
clearly succeeded by every other measure. Exit heading isn't checked at
all because a single instantaneous heading reading can be noisy,
especially coming out of a stall or sail-drop — net displacement is a
direct physical measurement of "did the boat actually make upwind
progress," a genuinely more robust signal than checking where the boat
happened to be pointed at the last sample.

**Crossing point / speed extremes — why kept as a separate post-
processing pass**, rather than folded into the delicate success/failure
scan logic above: keeping the two concerns apart meant the crossing-point
and speed-extreme computation could be added and iterated on without
risking any regression to the classification logic itself.

## 7. Local polynomial path fit and artifact detection

**Window/degree defaults arrived at through iteration, not chosen a
priori.**

**Why degree 5, not lower — the numerical confirmation.** Worth being
explicit that this is the *opposite* situation from an earlier
degree-3-vs-4 finding, not an extension of it: that finding showed degree
3 changed nothing at a *symmetric* window's exact center, because
odd-order terms provably vanish there. The window here is asymmetric by
construction (`numBefore=4`, `numAfter=3` for the N=8 default), so that
provable-vanishing property doesn't apply, and adding the degree-5 term
genuinely changes the fitted result — confirmed numerically against a
synthetic exponential-transient trajectory, where the fitted `a1`
(velocity) and `a2` (half of acceleration) both measurably shifted
between degree 4 and degree 5 for the same window and same data, rather
than matching to several decimal places the way the degree-3-vs-4 case
did.

**Why 8 points specifically.** The over-determination margin (2 "extra"
points) matches the earlier 7-point/degree-4 configuration, so the
smoothing-vs-snugness balance doesn't shift just because the degree went
up.

**Why centered, not trailing — a reverted design.** A trailing window
(evaluate at the window's own last point, using only preceding history)
was tried first and reverted. The appeal was avoiding a centered window's
own limitation — needing points from both sides means a point sitting
just before a gap/crash needs to look past it too, losing kinematics
there along with everything actually inside the gap. But **leverage**
(how much a fitted curve's value at a point depends on that point's *own*
observed value, vs. everyone else's in the window; formally the diagonal
of the regression hat matrix, `H = X(XᵀX)⁻¹Xᵀ`) at a window's true
trailing endpoint measured at 0.996 for the current default (8 points,
degree 5) — essentially 1, meaning the fit was nearly forced through that
one point's own value, giving away almost all the smoothing the wider
window was supposed to provide. Reverted back to centered, where leverage
at the evaluation point is meaningfully lower (0.55 for the same 8-point,
degree-5 configuration) — real smoothing, at the cost of needing forward
history and therefore losing kinematics for the last few points before a
gap.

Leverage is fundamentally tied to how informative a point is about *how
the curve is changing*, and points at a window's extremes are unavoidably
more informative about that than central ones, for any model expressive
enough to represent motion at all — the only way to get uniform leverage
is a model with no positional sensitivity whatsoever, literally just
fitting the mean, which can't represent a path. So this isn't a
shortcoming specific to this implementation; it's what any windowed
polynomial fit trades against, and centered is simply the better side of
that trade for this tool's purposes.

**Fitting across a true gap — tried skipping it, and reverted.** Fitting
across a genuine dt gap (device dropout — a crash, most commonly) asks
the curve to smoothly reconcile two disconnected local trends across a
time span with no data to constrain it in between, which reliably
produces a loop or a large excursion on the map — verified: a synthetic
crash scenario showed the fit overshooting to more than double the
plausible position range in the middle of the gap. But the *kinematics*
derived from it still came out as reasonable values in practice, and
skipping the fit entirely near every gap lost real data for the several
points right before a crash, which is often the most useful part to see.
Accepted as a known, cosmetic-only quirk rather than fixed.

**Tangential-acceleration deviation — the earlier, more elaborate
versions.** Went through several before landing on the current fit-vs-
edge comparison: a 4-edge windowed jump/dip comparison with a
heading-consistency filter, then a simpler EWMA-based deviation check —
each needing progressively more special cases to catch real artifacts
without also flagging genuine turns or genuine acceleration. The fit is a
better local trend line than either predecessor specifically because it
doesn't lag during real acceleration the way a trailing average does (a
backward-difference edge speed is structurally an average velocity over
the *past* interval, so it systematically undershoots during real
acceleration and overshoots during real deceleration) — confirmed against
synthetic exponential-approach acceleration, where the fit tracked the
true instantaneous speed far more closely than the edge speed did.

**Arc-length deviation — the sign-meaning confirmation.** Confirmed
against a synthetic spike test: -11.5 at the glitch itself and an even
larger -21.9 at its immediate neighbor, whose own window still contains
the glitch without it being the evaluation point.

## 8. Duplicate cleanup: timestamps and coordinates

**Duplicate timestamps — why speed-balancing, not distance-balancing.**
An earlier version balanced distance directly (`|distBefore -
distAfter|`), which seemed reasonable but has a real flaw: the typical
real cause of a duplicate timestamp is a point with a genuinely correct
position that got snapped to the wrong time, leaving `dtBefore` and
`dtAfter` asymmetric (one side effectively swallowed the sample that
should have been there). Balancing distance while ignoring that asymmetry
lets the longer-duration side's computed speed come out silently lower
(or the shorter side's silently higher) even when the true physical speed
through here was roughly steady — confirmed with a synthetic case
(`dtBefore=1s`, `dtAfter=2s`): distance-balancing picked a point implying
a jump from 15 m/s to 7.5 m/s across it, while speed-balancing picked the
point implying a flat 10 m/s on both sides, the actual constant-speed
answer the scenario was built from.

**Duplicate coordinates — why speed-balancing, not time-balancing.** An
earlier version balanced time directly (`|dtBefore - dtAfter|`), which
turns out to have the same flaw the timestamp cleanup's own earlier
version did, just with distance and time swapped: a synthetic case with
`distBefore=10m`, `distAfter=20m` showed time-balancing picking the
timestamp splitting the interval exactly in half, which implies a flat
speed *doubling* across it (5 m/s to 10 m/s) — because equal time over
unequal distances is unequal speed, not equal. Speed-balancing instead
lets the resulting wider gap be handled by the ordinary gap/true-gap
machinery instead of producing a fake zero-velocity segment.

## 10. UI controls reference

**Wind-estimation constants used to be UI-exposed.** A handful of them
were exposed while the algorithm was under active iteration; now settled,
they're plain constants in code instead.

## 11. UI layout and mobile support

**Header stats bar's "recompute on every redraw" — the staleness bug it
fixed.** Folded in after fixing a real staleness bug: it used to run
once, early, before the wind-direction textbox had been initialized from
the calculated estimate, so its own transition classification saw no
wind direction and reported zero jibes/tacks, and nothing ever refreshed
it afterward.

**Why session duration/length are narrower than the track's own total.**
Deliberately trimmed of any warm-up, breaks, or dead time before/after
the session itself; session length sums *every* edge in the span, not
just run-classified ones, matching session duration's own "the whole
trimmed session" scope.

**Why run count is a percentage of session duration, not the full
track.** Diluting by dead time the session itself doesn't include would
understate how much of the actual session was spent running.

**Panel order — why wind estimate and run detection come before track.**
They're the smaller, usually-collapsed panels, and reading order roughly
matches "set up wind and tune detection, then look at the track" rather
than the reverse.

**Run detection's own panel — why it exists as a separate panel.** Added
once the run-detection controls, originally a single flat row buried in
the threshold/action controls panel at the bottom of the page, grew past
the point a flat row could present clearly.

**Kinematic plots — the five-to-one-canvas collapse, and later
additions.** Originally five separate always-visible canvases, collapsed
into one canvas plus a dropdown. Path length deviation and the two jerk
plots were added later, after that original collapse, bringing the total
to eight. The `KINEMATIC_PLOTS` config maps each dropdown option to
values that used to be hardcoded per canvas.

**Touch handling — why separate handlers, not merged into the existing
mouse/wheel ones.** Lower risk of regressing already-working desktop
behavior, and pinch-zoom needed dedicated multi-touch logic regardless of
how single-pointer drag was handled.

**OSM tiles — checking whether the projection mismatch actually
matters.** Before relying on drawing each tile as a simple axis-aligned
image despite the track using a different projection than the tiles:
projected a real tile's four corners into the track's own ENU space at a
representative mid-latitude origin a few km away, and found opposite edge
lengths matched to within 0.02% and edge angles to within 0.001°, well
under anything visible.

**Track color modes — the removed device-speed gradient check.** No
separate device-speed check in the gradient anymore, even where device
speed is present in the file — an earlier version had its own,
gradient-only device-speed-disagreement threshold, dropped once a second,
independent speed estimate already existed for every edge regardless
(the local path fit — §7), and superseded again since: inheriting from
run detection picks up its own device-speed check (§4) automatically
wherever that governs, without this needing any device-speed logic of
its own.

**Track color modes — gradient edges switched to fit speed too, not just
the range.** A deliberate change from an earlier version where only the
range came from the fit and each edge's own color still came from the
raw, edge-based reading. The fit is a second, independent speed estimate
that naturally smooths out an isolated glitched edge, so one bad point
can't drag the whole color scale's bounds along with it — and reading it
consistently for the edge color too, not just the range, keeps a single
glitched point from painting itself with a wildly different color than
its genuinely similar-speed neighbors.

**`preferFitSpeed` — why all three UI surfaces switched together.** A
second, independent speed estimate exists for every edge regardless of
whether device speed happens to be present, and prefers it consistently
rather than mixing sources depending on which UI element happens to be
showing it.

## 12. Track artifacts: a taxonomy and a design not yet built

Nothing in this section is implemented — §7's tangential-acceleration and
arc-length deviation detectors exist and work; everything below is design
discussion toward a *correction* mechanism, which does not exist yet.
Recorded here so the reasoning isn't lost before it's built.

### What the detectors are actually catching

Working through real flagged segments surfaced (at least) three distinct
underlying causes, not one:

- **GPS jitter.** A point's *timestamp* is trustworthy but its *position*
  is slightly off — usually on an otherwise straight-ish stretch. The
  point still sits close to the true route; it's placed at the wrong
  point *along* that route for its timestamp. This warps the two edges
  touching it (shrinks one, stretches the other, or vice versa),
  producing jagged tangential acceleration without any real path-shape
  problem. This is the case the deviation detectors were originally built
  for, and the case any future correction mechanism should target.
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
    device is no longer measuring "where is the board" for that stretch.
    Not degraded signal about the thing of interest; not a signal about
    that thing at all for the duration. Untested as of this writing — a
    natural follow-up whenever a track with a genuine swim/uphaul segment
    is available, to see whether it produces the dense, sustained
    flagging the controlled-stall case didn't.
- **Crash gaps** (device dropout). Already and separately caught by `dt`
  (§4's `edgeGapFails`, reused throughout §6, §7). The deviation
  detectors *can* also flag these (a large `dt` gap can produce a fit
  loop or excursion — §7), but this is redundant detection of something
  already known via a cleaner signal, not a new capability. Useful as a
  cross-check but `dt` should keep priority — a flagged point that's
  already part of a known `crash_recovery` zone shouldn't be routed into
  whatever correction workflow eventually exists for jitter.

The practical implication: only the jitter case is a good target for
position *correction*. The stall/swim case, if the controlled-vs-active
distinction above holds up, may not need new discrimination logic at all
— the *existing* deviation detector, applied to a low-speed segment, may
already read differently for "held still" vs. "actively moving in the
water" without anything new being built. Worth testing directly before
building anything to handle it.

### A correction design: separate shape from pacing

The first design considered — replace a flagged point's position with
wherever §7's own fit predicts for that timestamp — has a real flaw: it
would discard the point's own recorded position, which (per the jitter
diagnosis above) is usually *not* the actual problem; the position is
close to correct, only its pacing along the route is off. Overwriting it
with a fresh fit evaluation throws away good information and — since the
fit can itself loop or overshoot under some conditions (§7) — risks
moving a fine point to a *worse* spot.

The alternative worked out instead: treat "what the route looks like" and
"how far along it a point should be at a given time" as two separate
fits, not one.

1. **Shape.** A curve through the segment's own recorded positions,
   parameterized by something neutral like cumulative chord length — not
   time. Answers "what does the route look like here," independent of
   pacing. Includes the flagged points' own positions, since (for the
   jitter case) those are trusted.
2. **Pacing.** A separate fit of arc-length-along-the-shape-curve as a
   function of time, built using only the segment's non-flagged (trusted)
   time/arc-length pairs.
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
this design was meant to solve outright, not fully escaped. Possibly fine
in practice; possibly wants the shape curve's parameterization anchored
only to the segment's trusted points instead. Unresolved as of this
writing.

**Segment-length question, also unresolved:** a short, isolated glitch (a
point or two) seems like a safe target for this. A long stretch of
consecutive flagged points — a genuine multi-second dropout, especially
one where a real maneuver might have happened inside it — may not be
well-represented by a single polynomial at all. Whether there's a length
past which correction should refuse and fall back to exclusion instead of
attempting a fit, and if so where that length sits, hasn't been decided.

**A cheap validation step worth keeping in mind:** once a segment is
corrected, re-running §7's own tangential-acceleration detector against
the corrected result is a direct, nearly-free check of whether the
correction actually worked. Device speed, where present, is a second
independent check for the same purpose — it's not derived from position
at all, so it can't be fooled by anything wrong with position.

**Whatever gets built here should look different from every other action
this tool takes on a track.** Deletion (§7) is currently the most
invasive existing action; position correction goes further, since it
asserts a *specific* corrected value rather than just admitting distrust.
Any implementation should be opt-in, visually distinct on both the map
and the kinematics charts, and reversible the same way "restore original
track" already reverses the duplicate-cleanup passes (§8).

