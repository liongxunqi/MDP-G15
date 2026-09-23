# Android integration review

Base before integration: `981a0f8`. Incoming `origin/main`: `e13172f`.

Incoming commits:
- `dc6abbd`: recycle the smallest available positive obstacle ID.
- `e13172f`: require 1–2 uppercase ASCII alphanumeric target characters and reject unknown obstacle references in controller status.

## Resolution

The combined version retains both sets of features; selecting either entire side
of the conflicts would lose required behavior.

| Area | Decision and rationale |
| --- | --- |
| IntegratedControllerScreen conflict | Keep the synchronous manual movement guard and combined grid/D-pad UI. Retain unknown-target rejection, but bind a live obstacle lookup instead of copying IDs after Compose recomposes. |
| ArenaViewModel persistence conflict | Remove the obsolete increasing-ID counter as the incoming change requires. Keep the local rule that estimated robot poses are not persisted as confirmed telemetry. |
| ID allocation | Keep the teammate's smallest-free-ID allocation. Surviving obstacles are not renumbered, and a new obstacle does not inherit a deleted obstacle's target or face. |
| Target validation | Keep the teammate's uppercase alphanumeric rule. Centralize it for the wire codec, domain actions and saved-state restoration. Invalid old labels are cleared without deleting their obstacles. |
| Failing incoming test | Update its expected positive-integer error wording to match the implementation; retain and expand rejection assertions. |

## Tests and edge cases

Added coverage for immediate lookup during outgoing map writes (before another
composition), additions/removals through undo and redo, restoring a legacy saved
counter, cleared detection metadata on ID reuse, reconnect snapshots, invalid
targets preserving valid detections, unknown references, lowercase/negative/long/
Unicode labels, domain-level validation, and the 50-obstacle cap after ID reuse.
Recycled obstacles must still prevent a colliding manual movement.

The previous movement, animation, telemetry, portrait/landscape layout, connection,
logs and obstacle tests remain part of the combined suite.

Final validation:
- 66 unit tests passed (12 app, 54 arena).
- 22 emulator tests passed in the combined candidate checkout.
- 3 additional portrait checks passed from the integrated main checkout, covering
  immediate target lookup, movement/Stop boundary guards, and target-face updates.
- Debug builds and both modules' lint checks passed from the main checkout.
- All 4 protocol compatibility checks passed against the read-only
  `origin/protocol-v3-fu` reference (`90478b6`).

## Protocol limitation

An old delayed `TARGET,1,11` and a new valid `TARGET,1,11` are indistinguishable
after ID 1 has been reused: the wire format carries no obstacle generation or
mission identifier. Android blocks map editing during known manual/autonomous
movement, reducing exposure, but cannot guarantee rejection of all delayed
results without receiver/protocol changes. No Raspberry Pi code is changed.

## Preservation

The original uncommitted files and their hashes were saved under
`/tmp/mdp-integration-p3y4h5tk/original-files`; `local.patch` captures the original
tracked changes. All trial merging and initial tests ran in the sibling
`candidate` checkout. The original working files were verified unchanged before
integration. Main was fast-forwarded to `e13172f`, and the tested combined changes
were restored to the working tree. They remain uncommitted and unstaged. The
user's `.gitignore` changes were preserved byte-for-byte. No RPi or algorithm
source was changed by this integration.

An additional safety stash retains the original work:
`3ebdcdf763433dab92e847444f133761bf308e26`
(`Safety backup before tested Android integration e13172f`).
