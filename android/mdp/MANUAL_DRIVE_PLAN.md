# Checklist repair sequence

Android only. Obstacle count, ID allocation/validation and target-ID validation belong to the teammate and are outside this change.

1. Send-time movement guard: use the checked-in manual bridge's F/R20, turns90/45 and TIGHT radius29.1cm. Check a swept rotating 2x2 footprint, reserve before sending, reject unknown pose, pending movement and disconnection. Require STATUS,OK to release an outstanding command; Stop/failure/disconnection require fresh position. Test spam, edges, obstacles, turns and lifecycle before proceeding.
2. Keep grid, manual pad, status and Stop on Arena. Landscape: grid left, bounded control column right. Portrait: bounded grid above controls. Scroll only arena editing details; keep driving visible. Retain Bluetooth/custom messages/Begin/Path on Controls. Verify both orientations and existing obstacle interactions.
3. Show locally estimated movement immediately, preserving separate reported pose semantics. Pending commands disable directions but not Stop. Never treat a preview timer as robot acknowledgement. Verify absent, delayed and invalid replies and cancellation.
4. Animate the checked-in arc geometry with continuous headings (including diagonals), not cardinal snapping. Preserve unrounded positions across commands. Verify reverse arcs, U-turns and smooth intermediate headings.
5. Display raw latest RX independently of parsed feedback. Preserve strict validation but clear stale parse errors on later successful messages. Test malformed then valid and multiple-message framing.

Physical position cannot be guaranteed by Android without measured telemetry. Defaults must match bridge environment/calibration. The manual bridge acknowledges completion but does not report its resulting position. No Raspberry Pi changes are included.

## Completion and validation

All five stages are implemented. Unit coverage includes all cardinal edges,
swept collisions, repeated callbacks before recomposition, pending/failed moves,
fresh-pose recovery, estimated versus reported coordinates, reverse arcs,
U-turns, mid-arc headings, and malformed-to-valid message recovery. Emulator
tests verified all eight controls, visible grid and persistent Stop in portrait
and landscape. The full 20-test app/arena interaction suite passed. Builds,
unit tests, lint and four protocol-v3 compatibility checks passed.

Additional edge cases addressed: custom movement text uses the same gate;
autonomous/manual movement cannot interleave; editing is blocked during motion;
invalid robot telemetry cannot leave driving enabled against a stale pose.
No obstacle count, ID allocation or target-ID validation rules were changed.

Hardware acceptance remains necessary: confirm manual bridge calibration,
establish a starting pose, verify one physical move and its completion reply,
then test obstacle/edge refusal and Stop. Android preview timing is illustrative.

## Final requirement audit

| Requirement | Implementation and evidence |
| --- | --- |
| Prevent off-map and colliding D-pad commands, including spam | `ManualDriveGuard` reserves synchronously before sending; `DrivePath.fits` checks swept rotating footprints. `ManualDriveTest` and `ArenaViewModelTest` cover unknown poses, edges, obstacles, repeated taps, malformed telemetry, disconnects and failure. |
| Grid, D-pad, status and Stop together; details scroll internally | `ArenaScreen` and `ArenaDrivePad` keep the grid and driving pad outside the scrollable details content. Emulator tests for all controls, obstacle editing and long status passed in portrait and landscape; screenshots were inspected. |
| Immediate local grid update, smooth movement and loading protection | `drive` publishes an explicitly estimated pose before transport submission. Both acknowledgement and preview completion are required before another movement. Tests prove missing replies remain locked and an early reply cannot skip the animation gate. |
| Realistic turning and eight visual headings | `DrivePath.at` retains fractional arc coordinates; `RobotMotion` samples continuous headings. Tests cover intermediate 45-degree heading, reverse arcs, U-turn displacement, wraparound and straight interpolation. Wire obstacle faces remain the original four directions. |
| Accurate latest received message and recovery from stale errors | `latestReceived` is independent of parsing feedback. Tests cover malformed input followed by valid/batched robot and status messages. Only documented `STATUS,OK` acknowledges movement, not ordinary `MSG,[OK]` text. |
| Preserve scope | RPi and algorithm files have no diff. Obstacle count, ID allocation/validation and target-ID validation rules are unchanged. |

Final validation: 59 unit tests passed; the original 20 app/arena emulator tests
passed, followed by the added long-status test and targeted layout/driving checks
in both orientations (21 unique interaction tests covered). Android debug builds,
both modules' lint checks, `git diff --check`, and all four protocol-v3 reference
checks passed. Final portrait and landscape screenshots were visually inspected.

## Immediate obstacle placement

Each Add obstacle press now creates and selects one obstacle in the first free
cell, scanning left to right from the bottom row. No second grid tap is needed.
Drag it to its intended location and choose its target face afterward; an unset
face retains the existing SKIP wire representation. Repeated presses use current
state synchronously and each accepted addition has its own undo entry.

The automatic action reuses the reducer's existing placement validation and
ViewModel edit/synchronization path. It respects the 50-obstacle cap, skips
occupied cells including fractional/rotated robot footprints, rejects a full
arena, and cannot edit during pending movement, preview animation or autonomous
runs. ID reuse and target-label rules are unchanged. Newly added obstacles do
not inherit metadata from deleted obstacles. Offline additions persist and the
full map is sent on reconnect; dragging and face changes still update that map.

Regression coverage includes 100 rapid additions, cap rejection without extra
outbound messages/history entries, hole reuse, a robot filling the entire arena,
rotated footprint exclusion, undo/redo, restoration/reconnect and movement locks.
UI tests add multiple obstacles without grid taps and then edit the selected
obstacle's face. Existing grid drag tests remain in the emulator suite.

An unknown robot position cannot be collision-checked until telemetry arrives;
this preserves existing behavior. Transport delivery and delayed recycled-ID
results retain the protocol limitations described in INTEGRATION_REVIEW.md.

Validation: 72 unit tests, 22 emulator tests in portrait, and 2 additional
landscape Add/face-edit tests passed. Debug builds and both module lint checks
passed. No physical robot delivery claim is implied by emulator testing.

## Drag label clarification

The grid continues to show the recognized target, falling back to the obstacle
ID. During a valid obstacle drag the label is the recognized target, falling
back only to `?` (target unknown). Selecting N/E/S/W changes the highlighted
edge; it never substitutes a direction letter for the target in the drag label.
No stored target, direction, ID or outbound protocol field is changed by this
presentation change.

Emulator regression coverage exercises unset and all four directions with
unknown, alphabetic and two-digit numeric targets, including a target equal to
the obstacle ID. It checks the actual Canvas text for the grid, drag and
cancelled drag. Existing tests cover face-edge rendering, valid movement,
outside-grid deletion, zoomed deletion, robot drag cancellation and viewport
clipping. An additional app test reaches the 50-obstacle cap through the Add
button and checks map snapshots and button availability after undo/redo.

Combined validation after the drag-label change: 72 unit tests, 24 portrait
emulator tests (14 app, 10 grid) and 2 additional landscape app tests passed.
Both modules' lint checks passed, and debug application/test APKs built as part
of connected testing. The emulator was restored to portrait afterward.
