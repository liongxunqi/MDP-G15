# Task 1: Android assessment workflow

## Operator flow

1. Connect on Controls, then open Arena. Use **normal** framing for the Task 1 RPi
   line protocol (AMD tool mode is for the separate manual test tool), then finish
   obstacle placement/facing.
2. Complete 4–8 obstacles and select an image face for each. The Arena has one
   **Start Task 1** button with the reason displayed if setup is incomplete. RPi's
   existing automatic planning after map updates remains unchanged. No planning
   notification is required to enable Start. Explicit `PATH` is still available
   through the Controls custom-command field for debugging.
3. After supervisor approval, tap **Start Task 1** once. `BEGIN` is sent once and
   Android immediately reserves the run, preventing another Start or Path request.
   No valid pose is required to send BEGIN; pose-dependent manual controls remain separate.
4. Android stays on Arena and automatically fits the full map at Start and completion. START/RUNNING reports confirm progress; TARGET updates
   the corresponding obstacle immediately. DONE retains the final map and labels.
5. Keep the map displayed for inspection and screenshot evidence. Only after
   interaction is permitted, use **New attempt** and confirm **Clear results**.
   This retains obstacle positions/faces, clears recognition labels and edit history,
   and sends no robot command. It does not reset RPi's mission state or consume an
   official retry. Coordinate robot-side preparation before starting another run.

Start is unavailable offline, outside the 4–8 obstacle count, with missing faces,
during movement, during unfinished placement,
while an earlier run is active/uncertain, or while old recognition results remain.
The general editor still permits up to 50 obstacles; only Task 1 starting is restricted.
“New attempt” is also available for pre-existing results without a terminal status.
Editing/undoing a map after requesting planning returns its planning label to Setup.

The foreground controller holds Android's keep-screen-on flag across all tabs,
connection changes and completion. Disposing the controller restores the previous flag.
This prevents ordinary inactivity timeout; it cannot prevent physical power-button
locking, OS shutdown, battery depletion, or the app being put in the background.
No Android timer sends a stop command or claims the robot completed autonomously.

## Interruption and result handling

- Disconnecting during a run keeps it active but marks status unknown. Reconnection
  does not send CLEAR, PATH or BEGIN for that run. A fresh START/RUNNING/DONE/FAILED
  report reconciles status. No acknowledgement popup or recovery tap is requested.
- Late RUNNING/START reports cannot reopen a completed/failed/stopped attempt. The
  first terminal outcome is retained separately from diagnostic robot messages.
  Late TARGET/pose updates may still complete the displayed result map.
- Completed labels and Task 1 phase survive SavedStateHandle restoration. An active
  attempt restored this way is marked unknown, not silently restarted. This is Android
  saved-state restoration, not a durable archive after force-stop or app-data clearing.
- An exception while sending BEGIN keeps the run uncertain: partial delivery cannot
  be ruled out, so automatic retries are unsafe. PATH send exceptions allow a retry.
- No protocol attempt ID exists. A delayed TARGET/DONE from an earlier attempt cannot
  be reliably distinguished after New attempt. RPi must finish/drain the prior session
  before a new run; robust cross-attempt isolation requires a shared protocol change.
- Queue acceptance is not delivery/mission acknowledgement. RPi must provide current
  status after reconnect; Android cannot recover missed results by itself. Lost messages,
  robot-side start deduplication and attempt-aware planner readiness remain integration work.

## Evidence responsibilities (fix 11)

Android provides the Task 1 live/final result map. There is no Android photo gallery.
Take the result-map screenshot using the tablet's normal screenshot controls after
permission for post-run interaction. Preserve it before New attempt or Reset arena.

The PC team owns the camera montage and must verify automatic display, representative
full-scene images with bounding boxes, association with Android image IDs, and screenshot
capture for the supervisor. Current `pc/task1_pc.py` saves `stitched_result.jpg`
without opening it. Its detector saves centre-cropped annotations; it should instead
retain the original scene/background for evidence (inference cropping can be separate).
The PC also needs to verify its annotation directory: the detector uses an absolute
path under `pc/runs/predict`, while the server currently looks up a relative path.
These PC observations are a handoff, not Android changes. No PC/RPi code is modified here.

## Verification

Automated coverage includes planning/start separation, synchronous duplicate-start
protection, offline/placement/movement guards, result retention, late status handling,
new-attempt clearing without map mutation/transmission, saved-state restoration,
reconnect without CLEAR/BEGIN replay, send exceptions, and Compose screen-awake lifecycle.
Tablet UI tests cover the Arena buttons, live results, confirmation cancellation,
Controls custom-BEGIN routing to Arena, and existing map/manual-control regressions.

Before assessment, perform a physical integration rehearsal:

- Run longer than the tablet's normal timeout, including final result presentation.
- Verify actual official TARGET IDs against the obstacle and PC image evidence.
- Confirm PATH then BEGIN, delayed planning, and one BEGIN only.
- Interrupt Bluetooth while moving; verify robot-side autonomy, fresh status/results
  on reconnect, and no CLEAR or repeated BEGIN from Android.
- Confirm physical autonomous stop and PC montage display without operator intervention.

## Optional planning notification: Android receive contract

No RPi changes are required for the current Start flow. The Android extension accepts:

```text
PLANNER,READY
PLANNER,READY,<map_sha256>
```

These are complete newline-delimited messages, not `MSG` display text and not mission
START/DONE statuses. `READY` is exact uppercase. The optional hash must be 64 lowercase
hex characters. A full-message regex (`PLANNER,READY(?:,([0-9a-f]{64}))?` via
`matchEntire`) validates the payload after outer transport whitespace/CRLF normalization.
Internal spaces, lowercase tags, uppercase/non-hex hashes, missing/extra fields and
embedded line breaks are rejected without changing the previous readiness display.
Separate newline-delimited messages are validated independently.

The basic form displays “RPi reports path ready — map version unverified”. It is only
an informational report: Android cannot associate an unversioned report with a map.
The optional hash is SHA-256 of UTF-8 `ObstacleSync.encode(obstacles)`: `CLEAR` followed
by LF-separated `OBSTACLE,id,x*10,y*10,FACING` lines sorted by numeric ID, with no trailing
newline; full uppercase direction names. It excludes recognition results and robot pose.
A matching hash displays “RPi reports path ready for this obstacle map”; a mismatching
hash is ignored. No additional outgoing messages are introduced.

Readiness never starts a run, bypasses local checks, releases run locks, or blocks Start
when absent. It is accepted only while connected in Setup/Path requested/Start requested.
It is ignored during Running, unknown connection state and terminal result presentation.
Edits, undo/redo, manual movement, reconnect, new attempts and explicit PATH requests
invalidate the informational readiness. It is intentionally not restored after process
recreation. A fingerprint confirms map contents only, not freshness, robot pose or attempt;
a delayed report for the same map still needs a future attempt/request identifier to
prove freshness. The basic form cannot reject delayed reports after an edit, so its
unverified label is deliberate. Physical robot execution still requires BEGIN.
