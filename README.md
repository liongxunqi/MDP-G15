# MDP-G15

## Layout

```
algo/                  path planner (pure Python, runs on the PC)
├── path_planner.py    visit order, photo distances, STM token output
├── grid_search.py     A* motion search with arcs and collision checks
├── stm_tokens.py      STM token builders; mirrors limits in rpi/.../communications/stm.py
├── tests/             test_path_planner.py, test_mission_sim.py
└── legacy/            old prototypes and the week 6 simulator (not used by Task 1)
pc/                    PC server: task1_pc.py, YOLO (image_recognition/, weights/)
rpi/mdp_rpi/           everything that runs on the Raspberry Pi (task1.py, task_a5.py, ...)
android/               tablet app
stm/                   STM32 firmware and PROTOCOL.md
```

## Running Task 1

From the repo root:

```bash
python3 rpi/mdp_rpi/task1.py      # on the RPi, first: it is the TCP server
python3 pc/task1_pc.py            # on the PC, second
```

Then connect Android, send the obstacles and press Begin. See `rpi/mdp_rpi/README.md` for setup.

Each run writes a log with an end-of-run summary of warnings and errors: `rpi/mdp_rpi/logs/` on the RPi, `pc/logs/` on the PC.

## Tests (no robot needed)

```bash
python3 algo/tests/test_path_planner.py         # planner unit tests
python3 algo/tests/test_mission_sim.py --random 0   # replay the fixed layouts
python3 algo/tests/test_mission_sim.py          # plus 30 random layouts
python3 pc/test_run_log.py                      # PC server against a fake RPi
```
