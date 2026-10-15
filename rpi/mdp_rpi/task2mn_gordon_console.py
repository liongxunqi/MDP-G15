"""
task2mn_gordon_console.py - test_stm.py's interactive prompt, taught the
protocol 9 and 10 commands the shared communications/stm.py predates:
!IRE / ?IRE / ?IRW (task2mn_link) and V/C movement suffixes, ?SEG and
192-byte lines (task2mn_gordon_link).

    python3 task2mn_gordon_console.py            # TIGHT profile (0), as tasks use

Then type lines exactly as PROTOCOL.md shows them, e.g.

    ?VER                      VER,MDPG15-STM32,10 on the new firmware
    !FAST1                    fast straights (protocol 9 tokens)
    F150                      ...then RST while it runs, then ?DIST
    !IRE21028                 arm the right-IR edge latch, 10..28 cm
    F80                       ...then ?IRE
    F30V15C,FR90V15C,F60V15   protocol 10 only: one continuous line
    q                         quit
"""

import sys

import task2mn_gordon_link

if __name__ == "__main__":
    task2mn_gordon_link.enable()
    import test_stm
    test_stm.main()
