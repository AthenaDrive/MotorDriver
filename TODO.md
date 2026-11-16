# TODO
- PCB next rev: fix ETH_1/port-2 W5500 RX termination (missing 49.9R to CT, CT shorted to RXN); BOM 8x 49.9R.
- PCB next rev: 1k/10n RC on each INA241 output at the ADC pin (kills switching-edge spikes; firmware now averages 4 samples for the 30 kHz ripple).
- FW: fix discovery boot race, task-config lifetime, TCP server VLA. Details: AGENTS.md §9.
- Potentially use IP175GHI ethernet switch