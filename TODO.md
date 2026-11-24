# TODO
- FW: verify DHCP + controller-IP learning on a fresh PC (NIC on "automatic"): board 0 should log "DHCP server on ETH_0: ESP_OK", the PC should get 192.168.0.3+, and telemetry must follow it.
- PCB next rev: fix ETH_1/port-2 W5500 RX termination (missing 49.9R to CT, CT shorted to RXN); BOM 8x 49.9R.
- PCB next rev: 1k/10n RC on each INA241 output at the ADC pin (kills switching-edge spikes; firmware now averages 4 samples for the 30 kHz ripple).
- Potentially use IP175GHI ethernet switch