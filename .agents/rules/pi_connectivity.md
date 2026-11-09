# Raspberry Pi Connectivity Rule

- Test Pi connectivity with a single quick check (e.g. `ssh -o ConnectTimeout=2 robot ...` or `Test-Connection -Count 1`).
- If the Pi is unreachable or the connection times out:
  - DO NOT retry in a loop.
  - DO NOT run network/subnet sweeps or repeated ping loops.
  - Report immediately that the Pi is unreachable (noting common hardware causes: brownout/undervoltage from weak USB power, powered off, or disconnected Wi-Fi) and ask the user how they would like to proceed.

