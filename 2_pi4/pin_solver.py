import csv
import re
import os
from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence, Tuple

# Complete Raspberry Pi 5 (RP1 I/O Controller) pinout database
RAW_PINOUT_CSV = """Header_or_Connector,Physical_Pin,Signal_Name,GPIO_Number,Type,Primary_Function,ALT0,ALT1,ALT2,ALT3,ALT4,Notes
40-Pin Header,1,3V3,N/A,Power,3.3V DC Power,N/A,N/A,N/A,N/A,N/A,Max 50mA total draw recommended across 3.3V pins
40-Pin Header,2,5V,N/A,Power,5V DC Power,N/A,N/A,N/A,N/A,N/A,Direct 5V power rail supplied from USB-C PSU
40-Pin Header,3,GPIO2,GPIO2,I/O,I2C1_SDA,SDA1,I2C2_SDA,UART3_CTS,SPI3_MISO,DPI_D2,Fixed 1.8k onboard pull-up resistor to 3.3V
40-Pin Header,4,5V,N/A,Power,5V DC Power,N/A,N/A,N/A,N/A,N/A,Direct 5V power rail supplied from USB-C PSU
40-Pin Header,5,GPIO3,GPIO3,I/O,I2C1_SCL,SCL1,I2C2_SCL,UART3_RTS,SPI3_SCLK,DPI_D3,Fixed 1.8k onboard pull-up resistor to 3.3V
40-Pin Header,6,GND,N/A,Ground,0V Ground,N/A,N/A,N/A,N/A,N/A,System Ground
40-Pin Header,7,GPIO4,GPIO4,I/O,GPCLK0,GPCLK0,SPI3_CE0,UART3_TX,I2C3_SDA,DPI_D4,General Purpose Clock 0 / PWM0_0 channel
40-Pin Header,8,GPIO14,GPIO14,I/O,UART0_TX,TXD0,SPI4_MISO,I2C4_SDA,DPI_D14,PWM1_2,Default primary Linux serial UART Transmit
40-Pin Header,9,GND,N/A,Ground,0V Ground,N/A,N/A,N/A,N/A,N/A,System Ground
40-Pin Header,10,GPIO15,GPIO15,I/O,UART0_RX,RXD0,SPI4_SCLK,I2C4_SCL,DPI_D15,PWM1_3,Default primary Linux serial UART Receive
40-Pin Header,11,GPIO17,GPIO17,I/O,GPIO,SPI0_SCK,UART1_RTS,SPI4_CE0,I2C5_SDA,DPI_D17,General Purpose I/O / PWM0_1
40-Pin Header,12,GPIO18,GPIO18,I/O,PCM_CLK / PWM0,PCM_CLK,SPI4_CE1,UART1_CTS,I2C5_SCL,DPI_D18,Hardware PWM0_0 Output / I2S Clock
40-Pin Header,13,GPIO27,GPIO27,I/O,GPIO,SPI5_CE1,UART2_RTS,I2C6_SCL,DPI_D23,PWM1_3,General Purpose I/O
40-Pin Header,14,GND,N/A,Ground,0V Ground,N/A,N/A,N/A,N/A,N/A,System Ground
40-Pin Header,15,GPIO22,GPIO22,I/O,GPIO,SD1_CLK,ARM_TRACECTL,DPI_D18,SPI0_CE1,PWM1_0,General Purpose I/O
40-Pin Header,16,GPIO23,GPIO23,I/O,GPIO,SD1_CMD,ARM_TRACECLK,DPI_D19,SPI0_MISO,PWM1_1,General Purpose I/O
40-Pin Header,17,3V3,N/A,Power,3.3V DC Power,N/A,N/A,N/A,N/A,N/A,3.3V Power Rail
40-Pin Header,18,GPIO24,GPIO24,I/O,GPIO,SD1_DAT0,ARM_TRACED0,DPI_D20,SPI0_SCLK,PWM1_2,General Purpose I/O
40-Pin Header,19,GPIO10,GPIO10,I/O,SPI0_MOSI,SPI0_MOSI,I2C3_SDA,UART4_TX,SPI2_SCLK,DPI_D10,SPI0 Controller Output / PWM0_2
40-Pin Header,20,GND,N/A,Ground,0V Ground,N/A,N/A,N/A,N/A,N/A,System Ground
40-Pin Header,21,GPIO9,GPIO9,I/O,SPI0_MISO,SPI0_MISO,I2C3_SCL,UART4_RX,SPI2_CE0,DPI_D9,SPI0 Controller Input / PWM0_1
40-Pin Header,22,GPIO25,GPIO25,I/O,GPIO,SD1_DAT1,ARM_TRACED1,DPI_D21,SPI0_CE0,PWM1_3,General Purpose I/O
40-Pin Header,23,GPIO11,GPIO11,I/O,SPI0_SCLK,SPI0_SCLK,I2C4_SDA,UART4_RTS,SPI2_CE1,DPI_D11,SPI0 Clock / PWM0_3
40-Pin Header,24,GPIO8,GPIO8,I/O,SPI0_CE0_N,SPI0_CE0_N,I2C4_SCL,UART4_CTS,SPI2_MISO,DPI_D8,SPI0 Chip Select 0 / PWM0_0
40-Pin Header,25,GND,N/A,Ground,0V Ground,N/A,N/A,N/A,N/A,N/A,System Ground
40-Pin Header,26,GPIO7,GPIO7,I/O,SPI0_CE1_N,SPI0_CE1_N,SPI3_MISO,UART3_RX,I2C3_SCL,DPI_D7,SPI0 Chip Select 1 / PWM0_3
40-Pin Header,27,GPIO0,GPIO0,I/O,ID_SD,I2C0_SDA,I2C1_SDA,UART2_TX,SPI3_CE1,DPI_D0,Reserved for HAT ID EEPROM auto-configuration
40-Pin Header,28,GPIO1,GPIO1,I/O,ID_SC,I2C0_SCL,I2C1_SCL,UART2_RX,SPI3_MOSI,DPI_D1,Reserved for HAT ID EEPROM auto-configuration
40-Pin Header,29,GPIO5,GPIO5,I/O,GPCLK1,GPCLK1,SPI3_MOSI,UART3_CTS,I2C4_SDA,DPI_D5,General Purpose Clock 1 / PWM0_1
40-Pin Header,30,GND,N/A,Ground,0V Ground,N/A,N/A,N/A,N/A,N/A,System Ground
40-Pin Header,31,GPIO6,GPIO6,I/O,GPCLK2,GPCLK2,SPI3_SCLK,UART3_RTS,I2C4_SCL,DPI_D6,General Purpose Clock 2 / PWM0_2
40-Pin Header,32,GPIO12,GPIO12,I/O,PWM0_0,PWM0_0,I2C5_SDA,UART5_TX,SPI4_MOSI,DPI_D12,Hardware PWM0 Channel 0 / PWM1_0
40-Pin Header,33,GPIO13,GPIO13,I/O,PWM0_1,PWM0_1,I2C5_SCL,UART5_RX,SPI4_MISO,DPI_D13,Hardware PWM0 Channel 1 / PWM1_1
40-Pin Header,34,GND,N/A,Ground,0V Ground,N/A,N/A,N/A,N/A,N/A,System Ground
40-Pin Header,35,GPIO19,GPIO19,I/O,PCM_FS / PWM1,PCM_FS,SPI4_MOSI,UART1_TX,I2C6_SDA,DPI_D19,Hardware PWM1_1 Output / I2S Frame Sync
40-Pin Header,36,GPIO16,GPIO16,I/O,GPIO,SPI4_MISO,UART1_RX,I2C5_SCL,DPI_D16,PWM0_0,General Purpose I/O
40-Pin Header,37,GPIO26,GPIO26,I/O,GPIO,SPI5_CE0,UART2_TX,I2C6_SDA,DPI_D22,PWM1_2,General Purpose I/O
40-Pin Header,38,GPIO20,GPIO20,I/O,PCM_DIN,PCM_DIN,SPI4_MOSI,UART2_RX,I2C6_SDA,DPI_D20,I2S Audio Data Input / GPCLK0
40-Pin Header,39,GND,N/A,Ground,0V Ground,N/A,N/A,N/A,N/A,N/A,System Ground
40-Pin Header,40,GPIO21,GPIO21,I/O,PCM_DOUT,PCM_DOUT,SPI4_SCLK,UART2_TX,I2C6_SCL,DPI_D21,I2S Audio Data Output / GPCLK1
Onboard Button,POWER_BTN,PWR_BTN,N/A,Input,Power Switch,N/A,N/A,N/A,N/A,N/A,Onboard tactile power switch for power state, wake, and soft shutdown
Debug Header (J2),1,DEBUG_TX,N/A,Output,UART_TX,N/A,N/A,N/A,N/A,N/A,3-pin dedicated UART debug port Transmit signal
Debug Header (J2),2,DEBUG_GND,N/A,Ground,0V Ground,N/A,N/A,N/A,N/A,N/A,3-pin dedicated UART debug port Ground
Debug Header (J2),3,DEBUG_RX,N/A,Input,UART_RX,N/A,N/A,N/A,N/A,N/A,3-pin dedicated UART debug port Receive signal
RTC Connector (J5),1,RTC_BAT+,N/A,Power,RTC_BAT,N/A,N/A,N/A,N/A,N/A,Dedicated 2-pin Real-Time Clock battery power input
RTC Connector (J5),2,RTC_GND,N/A,Ground,0V Ground,N/A,N/A,N/A,N/A,N/A,Dedicated Real-Time Clock battery Ground
Fan Connector (J1),1,FAN_GND,N/A,Ground,0V Ground,N/A,N/A,N/A,N/A,N/A,4-pin PWM active cooling fan Ground
Fan Connector (J1),2,FAN_5V,N/A,Power,5V DC Power,N/A,N/A,N/A,N/A,N/A,4-pin PWM active cooling fan 5V supply
Fan Connector (J1),3,FAN_TACH,N/A,Input,Fan Tachometer,N/A,N/A,N/A,N/A,N/A,Speed feedback tachometer signal from fan
Fan Connector (J1),4,FAN_PWM,N/A,Output,Fan PWM Control,N/A,N/A,N/A,N/A,N/A,Hardware PWM control signal for dynamic fan speed adjustment
"""

def parse_pinout_database(csv_text):
    database = {}
    lines = csv_text.strip().split('\n')
    for line in lines:
        line = line.strip()
        if not line or line.startswith("Header_or_Connector"):
            continue
        reader = csv.reader([line])
        try:
            row = next(reader)
        except Exception:
            continue
        if len(row) < 12:
            continue
            
        header_conn = row[0].strip()
        phys_pin = row[1].strip()
        signal_name = row[2].strip()
        gpio_num_str = row[3].strip()
        pin_type = row[4].strip()
        primary_func = row[5].strip()
        alt0 = row[6].strip()
        alt1 = row[7].strip()
        alt2 = row[8].strip()
        alt3 = row[9].strip()
        alt4 = row[10].strip()
        notes = row[11].strip()

        try:
            gpio_num = int(gpio_num_str.replace("GPIO", ""))
        except ValueError:
            gpio_num = None

        pin_info = {
            "Header": header_conn,
            "PhysicalPin": phys_pin,
            "Signal": signal_name,
            "GPIO": gpio_num,
            "Type": pin_type,
            "PrimaryFunc": primary_func,
            "ALT0": alt0,
            "ALT1": alt1,
            "ALT2": alt2,
            "ALT3": alt3,
            "ALT4": alt4,
            "Notes": notes
        }
        
        if header_conn == "40-Pin Header":
            database[f"PIN_{phys_pin}"] = pin_info
            database[phys_pin] = pin_info
        
        if gpio_num is not None:
            database[f"GPIO{gpio_num}"] = pin_info
            database[str(gpio_num)] = pin_info
            
        database[signal_name] = pin_info
        
    return database

PIN_DB = parse_pinout_database(RAW_PINOUT_CSV)

@dataclass(frozen=True)
class PinRequest:
    """One exclusive physical-wire assignment in the Pi pinout CSP."""

    name: str
    candidates: Tuple[int, ...]
    function: str
    description: str


class PinConstraintError(RuntimeError):
    """Raised when the requested hardware cannot be assigned uniquely."""


class PinoutSolver:
    """Small exact CSP solver for exclusive GPIO assignments.

    Each request is a node in a bipartite graph (signal -> allowed GPIOs).  The
    backtracking search uses the minimum-remaining-values heuristic and retains
    the lowest-cost complete matching, where earlier candidates are preferred.
    Shared-bus wires are represented once, so they are intentionally shared by
    devices without making two logical assignments to one physical pin.
    """

    def __init__(self, valid_gpios: Iterable[int], reserved_gpios: Iterable[int]):
        self.valid_gpios = set(valid_gpios)
        self.reserved_gpios = set(reserved_gpios)
        self._owners: Dict[int, str] = {}

    def reserve(self, request: PinRequest) -> None:
        if len(request.candidates) != 1:
            raise ValueError(f"Fixed request {request.name} needs exactly one GPIO")
        gpio = request.candidates[0]
        if gpio not in self.valid_gpios:
            raise PinConstraintError(f"{request.name}: GPIO{gpio} is not on the 40-pin header")
        if gpio in self.reserved_gpios:
            raise PinConstraintError(f"{request.name}: GPIO{gpio} is reserved by the system")
        if gpio in self._owners:
            raise PinConstraintError(
                f"GPIO{gpio} collision: {request.name} conflicts with {self._owners[gpio]}"
            )
        self._owners[gpio] = request.name

    def solve(self, requests: Sequence[PinRequest]) -> Dict[str, int]:
        candidates: Dict[str, List[int]] = {}
        request_by_name = {request.name: request for request in requests}
        if len(request_by_name) != len(requests):
            raise PinConstraintError("Every pin-request name must be unique")

        for request in requests:
            options = [
                gpio for gpio in request.candidates
                if gpio in self.valid_gpios
                and gpio not in self.reserved_gpios
                and gpio not in self._owners
            ]
            if not options:
                raise PinConstraintError(
                    f"No available GPIO candidates remain for {request.name}: {request.candidates}"
                )
            candidates[request.name] = options

        best_assignment: Dict[str, int] | None = None
        best_cost: int | None = None

        def search(pending: Tuple[str, ...], chosen: Dict[str, int], cost: int) -> None:
            nonlocal best_assignment, best_cost
            if best_cost is not None and cost >= best_cost:
                return
            if not pending:
                best_assignment = dict(chosen)
                best_cost = cost
                return

            # Minimum remaining values: expose impossible / over-subscribed
            # configurations early instead of printing a misleading report.
            name = min(
                pending,
                key=lambda item: sum(
                    gpio not in self._owners and gpio not in chosen.values()
                    for gpio in candidates[item]
                ),
            )
            request = request_by_name[name]
            remaining = tuple(item for item in pending if item != name)

            for preference_cost, gpio in enumerate(candidates[name]):
                if gpio in self._owners or gpio in chosen.values():
                    continue
                chosen[name] = gpio
                search(remaining, chosen, cost + preference_cost)
                del chosen[name]

        search(tuple(request_by_name), {}, 0)
        if best_assignment is None:
            detail = ", ".join(f"{name}={options}" for name, options in candidates.items())
            raise PinConstraintError(f"No one-to-one GPIO assignment exists ({detail})")

        for name, gpio in best_assignment.items():
            self._owners[gpio] = name
        return best_assignment

    @property
    def owners(self) -> Dict[int, str]:
        return dict(self._owners)


HEADER_GPIOS = tuple(range(0, 28))


def request(name: str, gpio: int, function: str, description: str) -> PinRequest:
    """Declare a fixed assignment and route it through the collision checker."""
    return PinRequest(name, (gpio,), function, description)


def physical_pin(gpio: int) -> int:
    info = PIN_DB.get(f"GPIO{gpio}")
    if not info or info["PhysicalPin"].isdigit() is False:
        raise PinConstraintError(f"GPIO{gpio} has no 40-pin-header entry in the pin database")
    return int(info["PhysicalPin"])


def solve_pi_pinout(use_usb_cables: bool = True, enable_hardware_tof_uart: bool = False):
    """Return a collision-free assignment for this Pi 4/5 hardware topology.

    Hardware Connections:
      Dual USB Cable Mode (Recommended / Default):
        - ESP32-Main: Direct USB Cable to Pi 4 USB Port (/dev/ttyUSB0 or /dev/ttyACM0 @ 921,600 baud)
        - ESP32-ToF:  Direct USB Cable to Pi 4 USB Port (/dev/ttyUSB1 or /dev/ttyACM1 @ 921,600 baud)
        - Zero GPIO pins or flying wires consumed on the 40-pin header!

      Legacy GPIO UART Mode:
        1. Primary High-Speed UART (/dev/ttyAMA0): Duplex link to ESP32-Main MCU (Chassis Telemetry @ 921,600 baud)
        2. Handshake Interrupt: GPIO20 Input triggered by ESP32-Main batch-ready pulse
        3. Secondary UART2 (/dev/ttyAMA2): Hardware UART to ESP32-ToF Hub (@ 921,600 baud)
    """

    reserved = set()
    solver = PinoutSolver(HEADER_GPIOS, reserved)

    if use_usb_cables:
        # Dual USB cable mode: 0 header pins assigned to MCU comms
        return {}

    fixed_requests = [
        # ESP32-Main Telemetry Highway (Primary UART0)
        request("ESP32_MAIN_UART_TX", 14, "UART0_TX", "Duplex TX -> ESP32-Main RX (GPIO 44) @ 921,600 baud"),
        request("ESP32_MAIN_UART_RX", 15, "UART0_RX", "Duplex RX <- ESP32-Main TX (GPIO 43) @ 921,600 baud"),
        request("ESP32_MAIN_DRDY",    20, "GPIO20",   "Batch data-ready handshake pulse from ESP32-Main (GPIO 1)"),
    ]

    if enable_hardware_tof_uart:
        fixed_requests.extend([
            # ESP32-ToF Hub Telemetry Link (Secondary UART2 via dtoverlay=uart2)
            request("ESP32_TOF_UART_TX", 0, "UART2_TX", "Secondary TX -> ESP32-ToF RX (GPIO 44) @ 921,600 baud"),
            request("ESP32_TOF_UART_RX", 1, "UART2_RX", "Secondary RX <- ESP32-ToF TX (GPIO 43) @ 921,600 baud"),
        ])

    for fixed in fixed_requests:
        solver.reserve(fixed)

    requests_by_name = {item.name: item for item in fixed_requests}
    assignments = {
        item.name: item.candidates[0]
        for item in fixed_requests
    }

    # This is deliberately an assertion, not just a summary: regressions that
    # reintroduce a duplicate GPIO now fail in development and CI.
    if len(assignments) != len(set(assignments.values())):
        raise PinConstraintError("Internal error: duplicate GPIO survived the solver")

    return {
        name: (
            gpio,
            physical_pin(gpio),
            requests_by_name[name].function,
            requests_by_name[name].description,
        )
        for name, gpio in assignments.items()
    }


def print_report(use_usb_cables: bool = True) -> None:
    assignments = solve_pi_pinout(use_usb_cables=use_usb_cables, enable_hardware_tof_uart=not use_usb_cables)
    used = {gpio for gpio, _, _, _ in assignments.values()}
    free = sorted(set(HEADER_GPIOS) - used)

    print("\n" + "=" * 100)
    print("  RASPBERRY PI 4 / 5 PINOUT CSP - VERIFIED COLLISION-FREE ASSIGNMENT")
    mode_str = "DUAL USB CABLE ARCHITECTURE (ZERO HEADER PINS USED)" if use_usb_cables else "LEGACY GPIO UART WIRING"
    print(f"  Topology Mode: {mode_str}")
    print("=" * 100)
    if use_usb_cables:
        print("  [USB SERIAL ARCHITECTURE]")
        print("  * ESP32-Main : Connected via USB Cable -> /dev/ttyUSB0 or /dev/ttyACM0 (921,600 baud)")
        print("  * ESP32-ToF  : Connected via USB Cable -> /dev/ttyUSB1 or /dev/ttyACM1 (921,600 baud)")
        print("  * Header Pins: 0 jumper wires on 40-pin header. 100% plug-and-play.")
        print("  " + "-" * 96)
        print(f"  Assigned Header GPIOs: 0 pins")
        print(f"  Free Available Header GPIOs ({len(free)}/28 pins): {free}")
    else:
        print(f"  {'Signal':<21} | {'Physical pin':<14} | {'GPIO':<9} | {'Function':<12} | Description")
        print("  " + "-" * 96)
        for name, (gpio, pin, function, description) in sorted(assignments.items(), key=lambda item: item[1][1]):
            print(f"  {name:<21} | Pin {pin:<10} | GPIO {gpio:<4} | {function:<12} | {description}")
        print("  " + "-" * 96)
        print(f"  Assigned GPIOs: {len(used)} unique pins - verified collision-free")
        print(f"  Free Available GPIOs ({len(free)} pins): {free}")
    print("=" * 100 + "\n")


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser(description="Raspberry Pi 4 Pinout Solver")
    parser.add_argument("--gpio-uart", action="store_true", help="Show legacy GPIO header UART wiring instead of USB cables")
    args = parser.parse_args()
    print_report(use_usb_cables=not args.gpio_uart)


if __name__ == "__main__":
    main()
