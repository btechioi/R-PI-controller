import spidev
import serial
import time
import threading

START_BYTE = 0xAA

# SPI Display (UI) - SPI bus 0 device 0
spi_display = spidev.SpiDev()
spi_display.open(0, 0)
spi_display.max_speed_hz = 1000000
spi_display.mode = 0b00

# SPI Pico IO subsystem - SPI bus 0 device 1
spi_pico = spidev.SpiDev()
spi_pico.open(0, 1)
spi_pico.max_speed_hz = 1000000
spi_pico.mode = 0b00

# UART wireless comm controller (ESP32-C3)
uart_com = serial.Serial('/dev/serial0', baudrate=115200, timeout=0.1)

# Global telemetry/state variables
controller_batt = 95
robot_batt = 80
connected = True
robot_temp = 37
robot_volt = 7.3
robot_rssi = -63
robot_rpm = 100
robot_mode = "Idle"

# IO Inputs (from Pico)
joystick_data = [0]*8  # 4 joysticks × (X,Y)
button_states = [False]*12  # 6 toggles + 6 buttons
encoder_positions = [0, 0]

spi_lock = threading.Lock()

def calc_checksum(data):
    cs = 0
    for b in data:
        cs ^= b
    return cs

def parse_int16(msb, lsb):
    val = (msb << 8) | lsb
    if val & 0x8000:
        val = -((~val & 0xFFFF) + 1)
    return val

# --- Pico IO Subsystem ---

def poll_pico_inputs():
    global joystick_data, encoder_positions, button_states

    request = [0xAA, 0x01, 0x18]  # Request packet: start, cmd, payload length=24
    cs = calc_checksum(request)
    request.append(cs)

    expected_response_len = 24  # Adjust if your Pico sends more or less

    with spi_lock:
        response = spi_pico.xfer2(request + [0x00] * expected_response_len)

    if len(response) < expected_response_len + 4:
        print("[PICO] Response too short:", response)
        return

    if response[0] != 0xAA or response[1] != 0x01:
        print("[PICO] Invalid start/command bytes")
        return

    length = response[2]
    if length != expected_response_len:
        print(f"[PICO] Unexpected length {length} != {expected_response_len}")
        return

    cs = response[3 + length + 1]
    calc_cs = 0
    for b in response[:3 + length + 1]:
        calc_cs ^= b
    if cs != calc_cs:
        print("[PICO] Checksum failed")
        return

    payload = response[3:3 + length]

    # Parse 4 joysticks (8 bytes = 4 × int16)
    joysticks = []
    for i in range(0, 8, 2):
        joysticks.append(parse_int16(payload[i], payload[i + 1]))

    # Parse 2 encoders (4 bytes = 2 × int16)
    encoders = []
    for i in range(8, 12, 2):
        encoders.append(parse_int16(payload[i], payload[i + 1]))

    # Toggles (6 bytes)
    toggles = payload[12:18]

    # Buttons (6 bytes)
    buttons = payload[18:24]

    joystick_data = joysticks
    encoder_positions = encoders
    button_states = [bool(x) for x in toggles + buttons]

    print(f"[PICO] Joysticks: {joystick_data}")
    print(f"[PICO] Encoders: {encoder_positions}")
    print(f"[PICO] Toggles+Buttons: {button_states}")

# --- Pack and send control data to wireless subsystem ---

def to_int16_bytes(val):
    val &= 0xFFFF
    return [(val >> 8) & 0xFF, val & 0xFF]

def pack_control_packet(joysticks, encoders, toggles, buttons):
    payload = []

    for val in joysticks:
        payload.extend(to_int16_bytes(val))

    for val in encoders:
        payload.extend(to_int16_bytes(val))

    toggles_mask = 0
    for i, t in enumerate(toggles):
        if t:
            toggles_mask |= (1 << i)
    buttons_mask = 0
    for i, b in enumerate(buttons):
        if b:
            buttons_mask |= (1 << i)

    payload.append(toggles_mask)
    payload.append(buttons_mask)

    packet = [0xAA, 0x04, len(payload)] + payload
    cs = calc_checksum(packet)
    packet.append(cs)

    return packet

def send_control_to_wireless():
    packet = pack_control_packet(joystick_data, encoder_positions,
                                 button_states[:6], button_states[6:])
    uart_com.write(bytearray(packet))
    print(f"[CONTROL] Sent control packet to wireless: {packet}")

# --- Parse telemetry from wireless subsystem ---

def parse_wireless_telemetry(packet):
    global controller_batt, robot_batt, connected
    global robot_temp, robot_volt, robot_rssi, robot_rpm, robot_mode

    if len(packet) < 10:
        return False
    if packet[0] != 0xAA or packet[1] != 0x03:
        return False
    length = packet[2]
    if length + 4 != len(packet):
        return False
    cs = 0
    for b in packet[:-1]:
        cs ^= b
    if cs != packet[-1]:
        print("[WIRELESS] Checksum fail")
        return False

    controller_batt = packet[3]
    robot_batt = packet[4]
    connected = bool(packet[5])
    robot_temp = packet[6]
    robot_volt = packet[7] / 10.0
    robot_rssi = packet[8] if packet[8] < 128 else packet[8] - 256
    robot_rpm = packet[9]

    mode_len = length - 7
    if mode_len > 0:
        robot_mode = packet[10:10 + mode_len].decode('ascii')
    else:
        robot_mode = "Unknown"

    print(f"[WIRELESS] Telemetry updated: Mode={robot_mode}")
    return True

def read_wireless_messages():
    buffer = bytearray()
    while uart_com.in_waiting:
        buffer += uart_com.read(uart_com.in_waiting)

    while len(buffer) >= 5:
        if buffer[0] != 0xAA:
            buffer.pop(0)
            continue

        if len(buffer) < 3:
            break

        length = buffer[2]
        packet_len = length + 4
        if len(buffer) < packet_len:
            break

        packet = buffer[:packet_len]
        if packet[1] == 0x03:
            parse_wireless_telemetry(packet)
        buffer = buffer[packet_len:]

# --- Display UI Subsystem ---

def send_telemetry_to_display():
    mode_bytes = robot_mode.encode('ascii')
    payload = [
        controller_batt,
        robot_batt,
        1 if connected else 0,
        robot_temp,
        int(robot_volt * 10),
        robot_rssi & 0xFF,
        robot_rpm
    ] + list(mode_bytes)

    packet = [START_BYTE, 0x02, len(payload)] + payload
    cs = calc_checksum(packet)
    packet.append(cs)

    with spi_lock:
        spi_display.xfer2(packet)

def receive_ui_events():
    read_len = 16
    with spi_lock:
        resp = spi_display.xfer2([0x00]*read_len)
    for i in range(len(resp)-3):
        if resp[i] == START_BYTE and resp[i+1] == 0x01:
            length = resp[i+2]
            if i + 3 + length >= len(resp):
                continue
            payload = resp[i+3:i+3+length]
            checksum = resp[i+3+length]
            calc_cs = 0
            for b in resp[i:i+3+length]:
                calc_cs ^= b
            if checksum == calc_cs:
                return payload
    return None

def process_ui_event(event):
    if not event:
        return
    event_id = event[0]
    if event_id == 0x10:
        print("[UI] Start button pressed")
        send_command_to_wireless(b'START')
    elif event_id == 0x20:
        print("[UI] Reset button pressed")
        send_command_to_wireless(b'RESET')
    elif event_id == 0x30:
        print("[UI] Stop button pressed")
        send_command_to_wireless(b'STOP')
    elif event_id == 0x40:
        print("[UI] Change Mode button pressed")
        toggle_robot_mode()
    elif event_id == 0x50:
        print("[UI] Ping button pressed")
        send_command_to_wireless(b'PING')

def toggle_robot_mode():
    global robot_mode
    modes = ["Idle", "Follow Line", "Obstacle Avoid", "Manual"]
    idx = modes.index(robot_mode) if robot_mode in modes else 0
    idx = (idx + 1) % len(modes)
    robot_mode = modes[idx]
    print(f"[MODE] Switched to {robot_mode}")

def send_command_to_wireless(cmd_bytes):
    packet = b'\xAA\x10' + bytes([len(cmd_bytes)]) + cmd_bytes
    cs = 0
    for b in packet:
        cs ^= b
    packet += bytes([cs])
    uart_com.write(packet)
    print(f"[WIRELESS] Sent command: {cmd_bytes}")

# --- Main loop threads ---

def display_loop():
    while True:
        send_telemetry_to_display()
        event = receive_ui_events()
        process_ui_event(event)
        time.sleep(0.05)

def pico_loop():
    while True:
        poll_pico_inputs()
        send_control_to_wireless()
        time.sleep(0.05)

def wireless_loop():
    while True:
        read_wireless_messages()
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        threads = []
        threads.append(threading.Thread(target=display_loop, daemon=True))
        threads.append(threading.Thread(target=pico_loop, daemon=True))
        threads.append(threading.Thread(target=wireless_loop, daemon=True))

        for t in threads:
            t.start()

        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting, closing SPI and UART...")
        spi_display.close()
        spi_pico.close()
        uart_com.close()
