import spidev
import time

# Open SPI bus 0, device 0 (adjust if needed)
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000  # 1 MHz (tune as needed)
spi.mode = 0b00

START_BYTE = 0xAA

def calc_checksum(data):
    cs = 0
    for b in data:
        cs ^= b
    return cs

def send_telemetry(controller_batt, robot_batt, connected,
                   robot_temp, robot_volt, robot_rssi, robot_rpm, robot_mode_str):
    payload = [
        controller_batt,
        robot_batt,
        1 if connected else 0,
        robot_temp,
        int(robot_volt * 10),
        robot_rssi & 0xFF,   # signed to unsigned byte
        robot_rpm
    ]
    mode_bytes = robot_mode_str.encode('ascii')
    payload.extend(mode_bytes)

    packet = [START_BYTE, 0x02, len(payload)] + payload
    cs = calc_checksum(packet)
    packet.append(cs)

    spi.xfer2(packet)
    print(f"Sent telemetry: {packet}")

def receive_ui_events():
    # Try to read any data sent from ESP32 slave
    # We'll attempt a dummy read with zeros sent, SPI is full duplex
    read_len = 10  # max expected event packet length
    resp = spi.xfer2([0x00]*read_len)

    # Look for valid packets in response (simple scan)
    for i in range(len(resp)-3):
        if resp[i] == START_BYTE and resp[i+1] == 0x01:  # UI Event cmd
            length = resp[i+2]
            if i + 3 + length >= len(resp):
                continue
            payload = resp[i+3:i+3+length]
            checksum = resp[i+3+length]
            calc_cs = 0
            for b in resp[i:i+3+length]:
                calc_cs ^= b
            if checksum == calc_cs:
                print(f"UI Event received: {payload}")
                return payload  # You can extend to handle events
    return None

try:
    while True:
        # Example telemetry values
        send_telemetry(
            controller_batt=90,
            robot_batt=75,
            connected=True,
            robot_temp=36,
            robot_volt=7.4,
            robot_rssi=-60,
            robot_rpm=150,
            robot_mode_str="Follow Line"
        )

        time.sleep(0.1)  # 10 Hz update

        event = receive_ui_events()
        if event:
            # Process UI events here
            event_id = event[0]
            if event_id == 0x10:
                print("Start button pressed")
            elif event_id == 0x20:
                print("Reset button pressed")
            elif event_id == 0x30:
                print("Stop button pressed")
            elif event_id == 0x40:
                print("Change Mode button pressed")
            elif event_id == 0x50:
                print("Ping button pressed")

        time.sleep(0.1)

except KeyboardInterrupt:
    spi.close()
    print("SPI closed and program terminated")
