import spidev
import time
import struct # For unpacking byte data into numbers

# --- SPI Configuration for Raspberry Pi Zero W (Master) ---
# Check your Pi's hardware SPI bus and device numbers.
# SPI0 on Pi typically:
# MOSI: GPIO10 (SPI0_MOSI)
# MISO: GPIO9 (SPI0_MISO)
# SCLK: GPIO11 (SPI0_SCLK)
# CE0: GPIO8 (SPI0_CE0) or CE1: GPIO7 (SPI0_CE1)
#
# IMPORTANT: The Pico code uses GP1 as its CS pin. You'll connect
# one of your Pi's available GPIOs (e.g., GPIO8 for CE0) to Pico's GP1.
# This script assumes you connect Pi's CE0 (GPIO8) to Pico's GP1.
SPI_BUS = 0     # SPI bus number (0 or 1 for Pi Zero W)
SPI_DEVICE = 0  # SPI device number (0 for CE0, 1 for CE1)
SPI_SPEED_HZ = 1000000 # 1 MHz, adjust as needed, ensure Pico can handle it
SPI_MODE = 0    # SPI mode (0 or 3), ensure it matches Pico (Pico is MODE0)

# --- Protocol Constants (MUST MATCH Pico Code) ---
START_BYTE = 0xAA
END_BYTE = 0x55
COMMAND_REQUEST_DATA = 0xA1

# Calculated Payload Length (Pico's buildPayload output)
# Joysticks (8 * 2 bytes) + Buttons (6 * 1 byte) + Switches (6 * 1 byte) +
# Encoders (2 * 2 bytes) + Accel (3 * 2 bytes)
PAYLOAD_LEN = 8 * 2 + 6 * 1 + 6 * 1 + 2 * 2 + 3 * 2 # 16 + 6 + 6 + 4 + 6 = 38 bytes

# Frame Structure: START_BYTE (1) + PAYLOAD (38) + CHECKSUM (1) + END_BYTE (1) + PADDING (1)
FRAME_LEN = 1 + PAYLOAD_LEN + 1 + 1 + 1 # 1 + 38 + 1 + 1 + 1 = 42 bytes

# --- Initialize SPI ---
try:
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED_HZ
    spi.mode = SPI_MODE
    print(f"SPI initialized: Bus {SPI_BUS}, Device {SPI_DEVICE}, Speed {SPI_SPEED_HZ} Hz, Mode {SPI_MODE}")
except FileNotFoundError:
    print("Error: /dev/spidev not found. Ensure SPI is enabled in raspi-config.")
    print("  Run 'sudo raspi-config', navigate to 'Interface Options' -> 'SPI' and enable it.")
    exit()
except Exception as e:
    print(f"Error initializing SPI: {e}")
    exit()

# --- Utility Functions ---

def calculate_checksum(data_bytes):
    """Calculates the XOR checksum of a bytearray."""
    checksum = 0
    for byte in data_bytes:
        checksum ^= byte
    return checksum

def request_and_receive_data():
    """
    Sends the request command (0xA1) and receives the full 42-byte frame from Pico.
    Returns the raw 42-byte response array.
    """
    # Create a list of bytes to send.
    # First byte is the command.
    # Subsequent bytes are dummy bytes (0x00) to clock out the Pico's response.
    tx_bytes = [COMMAND_REQUEST_DATA] + [0x00] * (FRAME_LEN - 1)
    
    try:
        # Perform the SPI transfer.
        # This sends tx_bytes and simultaneously receives rx_bytes.
        rx_bytes = spi.xfer2(tx_bytes)
        return bytes(rx_bytes) # Convert list of ints to bytes object
    except Exception as e:
        print(f"SPI transfer error: {e}")
        return None

def parse_pico_data_frame(frame_bytes):
    """
    Parses the 42-byte frame received from the Pico, validates it,
    and extracts sensor readings.
    Returns a dictionary of parsed sensor data or None if invalid.
    """
    if not frame_bytes or len(frame_bytes) != FRAME_LEN:
        print(f"Parse Error: Invalid frame length. Expected {FRAME_LEN}, got {len(frame_bytes)}")
        return None

    # 1. Check Start and End Bytes
    if frame_bytes[0] != START_BYTE:
        print(f"Parse Error: Invalid Start Byte. Expected 0x{START_BYTE:02X}, got 0x{frame_bytes[0]:02X}")
        return None
    if frame_bytes[FRAME_LEN - 2] != END_BYTE: # End byte is 2nd to last
        print(f"Parse Error: Invalid End Byte. Expected 0x{END_BYTE:02X}, got 0x{frame_bytes[FRAME_LEN - 2]:02X}")
        return None
    
    # 2. Extract Payload and Checksum
    payload_data = frame_bytes[1 : 1 + PAYLOAD_LEN] # Bytes 1 to 38 (inclusive)
    received_checksum = frame_bytes[1 + PAYLOAD_LEN] # Byte 39 (index 39)

    # 3. Verify Checksum
    calculated_checksum = calculate_checksum(payload_data)
    if calculated_checksum != received_checksum:
        print(f"Parse Error: Checksum mismatch! Calculated 0x{calculated_checksum:02X}, Received 0x{received_checksum:02X}")
        return None

    print(f"Frame valid! Checksum OK (0x{calculated_checksum:02X}). Parsing data...")

    # 4. Parse Payload (38 bytes)
    parsed_data = {}
    current_idx = 0

    # Joysticks (8 * uint16_t, each split into MSB, LSB) - 16 bytes
    parsed_data['joysticks'] = []
    for i in range(8):
        # struct.unpack('>H', ...) parses 2 bytes as a Big-endian Unsigned Short (uint16_t)
        val = struct.unpack('>H', payload_data[current_idx:current_idx+2])[0]
        parsed_data['joysticks'].append(val)
        current_idx += 2

    # Buttons (6 * uint8_t) - 6 bytes
    parsed_data['buttons'] = []
    for i in range(6):
        # Button value is 1 if not pressed, 0 if pressed
        is_pressed = not bool(payload_data[current_idx]) # Invert logic to be intuitive (True if pressed)
        parsed_data['buttons'].append(is_pressed)
        current_idx += 1

    # Switches (6 * uint8_t) - 6 bytes
    parsed_data['switches'] = []
    for i in range(6):
        # Switch value is 1 if open, 0 if closed
        is_closed = not bool(payload_data[current_idx]) # Invert logic to be intuitive (True if closed)
        parsed_data['switches'].append(is_closed)
        current_idx += 1

    # Encoders (2 * int16_t, each split into MSB, LSB) - 4 bytes
    parsed_data['encoders'] = []
    for i in range(2):
        # struct.unpack('>h', ...) parses 2 bytes as a Big-endian Signed Short (int16_t)
        val = struct.unpack('>h', payload_data[current_idx:current_idx+2])[0]
        parsed_data['encoders'].append(val)
        current_idx += 2
    
    # MPU6050 Accelerometer (3 * int16_t, each split into MSB, LSB) - 6 bytes
    parsed_data['accelerometer'] = {}
    parsed_data['accelerometer']['x'] = struct.unpack('>h', payload_data[current_idx:current_idx+2])[0]
    current_idx += 2
    parsed_data['accelerometer']['y'] = struct.unpack('>h', payload_data[current_idx:current_idx+2])[0]
    current_idx += 2
    parsed_data['accelerometer']['z'] = struct.unpack('>h', payload_data[current_idx:current_idx+2])[0]
    current_idx += 2

    # Verify all bytes were parsed
    if current_idx != PAYLOAD_LEN:
        print(f"Internal Parse Error: Mismatch in payload length. Parsed {current_idx}, Expected {PAYLOAD_LEN}")
        return None

    return parsed_data

# --- Main Loop Example ---
if __name__ == "__main__":
    print("Raspberry Pi Zero W: SPI Master for Pico I/O Subsystem")

    try:
        while True:
            # Request data from Pico
            print("\n--- Requesting data from Pico ---")
            received_frame = request_and_receive_data()

            if received_frame:
                # Parse and display the data
                parsed_sensor_data = parse_pico_data_frame(received_frame)

                if parsed_sensor_data:
                    print("\n--- Parsed Sensor Data ---")
                    for key, value in parsed_sensor_data.items():
                        if key == 'accelerometer':
                            print(f"  {key.capitalize()}: X={value['x']}, Y={value['y']}, Z={value['z']}")
                        else:
                            print(f"  {key.capitalize()}: {value}")
                else:
                    print("Failed to parse sensor data.")
            else:
                print("No frame received or SPI transfer error.")

            time.sleep(0.1) # Poll every 100ms. Adjust as needed.

    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        spi.close()
        print("SPI connection closed.")