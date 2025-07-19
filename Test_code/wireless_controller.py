# Python script for Raspberry Pi Zero W (Controller)
import serial
import time
import crcmod.predefined # pip install crcmod

# --- UART Configuration ---
SERIAL_PORT = '/dev/serial0' # Or '/dev/ttyUSB0' if using a USB-to-serial adapter
BAUD_RATE = 230400

# --- Protocol Constants (MUST MATCH ESP32-C3) ---
FRAME_START = 0x7E
FRAME_ESCAPE = 0x7D
FRAME_XOR = 0x20

# Command Opcodes from Pi to ESP32-C3
CMD_SET_RADIO_LORA      = 0x01
CMD_SET_RADIO_NRF24     = 0x02
CMD_SET_RADIO_ESPNOW    = 0x03
CMD_SEND_DATA           = 0x04
CMD_SET_NRF24_TX_ADDR   = 0x05
CMD_SET_NRF24_RX_ADDR   = 0x06
CMD_SET_ESPNOW_PEER_MAC = 0x07

# Response Opcodes from ESP32-C3 to Pi Zero W
RSP_ACK                 = 0xA0
RSP_NACK                = 0xA1
RSP_RECEIVED_DATA       = 0xB0

# CRC-16 XMODEM (0x1021 polynomial, 0x0000 initial value, no reverse, no XOR out)
crc16 = crcmod.predefined.mkCrcFun('crc-16-xmodem')

# --- Serial Port Setup ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) # Shorter timeout for non-blocking read
    print(f"Opened serial port {SERIAL_PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    print("Ensure the port is correct and you have permissions (e.g., 'sudo usermod -a -G dialout $USER')")
    exit()

# --- Functions to send data to ESP32-C3 ---

def encode_framed_data(payload):
    """Encodes a raw payload (bytes) into the framed protocol with CRC."""
    data_to_crc = bytearray(payload)
    
    # Calculate CRC on the raw payload
    crc = crc16(data_to_crc)
    
    # Append CRC (LSB then MSB)
    data_to_crc.append(crc & 0xFF)
    data_to_crc.append((crc >> 8) & 0xFF)

    # Apply byte stuffing
    framed_bytes = bytearray([FRAME_START])
    for b in data_to_crc:
        if b == FRAME_START or b == FRAME_ESCAPE:
            framed_bytes.append(FRAME_ESCAPE)
            framed_bytes.append(b ^ FRAME_XOR)
        else:
            framed_bytes.append(b)
    
    return framed_bytes

def send_command_to_esp32(command_opcode, data=b''):
    """Constructs and sends a command to the ESP32-C3."""
    payload = bytearray([command_opcode]) + bytearray(data)
    framed_msg = encode_framed_data(payload)
    
    print(f"Sending command 0x{command_opcode:02X} with data: {data.hex()}")
    ser.write(framed_msg)

# --- Functions to receive and decode data from ESP32-C3 ---

# Static variables for the state machine
_in_frame = False
_escape = False
_temp_buffer = bytearray()

def decode_framed_data():
    """Reads from serial and decodes a framed message, returns payload if complete and valid."""
    global _in_frame, _escape, _temp_buffer

    while ser.in_waiting > 0:
        b = ser.read(1)[0] # Read one byte

        if _in_frame:
            if b == FRAME_ESCAPE:
                _escape = True
            elif b == FRAME_START:
                # End of frame or unexpected start
                _in_frame = False
                
                if len(_temp_buffer) < 2: # Need at least 2 bytes for CRC
                    print("Decode: Frame too short for CRC. Discarding.")
                    _temp_buffer.clear()
                    return None
                
                # Extract CRC from the end
                received_crc_lsb = _temp_buffer[-2]
                received_crc_msb = _temp_buffer[-1]
                received_crc = (received_crc_msb << 8) | received_crc_lsb
                
                data_part = _temp_buffer[:-2] # Data excluding CRC
                calculated_crc = crc16(data_part)

                if calculated_crc == received_crc:
                    print(f"Decode: CRC OK. Payload len: {len(data_part)}, CRC: 0x{received_crc:04X}")
                    payload = bytes(data_part)
                    _temp_buffer.clear()
                    return payload # Return the valid payload
                else:
                    print(f"Decode: CRC MISMATCH! Expected 0x{calculated_crc:04X}, Got 0x{received_crc:04X}. Discarding.")
                    _temp_buffer.clear()
                    return None
            else:
                if _escape:
                    _temp_buffer.append(b ^ FRAME_XOR)
                    _escape = False
                else:
                    _temp_buffer.append(b)
                
                if len(_temp_buffer) >= 256: # Max frame size protection
                    print("Decode: Buffer overflow. Discarding frame.")
                    _in_frame = False
                    _temp_buffer.clear()
                    return None
        else:
            if b == FRAME_START:
                _in_frame = True
                _temp_buffer.clear()
                _escape = False
    return None # No complete frame yet

def process_received_payload(payload):
    """Processes a decoded payload received from the ESP32-C3."""
    if not payload:
        return

    response_opcode = payload[0]
    response_data = payload[1:]

    if response_opcode == RSP_ACK:
        original_command = response_data[0] if response_data else 0xFF
        print(f"ESP32-C3 ACK received for command 0x{original_command:02X}")
    elif response_opcode == RSP_NACK:
        original_command = response_data[0] if response_data else 0xFF
        print(f"ESP32-C3 NACK received for command 0x{original_command:02X}")
    elif response_opcode == RSP_RECEIVED_DATA:
        print(f"Received wireless data (via ESP32-C3): {response_data.decode('utf-8', errors='ignore')}")
    else:
        print(f"Unknown response opcode 0x{response_opcode:02X} with data: {response_data.hex()}")

# --- Main Loop for Controller ---
if __name__ == "__main__":
    print("Raspberry Pi Zero W Controller Started.")
    print("Waiting for serial data...")

    try:
        # Example communication flow:
        time.sleep(1) # Give ESP32 time to boot and for Serial to be ready

        # 1. Set the radio to LoRa
        print("\n--- Setting radio to LoRa ---")
        send_command_to_esp32(CMD_SET_RADIO_LORA)
        time.sleep(0.5) # Wait for response
        process_received_payload(decode_framed_data()) # Process potential ACK

        # 2. Send some data via LoRa
        print("\n--- Sending data via LoRa ---")
        message_to_send = b"Hello from Pi via LoRa!"
        send_command_to_esp32(CMD_SEND_DATA, message_to_send)
        time.sleep(0.5) # Wait for response
        process_received_payload(decode_framed_data()) # Process potential ACK

        # 3. Set the radio to NRF24
        print("\n--- Setting radio to NRF24 ---")
        send_command_to_esp32(CMD_SET_RADIO_NRF24)
        time.sleep(0.5)
        process_received_payload(decode_framed_data())

        # 4. Set NRF24 TX Address (example)
        print("\n--- Setting NRF24 TX Address ---")
        nrf_tx_addr = b'RPI01' # 5-byte address
        send_command_to_esp32(CMD_SET_NRF24_TX_ADDR, nrf_tx_addr)
        time.sleep(0.5)
        process_received_payload(decode_framed_data())

        # 5. Send some data via NRF24
        print("\n--- Sending data via NRF24 ---")
        message_to_send_nrf = b"Hello NRF24 from Pi!" # Max 32 bytes for NRF24
        send_command_to_esp32(CMD_SEND_DATA, message_to_send_nrf)
        time.sleep(0.5)
        process_received_payload(decode_framed_data())

        # 6. Set the radio to ESP-NOW
        print("\n--- Setting radio to ESP-NOW ---")
        send_command_to_esp32(CMD_SET_RADIO_ESPNOW)
        time.sleep(0.5)
        process_received_payload(decode_framed_data())

        # 7. Set ESP-NOW Peer MAC (MUST BE THE ACTUAL MAC OF YOUR ESP-NOW RECEIVER!)
        # Replace with your actual peer MAC
        print("\n--- Setting ESP-NOW Peer MAC ---")
        esp_now_peer_mac = bytes([0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC]) # Example MAC
        send_command_to_esp32(CMD_SET_ESPNOW_PEER_MAC, esp_now_peer_mac)
        time.sleep(0.5)
        process_received_payload(decode_framed_data())

        # 8. Send some data via ESP-NOW
        print("\n--- Sending data via ESP-NOW ---")
        message_to_send_espnow = b"Hello ESP-NOW from Pi!"
        send_command_to_esp32(CMD_SEND_DATA, message_to_send_espnow)
        time.sleep(0.5)
        process_received_payload(decode_framed_data())

        print("\n--- Entering continuous receive loop ---")
        while True:
            payload = decode_framed_data()
            if payload:
                process_received_payload(payload)
            time.sleep(0.01) # Small delay to prevent busy-waiting

    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed.")