"""
Python script to communicate with Victron VE.Direct devices (like BlueSolar/SmartSolar MPPTs)
using the HEX protocol over a UART (serial) connection.

This script is based *only* on the "BlueSolar-HEX-protocol.pdf" (Rev 18) document.

Features:
- Uses classes to structure the device communication.
- Uses the 'construct' library to build and parse binary data packets.
- Implements the HEX protocol frame formatting, parsing, and checksum validation.
- Provides a generic get_register() method that uses a dictionary of construct
  parsers to automatically decode register values.
- Provides a generic set_register() method to write values to the device.
"""

import serial
import time
import binascii
import sys
from construct import (
    Struct, Int8ul, Int16ul, Int16sl, Int32ul, GreedyBytes, CString, Construct,
    Switch, Pass, this, BitStruct, Adapter
)
from typing import Dict, Any, Union

class EnumAdapter(Adapter):
    """
    Generic Adapter to map between Integers and Strings (Enums).
    Decodes: Integer -> String (e.g., 3 -> "Bulk")
    Encodes: String -> Integer (e.g., "Bulk" -> 3)
    """
    def __init__(self, subcon, decode_map: Dict[int, str], encode_map: Dict[str, int], default_prefix: str = "Unknown"):
        super().__init__(subcon)
        self.decode_map = decode_map
        self.encode_map = encode_map
        self.default_prefix = default_prefix

    def _decode(self, obj, context, path):
        # Return the string if found, else "Unknown (int)"
        return self.decode_map.get(obj, f"{self.default_prefix} ({obj})")

    def _encode(self, obj, context, path):
        # If the user passed a raw int, pass it through (flexibility)
        if isinstance(obj, int):
            return obj
        # Look up the string to find the integer
        if obj in self.encode_map:
            return self.encode_map[obj]
        raise ValueError(f"Invalid value '{obj}'. Valid options: {list(self.encode_map.keys())}")

# Device States (Page 8, for 0x0201)
DEVICE_STATES: Dict[int, str] = {
    0: "Not charging (Off)",
    2: "Fault",
    3: "Bulk",
    4: "Absorption",
    5: "Float",
    6: "Storage",
    7: "Manual Equalise",
    245: "Wake-up",
    247: "Auto Equalise",
    250: "Blocked (Updating)",
    252: "External Control",
    255: "Unavailable",
}
DEVICE_STATES_REV: Dict[str, int] = {v: k for k, v in DEVICE_STATES.items()}

# Device States for Remote Control (Page 25, for 0x200C)
DEVICE_STATES_LINK: Dict[int, str] = {
    0: "Not charging (Off)",
    2: "Fault",
    3: "Bulk",
    4: "Absorption",
    5: "Float",
    6: "Storage",
    7: "Manual Equalise",
    11: "Power Supply",
    245: "Wake-up",
    246: "Repeated Absorption",
    247: "Auto Equalise",
    248: "Battery Safe",
    249: "Load Detect",
    252: "External Control",
    255: "Unavailable",
}
DEVICE_STATES_LINK_REV: Dict[str, int] = {v: k for k, v in DEVICE_STATES_LINK.items()}

# Load Output Control Modes (Page 15)
# Note: Bit 7 is a "timer active" flag, so we only parse the lower 4 bits.
LOAD_OUTPUT_MODES: Dict[int, str] = {
    0: "OFF",
    1: "AUTO (Batterylife)",
    2: "ALT1 (Off<1.1V, On>13.1V)",
    3: "ALT2 (Off<11.8V, On>14.0V)",
    4: "ON",
    5: "USER1 (User defined)",
    6: "USER2 (User defined)",
    7: "AES (Automatic Energy Selector)",
}
LOAD_OUTPUT_MODES_REV: Dict[str, int] = {v: k for k, v in LOAD_OUTPUT_MODES.items()}

# Network Mode Flags (Page 23, for 0x200E)
NETWORK_MODE_FLAGS: Dict[int, str] = {
    1: "Networked",
    2: "Slave mode",
    4: "External control mode",
    8: "BMS controlled",
    16: "Charge group master",
    32: "Charge instance master",
    64: "Standby",
    128: "Reserved"
}
NETWORK_MODE_FLAGS_REV: Dict[str, int] = {v: k for k, v in NETWORK_MODE_FLAGS.items()}

# Network Info Flags (Page 25, for 0x200D)
# Keys represent the DECIMAL VALUE of the bit (1, 2, 4, 8...)
NETWORK_INFO_FLAGS: Dict[int, str] = {
    1: "Unit is controlled by a BMS",
    2: "Unit voltage set-point is controlled remotely",
    4: "Unit operates as charge slave",
    8: "Unit operates as charge master",
    16: "Unit is using ICHARGE information",
    32: "Unit is using ISENSE information",
    64: "Unit is using TSENSE information",
    128: "Unit is using VSENSE information",
    256: "Unit is held in STANDBY while the network initialises"
}

# Error Payload Meanings (Page 4)
ERROR_PAYLOAD_VALUES: Dict[bytes, str] = {
    b'\xaa\xaa': "Frame Error (Invalid command or structure)",
    b'\x00': "Unable to enter bootloader",
    b'\x00\x00': "Unable to enter bootloader" 
}

# Charger Error Codes (Page 13)
ERROR_CODE_VALUES: Dict[int, str] = {
    0: "No error",
    2: "Battery voltage too high",
    3: "Battery temperature sensor issue",
    4: "Battery temperature sensor issue",
    5: "Battery temperature sensor issue",
    6: "Battery voltage sensor issue",
    7: "Battery voltage sensor issue",
    8: "Battery voltage sensor issue",
    14: "Battery temperature too low (charging not allowed)",
    17: "Charger internal temperature too high",
    18: "Charger excessive output current",
    19: "Charger current polarity reversed",
    20: "Charger bulk time expired",
    21: "Charger current sensor issue",
    22: "Charger internal temperature sensor issue",
    23: "Charger internal temperature sensor issue",
    26: "Charger terminals overheated",
    27: "Charger short-circuit",
    28: "Converter issue",
    29: "Battery over-charge protection",
    33: "Input voltage too high",
    34: "Input excessive current",
    38: "Input shutdown (due to excessive battery voltage)",
    39: "Input shutdown (current flowing while converter off)",
    66: "Incompatible device in the network",
    67: "BMS connection lost",
    68: "Network misconfigured",
    116: "Calibration data lost",
    117: "Incompatible firmware",
    119: "Settings data invalid / corrupted",
}

class VeDirectDev:
    """
    A class to interact with a Victron VE.Direct device via the HEX protocol.
    
    The protocol logic is based on "BlueSolar-HEX-protocol.pdf" (Rev 18).
    
    Key protocol details:
    - Frame: :[command][data...][checksum]\n
    - Data is sent as ASCII-HEX characters (e.g., 0xFF is sent as 'F', 'F').
    - Checksum: (sum of all *binary* bytes from command to checksum) % 256 == 0x55
    - Numbers (like register IDs) are Little Endian.
    """

    # Commands (Page 3)
    COMMANDS: Dict[str, int] = {
        'PING': 0x01,
        'APPVERSION': 0x03,
        'PRODUCTID': 0x04,
        'RESTART': 0x06,
        'GET': 0x07,
        'SET': 0x08,
    }

    # Responses (Page 4)
    RESPONSES: Dict[str, int] = {
        'PING_RSP': 0x05,
        'GET_RSP': 0x07,
        'SET_RSP': 0x08,
        'DONE': 0x01,
        'ERROR': 0x04,
        'ASYNC': 0x0A,
    }
    
    # Register definitions for READABLE registers ('Get' command)
    # 'id': The 16-bit register ID.
    # 'parser': The construct object used to parse the raw bytes.
    # 'scale': The multiplier to apply to the parsed value.
    REGISTERS: Dict[str, Dict[str, Any]] = {
        'SERIAL_NUMBER': {
            'id': 0x010A,
            'parser': CString("ascii"),
            'scale': 1
        },
        'MODEL_NAME': {
            'id': 0x010B,
            'parser': CString("ascii"),
            'scale': 1
        },
        'DEVICE_STATE': {
            'id': 0x0201,
            'parser': EnumAdapter(Int8ul, DEVICE_STATES, DEVICE_STATES_REV, "Unknown State"),
            'scale': 1
        },
        'DEVICE_STATE_LINK': {
            'id': 0x200C,
            'parser': EnumAdapter(Int8ul, DEVICE_STATES_LINK, DEVICE_STATES_LINK_REV, "Unknown State"),
            'scale': 1
        },
        'CHARGER_ERROR_CODE': {
            'id': 0xEDDA,
            'parser': EnumAdapter(Int8ul, ERROR_CODE_VALUES, {}, "Unknown Error"), # Read-only, empty encode map
            'scale': 1
        },
        'CHARGER_VOLTAGE': {
            'id': 0xEDD5,
            'parser': Int16sl,
            'scale': 0.01
        },
        'CHARGER_CURRENT': {
            'id': 0xEDD7,
            'parser': Int16sl,
            'scale': 0.1
        },
        'PANEL_VOLTAGE': {
            'id': 0xEDBB,
            'parser': Int16sl,
            'scale': 0.01
        },
        'PANEL_POWER': {
            'id': 0xEDBC,
            'parser': Int32ul,
            'scale': 0.01
        },
        'PANEL_CURRENT': {
            'id': 0xEDBD,
            'parser': Int16sl,
            'scale': 0.1
        },
        'YIELD_TODAY': {
            'id': 0xEDD3,
            'parser': Int16ul,
            'scale': 0.01
        },
        'MAXIMUM_POWER_TODAY': {
            'id': 0xEDD2,
            'parser': Int16ul,
            'scale': 1
        },
        'MAXIMUM_POWER_YESTERDAY': {
            'id': 0xEDD0,
            'parser': Int16ul,
            'scale': 1
        },
        'CHARGER_INTERNAL_TEMP': {
            'id': 0xEDDB,
            'parser': Int16sl,
            'scale': 0.01
        },
        'BATTERY_VOLTAGE_SETTING': {
            'id': 0xEDEF,
            'parser': Int8ul,
            'scale': 1
        },
        'LOAD_OUTPUT_CONTROL': {
            'id': 0xEDAB,
            'parser': EnumAdapter(Int8ul, LOAD_OUTPUT_MODES, LOAD_OUTPUT_MODES_REV, "Unknown Mode"),
            'scale': 1
        },
        'CHARGE_CURRENT_LIMIT': {
            'id': 0x2015,
            'parser': Int16ul,
            'scale': 0.1
        },
        'NETWORK_INFO': {
            'id': 0x200D,
            'parser': EnumAdapter(Int16ul, NETWORK_INFO_FLAGS, {}, "Unknown Info"),
            'scale': 1
        },
        'NETWORK_MODE': {
            'id': 0x200E,
            'parser': EnumAdapter(Int8ul, NETWORK_MODE_FLAGS, NETWORK_MODE_FLAGS_REV, "UnknownMode"),
            'scale': 1
        },
        'NETWORK_STATUS': {
            'id': 0x200F,
            'parser': Int8ul,
            "scale":1
        }
    }
    
    # Register definitions for WRITABLE registers ('Set' command)
    # (Structure is the same as REGISTERS)
    SET_REGISTERS: Dict[str, Dict[str, Any]] = {
        'BATTERY_VOLTAGE_SETTING': REGISTERS['BATTERY_VOLTAGE_SETTING'],
        'CHARGE_CURRENT_LIMIT': REGISTERS['CHARGE_CURRENT_LIMIT'],
        'LOAD_OUTPUT_CONTROL': REGISTERS['LOAD_OUTPUT_CONTROL'],
        'DEVICE_STATE_LINK': REGISTERS['DEVICE_STATE_LINK'],
        'NETWORK_MODE': REGISTERS['NETWORK_MODE']
    }

    
    GetRequestPayload: Construct = Struct(
        "id" / Int16ul,
        "flags" / Int8ul
    )

    PingResponsePayload: Construct = Struct(
        "version" / Int16ul
    )

    GetResponsePayload: Construct = Struct(
        "id" / Int16ul,
        "flags" / Int8ul,
        "value_raw" / GreedyBytes
    )
    
    ErrorResponsePayload: Construct = Struct(
        "error_code" / GreedyBytes
    )
    
    FullResponse: Construct = Struct(
        "response_cmd" / Int8ul,
        "payload" / Switch(
            this.response_cmd,
            {
                RESPONSES['PING_RSP']: PingResponsePayload,
                RESPONSES['GET_RSP']: GetResponsePayload,
                RESPONSES['SET_RSP']: GetResponsePayload,
                RESPONSES['ASYNC']: GetResponsePayload,
                RESPONSES['ERROR']: ErrorResponsePayload,
                RESPONSES['DONE']: Pass,
            },
            default=GreedyBytes
        )
    )

    
    def __init__(self, port: str, baudrate: int = 19200, timeout: int = 1, debug: bool = False) -> None:
        """
        Initializes the VeDirectDev.
        
        Args:
            port (str): The serial port name (e.g., '/dev/ttyUSB0' or 'COM3').
            baudrate (int): The UART baud rate. 19200 is standard for VE.Direct.
            timeout (int): Serial read timeout in seconds.
        """
        self.port: str = port
        self.ser: serial.Serial
        self.debug: bool = debug

        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout, parity="N")
            if not self.ser.is_open:
                raise serial.SerialException(f"Failed to open port {self.port}")
            print(f"Successfully connected to {self.port}")
        except serial.SerialException as e:
            print(f"Error connecting to serial port: {e}", file=sys.stderr)
            raise


    def disconnect(self) -> None:
        """Closes the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"Disconnected from {self.port}")

    def _calculate_checksum(self, data_bytes: bytes) -> int:
        """
        Calculates the required checksum byte.
        The sum of all binary bytes (cmd + payload + checksum) must equal 0x55.
        
        Args:
            data_bytes (bytes): The binary bytes of (command + payload).
            
        Returns:
            int: The checksum byte.
        """
        # (sum + checksum) % 256 = 0x55
        # checksum = (0x55 - (sum % 256)) % 256
        sum_bytes = sum(data_bytes) % 256
        return (0x55 - sum_bytes) % 256

    def _format_frame(self, command_byte: bytes, payload: bytes = b"") -> bytes:
        """
        Formats a command and payload into the full ASCII-HEX frame.
        
        Args:
            command_byte (bytes): A single byte for the command (e.g., b'\x07').
            payload (bytes): The binary payload.
            
        Returns:
            bytes: The complete, ready-to-send ASCII frame (e.g., b':70B010042\n').
        """
        data_to_check: bytes = command_byte + payload
        checksum: int = self._calculate_checksum(data_to_check)
        
        # Per Page 3/27, the command is a *single hex nibble*
        # e.g., Ping (0x01) -> '1'. Get (0x07) -> '7'.
        cmd_char: bytes = b"%1X" % command_byte[0]

        if payload:
            payload_hex = binascii.hexlify(payload).upper()
        else:
            payload_hex: bytes = b""
            
        # Checksum is a full byte (two hex chars)
        checksum_hex: bytes = b"%02X" % checksum
        
        # Add start and end markers
        return b':' + cmd_char + payload_hex + checksum_hex + b'\n'

    def _parse_frame(self, frame_ascii: bytes) -> bytes:
        """
        Parses a received ASCII-HEX frame, validates checksum, and returns binary payload.
        
        Args:
            frame_ascii (bytes): The raw frame read from serial (e.g., b':70B01960045\n').
            
        Returns:
            bytes: The binary data part (command + payload).
            
        Raises:
            ValueError: If the frame is invalid or checksum fails.
        """
        if not frame_ascii.startswith(b':') or not frame_ascii.endswith(b'\n'):
            raise ValueError("Invalid frame: missing start/end markers.")
            
        # Strip markers
        frame_hex: bytes = frame_ascii.strip(b':\n')
        
        # The command is one nibble (1 char), the rest is hex bytes (2 chars)
        # So the total length must be odd.
        if len(frame_hex) % 2 == 0:
            raise ValueError(f"Invalid frame: Not valid HEX. Even-length string. Got: {frame_hex.decode()}")

        try:
            # Pad the odd-length string with a leading '0' to make it even
            frame_hex_padded = b'0' + frame_hex
            
            binary_data: bytes = binascii.unhexlify(frame_hex_padded)
            
        except (binascii.Error, ValueError) as e:
            raise ValueError(f"Invalid frame: Not valid HEX. {e}")
            
        if len(binary_data) < 1: # Must have at least command
            raise ValueError("Invalid frame: too short.")
            
        data_part: bytes = binary_data[:-1] # Everything except the last byte
        checksum_part: int = binary_data[-1] # The last byte
        
        # (sum(data_part) + checksum_part) % 256 must == 0x55
        expected_sum: int = (sum(data_part) + checksum_part) % 256
        if expected_sum != 0x55:
            raise ValueError(f"Checksum mismatch! Expected 0x55, got 0x{expected_sum:02X}")
            
        return data_part # Return the binary (command + payload)

    def _send_and_receive(self, command_byte: bytes, payload: bytes = b"") -> Any:
        """
        Sends a formatted frame and waits for a specific response.
        Ignores asynchronous (0x0A) messages while waiting.
        Ignores also continuous streaming data from device (cannot turn it off)
        
        Args:
            command_byte (bytes): The single command byte (e.g., b'\x07').
            payload (bytes): The binary payload (if any).
            
        Returns:
            (Any): The fully parsed construct object from FullResponse.parse().
            
        Raises:
            Exception: On timeout or serial error.
        """
        if not self.ser or not self.ser.is_open:
            raise serial.SerialException("Serial port is not connected.")
            
        frame_to_send: bytes = self._format_frame(command_byte, payload)
        
        self.ser.reset_input_buffer()
        
        if self.debug: #debugging
            print(f"DEBUG TX: {frame_to_send.strip().decode('latin-1')}")
        self.ser.write(frame_to_send)
        self.ser.flush()
        
        time.sleep(0.05)

        max_debug_skip_lines = 21
        text_skip_count = 0
        while True:
            response_frame: bytes = self.ser.readline()
            if self.debug:  #debugging
                print(f"DEBUG RX: {response_frame.strip().decode('latin-1')}")

            if not response_frame:
                raise TimeoutError("No response from device (timeout).")
            
            start_index = response_frame.find(b':')

            if start_index == -1:
                if self.debug and text_skip_count < max_debug_skip_lines:
                    print(f"DEBUG SKIP TEXT: {response_frame.strip().decode('latin-1')}")
                    text_skip_count += 1
                elif self.debug and text_skip_count == max_debug_skip_lines:
                    print(f"DEBUG SKIP TEXT: (skipping {max_debug_skip_lines} lines TEXT...)")
                    text_skip_count += 1
                elif text_skip_count >= max_debug_skip_lines:
                    raise Exception(f"DEBUG SKIP TEXT exceeded {max_debug_skip_lines}")
                continue
            else:

                if start_index > 0:
                    text_junk = response_frame[:start_index].strip().decode('latin-1')
                    if self.debug:
                        print(f"DEBUG TEXT/HEX JUNK: '{text_junk}'")
                    response_frame = response_frame[start_index:]

                if not response_frame.endswith(b'\n'):
                    if self.debug:
                        print(f"DEBUG WARNING: HEX fragment missing newline: {response_frame.strip().decode('latin-1')}")

            try:
                parsed_binary: bytes = self._parse_frame(response_frame)
                
                parsed_response: Any = self.FullResponse.parse(parsed_binary)
                
                if parsed_response.response_cmd == self.RESPONSES['ASYNC']:
                    # print("DEBUG: Ignoring Async frame, listening again...")
                    continue 
                else:
                    return parsed_response
                    
            except (ValueError, serial.SerialException) as e:
                continue 

    def _get_register_raw(self, register_id: int) -> bytes:
        """
        Internal helper to perform a 'Get' request and return the raw value.
        
        Args:
            register_id (int): The 16-bit register ID.
            
        Returns:
            bytes: The raw `value` part of the response.
        """
        req_payload: bytes = self.GetRequestPayload.build({"id": register_id, "flags": 0})
        parsed_response: Any = self._send_and_receive(bytes([self.COMMANDS['GET']]), req_payload)

        if parsed_response.response_cmd != self.RESPONSES['GET_RSP']:
            if parsed_response.response_cmd == self.RESPONSES['ERROR']:
                raise Exception(f"Device returned an Error frame. Code: {parsed_response.payload.error_code}")
            raise Exception(f"Unexpected response command: 0x{parsed_response.response_cmd:02X}")
        
        parsed_get: Any = parsed_response.payload

        if parsed_get.id != register_id:
            raise Exception(f"Mismatched ID: Asked 0x{register_id:04X}, got 0x{parsed_get.id:04X}")
        if parsed_get.flags != 0:
             raise Exception(f"Register 0x{register_id:04X} Error Flag: 0x{parsed_get.flags:02X}")
        return parsed_get.value_raw
        
    
    #public set and get data methods
    def set_register(self, register_name: str, value: Any) -> bool:
        """
        Generic method to set a register's value.
        
        Args:
            register_name (str): The key from the SET_REGISTERS dictionary.
            value (Any): The value to set.
            
        Returns:
            bool: True on success.
            
        Raises:
            Exception: On write error (e.g., read-only, value out of range).
        """
        if register_name not in self.SET_REGISTERS:
            raise ValueError(f"Unknown writable register name: {register_name}")
        reg: Dict[str, Any] = self.SET_REGISTERS[register_name]
        
        if isinstance(reg['parser'], (EnumAdapter)):
                value_bytes = reg['parser'].build(value)
        elif isinstance(value, (int, float)):
            raw_int_value = int(value / reg['scale'])
            value_bytes = reg['parser'].build(raw_int_value)
        else:
                value_bytes = reg['parser'].build(value)

        id_bytes: bytes = Int16ul.build(reg['id'])
        flags_bytes: bytes = Int8ul.build(0)
        req_payload: bytes = id_bytes + flags_bytes + value_bytes

        parsed_response: Any = self._send_and_receive(bytes([self.COMMANDS['SET']]), req_payload)

        if parsed_response.response_cmd != self.RESPONSES['SET_RSP']:
            if parsed_response.response_cmd == self.RESPONSES['ERROR']:
                error_raw = parsed_response.payload.error_code
                error_msg = ERROR_PAYLOAD_VALUES.get(error_raw, f"Unknown (0x{error_raw.hex()})")
                raise Exception(f"Device returned an Error frame. Code: {error_msg}")
            raise Exception(f"Unexpected response: 0x{parsed_response.response_cmd:02X}")
            
        parsed_set_rsp: Any = parsed_response.payload
        if parsed_set_rsp.id != reg['id']:
            raise Exception(f"Mismatched ID in set response: 0x{parsed_set_rsp.id:04X}")
        if parsed_set_rsp.flags != 0:
            raise Exception(f"Set Register Error Flag: 0x{parsed_set_rsp.flags:02X}")
            
        return True

    def get_register(self, register_name: str) -> Any:
        """
        Generic method to get, parse, and scale a register by its name.
        
        Args:
            register_name (str): The key from the REGISTERS dictionary.
            
        Returns:
            (any): The parsed and scaled value.
        """
        if register_name not in self.REGISTERS:
            raise ValueError(f"Unknown register name: {register_name}")
        reg: Dict[str, Any] = self.REGISTERS[register_name]
        
        raw_value: bytes = self._get_register_raw(reg['id'])
        
        try:
            parsed_value: Any = reg['parser'].parse(raw_value)
        except Exception as e:
            raise Exception(f"Parse error for {register_name}: {e}")

        if isinstance(parsed_value, (int, float)):
            return parsed_value * reg['scale']
        return parsed_value

    def ping(self) -> int:
        """
        Sends a Ping (0x01) command to the device.
        
        Returns:
            int: The device firmware version (e.g., 0x0116 for v1.16).
        """
        parsed_response: Any = self._send_and_receive(bytes([self.COMMANDS['PING']]))
        
        if parsed_response.response_cmd != self.RESPONSES['PING_RSP']:
            raise Exception(f"Unexpected ping response: 0x{parsed_response.response_cmd:02X}")
            
        return parsed_response.payload.version
    

if __name__=="__main__":
    mppt = VeDirectDev("/dev//ttyAMA0")

    print(mppt.get_register('NETWORK_MODE'))
    #print(mppt.get_register('DEVICE_STATE'))
    