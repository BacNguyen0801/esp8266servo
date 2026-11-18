import argparse
import sys
import time

import serial

SERVO_SID = 0x31
SERVO_CHANNEL = 0x01
SERVO_CMD_HI = 0xF0
SERVO_CMD_LOW = 0x01
SERVO_VOLTAGE_CMD_HI = 0xC0
SERVO_VOLTAGE_CMD_LOW = 0x01
FRAME_LENGTH_BYTE = 0x05

DIRECTION_RIGHT = 0x00
DIRECTION_LEFT = 0x01
VOLTAGE_MIN_V = 3
VOLTAGE_MAX_V = 24


def normalize_port_name(port_name):
    value = str(port_name).strip()
    if not value:
        raise ValueError("COM port is required")
    value = value.upper()
    if value.startswith("COM"):
        return value
    if value.isdigit():
        return f"COM{value}"
    return value


def normalize_command_text(command_hex):
    if command_hex is None:
        raise ValueError("Command is required")

    cleaned = str(command_hex).strip()
    if cleaned.lower().startswith("udsreq(") and cleaned.endswith(")"):
        cleaned = cleaned[7:-1]

    cleaned = cleaned.replace(" ", "").replace("-", "").replace("_", "").lower()
    return cleaned


def parse_raw_command(command_hex):
    cleaned = normalize_command_text(command_hex)
    if len(cleaned) % 2 != 0:
        raise ValueError("Command must be a valid hex string")

    if len(cleaned) == 8:
        cleaned = cleaned + "00"

    if len(cleaned) not in (10, 12):
        raise ValueError("Command must be 10 or 12 hex chars (example: 3101f00100 or 053101f00100)")

    try:
        payload = bytes.fromhex(cleaned)
    except ValueError as exc:
        raise ValueError(f"Invalid hex command: {command_hex}") from exc

    if len(payload) == 6:
        if payload[0] != FRAME_LENGTH_BYTE:
            raise ValueError("Length-prefixed command must start with 05")
        frame = payload[1:]
        if len(frame) != 5:
            raise ValueError("Length-prefixed frame must resolve to 5 payload bytes")

        if frame[0] == 0x31 and frame[1] == 0x01 and frame[2] == 0xF0 and frame[3] == 0x01:
            if frame[4] not in (0x00, 0x01):
                raise ValueError("Direction value must be 00 or 01")
            return payload

        if frame[0] == 0x31 and frame[1] == 0x01 and frame[2] == 0xC0 and frame[3] == 0x01:
            if not (VOLTAGE_MIN_V <= frame[4] <= VOLTAGE_MAX_V):
                raise ValueError(f"Voltage target must be between {VOLTAGE_MIN_V} and {VOLTAGE_MAX_V} V")
            return payload

        raise ValueError("Command must start with 31 01 f0 01 or 31 01 c0 01 after the length byte")

    if len(payload) == 5:
        if payload[0] == 0x31 and payload[1] == 0x01 and payload[2] == 0xF0 and payload[3] == 0x01:
            if payload[4] not in (0x00, 0x01):
                raise ValueError("Direction value must be 00 or 01")
            return bytes([FRAME_LENGTH_BYTE]) + payload

        if payload[0] == 0x31 and payload[1] == 0x01 and payload[2] == 0xC0 and payload[3] == 0x01:
            if not (VOLTAGE_MIN_V <= payload[4] <= VOLTAGE_MAX_V):
                raise ValueError(f"Voltage target must be between {VOLTAGE_MIN_V} and {VOLTAGE_MAX_V} V")
            return bytes([FRAME_LENGTH_BYTE]) + payload

        raise ValueError("Command must start with 31 01 f0 01 or 31 01 c0 01")

    raise ValueError("Command must resolve to either 5 bytes (31 01 <F0/C0> 01 <value>) or 6 bytes (05 31 01 <F0/C0> 01 <value>)")

def has_uds_response(received):
    if b"UDSResp" in received:
        return True

    for index in range(max(0, len(received) - 3)):
        if (received[index] == 0x71 and
                received[index + 1] == SERVO_CHANNEL and
                received[index + 2] in (SERVO_CMD_HI, SERVO_VOLTAGE_CMD_HI) and
                received[index + 3] == SERVO_CMD_LOW):
            return True

    return False


def send_serial_bytes(port_name, payload):
    port_name = normalize_port_name(port_name)

    try:
        print(f"Opening {port_name}...")

        with serial.Serial(
            port=port_name,
            baudrate=921600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.02,
            write_timeout=0.5
        ) as ser:
            ser.reset_input_buffer()

            print(f"PORT OPENED: {port_name}")
            written = ser.write(payload)
            ser.flush()

            print(f"Bytes transmitted: {written}/{len(payload)}")

            deadline = time.monotonic() + 5.0
            received = bytearray()

            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    print("Timeout: UDS response not received within 1 second")
                    print(f"PORT CLOSED: {port_name}")
                    return False

                chunk = ser.read(min(64, 256))
                if chunk:
                    received.extend(chunk)

                    if has_uds_response(received):
                        print("UDS response received")
                        break

                    try:
                        text = chunk.decode("ascii").replace("\r", "").strip()
                    except UnicodeDecodeError:
                        text = ""

                    if text:
                        print(f"UART received: {text}")
                    else:
                        hex_text = " ".join(f"{byte:02X}" for byte in chunk)
                        print(f"UART received: HEX: {hex_text}")

        print(f"PORT CLOSED: {port_name}")
        return True

    except Exception as exc:
        print(f"Serial error: {type(exc).__name__}: {exc}")
        raise


def send_raw_command_via_serial(port_name, command_hex):
    payload = parse_raw_command(command_hex)
    return send_serial_bytes(port_name, payload)



try:
    from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
    from PyQt5.QtWidgets import QApplication, QComboBox, QHBoxLayout, QLabel, QPushButton, QSpinBox, QTextEdit, QVBoxLayout, QWidget
except ImportError:
    QSerialPort = None
    QSerialPortInfo = None
    QApplication = None
    QComboBox = None
    QHBoxLayout = None
    QLabel = None
    QPushButton = None
    QSpinBox = None
    QVBoxLayout = None
    QWidget = None
    QTextEdit = None


class ServoControlWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.setWindowTitle("ESP8266 Servo UART Control")
        self.resize(460, 420)
        self.setStyleSheet(
            """
            QWidget {
                background: #f5f7fb;
                color: #1f2937;
                font-family: "Segoe UI", sans-serif;
                font-size: 15px;
            }
            QLabel {
                color: #374151;
                font-weight: 700;
                font-size: 15px;
            }
            QComboBox, QSpinBox {
                background: #ffffff;
                border: 1px solid #dfe5ee;
                border-radius: 8px;
                padding: 10px 12px;
                min-height: 36px;
                color: #111827;
                font-size: 15px;
            }
            QComboBox::drop-down {
                border: none;
                width: 24px;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 6px solid transparent;
                border-right: 6px solid transparent;
                border-top: 7px solid #6b7280;
                margin-right: 8px;
            }
            QPushButton {
                background: #2563eb;
                color: #ffffff;
                border: none;
                border-radius: 8px;
                padding: 12px 18px;
                min-height: 40px;
                font-weight: 700;
                font-size: 15px;
                letter-spacing: 0.2px;
            }
            QPushButton:hover {
                background: #1d4ed8;
            }
            QPushButton:pressed {
                background: #1e40af;
            }
            QPushButton#connectToggleBtn {
                background: #10b981;
                min-width: 100px;
            }
            QPushButton#connectToggleBtn:hover {
                background: #059669;
            }
            QPushButton#leftBtn {
                background: #22c55e;
            }
            QPushButton#leftBtn:hover {
                background: #16a34a;
            }
            QPushButton#rightBtn {
                background: #f59e0b;
            }
            QPushButton#rightBtn:hover {
                background: #d97706;
            }
            QTextEdit {
                background: #111827;
                color: #d1fae5;
                border: 1px solid #374151;
                border-radius: 8px;
                padding: 8px;
                font-family: "Consolas", "Courier New", monospace;
                font-size: 13px;
            }
            QTextEdit[readOnly="true"] {
                background: #111827;
                color: #d1fae5;
            }
            """
        )

        self.port_combo = QComboBox()

        self.command_log = QTextEdit()
        self.command_log.setReadOnly(True)
        self.command_log.setPlaceholderText("UART command log")
        self.command_log.setMinimumHeight(150)

        self.connection_btn = QPushButton("Connect")
        self.connection_btn.setObjectName("connectToggleBtn")

        self.target_voltage_spin = QSpinBox()
        self.target_voltage_spin.setRange(VOLTAGE_MIN_V, VOLTAGE_MAX_V)
        self.target_voltage_spin.setValue(12)

        left_btn = QPushButton("Rotate Left")
        left_btn.setObjectName("leftBtn")
        right_btn = QPushButton("Rotate Right")
        right_btn.setObjectName("rightBtn")
        voltage_btn = QPushButton("SET VOLT")

        self.connection_btn.clicked.connect(self.toggle_connection)
        left_btn.clicked.connect(lambda: self.send_direction(DIRECTION_LEFT))
        right_btn.clicked.connect(lambda: self.send_direction(DIRECTION_RIGHT))
        voltage_btn.clicked.connect(self.send_voltage_target)

        port_row = QHBoxLayout()
        port_row.addWidget(QLabel("COM Port:"))
        port_row.addWidget(self.port_combo, 1)
        port_row.addWidget(self.connection_btn)

        voltage_row = QHBoxLayout()
        voltage_row.addWidget(QLabel("Target Voltage (V):"))
        voltage_row.addWidget(self.target_voltage_spin)
        voltage_row.addWidget(voltage_btn)

        button_row = QHBoxLayout()
        button_row.addWidget(left_btn)
        button_row.addWidget(right_btn)

        root_layout = QVBoxLayout(self)
        root_layout.addLayout(port_row)
        root_layout.addLayout(voltage_row)
        root_layout.addLayout(button_row)
        root_layout.addWidget(QLabel("Command Log:"))
        root_layout.addWidget(self.command_log, 1)

        self.refresh_ports()

    def append_log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.command_log.append(f"[{timestamp}] {message}")

    def format_received_bytes(self, bytes_data):
        hex_text = " ".join(f"{b:02X}" for b in bytes_data)
        try:
            ascii_text = bytes_data.decode("ascii").replace("\r", "").strip()
        except UnicodeDecodeError:
            ascii_text = ""

        if ascii_text:
            return f"{ascii_text}"
        return f"HEX: {hex_text}"

    def refresh_ports(self):
        ports = [port.portName() for port in QSerialPortInfo.availablePorts()]
        self.port_combo.clear()
        self.port_combo.addItems(ports)

        if ports:
            self.port_combo.setCurrentIndex(0)
        else:
            self.append_log("NO COM PORTS FOUND")

    def toggle_connection(self):
        if self.serial_port and self.serial_port.isOpen():
            self.disconnect_port()
        else:
            self.connect_port()

    def connect_port(self):
        port_name = self.port_combo.currentText()
        if not port_name:
            self.append_log("SELECT A COM PORT FIRST")
            return

        if self.serial_port is not None and self.serial_port.isOpen():
            self.serial_port.close()

        self.serial_port = QSerialPort()
        self.serial_port.setPortName(port_name)
        self.serial_port.setBaudRate(921600)
        self.serial_port.setDataBits(QSerialPort.Data8)
        self.serial_port.setParity(QSerialPort.NoParity)
        self.serial_port.setStopBits(QSerialPort.OneStop)
        self.serial_port.setFlowControl(QSerialPort.NoFlowControl)

        self.serial_port.readyRead.connect(self.handle_serial_data)

        if self.serial_port.open(QSerialPort.ReadWrite):
            self.connection_btn.setText("Disconnect")
            self.append_log(f"CONNECTED: {port_name}")
        else:
            self.connection_btn.setText("Connect")
            self.append_log(f"CONNECTION FAILED: {port_name}")
            self.serial_port = None

    def disconnect_port(self):
        if not self.serial_port or not self.serial_port.isOpen():
            self.connection_btn.setText("Connect")
            self.append_log("NO ACTIVE CONNECTION")
            return

        port_name = self.serial_port.portName()
        self.serial_port.close()
        self.connection_btn.setText("Connect")
        self.append_log(f"DISCONNECTED: {port_name}")

    def handle_serial_data(self):
        if not self.serial_port or not self.serial_port.isOpen():
            return

        data = self.serial_port.readAll()
        if data.size() == 0:
            return

        bytes_data = bytes(data)
        if not bytes_data:
            return

        receive_text = self.format_received_bytes(bytes_data)
        self.append_log(f"RECEIVED: {receive_text}")

    def send_direction(self, direction):
        if not self.serial_port or not self.serial_port.isOpen():
            self.append_log("CONNECT TO A COM PORT FIRST")
            return

        packet = bytes([FRAME_LENGTH_BYTE, SERVO_SID, SERVO_CHANNEL, SERVO_CMD_HI, SERVO_CMD_LOW, direction])
        written = self.serial_port.write(packet)
        self.serial_port.flush()

        if written >= 0:
            uart_hex = " ".join(f"{b:02X}" for b in packet)
            self.append_log(f"SENT: {uart_hex}")
        else:
            self.append_log("FAILED TO SEND COMMAND")

    def send_voltage_target(self):
        if not self.serial_port or not self.serial_port.isOpen():
            self.append_log("CONNECT TO A COM PORT FIRST")
            return

        target_v = self.target_voltage_spin.value()
        packet = bytes([FRAME_LENGTH_BYTE, SERVO_SID, SERVO_CHANNEL, SERVO_VOLTAGE_CMD_HI, SERVO_VOLTAGE_CMD_LOW, target_v])
        written = self.serial_port.write(packet)
        self.serial_port.flush()

        if written >= 0:
            uart_hex = " ".join(f"{b:02X}" for b in packet)
            self.append_log(
                f"SENT: {uart_hex} | target {target_v} V"
            )
        else:
            self.append_log("FAILED TO SEND VOLTAGE TARGET")

    def closeEvent(self, event):
        if self.serial_port and self.serial_port.isOpen():
            self.serial_port.close()
        event.accept()


def build_cli_parser():
    parser = argparse.ArgumentParser(description="ESP8266 servo UART controller")
    parser.add_argument("-COM", "--com", dest="com_port", help="COM port, for example 4 or COM4")
    parser.add_argument("-cmd", "--cmd", dest="command_hex", help="Raw UART command in hex")
    return parser


if __name__ == "__main__":
    cli_parser = build_cli_parser()
    cli_args, _ = cli_parser.parse_known_args()

    if cli_args.com_port or cli_args.command_hex:
        if not cli_args.com_port or not cli_args.command_hex:
            cli_parser.error("Both -COM and -cmd are required when using CLI mode")
        try:
            response_received = send_raw_command_via_serial(cli_args.com_port, cli_args.command_hex)
            sys.exit(0 if response_received else 1)
        except Exception as exc:
            print(f"Error: {exc}", file=sys.stderr)
            sys.exit(1)

    if QApplication is None or QWidget is None:
        print("GUI support is unavailable in this environment.", file=sys.stderr)
        sys.exit(1)

    app = QApplication(sys.argv)
    window = ServoControlWindow()
    window.show()
    sys.exit(app.exec_())
