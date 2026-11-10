import argparse
import sys
import time

import serial

SERVO_SID = 0x31
SERVO_CHANNEL = 0x01
SERVO_CMD_HI = 0xF0
SERVO_CMD_LOW = 0x01
FRAME_LENGTH_BYTE = 0x05

DIRECTION_RIGHT = 0x00
DIRECTION_LEFT = 0x01


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


def parse_raw_command(command_hex):
    if command_hex is None:
        raise ValueError("Command is required")

    cleaned = str(command_hex).strip().replace(" ", "").replace("-", "").lower()
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
        if frame[0] != 0x31 or frame[1] != 0x01 or frame[2] != 0xF0 or frame[3] != 0x01:
            raise ValueError("Command must start with 31 01 f0 01 after the length byte")
        if frame[4] not in (0x00, 0x01):
            raise ValueError("Final byte must be 00 or 01")
        return payload

    if len(payload) == 5:
        if payload[0] != 0x31 or payload[1] != 0x01 or payload[2] != 0xF0 or payload[3] != 0x01:
            raise ValueError("Command must start with 31 01 f0 01")
        if payload[4] not in (0x00, 0x01):
            raise ValueError("Final byte must be 00 or 01")
        return bytes([FRAME_LENGTH_BYTE]) + payload

    raise ValueError("Command must resolve to either 5 bytes (31 01 f0 01 <00/01>) or 6 bytes (05 31 01 f0 01 <00/01>)")

def send_raw_command_via_serial(port_name, command_hex):
    port_name = normalize_port_name(port_name)
    payload = parse_raw_command(command_hex)

    try:
        print(f"Opening {port_name}...")

        with serial.Serial(
            port=port_name,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
            write_timeout=2
        ) as ser:
            print(f"PORT OPENED: {port_name}")
            written = ser.write(payload)
            ser.flush()

            print(f"Bytes transmitted: {written}/{len(payload)}")

            # Allow transmission to complete before closing
            time.sleep(0.1)

        print(f"PORT CLOSED: {port_name}")

    except Exception as exc:
        print(f"Serial error: {type(exc).__name__}: {exc}")
        raise



try:
    from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
    from PyQt5.QtWidgets import QApplication, QComboBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget
except ImportError:
    QSerialPort = None
    QSerialPortInfo = None
    QApplication = None
    QComboBox = None
    QHBoxLayout = None
    QLabel = None
    QPushButton = None
    QVBoxLayout = None
    QWidget = None


class ServoControlWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.setWindowTitle("ESP8266 Servo UART Control")
        self.resize(420, 180)
        self.setStyleSheet(
            """
            QWidget {
                background: #f3f4f6;
                color: #111827;
                font-family: "Segoe UI";
                font-size: 12px;
            }
            QPushButton, QComboBox, QLabel {
                font-size: 12px;
            }
            QLabel {
                color: #111827;
            }
            QComboBox {
                background: #ffffff;
                border: 1px solid #d1d5db;
                border-radius: 8px;
                padding: 7px 10px;
                min-height: 30px;
            }
            QComboBox::drop-down {
                border: none;
                width: 20px;
            }
            QPushButton {
                background: #2563eb;
                color: #ffffff;
                border: none;
                border-radius: 10px;
                padding: 8px 16px;
                min-height: 32px;
                font-weight: 600;
            }
            QPushButton:hover {
                background: #1d4ed8;
            }
            QPushButton#connectToggleBtn {
                background: #16a34a;
            }
            QPushButton#connectToggleBtn:hover {
                background: #15803d;
            }
            QPushButton#leftBtn {
                background: #10b981;
            }
            QPushButton#leftBtn:hover {
                background: #059669;
            }
            QPushButton#rightBtn {
                background: #f59e0b;
            }
            QPushButton#rightBtn:hover {
                background: #d97706;
            }
            """
        )

        self.port_combo = QComboBox()
        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("padding-top: 8px; font-weight: 600;")

        self.connection_btn = QPushButton("Connect")
        self.connection_btn.setObjectName("connectToggleBtn")

        left_btn = QPushButton("Rotate Left")
        left_btn.setObjectName("leftBtn")
        right_btn = QPushButton("Rotate Right")
        right_btn.setObjectName("rightBtn")

        self.connection_btn.clicked.connect(self.toggle_connection)
        left_btn.clicked.connect(lambda: self.send_direction(DIRECTION_LEFT))
        right_btn.clicked.connect(lambda: self.send_direction(DIRECTION_RIGHT))

        port_row = QHBoxLayout()
        port_row.addWidget(QLabel("COM Port:"))
        port_row.addWidget(self.port_combo, 1)
        port_row.addWidget(self.connection_btn)

        button_row = QHBoxLayout()
        button_row.addWidget(left_btn)
        button_row.addWidget(right_btn)

        root_layout = QVBoxLayout(self)
        root_layout.addLayout(port_row)
        root_layout.addLayout(button_row)
        root_layout.addWidget(self.status_label)

        self.refresh_ports()

    def refresh_ports(self):
        ports = [port.portName() for port in QSerialPortInfo.availablePorts()]
        self.port_combo.clear()
        self.port_combo.addItems(ports)

        if ports:
            self.port_combo.setCurrentIndex(0)
            self.status_label.setText("Ready")
        else:
            self.status_label.setText("No COM ports found")

    def toggle_connection(self):
        if self.serial_port and self.serial_port.isOpen():
            self.disconnect_port()
        else:
            self.connect_port()

    def connect_port(self):
        port_name = self.port_combo.currentText()
        if not port_name:
            self.status_label.setText("Select a COM port first")
            return

        if self.serial_port is not None and self.serial_port.isOpen():
            self.serial_port.close()

        self.serial_port = QSerialPort()
        self.serial_port.setPortName(port_name)
        self.serial_port.setBaudRate(QSerialPort.Baud115200)
        self.serial_port.setDataBits(QSerialPort.Data8)
        self.serial_port.setParity(QSerialPort.NoParity)
        self.serial_port.setStopBits(QSerialPort.OneStop)
        self.serial_port.setFlowControl(QSerialPort.NoFlowControl)

        if self.serial_port.open(QSerialPort.ReadWrite):
            self.connection_btn.setText("Disconnect")
            self.status_label.setText(f"Connected to {port_name}")
        else:
            self.connection_btn.setText("Connect")
            self.status_label.setText(f"Failed to connect to {port_name}")
            self.serial_port = None

    def disconnect_port(self):
        if not self.serial_port or not self.serial_port.isOpen():
            self.connection_btn.setText("Connect")
            self.status_label.setText("No active connection")
            return

        self.serial_port.close()
        self.connection_btn.setText("Connect")
        self.status_label.setText("Disconnected")

    def send_direction(self, direction):
        if not self.serial_port or not self.serial_port.isOpen():
            self.status_label.setText("Connect to a COM port first")
            return

        packet = bytes([FRAME_LENGTH_BYTE, SERVO_SID, SERVO_CHANNEL, SERVO_CMD_HI, SERVO_CMD_LOW, direction])
        written = self.serial_port.write(packet)
        self.serial_port.flush()

        if written >= 0:
            direction_name = "anti-clockwise" if direction == DIRECTION_LEFT else "clockwise"
            cmd_code = "01" if direction == DIRECTION_LEFT else "00"
            self.status_label.setText(
                f"Sent: 05 31 01 f0 01 {cmd_code} - rotate {direction_name}"
            )
        else:
            self.status_label.setText("Failed to send command")

    def closeEvent(self, event):
        if self.serial_port and self.serial_port.isOpen():
            self.serial_port.close()
        event.accept()


def build_cli_parser():
    parser = argparse.ArgumentParser(description="ESP8266 servo UART controller")
    parser.add_argument("-COM", "--com", dest="com_port", help="COM port, for example 4 or COM4")
    parser.add_argument("-cmd", "--cmd", dest="command_hex", help="Raw command in hex, for example 053101f00100 or 3101f00100")
    return parser


if __name__ == "__main__":
    parser = build_cli_parser()
    args, remaining = parser.parse_known_args()

    if args.com_port or args.command_hex:
        if not args.com_port or not args.command_hex:
            parser.error("Both -COM and -cmd are required when using CLI mode")
        try:
            send_raw_command_via_serial(args.com_port, args.command_hex)
            sys.exit(0)
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
