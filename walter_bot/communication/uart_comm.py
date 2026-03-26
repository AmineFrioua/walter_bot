"""
UART Communication Handler for Raspberry Pi ↔ ESP32
Handles serial communication with the ESP32 slave
"""

import serial
import threading
import queue
import time


class UARTCommunication:
    """Manages UART communication with ESP32"""

    def __init__(self, port, baudrate):
        """
        Initialize UART communication

        Args:
            port (str): Serial port (e.g., '/dev/ttyAMA0')
            baudrate (int): Baud rate (e.g., 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.rx_queue = queue.Queue()
        self.tx_queue = queue.Queue()
        self.rx_thread = None
        self.tx_thread = None
        self.running = False

    def connect(self):
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            time.sleep(2)  # Wait for ESP32 to reset
            self.running = True

            # Start RX/TX threads
            self.rx_thread = threading.Thread(target=self._rx_handler, daemon=True)
            self.tx_thread = threading.Thread(target=self._tx_handler, daemon=True)
            self.rx_thread.start()
            self.tx_thread.start()

            print(f"✓ UART connected: {self.port} @ {self.baudrate} baud")
        except serial.SerialException as e:
            print(f"✗ UART connection failed: {e}")
            raise

    def send(self, data):
        self.tx_queue.put(data)

    def receive(self, timeout=1):
        """
        Receive data from queue

        Args:
            timeout (float): Timeout in seconds

        Returns:
            str: Received data or None
        """
        try:
            return self.rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _rx_handler(self):
        while self.running:
            try:
                if self.serial.in_waiting:
                    data = self.serial.readline().decode('utf-8').strip()
                    if data:
                        self.rx_queue.put(data)
            except Exception as e:
                print(f"RX error: {e}")
            time.sleep(0.01)

    def _tx_handler(self):
        while self.running:
            try:
                data = self.tx_queue.get(timeout=0.1)
                self.serial.write(data.encode('utf-8'))
            except queue.Empty:
                pass
            except Exception as e:
                print(f"TX error: {e}")

    def close(self):
        """Close serial connection"""
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
        print("✓ UART closed")
