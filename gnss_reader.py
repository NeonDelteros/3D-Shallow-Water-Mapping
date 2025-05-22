# ===========================
# File: gnss_reader.py
# Description: Read and parse NMEA GNSS data from USB or log file
# Supports multiple NMEA sentence types: GGA, RMC, VTG, GSV, HDT
# ===========================

import serial
from serial.tools import list_ports
from dataclasses import dataclass
from datetime import datetime, date, time
from typing import List, Optional, Generator

# ---------- Data classes for each NMEA sentence type ----------

@dataclass
class GGAData:
    """Global Positioning System Fix Data"""
    time_utc: Optional[time]
    latitude: Optional[float]
    latitude_dir: Optional[str]
    longitude: Optional[float]
    longitude_dir: Optional[str]
    fix_quality: Optional[int]
    num_satellites: Optional[int]
    horizontal_dilution: Optional[float]
    altitude: Optional[float]
    altitude_units: Optional[str]
    geoid_separation: Optional[float]
    geoid_units: Optional[str]

@dataclass
class RMCData:
    """Recommended Minimum Navigation Information"""
    time_utc: Optional[time]
    status: Optional[str]
    latitude: Optional[float]
    latitude_dir: Optional[str]
    longitude: Optional[float]
    longitude_dir: Optional[str]
    speed_knots: Optional[float]
    track_angle: Optional[float]
    date_utc: Optional[date]
    mag_variation: Optional[float]
    mag_var_dir: Optional[str]

@dataclass
class VTGData:
    """Track Made Good and Ground Speed"""
    track_true: Optional[float]
    track_magnetic: Optional[float]
    speed_knots: Optional[float]
    speed_kmph: Optional[float]
    mode: Optional[str]

@dataclass
class SatelliteInfo:
    """Information for a single satellite (used in GSV)"""
    prn: Optional[int]
    elevation: Optional[int]
    azimuth: Optional[int]
    snr: Optional[int]

@dataclass
class GSVData:
    """Satellites in View"""
    total_messages: Optional[int]
    message_number: Optional[int]
    total_satellites: Optional[int]
    satellites: List[SatelliteInfo]

@dataclass
class HDTData:
    """Heading, True (from compass or gyro)"""
    heading: Optional[float]

# ---------- GNSSReader: Core class for reading and parsing NMEA ----------

class GNSSReader:
    """
    Reads and parses NMEA 0183 sentences from USB serial or log file.
    Supports GGA, RMC, VTG, GSV, HDT.
    Automatically detects USB GNSS devices.
    """
    def __init__(self, port: Optional[str] = None, baudrate: int = 9600, log_file: Optional[str] = None, checksum_required: bool = False):
        self.checksum_required = checksum_required
        self.ser = None
        self.log_file = log_file

        if log_file:
            # Open log file for playback mode
            self.file = open(log_file, 'r')
        else:
            # Auto-detect USB GNSS port if not provided
            if port is None:
                ports = self.detect_gnss_ports()
                if not ports:
                    raise IOError("No GNSS USB serial ports found")
                port = ports[0]
            # Open serial port for live reading
            self.ser = serial.Serial(port, baudrate=baudrate, timeout=1)

    @staticmethod
    def detect_gnss_ports() -> List[str]:
        """Scan available serial ports and return list of likely GNSS devices."""
        ports = []
        for info in list_ports.comports():
            desc = (info.description or "").lower()
            if any(keyword in desc for keyword in ['gps', 'gnss', 'u-blox', 'serial']):
                ports.append(info.device)
        # Fallback to ttyUSB/ttyACM if needed
        if not ports:
            for info in list_ports.comports():
                device = info.device
                if device.startswith('/dev/ttyUSB') or device.startswith('/dev/ttyACM'):
                    ports.append(device)
        return ports

    def read_sentences(self) -> Generator[object, None, None]:
        """Generator that reads, parses, and yields NMEA sentence objects."""
        while True:
            try:
                line = self.file.readline() if self.log_file else self.ser.readline().decode('ascii', errors='ignore')
                if not line:
                    break
                line = line.strip()
                if not line.startswith('$'):
                    continue
                parsed = self.parse_sentence(line)
                if parsed:
                    yield parsed
            except Exception:
                continue

    def parse_sentence(self, line: str):
        """Core dispatcher to handle different NMEA message types."""
        if '*' in line:
            body, cksum_str = line[1:].split('*', 1)
            calc_cksum = 0
            for ch in body:
                calc_cksum ^= ord(ch)
            if self.checksum_required:
                sent_cksum = int(cksum_str.strip(), 16)
                if calc_cksum != sent_cksum:
                    raise ValueError("Checksum does not match")
        else:
            body = line[1:]

        fields = body.split(',')
        msg_type = fields[0][2:]

        # Route to sentence-specific parser
        if msg_type == 'GGA':
            return self._parse_gga(fields)
        elif msg_type == 'RMC':
            return self._parse_rmc(fields)
        elif msg_type == 'VTG':
            return self._parse_vtg(fields)
        elif msg_type == 'GSV':
            return self._parse_gsv(fields)
        elif msg_type == 'HDT':
            return self._parse_hdt(fields)
        return None

    # ---------- Helper parsing methods ----------

    def _parse_time(self, timestr: str) -> Optional[time]:
        if not timestr:
            return None
        try:
            hh = int(timestr[0:2])
            mm = int(timestr[2:4])
            ss = float(timestr[4:])
            return time(hh, mm, int(ss), int((ss - int(ss)) * 1e6))
        except:
            return None

    def _parse_date(self, datestr: str) -> Optional[date]:
        if not datestr:
            return None
        try:
            day = int(datestr[0:2])
            month = int(datestr[2:4])
            year = int(datestr[4:6]) + 2000
            return date(year, month, day)
        except:
            return None

    def _parse_coord(self, value: str, direction: str) -> Optional[float]:
        if not value or not direction:
            return None
        try:
            deglen = len(value.split('.')[0]) - 2
            deg = float(value[:deglen])
            mins = float(value[deglen:])
            dec = deg + mins / 60.0
            if direction in ['S', 'W']:
                dec = -dec
            return dec
        except:
            return None

    # ---------- Sentence parsers ----------

    def _parse_gga(self, fields):
        """Parse GGA sentence and return GGAData"""
        return GGAData(
            time_utc=self._parse_time(fields[1]),
            latitude=self._parse_coord(fields[2], fields[3]),
            latitude_dir=fields[3],
            longitude=self._parse_coord(fields[4], fields[5]),
            longitude_dir=fields[5],
            fix_quality=int(fields[6]) if fields[6] else None,
            num_satellites=int(fields[7]) if fields[7] else None,
            horizontal_dilution=float(fields[8]) if fields[8] else None,
            altitude=float(fields[9]) if fields[9] else None,
            altitude_units=fields[10] if len(fields) > 10 else None,
            geoid_separation=float(fields[11]) if len(fields) > 11 and fields[11] else None,
            geoid_units=fields[12] if len(fields) > 12 else None
        )

    def _parse_rmc(self, fields):
        """Parse RMC sentence and return RMCData"""
        return RMCData(
            time_utc=self._parse_time(fields[1]),
            status=fields[2],
            latitude=self._parse_coord(fields[3], fields[4]),
            latitude_dir=fields[4],
            longitude=self._parse_coord(fields[5], fields[6]),
            longitude_dir=fields[6],
            speed_knots=float(fields[7]) if fields[7] else None,
            track_angle=float(fields[8]) if fields[8] else None,
            date_utc=self._parse_date(fields[9]),
            mag_variation=float(fields[10]) if len(fields) > 10 and fields[10] else None,
            mag_var_dir=fields[11] if len(fields) > 11 else None
        )

    def _parse_vtg(self, fields):
        """Parse VTG sentence and return VTGData"""
        return VTGData(
            track_true=float(fields[1]) if fields[1] else None,
            track_magnetic=float(fields[3]) if fields[3] else None,
            speed_knots=float(fields[5]) if fields[5] else None,
            speed_kmph=float(fields[7]) if fields[7] else None,
            mode=fields[9] if len(fields) > 9 else None
        )

    def _parse_gsv(self, fields):
        """Parse GSV sentence and return GSVData"""
        sats = []
        i = 4
        while i + 3 < len(fields):
            sats.append(SatelliteInfo(
                prn=int(fields[i]) if fields[i] else None,
                elevation=int(fields[i+1]) if fields[i+1] else None,
                azimuth=int(fields[i+2]) if fields[i+2] else None,
                snr=int(fields[i+3]) if fields[i+3] else None
            ))
            i += 4
        return GSVData(
            total_messages=int(fields[1]) if fields[1] else None,
            message_number=int(fields[2]) if fields[2] else None,
            total_satellites=int(fields[3]) if fields[3] else None,
            satellites=sats
        )

    def _parse_hdt(self, fields):
        """Parse HDT sentence and return HDTData"""
        return HDTData(
            heading=float(fields[1]) if fields[1] else None
        )

# ------------------------------------------------------------------
# Optional Dual GNSS Support: Multi-port, Multi-threaded Integration
# ------------------------------------------------------------------
# Use GNSSReaderDual to read from multiple GNSS devices in parallel.

import threading
import queue

class GNSSReaderDual:
    """
    Reads and parses NMEA sentences from two or more GNSS USB devices concurrently.
    Parsed data is yielded as (port, message_object) tuples.
    """
    def __init__(self, ports: Optional[List[str]] = None, baudrate: int = 9600, checksum_required: bool = False):
        self.checksum_required = checksum_required
        self.baudrate = baudrate
        self.ports = ports if ports else self._detect_multiple_ports()
        self.queue = queue.Queue()
        self.threads = []

        # Start a reader thread for each detected GNSS port
        for port in self.ports:
            thread = threading.Thread(target=self._reader_loop, args=(port,), daemon=True)
            thread.start()
            self.threads.append(thread)

    def _detect_multiple_ports(self) -> List[str]:
        """Auto-detect multiple likely GNSS ports (USB only)."""
        ports = []
        for info in list_ports.comports():
            desc = (info.description or "").lower()
            if any(k in desc for k in ['gps', 'gnss', 'u-blox']):
                ports.append(info.device)
        if not ports:
            for info in list_ports.comports():
                if info.device.startswith('/dev/ttyUSB') or info.device.startswith('/dev/ttyACM'):
                    ports.append(info.device)
        return ports

    def _reader_loop(self, port: str):
        """Reads from one GNSS device and parses incoming sentences."""
        try:
            ser = serial.Serial(port, self.baudrate, timeout=1)
            while True:
                try:
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    if not line.startswith('$'):
                        continue
                    msg = GNSSReader().parse_sentence(line)
                    if msg:
                        self.queue.put((port, msg))
                except:
                    continue
        except Exception as e:
            print(f"[ERROR] Serial open failed on {port}: {e}")

    def read_sentences(self) -> Generator[tuple, None, None]:
        """Yields (port, message) tuples from the internal queue."""
        while True:
            try:
                yield self.queue.get(timeout=1)
            except queue.Empty:
                continue