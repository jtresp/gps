import serial
import pynmea2
import math
import time

class GPSModule:
    def __init__(self, port, baudrate):
        """
        Initialisiert das GPS-Modul.
        
        :param port: Serieller Port des GPS-Moduls (z.B. '/dev/ttyUSB0')
        :param baudrate: Baudrate der seriellen Kommunikation (z.B. 4800)
        """
        self.serial_port = serial.Serial(port, baudrate, timeout=1)

    def read_data(self):
        """
        Liest eine Zeile NMEA-Daten vom GPS-Modul.
        
        :return: NMEA-Datenzeile
        """
        line = self.serial_port.readline().decode('ascii', errors='replace')
        return line

    def get_coordinates(self):
        """
        Liest die Längen- und Breitengrade vom GPS-Modul.
        
        :return: Tuple aus (latitude, longitude)
        """
        while True:
            data = self.read_data()
            if data.startswith('$GPGGA'):
                msg = pynmea2.parse(data)
                latitude = msg.latitude
                longitude = msg.longitude
                return latitude, longitude

def geo_to_cartesian(lat, lon, lat0, lon0):
    """
    Wandelt geographische Koordinaten in kartesische Koordinaten um.
    
    :param lat: Breite in Dezimalgrad
    :param lon: Länge in Dezimalgrad
    :param lat0: Referenzbreite in Dezimalgrad
    :param lon0: Referenzlänge in Dezimalgrad
    :return: Tuple aus (x, y) in Metern
    """
    # WGS84 Konstanten
    R = 6378137  # Erdradius in Metern

    # Umwandlung von Grad in Radian
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)

    # Berechnung der kartesischen Koordinaten
    x = R * (lon_rad - lon0_rad) * math.cos((lat_rad + lat0_rad) / 2)
    y = R * (lat_rad - lat0_rad)
    
    return x, y

def track_points(gps, duration=1):
    """
    Erfasst die zurückgelegten Punkte innerhalb eines bestimmten Zeitraums.
    
    :param gps: GPSModule-Objekt
    :param duration: Zeitdauer für die Erfassung in Sekunden
    :return: Liste der erfassten Punkte [(latitude, longitude)]
    """
    start_time = time.time()
    points = []

    while time.time() - start_time < duration:
        latitude, longitude = gps.get_coordinates()
        points.append((latitude, longitude))
        # Warte, um nicht zu oft Daten zu erfassen (z.B. alle 0.5 Sekunden)
        time.sleep(0.5)

    return points

# Beispielkonfiguration
port = '/dev/ttyUSB0'  # Serieller Port des GPS-Moduls
baudrate = 4800  # Baudrate der seriellen Kommunikation

# Initialisiere das GPS-Modul
gps = GPSModule(port, baudrate)

# Referenzkoordinaten (z.B. der Startpunkt)
lat0, lon0 = 52.5200, 13.4050  # Beispielkoordinaten (Berlin)

# Erfasst die zurückgelegten Punkte innerhalb von 1 Sekunde
duration = 1  # Zeitdauer in Sekunden
points = track_points(gps, duration)

# Umwandeln der erfassten geographischen Koordinaten in kartesische Koordinaten und Ausgabe
for point in points:
    x, y = geo_to_cartesian(point[0], point[1], lat0, lon0)
    print(f"Geographische Koordinaten: (Latitude: {point[0]}, Longitude: {point[1]})")
    print(f"Kartesische Koordinaten: (X: {x} Meter, Y: {y} Meter)")
