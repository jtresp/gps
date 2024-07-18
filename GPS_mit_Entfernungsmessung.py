import serial
import time
import pynmea2
from math import radians, sin, cos, sqrt, atan2

# Konfiguration der seriellen Schnittstelle
ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)

def haversine(lat1, lon1, lat2, lon2):
    # Konvertiere von Grad zu Radiant
    R = 6371e3  # Erdradius in Metern
    phi1 = radians(lat1)
    phi2 = radians(lat2)
    delta_phi = radians(lat2 - lat1)
    delta_lambda = radians(lon2 - lon1)

    a = sin(delta_phi / 2) ** 2 + cos(phi1) * cos(phi2) * sin(delta_lambda / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance = R * c  # Abstand in Metern
    return distance

def parse_gps_data(data):
    try:
        msg = pynmea2.parse(data)
        if isinstance(msg, pynmea2.types.talker.GGA):
            return (msg.latitude, msg.longitude, msg.timestamp)
    except pynmea2.ParseError as e:
        print(f"Failed to parse GPS data: {e}")
    return None

def main():
    previous_point = None
    while True:
        # Lesen der GPS-Daten
        gps_data = ser.readline().decode('ascii', errors='replace').strip()
        current_point = parse_gps_data(gps_data)
        
        if current_point:
            if previous_point:
                lat1, lon1, _ = previous_point
                lat2, lon2, timestamp = current_point
                distance = haversine(lat1, lon1, lat2, lon2)
                print(f"Time: {timestamp}, Distance: {distance:.2f} meters")
            previous_point = current_point

        time.sleep(1)

if __name__ == "__main__":
    main()
