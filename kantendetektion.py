import cv2
import numpy as np

def detect_edges(frame):
    # Bild in Graustufen konvertieren
    # cv2.cvtColor konvertiert das Farbbild in ein Graustufenbild, da die Kantendetektion in der Regel auf Graustufenbildern durchgeführt wird.
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Gaussian Blur anwenden, um Rauschen zu reduzieren
    # cv2.GaussianBlur wendet einen Gaußschen Weichzeichner auf das Bild an, um das Rauschen zu reduzieren und so die Kantendetektion zu verbessern.
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Canny-Kantendetektion anwenden
    # cv2.Canny führt die Canny-Kantendetektion durch. Die Schwellenwerte 50 und 150 bestimmen die Empfindlichkeit des Detektors.
    edges = cv2.Canny(blurred, 50, 150)
    
    # Hough-Transformation anwenden, um Linien zu detektieren
    # cv2.HoughLinesP verwendet die Wahrscheinlichkeits-Hough-Transformation, um Linien im Kantenbild zu finden.
    # Parameter:
    # - 1: Abstandauflösung in Pixeln.
    # - np.pi / 180: Winkelauflösung in Radiant.
    # - threshold=50: Mindestanzahl an Schnittpunkten, um eine Linie zu detektieren.
    # - minLineLength=50: Mindestlänge einer Linie in Pixeln.
    # - maxLineGap=10: Maximale Lücke zwischen Segmenten, um als eine Linie betrachtet zu werden.
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
    
    # Linien in das ursprüngliche Bild einzeichnen
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # Berechne den Winkel der Linie in Grad
            # np.arctan2 gibt den Winkel in Radiant zurück, wir konvertieren ihn zu Grad.
            angle = np.arctan2(y2 - y1, x2 - x1) * 180.0 / np.pi
            # Nur horizontale Linien (ungefähr 0 Grad oder 180 Grad)
            if -10 < angle < 10 or 170 < abs(angle) < 190:
                # cv2.line zeichnet eine Linie auf das Bild. (0, 255, 0) ist die Farbe der Linie (grün), und 2 ist die Dicke der Linie.
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    return frame, edges

def main():
    # Webcam starten
    # cv2.VideoCapture(0) öffnet die Webcam. Der Parameter 0 wählt die Standard-Webcam des Systems.
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Fehler: Konnte die Webcam nicht öffnen.")
        return
    
    while True:
        # Frame von der Webcam lesen
        ret, frame = cap.read()
        if not ret:
            print("Fehler: Konnte Frame nicht lesen.")
            break
        
        # Kanten und Linien detektieren
        result_frame, edge_frame = detect_edges(frame)
        
        # Ergebnis anzeigen
        # cv2.imshow zeigt das Bild in einem Fenster an. 'Detected Edges' ist der Titel des Fensters für das Kantenbild.
        cv2.imshow('Detected Edges', edge_frame)
        # 'Result' ist der Titel des Fensters für das Ergebnisbild mit den eingezeichneten Linien.
        cv2.imshow('Result', result_frame)
        
        # Mit 'q' beenden
        # cv2.waitKey wartet auf eine Tasteneingabe. Wenn 'q' gedrückt wird, wird die Schleife beendet.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Ressourcen freigeben
    # cap.release gibt die Webcam frei.
    cap.release()
    # cv2.destroyAllWindows schließt alle von OpenCV geöffneten Fenster.
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
