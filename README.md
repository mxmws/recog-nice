# 3D Erfassungsprogramm

## ------Allgemeine Informationen------


Da der gemittelte Scan schon mit Hilfe der Realsense SDK gebildet, wird für den Rest des Programmes nur noch die PCL benötigt. Daher findet die Entwicklung unter VS auf Windows statt. Verwendet wird eine fertig gebaute Point Cloud Library von der GfAI, die nur mit VS2015 kompatibel ist. Um das Programm auf dem Raspberry zum laufen zu bringen, haben wir ein Script geschrieben, das automatisch die cmake Datei erstellt und Änderungen an den Dateipfaden vornimmt. In der untenstehenden Anleitung ist genau beschrieben, wie Sie das Programm aus dem VS Projekt auf dem Raspberry Pi zum Laufen bringen.

Das Programm läuft sowohl auf Windows als auch auf dem Raspberry Pi.
Mit dem Raspberry muss zunächst die leere Fläche gescannt werden ('plain.ply'), auf dem das Objekt später stehen soll.
Dann muss die Fläche gekennzeichnet werden, in dem man beliebiege Objekte so platziert ('objects.ply') , dass die von Ihnen verdeckte Fläche, den gewünschten Bereich zum Scannen repräsentiert.
Dabei ist zu beachten, dass sich im toten Winkel hinter dem Objekt nur flacher Boden befinden darf.
Danach platziert man das gewünschte Objekt auf den Scanbereich ('halloDasIstUnserScan.ply'). Nun wird der Boden sowie der Hintergrund von diesem Scan entfernt, sodass nur noch das relevante Objekt in der
Punktwolke vorhanden ist.

Auf Windows ist das Scannen mit der RealSense nicht möglich, daher befinden sich dort schon Testdateien für den leeren Scan ('plain.ply'), den Scan zum markieren des 
Scanbereichs ('objects.pls') und das Objekt, welches zuletzt gescannt wird ('halloDasIstUnserScan.ply').


## ---------------Anleitung---------------

1. Paketmanager starten:
- Start > Preferences > Add / Remove Software

2. Updates installieren
- Options > Refresh Package Lists
- Options > Check For Updates
- Install updates unten rechts anklicken - Der Vorgang kann eine Weile dauern. Holen Sie sich einen Kaffee

3. PCL Bibliothek herunterladen
- nach `libpcl` suchen
- alle 22 packages zur installation auswählen
- Apply unten rechts anklicken - Der Vorgang kann eine Weile dauern. Holen Sie sich noch einen Kaffee

4. Pakete entpacken
- zum Pfad gehen: `cd /var/cache/apt/archives/`
- alle deb Dateien von der PCL entpacken: `sudo dpkg -i libpcl*.deb`

5. Programm kompilieren
- Falls noch nicht vorhanden - Repository clonen
- zum Repository navigieren und Script starten mit `script_cmake.sh`

6. Programm ausführen
- in den Build Ordner navigieren (standardmäßig: `cd /home/pi/librealsense/build/swe_project`)
- executable ausführen (`./PCD2ASC`)
- erzeugte PLYs erscheinen im selben Ordner

## ---------------Visual Studio Projekt---------------

1. Projekt öffnen und **nicht** auf v141 upgraden > Rechtsklick auf Projekt > Eigenschaften > General > Platform Toolset "Visual Studio  2015 (v140) auswählen"
- Toolset muss eventuell davor mit dem Installer installiert werden

2. Systemumgebungsvariablen unter Windows öffnen > Erweitert > Umgebungsvariablen... > Systemvariablen: Neu... anklicken 
- Name der Variablen: `PCL_ROOT`
- Wert der Variable: Pfad in dem die PCL gespeichert ist z.B.`C:\Users\Minh\Desktop\PCL\PCL 1.8.1`


Viel Spaß!
