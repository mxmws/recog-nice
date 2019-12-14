# 3D Erfassungsprogramm

## ------Allgemeine Informationen------

Das Beispielprogramm, welches den gemittelten Scan erstellt, befindet sich im Ordner Examples. Es ist leider nur mit Hilfe der Realsense SDK ausführbar, die nicht so einfach zu installieren ist, wie die LibPCL. Wir können Ihnen die Funktionsfähigkeit aber gerne am Dienstag demonstrieren.

Da wir schon einen gemittelten Scan mit Hilfe der Realsense bilden können, brauchen wir für den Rest des Programmes nur noch die PCL. Daher findet die Entwicklung unter VS auf Windows statt. Wir verwenden eine fertig gebaute Point Cloud Library von der GfAI, die nur mit VS2015 kompatibel ist. Um das Programm auf dem Raspberry zum laufen zu bringen, haben wir ein Script geschrieben, das automatisch die cmake Datei erstellt und Änderungen an den Dateipfaden vornimmt. In der untenstehenden Anleitung ist genau beschrieben, wie Sie das Programm aus dem VS Projekt auf dem Raspberry Pi zum Laufen bringen.



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
- zum Repository navigieren und Script starten mit `sh vs_to_cmake.sh`
  (Sie können vorher das Script oben editieren, um den Zielpfad zu ändern)

6. Programm ausführen
- in den Build Ordner navigieren (standardmäßig: `cd /home/pi/ply_reader_test/build`)
- executable ausführen
- erzeugte PLY erscheint im selben Ordner

## ---------------Visual Studio Projekt---------------

1. Projekt öffnen und nicht auf v141 upgraden > Rechtsklick auf Projekt > Eigenschaften > General > Platform Toolset "Visual Studio  2015 (v140) auswählen"
- Toolset muss eventuell davor mit dem Installer installiert werden

2. Systemumgebungsvariablen unter Windows öffnen > Erweitert > Umgebungsvariablen... > Systemvariablen: Neu... anklicken 
- Name der Variablen: "PCL_ROOT"
- Wert der Variable: Pfad in dem die PCL gepsichert ist "C:\Users\Minh\Desktop\PCL\PCL 1.8.1"


Viel Spaß!
