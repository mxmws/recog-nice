.--------      -------
.|_____  \     |      \
.      |  |    |  |\   \
.------   |    |  | |   |
.|_____   |    |  | |   |
.      |  |    |  |/    |
.------   |    |       /
.|_______/     |______/


----------Anleitung----------

1. Paketmanager starten:
- Start > Preferences > Add / Remove Software

2. Updates installieren
- Options > Refresh Package Lists
- Options > Check For Updates
- Install updates unten rechts anklicken - Der Vorgang kann eine Weile dauern. Holen Sie sich einen Kaffee

3. PCL Bibliothek herunterladen
- nach 'libpcl' suchen
- alle 22 packages zur installation auswählen
- Apply unten rechts anklicken - Der Vorgang kann eine Weile dauern. Holen Sie sich noch einen Kaffee

4. Packete entpacken
- zum Pfad gehen: 'cd /var/cache/apt/archives/'
- alle deb Dateien von der PCL entpacken: 'sudo dpkg -i libpcl*.deb'

5. Programm kompilieren
- Falls noch nicht vorhanden - Repository clonen
- zum Repository navigieren und Scrip starten mit 'sh vs_to_cmake.sh'
  (Sie können vorher das Script editieren, um den Zielpfad zu ändern)

6. Programm ausführen
- in den Build Ordner navigieren (standartmäßig: /home/pi/ply_reader_test/build)
- executable ausführen
- erzeugte PLY befindet sich standartmäßig in /home/pi/ply_reader_test

Viel Spaß!
