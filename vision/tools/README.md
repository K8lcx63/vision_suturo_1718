### Erklärung

Das Tool nimmt sich eine Liste von PCD-Dateien (mit absolutem Pfad zur Datei auf dem Datenträger) und wandelt diese
in Normals- und Color-Histogramme um. Anschließend werden diese als CSV-Dateien mit suffix "_normals_histogram.csv"
oder "_colors_histogram.csv" im selben Ordner gespeichert.

### Bauen

Das Tool wird folgendermaßen gebaut (im Ordner "tools"):

> mkdir build
> cd build
> cmake ..
> make

### Ausführen

Das Tool wird folgendermaßen ausgeführt (im Ordner "build"):

> ./batch_processor /pfad/zur/PCD-Dateiliste.txt name_des_objekts

### Dateien

Es wird eine Liste mit absoluten Pfaden zu den PCD-Dateien benötigt. Eine Zeile reicht.
Beispiel für Inhalt von edeka_red_bowl.txt:
> /Pfad/zu/Datei/in/common_suturo1718/pcd_files/edeka_red_bowl/edeka_red_bowl_60_63.pcd

Alle Dateien in der Liste werden geladen und bearbeitet.