# Einführung #

Die Installation besteht aus drei Schritten:
  * Herunterladen bzw aktualisieren der Dateien vom svn server
  * Installation des/der kinematic Konverter
  * Erzeugen eines oder mehrerer Konfigurations-Verzeichnisses

Bevor man damit beginnt, sollte man wissen welche Ubuntu Version und welche EMC Version installiert ist.

# Herunterladen bzw Aktualisieren der Dateien #

## Erstmalige Installation ##

Alle Dateien werden mit svn Kommandos heruntergeladen. Falls svn noch nicht installiert ist, mit apt-get oder dem graphischen Paketmanager installieren.

emc2 sollte ebenfalls bereits installiert sein. Dann sollte es im home Verzeichnis ein Verzeichnis "emc2" geben, in dem Konfigurationen, nc files etc angelegt werden, etwa so:
```
/home/gerd/emc2/configs     --- Verzeichnis für Konfigurationen
/home/gerd/emc2/nc-files    --- Verzeichnis für nc files (g-code)
```
"gerd" steht für den Namen des angemeldeten Linux users.

Dann gibt man in einem Terminal ein:
```
cd ~/emc2
svn checkout http://emc2hotwinch.googlecode.com/svn/trunk/ emc2hotwinch
cd emc2hotwinch
```
Hier sollte es jetzt die folgenden Verzeichnisse geben:
```
XYUV
XYAB
kinematic
kinematic_Ub_08_04-EMC_2_3
```

## Aktualisierung ##

svn hat sich in speziellen Verzeichnissen (.svn) den heruntergeladenen Status gemerkt. Ich empfehle an diesen Verzeichnissen NICHTS zu ändern.

Mit
```
cd ~/emc2/emc2hotwinch
svn update
```
kann man stets den aktuellen Stand herunterladen.


# Installation der kinematic Konverter #

Die Kinematic Konverter werden nur benötigt, wenn man die Seilwinden Version realisieren will. Bei "normalen" Linearachsen werden diese nicht benötigt.

## Installieren der kompilierten Module ##
Für die Kombinationnen Ubuntu 8.04 / EMC 2.3.x und Ubuntu 10.04 / EMC 2.4.x sind vorkompilierte Module vorhanden, die lediglich ins entsprchende Verzeichnis kopiert werden müssen.

Ubuntu 10.04:
```
cd ~/emc2/emc2hotwinch/kinematic
sudo cp *.ko /usr/realtime-2.6.32-122-rtai/modules/emc2/
```

Ubuntu 8.04:
```
cd ~/emc2/emc2hotwinch/kinematic_Ub_08_04-EMC_2_3
sudo cp *.ko /usr/realtime-2.6.24-16-rtai/modules/emc2/
```

## Kompilieren und installieren vom Quellcode ##

Dazu muss auch emc2-dev installiert sein.

# Konfiguratonsverzeichnis und Konfigurationsdateien #

## XY-UV oder XY-AB ##

Es gibt eine Variante die die Achsenpaare XY-UV verwendet und eine die XY-AB verwendet.
Die urpsrünglich Variante war die XY-UV, gemäß cnc [Achsenstandard](http://de.wikipedia.org/wiki/Computerized_Numerical_Control#Maschinenachsen).

Allerdings hat diese wegen einiger EMC2 Eigenheiten zwei Nachteile:
  * EMC2 erlaubt in der derzeitigen Version keine Lücken in der Achsdefinition. Das bedeuted, wenn wir Achse U verwenden wollen, müssen wir auch die Achsen XYZABC konfigurieren. Das ist optisch nicht so schön und ein bisschen lästig, gravierender ist noch:
  * Im sogenannten "world" mode bei einer nicht-trivialen Kinematic kann man die Achsen UVW nicht joggen !

PeterD hat deshalb eine Variante mit den Achsen XY-AB erstellt, die ich hiermit weiter pflegen will.

Die Konfigurationsverzeichnisse von EMC liegen normalerweise in ~/emc2/configs. Dorthin kopiert man zunächst die kompletten Verzeichnisse XYUV aund XYAB.

Mit den ini files in diesen Verzeichnissen kann emc2 jetzt prinzipiell gestartet werden.

Normalerweise müssen jedoch vorher einige Anpassungen vorgenommen werden.

Siehe dazu wiki Anpassungen.


