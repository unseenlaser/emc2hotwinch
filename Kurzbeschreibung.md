## Kurzbeschreibung ##

EMC2 ist eine leistungsfähige, semi-profesionelle, PC basierende cnc Software.
Als Betriebssystem wird Ubuntu Linux mit einer Echtzeiterweiterung verwendet.

Aufgrund seiner Flexibilität und Anpassbarkeit eigent sich EMC2 für zahlreiche Anwendungen. Darunter z.B. Roboter, Metall- und Holzbearbeitungsmaschinen, Drahterodierschneiden und so weiter.

Die hier veröffentlichte Arbeit ist eine Anpassung von EMC2 an eine eine Heissdraht-Styropor-Schneideanlage, speziell zur Herstellung von Teilen für den Flugmodellbau.

Die Schneideanlage ist nicht wie üblich mit Linearführungen und -Antrieben ausgestattet, sondern arbeitet mit Seilwinden. Dadurch wird die Mechanik besonders einfach. Im Grunde genommen braucht man eigentlich gar keine Mechanik. Dafür braucht man auf der Softwareseite einen "Kinematik-Konverter".

Was heisst das ?
Die Bewegung des Schneidedrahtes wird üblicherweise in einem rechtwinkeligen, kartesischen Koordinatsystem vorgegeben (X- Y-Koordinate). Diese Koordinaten müssen in Seillängen umgerechnet werden (forward transformation), bzw umgekehrt muss aus den Seillängen die Position in X und Y Koordinaten berechnet werden (inverse tranformation)