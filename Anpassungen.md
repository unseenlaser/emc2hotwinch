# Allgemeine Anpassungen #

Die allgemeine Umgebung ist im ini wie folgt voreingestellt:
  * Axis als UI
  * nc files in ~/emc2/nc-files
  * kein joypad

mocca als UI ist in Vorbereitung, funktioniert aber noch nicht 100%

Wer ein joypad zur manuellen Steuerung verwenden möchte: siehe wiki Joypad.

# Parallel Port und Pinbelegung #

Am Parallelport werden Takt/Richtungssignale für 4 Schrittmotore ausgegeben.

Es sind zwei hal Files vorhanden, die die Pinbelegung definieren. Dadurch kann auf einfache Weise rechte und linke Seite der Schneide vertauscht werden. Wozu das gut ist: siehe wiki "Zuspitzung".

Mit den beiden Scripten re und li kann das auch zur Laufzeit (von emc/Axis) durchgeführt werden. Das ist natürlich nur sinnvoll, wenn die Anlage steht (dh die Motoren stehen)! Ausserdem sollte dabei die Position der X Achse gleich U(A) und Y gleich V(B) sein. Am besten per MDI Kommando in eine solche Position fahren, dann ausschalten. Erst dann eines der Scripte aufrufen. Wird das nicht beachtet, muss die Anlage neu referenziert werden!

Ansonsten ist die Pinzuordnung im Rahmen der Möglichkeiten des PPT frei wählbar. Es sollten aber immer beide pin files (rechts und links) konsistent gehalten werden. Falls ein Motor in die falsche Richtung dreht, kann der pin einfach mit der invert Funktion invertiert werden.

Zur "Reset" Funktion siehe unten "Timing".

# Geometrie #

## Allgemeines ##

Die Maschine hat eine XY Ebene und eine UV Ebene.
X bzw U Achse verlaufen waagrecht, Y bzw V senkrecht (positiv nach oben).
Zur XY Ebene gehören die Motoren Axis\_0 und Axis\_1, zur UV Ebene Axis\_6 und Axis\_7.
Der Motor Axis\_0 liegt in Richtung der negativen X Koordianten, Axis\_1 in Richtung positiver X Koordinaten. Entsprechend liegt Axis\_6 in Richtung negativer U Koordinaten und Axis\_7 in Richtung positiver U Koordinaten.

## Referenzpunkt und Nullpunkt ##

Diese beiden Punkte können, müssen aber nicht zusammenfallen.
Der Referenzpunkt wird durch das Kommando “Referenzfahrt” angefahren. Dabei wird nicht
wirklich gefahren, es werden lediglich die Koordinaten des Referenzpunkts eingestellt. Das
Verfahren muss vorher manuell (mit stromlosen Motoren) und/oder per jog im joint modus
erfolgen!

Die XY/UV Koordinaten des Referenzpunkts werden duch XU0 und YV0 definiert. Der
Referenzpunkt sollte möglichst in der Mitte zwischen beiden Motoren liegen und auf gleicher Höhe über der Arbeitsplatte. Den Nullpunkt kann man dann so verschieben, dass der Schneidebereich mit positiven X/V Koordinaten etwa in der Mitte zu liegen kommt.

## Kalibrierung ##

<img src='http://emc2hotwinch.googlecode.com/files/Geometrie.JPG' />

Für die Kalibrierung des Systems sind folgende Parameter entscheidend:
  * Abstand der Motoren in beiden Ebenen
  * Seillängen im Referenzpunkt
  * Anzahl der Schrittmotor-Schritte pro mm Seil

Abstand der Motoren ist eigentlich nicht ganz richtig. Wer es ganz genau nimmt, misst den Abstand der beiden Punkte, an denen das Seil die Trommel verlässt. Da bei mir die Seile “von aussen” aufgewickelt werden, addiert sich zum Abstand der Motoren noch “etwas weniger” als der Trommeldurchmesser. “Etwas weniger” deshalb, weil die Seile ja nicht esenkrecht nach unten laufen. Dieses “etwas weniger” ist allerdings von der Position des Schneidedrahts abhängig. Der kinematic konverter berücksichtigt diese geringfügige Verschiebung nicht.
Die Seillängen im Referenzpunkt werden gemessen von dem Punkt an dem das Seil die Trommel
verlässt bis zum Schneidedraht.

Zur Ermittlung Schrittmotor-Schritte pro mm Seil gibt es hier: http://cnc-hotwire.de/ eine gute Beschreibung. Ich empfehle allerdings eine regelmäßige Kontrolle durch nachmessen.

# Timing #