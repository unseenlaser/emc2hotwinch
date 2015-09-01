# Besonderheiten bei großer Zuspitzung #

Bei Profilen mit großer Zuspitzung treten zwangsläufig unterschiedliche Geschwindigkeiten an den beiden Seiten (root und tip) auf.Nehmen wir der Einfachheit halber an, die Vorderkante (Nasenleiste, grün im Bild, das schwarze Rechteck ist der Styroblock, von oben)) habe keine Pfeilung.

<img src='http://emc2hotwinch.googlecode.com/files/Beispiel1.jpg' />

Wie schaut die Bewegung aus, die Profili erzeugt ?

Zuerst wird der Schneidedraht in eine rechtwinkelige Startposition ausserhalb des Styro-Blocks ca. in Höhe der Endleiste gefahren.(Im Bild rot)
Dann folgt die erste Schneidebewegung, und zwar geradlinig bis zur Endleiste (blau). Bei dieser Bewegung muss die tip-Seite deutlich schneller fahren, da auf der root Seite meist nur ein paar mm zu fahren sind, auf der tip seite oft mehrere cm !

Anschließend wird das eigentliche Profil geschnitten. Jetzt ist die root Seite schneller, um wieviel hängt vom Ausmass der der Zuspitzung ab.
Dieser Geschwindigkeitsunterschied lässt sich grundsätzlich nicht vermeiden.
Der daraus resultierende Abbrandunterschied lässt sich in Profili einstellen.
Wenn der Draht wieder an der Endleiste (blau) angekommen ist, wird wieder aus dem Styro herausgefahren zur rechtwinkeligen Ausgangsposition.
Dabei ist jetzt wieder die tip Seite deutlich schneller.

## Anfahren ##

Unschön ist zunächst mal der extreme Geschwindigkeitsunterschied der beiden Seiten beim hinein/herausfahren.
Dafür habe ich folgende Lösung:
Der g-code wird manuell so abgeändert, dass die Start/End-Position nicht mehr rechtwinkelig ist, sondern entsprechend dem Winkel der Endleiste (im Bild gelb). Damit ist gewährleistet, dass der Draht schön gleichmässig auf beiden Seiten fährt.
Die entsprechende Änderung im g-Code ist schnell gemacht. (Beispiel fehlt noch)

Mit der neuen Profili Version (ab 2.26) erübrigt sich dieses Vorgehen, wenn man die neue Option verwendet, die Fläche an der Endleiste auszurichten.

## Schneiden des Profils ##

Dabei können prinzipiell unterschiedliche Geschwindikeiten auf root- und tip-Seite auftreten, weil die Drahtbewegung einerseits synchronsiert sein muss, andererseits aber die Wege unterschiedlich lang sind. Der Geschwindigkeitsbefehl bei EMC (F...) bezieht sich immer auf die XYZ Achsen, in unserem fall also auf das XY Koordinatensystem. Das UV System wird "mitbewegt", so dass die programmierten Endpunkte zeitgleich erreicht werden. Wenn man also die root Seite auf XY legt, dann fährt diese Seite mit der gewählten Geschwindigkeit, die tip Seite entsprechend langsamer.

### Umschalten der XY -UV Ebenen ###
Ich empfehle folgendes Verfahren:
Ich schneide immer mit root = X-Y. Damit daraus eine rechte und eine linke Hälfte wird, muss man an der Schneide die Seiten vertauschen. Das ist auch weiter nicht schwierig:
Man braucht nur einen anderen hal file mit geändertem pinning. d.h. ich habe jetzt zwei konfigurationen. Die sind identisch bis auf die pin belegung.
Die eingestellte Geschwindigkeit gilt immer für die root Seite, sowie auch für alle Bewegungen, die an beiden Enden gleich groß sind.
Hinein- und herausfahren muss nach dem oben beschriebenen Verfahren korrigiert werden, sonst erfolgt das auf der tip Seite viel zu schnell.

### Zwei unterschiedlich g-codes ###
Eine andere Möglichkeit ist, zwei verschiedene g-code files zu erzeugen. Das kann man in Profili machen, man muss aber die Geschwindigkeiten entsprechend der Zuspitzung korrigieren, da sich die vorgegebene Geschwindigkeit immer auf die XY Ebene bezieht.
Auch hier muss ggfls. das hinein- und herausfahren nach dem oben beschriebenen Verfahren korrigiert werden, sonst erfolgt das auf der tip Seite viel zu schnell, bzw auf der root seite zu langsam.