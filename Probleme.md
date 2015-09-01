# EMC startet nicht #

## Local APIC Problem ##

Das folgende bezieht sich auf die Kombination Ubuntu 10.04 und EMC 2.4

Wenn im error log eine zeile wie diese erscheint:

`[    0.000000] Local APIC disabled by BIOS -- you can enable it with "lapic"`

dann hilft meinstens:

in /etc/default/grub, diese Zeile:

`GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"`

ändern in

`GRUB_CMDLINE_LINUX_DEFAULT="quiet splash lapic"`

Achtung: man braucht dazu Administrator Rechte. Also zB den editor mit sudo starten.

`sudo gedit /etc/default/grub`, falls gedit der editor ist

dann `sudo update-grub` ausführen und neu booten


Das funktioniert nur, wenn das System ein LAPIC hat.

Wenn man daruafhin die folgende Meldung erhält:

`[ 0.000000] Local APIC not detected. Using dummy APIC emulation.`

hat man Pech mit diesem PC. Dann kann man entweder auf Ubuntu 8.04 zurückgehen, oder einen anderen PC probieren.