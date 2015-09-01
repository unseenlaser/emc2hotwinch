# Versionen #

emc2hotwinch ist geeignet zum Betrieb von cnc gesteuerten Heissdraht Styroporschneideanlagen mit EMC2. Dabei kann eine konventionelle Linearachsenmechanik verwendet werden, oder eine einfache Seilwindenmechanik.

emc2hotwinch unterstützt derzeit die EMC Version 2.3.x, basierend auf Ubuntu 8.04 und die Version 2.4.x, basierend auf Ubuntu 10.04

Bitte nicht die Software aus dem download Bereich verwenden, das wird derzeit nicht gepflegt.

Stattdessen mit
`svn checkout http://emc2hotwinch.googlecode.com/svn/trunk/ emc2hotwinch` den aktuellen code herunterladen.
Wenn ubuntu meldet, dass svn nicht installiert ist: einfach mit aptget wie angegeben installieren.

Es sollten dann die folgenden Verzeichnisse angelegt sein:
  * XYAB
  * XYUV
  * kinematic
  * kinematic\_Ub\_08\_04-EMC\_2\_3

Zur weiteren Installation siehe wiki "Installation"