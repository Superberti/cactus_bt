#ifndef CACTUSERRORS_H
#define CACTUSERRORS_H

#define ERR_OK                                  0  // Alles OK
#define ERR_GENERIC															1	// Nicht näher spezifizierter Fehler (nur Text)
#define ERR_CCREADTIMEOUT												2	// Lese-Timeout
#define ERR_CCWRITETIMEOUT											3	// Schreib-Timeout
#define ERR_BREAK																4	// Fehler für Abbruch
#define ERR_EOF																	5	// EndOfFile aufgetreten
#define ERR_UNKNOWNCOMMAND											6	// Unbekanntes Client-Kommando
#define ERR_OSERROR															7	// Betriebssystemfehler
#define ERR_EMPTYCMD														8	// Leere Befehlszeile
#define ERR_NUMPARAMS														9	// Falsche Zahl an Parametern für das aktuelle Kommando
#define ERR_WRONGPARAMTYPE											11	// Falscher Parametertyp (beim Client-Kommando)
#define ERR_CMD_NOTSUPPORTED										14	// Thread unterstuetzt dieses Kommando nicht
#define ERR_PARAM_OUT_OF_BOUNDS                 17  // Parameter außerhalb des Wertebereichs
#define ERR_EFFECT_RUNNING                      18  // Kommando kann nicht ausgeführt werden, da die Effektschleife läuft

#endif
