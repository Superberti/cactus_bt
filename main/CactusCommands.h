#ifndef COCOAANDWINDOXTYPES_H
#define COCOAANDWINDOXTYPES_H

/// Vereinbarungen fuer den Rueckgabewert eines Client-Kommandos
#define CLIENT_OK       "OK"        ///< Kommando erfolgreich
#define CLIENT_ERR      "ERR"       ///< Fehler im Kommando aufgetreten
#define CLIENT_PENDING  "PENDING"   ///< Befehl dauert laenger als der TCP/IP-Timeout, frag spaeter noch mal nach...

/// Status bei einem Kommandothread
#define THREAD_OK					"THREAD_OK"						///< Thread wurde erfolgreich ausgefuehrt
#define THREAD_ERR				"THREAD_ERR"					///< Thread wurde mit Fehler abgebrochen
#define THREAD_PENDING		"THREAD_PENDING"			///< Thread laeuft gerade
#define THREAD_NOTEXISTS	"THREAD_NOTEXISTS"		///< Momentan existiert gar kein Thread

/// Fuer Binaeuebertragungen CLIENT_OK vorangestellt:
/// Beispiel: "OK BINDATA 64"  -> nach dem return kommen 64 Bytes Daten
#define CLIENT_BINDATA  "BINDATA"   ///< Es kommen nach einem return Binaerwerte
#define CLIENT_BASE64   "BASE64"    ///< Es kommen Base64-kodierte Werte


/// Kommandos

/// Alle Kommandos sowie alle Parameter sind in Klartext als Strings gehalten.
/// Einzige Ausnahme ist hierbei die Antwort beim Lesen vom Server mit CLIENT_BINDATA
/// Nach diesem Konstrukt muessen die darauffolgenden Daten binaer gelesen werden.
/// Ist alles in Ordnung, so antwortet der Server mit: OK Rueckgabeparameter1..2..n.
/// Bei einem Fehler antwortet der Server mit ERR Fehlernummer Klartext.
/// Ein Clientkommando beginnt immer mit dem in CocoaCommands[] definierten Kommandostrings
/// und den Parametern, die mit Leerzeichen getrennt werden. Gibt es in einem Parameter
/// Leerzeichen, so ist der Parameter in Anfuehrungszeichen zu setzen.
/// Alle Befehle und Antworten muessen in einer Zeile stehen und enden mit einem Return.
enum TCactusCommand {
	/// Unbekanntes Kommando
	eCMD_UNKNOWN 						= 0,
 
  /// RGB-Wert einer spezifischen LED setzen (0-143)
  /// Parameter: LED-Nummer, R, G, B
  /// Beispiel: "SETLED 99 255 128 255"
  eCMD_SET_LED = 1,

  /// Einen Bereich (Start bis Ende) von LEDs setzen
  /// Parameter: Start-LED-Nummer, End-LED-Nummer, Rs, Gs, Bs ... Re, Ge, Be
  eCMD_SET_LED_RANGE = 2,

  /// LED-Effekt abspielen
  /// Parameter: Effektnummer (0=aus)
  eCMD_LED_EFFECT = 3,

  /// LED-Füllgrad in Prozent setzen (mit Farbe)
  /// Parameter: Füllgrad in Prozent, R, G, B
  eCMD_FILL_CACTUS = 4,

};

/// Kommandostruktur. Bindet den Befehl an eine feste Zeichenkette
struct TCommandStruct
{
	TCactusCommand 	mCmd;				  ///< Identifizierer des Kommandos.
	char						mCmdStr[40]; 	///< Dazugehoeriger Kommandostring
	int 						mNumParams;		///< Mindestanzahl zum Befehl zugehoeriger Parameter. Zusaetzliche Parameter, die der Server (noch) nicht versteht, werden ignoriert!
};//TWxSensorData

//----------------------------------------------------------------------------

/// Liste aller gueltigen Kommandos. Ueber dieses Feld koennen die
/// Kommandostrings direkt angesprochen werden, ohne direkt im
/// Quelltext ausgeschrieben zu werden. Damit vermeidet man
/// schwer zu findende Schreibfehler bei den Kommandostrings, die
/// dann erst im Betrieb auffallen wuerden.

const TCommandStruct CactusCommands[] = {
// Kommando									Kommandostring			Mindestzahl an Parametern   
	{eCMD_UNKNOWN,					"UNKNOWN"    				    ,0 },
  {eCMD_SET_LED,          "SETLED"	              ,4 },
  {eCMD_SET_LED_RANGE,    "SETLEDRANGE"	          ,5 },
  {eCMD_LED_EFFECT,       "LEDEFFECT"	            ,1 },
  {eCMD_FILL_CACTUS,      "FILLCACTUS"	          ,4 },
};

#endif
