package main

import (
	"encoding/hex"
	"fmt"
	"log"
	"os"
	"path"
	"regexp"
	"runtime"
	"time"
)

var verbose bool
var vverbose bool
var talkerIds = map[string]string{
	"AB": "Independent AIS Base Station",
	"AD": "Dependent AIS Base Station",
	"AG": "Autopilot - General",
	"AI": "Mobile AIS Station",
	"AN": "AIS Aid to Navigation",
	"AP": "Autopilot - Magnetic",
	"AR": "AIS Receiving Station",
	"AT": "AIS Transmitting Station",
	"AX": "AIS Simplex Repeater",
	"BD": "BeiDou (China)",
	"BI": "Bilge System",
	"BN": "Bridge navigational watch alarm system",
	"CA": "Central Alarm",
	"CC": "Computer - Programmed Calculator (obsolete)",
	"CD": "Communications - Digital Selective Calling (DSC)",
	"CM": "Computer - Memory Data (obsolete)",
	"CR": "Data Receiver",
	"CS": "Communications - Satellite",
	"CT": "Communications - Radio-Telephone (MF/HF)",
	"CV": "Communications - Radio-Telephone (VHF)",
	"CX": "Communications - Scanning Receiver",
	"DE": "DECCA Navigation (obsolete)",
	"DF": "Direction Finder",
	"DM": "Velocity Sensor, Speed Log, Water, Magnetic",
	"DP": "Dynamiv Position",
	"DU": "Duplex repeater station",
	"EC": "Electronic Chart Display & Information System (ECDIS)",
	"EP": "Emergency Position Indicating Beacon (EPIRB)",
	"ER": "Engine Room Monitoring Systems",
	"FD": "Fire Door",
	"FS": "Fire Sprinkler",
	"GA": "Galileo Positioning System",
	"GB": "BeiDou (China)",
	"GI": "NavIC, IRNSS (India)",
	"GL": "GLONASS, according to IEIC 61162-1",
	"GN": "Combination of multiple satellite systems (NMEA 1083)",
	"GP": "Global Positioning System receiver",
	"GQ": "QZSS regional GPS augmentation system (Japan)",
	"HC": "Heading - Magnetic Compass",
	"HD": "Hull Door",
	"HE": "Heading - North Seeking Gyro",
	"HF": "Heading - Fluxgate",
	"HN": "Heading - Non North Seeking Gyro",
	"HS": "Hull Stress",
	"II": "Integrated Instrumentation",
	"IN": "Integrated Navigation",
	"JA": "Alarm and Monitoring",
	"JB": "Water Monitoring",
	"JC": "Power Management",
	"JD": "Propulsion Control",
	"JE": "Engine Control",
	"JF": "Propulsion Boiler",
	"JG": "Aux Boiler",
	"JH": "Engine Governor",
	"LA": "Loran A (obsolete)",
	"LC": "Loran C (obsolete)",
	"MP": "Microwave Positioning System (obsolete)",
	"MX": "Multiplexer",
	"NL": "Navigation light controller",
	"OM": "OMEGA Navigation System (obsolete)",
	"OS": "Distress Alarm System (obsolete)",
	"P":  "Vendor specific",
	"QZ": "QZSS regional GPS augmentation system (Japan)",
	"RA": "RADAR and/or ARPA",
	"RB": "Record Book",
	"RC": "Propulsion Machinery",
	"RI": "Rudder Angle Indicator",
	"SA": "Physical Shore AUS Station",
	"SD": "Depth Sounder",
	"SG": "Steering Gear",
	"SN": "Electronic Positioning System, other/general",
	"SS": "Scanning Sounder",
	"ST": "Skytraq debug output",
	"TC": "Track Control",
	"TI": "Turn Rate Indicator",
	"TR": "TRANSIT Navigation System",
	"U#": "'#' is a digit 0 …​ 9; User Configured",
	"UP": "Microprocessor controller",
	"VA": "VHF Data Exchange System (VDES), ASM",
	"VD": "Velocity Sensor, Doppler, other/general",
	"VM": "Velocity Sensor, Speed Log, Water, Magnetic",
	"VR": "Voyage Data recorder",
	"VS": "VHF Data Exchange System (VDES), Satellite",
	"VT": "VHF Data Exchange System (VDES), Terrestrial",
	"VW": "Velocity Sensor, Speed Log, Water, Mechanical",
	"WD": "Watertight Door",
	"WI": "Weather Instruments",
	"WL": "Water Level",
	"YC": "Transducer - Temperature (obsolete)",
	"YD": "Transducer - Displacement, Angular or Linear (obsolete)",
	"YF": "Transducer - Frequency (obsolete)",
	"YL": "Transducer - Level (obsolete)",
	"YP": "Transducer - Pressure (obsolete)",
	"YR": "Transducer - Flow Rate (obsolete)",
	"YT": "Transducer - Tachometer (obsolete)",
	"YV": "Transducer - Volume (obsolete)",
	"YX": "Transducer",
	"ZA": "Timekeeper - Atomic Clock",
	"ZC": "Timekeeper - Chronometer",
	"ZQ": "Timekeeper - Quartz",
	"ZV": "Timekeeper - Radio Update, WWV or WWVH",
}

////////////////////////////////////////////////////////////////////////////////////////
////                      Garmin Proprietary Sentence Formaters                     ////
//// https://developer.garmin.com/downloads/legacy/uploads/2015/08/190-00684-00.pdf ////
////////////////////////////////////////////////////////////////////////////////////////

// Sensor Initialization Information
type GRMI struct {
	sentence     string
	lat          float32
	latDirection byte
	lon          float32
	lonDirection byte
	timeUTC      time.Time
	receiverCmd  byte // A = Auto Locate   R = Unit Rest
	checksum     byte
}

// Sensor Configuration Information
type GRMC struct {
	sentence                          string
	fixMode                           byte    // A = automatic  2 = 2D exclusively (host system must supply altitude)  3 = 3D exclusively
	altitude                          float32 // meters
	earthDatumIndex                   uint
	earthDatumSemiMajorAxis           float32
	earthDatumInverseFlatteningFactor float64
	earthDatumDeltaX                  float32
	earthDatumDeltaY                  float32
	earthDatumDeltaZ                  float32
	diffMode                          byte // A = automatic  D = differential exclusively
	baudRate                          byte // 1 = 1200  2 = 2400  3 = 4800  4 = 9600  5 = 19200  6 = 300  7 = 600
	velocityFilter                    byte // 0 = No filter  1 = Automatic filter  2-255 = Filter time constant (e.g. 10 = 10 second filter)
	deadReckoningValidTime            uint // 1 - 30 seconds
	checksum                          byte
}

// Additional Sensor Configuration Information
type GRMC1 struct {
	sentence                     string
	outputTime                   uint // 0-900 seconds
	binaryPhaseOutputData        bool // 1 = off  2 = on
	autoPosAvgWhenStopped        bool // 1 = off  2 = on
	dgpsBeaconFreq               float32
	dgpsBeaconBitRate            float32
	dgpsBeaconScanning           bool // 1 = off  2 = on
	nmea0183Ver2_30ModeIndicator bool // 1 = off  2 = on
	dgpsMode                     bool // W = WAAS Only  N = None (DGPS disabled)
	adaptTransEnabled            bool // 1 = off  2 = on
	autoPwrOff                   bool // 1 = off  2 = on
	pwrOnWithExtCharger          bool // 1 = off  2 = on
	checksum                     byte
}

// Output Sentence Enable/Disable
type GRMO struct {
	sentence                  string
	targetSentenceDescription string
	targetSentenceMode        byte // 0 = disable specified sentence  1 = enable specified sentence  2 = disable all output sentences  3 = enable all output sentences (except GPALM)  4 = restore factory default output sentences
	checksum                  byte
}

// Additional Waypoint Information
type GRMW struct {
	sentence      string
	waypointId    string
	symbolNumber  uint
	commentString string
	checksum      byte
}

// Estimated Error Information
type GRME struct {
	sentence      string
	estHorzPosErr float32 // meters
	estVertPosErr float32 // meters
	estPosErr     float32 // meters
	checksum      byte
}

// GPS Fix Data Sentence
type GRMF struct {
	sentence                string
	gpsWeekNumber           uint
	gpsSeconds              uint
	timeUTC                 time.Time
	gpsLeapSecCount         uint
	lat                     float32
	latDirection            byte
	lon                     float32
	lonDirection            byte
	mode                    byte // M = manual  A = automatic
	fixType                 byte // 0 = no fix  1 = 2D fix  2 = 3D fix
	speedOverGround         uint // km/h
	courseOverGround        uint // degrees true
	posDilutionOfPrecision  uint
	timeDilutionOfPrecision uint
	checksum                byte
}

//////////////////////////////////
///// IEC Sentence Formaters /////
//////////////////////////////////

// Waypoint arrival alarm
type AAM struct {
	sentence                 string
	arrivalCircleEntered     bool
	perpendicularPassed      bool
	arrivalCircleRadius      float32
	arrivalCircleRadiusUnits byte
	waypointId               string
	checksum                 byte
}

// AIS addressed and binary broadcast acknowledgement
type ABK struct {
	sentence           string
	MMSI               uint
	channel            uint
	msgId              float32
	msgSeqNum          uint
	ackType            uint
	ackTypeDescription string // 0-5, See IEC 61162-1 Pg 30
	checksum           byte
}

// AIS addressed binary and safety related message
type ABM struct {
	sentence       string
	totalSentences uint
	sentenceNum    uint
	seqMsgId       uint
	MMSI           uint
	channel        uint
	msgId          uint
	data           []byte
	checksum       byte
}

// AIS channel assignment message
type ACA struct {
	sentence                   string
	seqNum                     uint
	latNE                      float32
	latNEDirection             byte
	lonNE                      float32
	lonNEDirection             byte
	latSW                      float32
	latSWDirection             byte
	lonSW                      float32
	lonSWDirection             byte
	transitionZoneSize         uint
	channelA                   uint
	channelABandwidth          uint
	channelB                   uint
	channelBBandwidth          uint
	modeControlTxRx            uint
	modeControlTxRxDescription string // 0-5, See IEC 61162-1 Pg. 32, Note 5
	pwrLvlControl              uint   // 0 = high power,  1 = low power
	infoSrc                    byte
	infoSrcDescription         string // A-D, M, See IEC 61162-1 Pg. 33, Note 7
	inUse                      bool   // 0 = false, 1 = true
	inUseChangeTime            time.Time
	checksum                   byte
}

// Acknowledge alarm
type ACK struct {
	sentence string
	alarmId  uint
	checksum byte
}

// AIS channel management information source
type ACS struct {
	sentence        string
	seqNum          uint
	MMSI            uint
	receiptDateTime time.Time
	checksum        byte
}

// AIS interrogation request
type AIR struct {
	sentence               string
	MMSIStation1           uint
	msg1Station1           float32
	msg1Station1Subsection uint
	msg2Station1           float32
	msg2Station1Subsection uint
	MMSIStation2           uint
	msg1Station2           float32
	msg1Station2Subsection uint
	channel                byte
	msg1Station1Reply      float32
	msg2Station1Reply      float32
	msg1Station2Reply      float32
	checksum               byte
}

// Acknowledge detail alarm condition
type AKD struct {
	sentence                string
	ackTime                 time.Time
	alarmSrcSysIndicator    string
	alarmSrcSubSysIndicator string
	alarmInstanceNum        uint
	alarmType               uint
	ackSrcSysIndicator      string
	ackSrcSubSysIndicator   string
	ackInstanceNum          uint
	checksum                byte
}

// Report detailed alarm condition
type ALA struct {
	sentence                string
	eventTime               time.Time
	alarmSrcSysIndicator    string
	alarmSrcSubSysIndicator string
	alarmInstanceNum        uint
	alarmType               uint
	alarmCondition          byte // See IEC 61162-1, Pg. 36, Note 8
	alarmAck                byte // See IEC 61162-1, Pg. 36, Note 9
	alarmDescription        string
	checksum                byte
}

// Set alarm state
type ALR struct {
	sentence               string
	alarmChangeTime        time.Time
	alarmId                uint
	alarmThresholdExceeded bool // A = true, V = false
	alarmAcknowledged      bool // A = true, V = false
	checksum               byte
}

// Heading/track controller (autopilot) sentence B
type APB struct {
	sentence string
}

func Verbose(text string) {
	now := time.Now()
	if verbose {
		pc, file, line, ok := runtime.Caller(1)

		if !ok {
			Er(fmt.Errorf("error getting caller function\n"))
		}

		msg := fmt.Sprintf("%02d:%02d:%02d.%04d | %v | %s:%d | VERBOSE: %v\n", now.Hour(), now.Minute(), now.Second(), now.Nanosecond()/1000000, runtime.FuncForPC(pc).Name(), path.Base(file), line, text)
		fmt.Fprintf(os.Stderr, "%s", msg)
	}
}

func VVerbose(text string) {
	if vverbose {
		Verbose(text)
	}
}

func Er(err error) {
	now := time.Now()
	pc, file, line, ok := runtime.Caller(1)

	if !ok {
		log.Fatal("error getting caller function\n")
		os.Exit(1)
	}

	if err != nil {
		msg := fmt.Sprintf("%02d:%02d:%02d.%04d | %v | %s:%d | ERROR: %v\n", now.Hour(), now.Minute(), now.Second(), now.Nanosecond()/1000000, runtime.FuncForPC(pc).Name(), path.Base(file), line, err)
		fmt.Fprintf(os.Stderr, "%s", msg)
		os.Exit(2)
	}
}

/*
func GetChecksum(sentence string) byte {

	// Checksum includes commas, anthing inside this regex capture group:
	// ^[$!](.*)\*\[a-zA-Z0-9]{2}$

	var chksum byte
	regex := regexp.MustCompile(`^[$!](.*)\*\[a-zA-Z0-9]{2}$`)
	match := regex.FindStringSubmatch(sentence)
	if len(match) >= 2 {
		for i := 0; i < len(match[1]); i++ {
			chksum ^= match[1][i]
		}
	}
	return chksum
}
*/

func ValidateChecksum(sentence string) bool {
	Verbose(fmt.Sprintf("Validating checksum for sentence: %s", sentence))
	var chksum byte
	regex := regexp.MustCompile(`^[$!](.*)\*([a-zA-Z0-9]{2})$`)
	Verbose(fmt.Sprintf("Using regular expresion: %s", regex.String()))
	match := regex.FindStringSubmatch(sentence)
	if len(match) >= 3 {
		for i := 0; i < len(match[1]); i++ {
			VVerbose(fmt.Sprintf("%02X ^ %02X = %02X", chksum, match[1][i], chksum^match[1][i]))
			chksum ^= match[1][i]
		}
		givenChksum, err := hex.DecodeString(match[2])
		Er(err)

		if len(givenChksum) >= 1 && chksum == givenChksum[0] {
			Verbose("Checksums match")
			return true
		} else {
			Verbose("Checksums do not match")
			return false
		}
	} else {
		Verbose("Regular expression did not match")
		return false
	}
}

func main() {
	var args []string = os.Args[1:]

	for i := 0; i < len(args); i++ {
		var arg string = args[i]

		if arg == "-h" || arg == "--help" {
			fmt.Fprintf(os.Stdout, "\n\n%s -- Parse NMEA 0183 data.\n\n", os.Args[0])
			fmt.Fprintf(os.Stdout, " -h || --help     Print this help message\n")
			fmt.Fprintf(os.Stdout, " -v || --verbose  Verbose output\n")
			fmt.Fprintf(os.Stdout, "-vv || --vverbose Very verbose output\n")

			fmt.Fprintf(os.Stdout, "\n")
			os.Exit(0)
		}

		if arg == "-v" || arg == "--verbose" {
			verbose = true
		}

		if arg == "-vv" || arg == "--vverbose" {
			verbose = true
			vverbose = true
		}
	}

	for i := 0; i < len(args); i++ {
		var arg string = args[i]

		Verbose(fmt.Sprintf("Processing argument: %s", arg))
	}

	Verbose("TEST CHECKSUM:")
	ValidateChecksum("$GPGGA,210230,3855.4487,N,09446.0071,W,1,07,1.1,370.5,M,-29.5,M,,*7A")
	ValidateChecksum("$GPGSV,2,1,08,02,74,042,45,04,18,190,36,07,67,279,42,12,29,323,36*77")
	ValidateChecksum("$GPGSV,2,2,08,15,30,050,47,19,09,158,,26,12,281,40,27,38,173,41*7B")
	ValidateChecksum("$GPRMC,210230,A,3855.4487,N,09446.0071,W,0.0,076.2,130495,003.8,E*69")
}
