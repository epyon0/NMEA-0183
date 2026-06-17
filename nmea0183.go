package main

import (
	"bufio"
	"encoding/hex"
	"fmt"
	"log"
	"math"
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

///////////////////////////////////////////////
//// Garmin Proprietary Sentence Formaters ////
///////////////////////////////////////////////

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

// Aviation Height and VNAV Data
type GRMH struct {
	sentence                 string
	dataStatus               bool // A = data valid  V - data unusable
	vertSpeed                int  // feet/minute
	vnavProfileErr           int
	vertSpeedToVnavTarget    int     // feet/minute
	vertSpeedToNextWaypoint  int     // feet/minute
	approxHeightAboveTerrain uint    // feet, rounded to next lowest 100 feet
	desiredTrack             float32 // degrees true
	checksum                 byte
}

// Map Datum
type GRMM struct {
	sentence     string
	currMapDatum string
	checksum     byte
}

// Sensor Status Information
type GRMT struct {
	sentence                string
	prodModelSwVer          string
	romChecksumTest         bool // P = pass  F = fail
	recvFailDiscrete        bool // P = pass  F = fail
	storedDataLost          bool // R = retained  L = lost
	oscillatorDriftDiscrete bool // P = pass  F = excessive drift detected
	dataCollectionDiscrete  bool // C = collecting  null if not collecting
	gpsSensorTempC          float32
	gpsSensorConfigData     bool // R = retained  L = lost
	checksum                byte
}

// 3D velocity Information
type GRMV struct {
	sentence         string
	trueEastVelocity float32 // meters/second
	trueNortVelocity float32 // meters/second
	upVelocity       float32 // meters/second
	checksum         byte
}

// Altitude
type GRMZ struct {
	sentence     string
	currAltitude float32 // feet
	fixType      byte    // 1 = no fix  2 = 2D fix  3 = 3D fix
	checksum     byte
}

// DGPS Beacon Information
type GRMB struct {
	sentence               string
	beaconTuneFreq         float32 // kHz
	beaconBitRate          uint    // bits/second
	beaconSNR              uint
	beaconDataQuality      uint
	distToBeaconRefStation float32 // kilometers
	beaconRecvCommStatus   byte    // 0 = Check Wiring  1 = No Signal  2 = Tuning  3 = Receiving  4 = Scanning
	dgpsFixSrc             byte    // R = RTCM  W = WASS  N = Non-DGPS Fix
	dgpsMode               byte    // A = Automatic  W = WASS Only  R = RTCM Only  N = None (DGPS disabled)
	checksum               byte
}

//////////////////////////////////////////
///// IEC 61162-1 Sentence Formaters /////
//////////////////////////////////////////

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
	sentence                         string
	status1                          byte // A = Data valid  V = LORAN-C blink or SNR warning  V = general warning flag for other navigation systems when a reliable fix is not available
	status2                          byte // A = OK or not used  V = LORAN-C cycle lock warning flag
	magnitudeXTE                     float32
	directionToSteer                 byte // L/R
	xteUnits                         byte // nautical miles
	status3                          byte // A = arrival circle entered  V = arrival circle not passed
	status4                          byte // A = perpendicular passed at waypoint  V = perpendicular not entered
	bearingOriginToDest              float32
	bearingOriginToDestTrue          byte // M = magnetic  T = true
	destWaypointId                   string
	bearingPresentPosToDest          float32
	bearingPresentPosToDestTrue      byte // M = magnetic  T = true
	headingToSteerToDestWaypoint     float32
	headingToSteerToDestWaypointTrue byte // M = magnetic  T = true
	mode                             byte // A = Autonomous mode  D = Differential mode  E = Estimaged (dead reckoning) mode  M = Manual input mode  S = Simulator mode  N = Data not valid
	checksum                         byte
}

// AIS  broadcasting binary message
type BBM struct {
	sentence       string
	totalSentences uint
	sentenceNum    uint
	seqMsgId       uint
	channel        byte // 0 = no broadcast chan preference  1 = broadcast on AIS channel A  2 = broadcast on AIS channel B  3 = broadcast the message on both AIS channels A and B
	msgId          uint
	data           string
	fillBits       uint
	checksum       byte
}

// Bearing and distance to waypoint -- Dead reckoning
type BEC struct {
	sentence             string
	timeUTC              time.Time
	waypointLat          float32
	waypointLatDirection byte // N/S
	waypointLon          float32
	waypointLonDirection byte    // E/W
	bearingTrue          float32 // degrees
	bearingMagnetic      float32 // degrees
	distance             float32
	distanceUnits        byte // nautical miles
	waypointId           string
	checksum             byte
}

// Bearing origin to detination
type BOD struct {
	sentence        string
	bearingTrue     float32 // degrees
	bearingMagnetic float32 // degrees
	destWaypointId  string
	origWaypointId  string
	checksum        byte
}

// Bearing and distance to waypoint -- Great circle
type BWC struct {
	sentence             string
	timeUTC              time.Time
	waypointLat          float32
	waypointLatDirection byte // N/S
	waypointLon          float32
	waypointLonDirection byte    // E/W
	bearingTrue          float32 // degrees
	bearingMagnetic      float32 // degrees
	distance             float32
	distanceUnits        byte // nautical miles
	waypointId           string
	mode                 byte // A = Autonomous mode  D = Differential mode  E = Estimaged (dead reckoning) mode  M = Manual input mode  S = Simulator mode  N = Data not valid
	checksum             byte
}

// Bearing and distance to waypoint -- Rhumb line
type BWR struct {
	sentence             string
	timeUTC              time.Time
	waypointLat          float32
	waypointLatDirection byte // N/S
	waypointLon          float32
	waypointLonDirection byte    // E/W
	bearingTrue          float32 // degrees
	bearingMagnetic      float32 // degrees
	distance             float32
	distanceUnits        byte // nautical miles
	waypointId           string
	mode                 byte // A = Autonomous mode  D = Differential mode  E = Estimaged (dead reckoning) mode  M = Manual input mode  S = Simulator mode  N = Data not valid
	checksum             byte
}

// Bearing waypoint to waypoint
type BWW struct {
	sentence        string
	bearingTrue     float32 // degrees
	bearingMagnetic float32 // degrees
	toWaypointId    string
	fromWaypointId  string
	checksum        byte
}

// Configure broadcast rates for AIS AtoN station message command
type CBR struct {
	sentence          string
	mmsi              uint
	msgId             uint
	msgIdIdx          uint
	chanAStartTimeUTC time.Time
	chanASlotStart    int
	chanASlotInterval float32
	setup             byte // 0 = FATDMA  1 = RATDMA  2 = CSTDMA
	chanBStartTimeUTC time.Time
	chanBSlotStart    int
	chanBSlotInterval float32
	sentenceStatus    byte // R = Sentence is a status report of current settings  C = Sentence is a configuration command to change settings
	checksum          byte
}

// Water current layer -- Multi-layer water current data
type CUR struct {
	sentence           string
	validity           bool // A = Valid  V = not valid
	dataSetNum         uint
	layerNum           float32
	currDepth          float32 // meters
	currDirection      float32 // degrees
	directionReference byte    // T = true  R = relative
	currSpeed          float32 // knots
	refLayerDepth      float32 // meters
	heading            float32 // degrees
	headingReference   byte    // T = true  M = magnetic
	speedReference     byte    // B = Bottom track  W = Water track  P = Positioning system
	checksum           byte
}

// Depth below transducer
type DBT struct {
	sentence          string
	waterDepthFeet    float32
	waterDepthMeters  float32
	waterDepthFathoms float32
	checksum          byte
}

// Display dimming control
type DDC struct {
	sentence          string
	preset            byte // D = Day time setting  K = Dusk setting  N = Night time setting  O = Backlighting off setting
	brightnessPercent uint
	colorPalette      byte // D = Day time setting  K = Dusk setting  N = Night time setting  O = Backlighting off setting
	sentenceStatus    byte // R = Sentence is a status report of current settings  C = Sentence is a configuration command to change settings
	checksum          byte
}

// Door status detection
type DOR struct {
	sentence                string
	msgType                 byte // S = Status for section  E = Status for single door  F = Fault in system
	eventTime               time.Time
	typeOfDoorMonSys        string // WT = Watertight door  WS = Semi-watertight door  FD = Fire door  HD = Hull door  OT = Other
	firstDivisionIndicator  string // WT = Bulkhead/frame number  WS = Bulkhead/frame number  FD = Number/letter of zone  HD = Door indication/frame number  OT = Door indication/frame number
	secondDivisionIndicator uint   // WT = Deck number  WS = Deck number  FD = Deck number or control system loop number or other indicator HD = Deck number OT = Deck number
	doorNumber              uint   // Door number or door open count
	doorStatus              byte   // O = Open  C = Closed  S = Secured  F = Free status  X = Fault door
	watertightSwitch        byte   // O = Harbour mode (allowed open)  C = Sea mode (ordered closed)
	msgDescription          string
	checksum                byte
}

// Depth
type DPT struct {
	sentence           string
	waterDepth         float32 // Water depth relative to the transducer in meters
	transducerOffset   float32 // Offset from transducer in meters
	maxRangeScaleInUse float32
	checksum           byte
}

// Digital selective calling information
type DSC struct {
	sentence           string
	formateSpecifier   string
	address            uint
	category           string
	telecommand1       string  // Nature of distress or first telecommand
	telecommand2       string  // Type of communication or second telecommand
	posFreq            float32 // Position or channel/frequency   See note 8 & 9 IEC 61162-1 Pg. 45
	timePhone          string  // Time or telephone number
	mmsi               uint    // MMSI of ship in distress
	distress           string  // Nature of distress
	acknowledgement    byte    // R = Acknowledgeme request  B = Acknowledgement  S = Neither (end of sequence)
	expansionIndicator bool    // E = true  Null = false
	checksum           byte
}

// Expanded digital selective calling
type DSE struct {
	sentence       string
	totalSentences uint
	sentenceNum    uint
	queryReply     byte // Q = Query  R = Reply  A = Automatic
	mmsi           uint
	dataCode       []uint
	dataField      []string
	checksum       byte
}

// Datum reference
type DTM struct {
	sentence           string
	localDatum         string
	localDatumSubCode  byte
	latOffset          float32 // minutes
	latOffsetDirection byte    // N/S
	lonOffset          float32 // minutes
	lonOffsetDirection byte    // E/W
	altitudeOffset     float32 // meters
	refDatum           string
	checksum           byte
}

// Engine telegraph operation status
type ETL struct {
	sentence                      string
	eventTime                     time.Time
	msgType                       byte // O = Order  A = Answer-back
	posIndicatorOfEngineTelegraph uint // 00 = STOP ENGINE  01 = [AH] DEAD SLOW  02 = [AH] SLOW  03 = [AH] HALF  04 = [AH] FULL  05 = [AH] NAV. FULL  11 = [AS] DEAD SLOW  12 [AS] SLOW  13 = [AS] HALF  14 = [AS] FULL  15 = [AS] CRASH ASTERN
	posIndicatorOfSubTelegraph    uint // 20 = S/B (Stand-by engine)  30 = F/A (Full away - Navigation full)  40 = F/E (Finish with engine)
	operatingLocIndicator         uint // B = Bridge  P = Port wing  S = Starboard wing  C = Engine control room  E = Engine side / local  W = Wing (port or starboard not specified)
	numEngineProp                 uint // Number of engine or propeller shaft,  0 = single or on center-line  Odd = starboard  Even = port
	checksum                      byte
}

// General event message
type EVE struct {
	sentence         string
	eventTime        time.Time
	tagCode          string
	eventDescription string
	checksum         byte
}

// Fire detection
type FIR struct {
	sentence              string
	msgType               byte // S = Status for section  E = Status for each fire detector  F = Fault in system  D = Disabled
	eventTime             time.Time
	typeOfDetectionSystem string // FD = Generic fire  FH = Heat type  FS = Smoke type  FD = Smoke and heat  FM = Manual call point  GD = Any gas  GO = Oxygen gas  GS = Hydrogen sulphide gas  GH = Hydro-carbon gas  SF = Sprinkler flow switch  SV = Sprinkler manual valve release  CO = CO2 manual release  OT = Other
	division1Indicator    string // Any detection system = Number/letter of zone
	division2Indicator    string // Any detection system = Loop number
	detectorNumCount      uint   // Fire detector number or activation detection count, when msgType = E this identifies the dector, when msgType = S this contains the number of fire dectors activated
	condition             byte   // A = Activation  V = Non-activation  X = Fault (state unknown)
	alarmAcknowledged     bool   // A = true  V = false
	msgDescription        string
	checksum              byte
}

// Frequency set information
type FSI struct {
	sentence       string
	txFreq         uint // Hz
	rxFreq         uint // Hz
	mode           byte // d = F3E/G3E, simplex, telephone  e = F3E/G3E, duplex, telephone  m = J3E, telephone  o = H3E, telephone  q = F1B/J2B FEC NBDP, telex/teleprinter  s = F1B/J2B ARQ NBDP, telex/teleprinter  t = F1B/J2B recv only, teleprinter/DSC  w = F1B/J2B, teleprinter/DSC  x = A1A Morse, tape recorder  { = A1A Morse, Morse key/head set  | = F1C/F2C/F3C, facsimile machine
	pwrLvl         uint // 0 = standby  1 = lowest  9 = highest
	sentenceStatus byte // R = Sentence is a status report of current settings  C = Sentence is a configuration command to change settings
	checksum       byte
}

// GNSS satellite fault detection
type GBS struct {
	sentence         string
	timeUtc          time.Time
	expectedErrInLat float32 // meters
	expectedErrInLon float32 // meters
	expectedErrInAlt float32 // meters
	id               uint
	probMissDetect   float32
	estBias          float32
	estStdDevBias    float32
	gnssSystemId     byte // 1 (GP) = GPS  2 (GL) = GLONAS  3 (GA) = GALILEO
	gnssSignalId     byte // See IEC 61162-1 Table on Pg. 51
	checksum         byte
}

// Generic binary information
type GEN struct {
	sentence string
	index    uint16 // 4 char string of hex character to be converted to 16-bit number
	time     time.Time
	data     []byte // series of comma seperate 4 char hex strings to be converted to a byte slice
	checksum byte
}

// GNSS fix accuracy and integrity
type GFA struct {
	sentence                      string
	timeUTC                       time.Time
	horzProtectLvl                float32 // meters
	vertProtectLvl                float32 // meters
	axisOfErrSemiMajorStdDev      float32 // meters
	axisOfErrSemiMinorStdDev      float32 // meters
	axisOfErrSemiMajorOrientation float32 //meters
	altStdDev                     float32 // meters
	accuracyLvl                   float32 // meters
	integrityStatus               string
	checksum                      byte
}

// Global positioning system (GPS) fix data
type GGA struct {
	sentence          string
	timeUTC           time.Time
	lat               float32
	latDirection      byte // N/S
	lon               float32
	lonDirection      byte // E/W
	quality           int  // 0 = fix not available or invalid  1 = GPS SPS mode  2 = differential GPS, SPS mode  3 = GPS PPS mode  4 = Real Time Kinematic (RTK)  5 = Float RTK  6 = Estimated (dead reckoning) mode  7 = Manual input mode  8 = Simulator mode
	satsNum           int
	hdop              float32
	altSeaLvl         float32 // meters
	geoidalSeparation float32 // meters
	age               float32
	diffRefStationId  int
	checksum          byte
}

// Geographic position -- Latitude/longitude
type GLL struct {
	sentence     string
	lat          float32
	latDirection byte // N/S
	lon          float32
	lonDirection byte // E/W
	timeUTC      time.Time
	status       byte // A = data valid  V = data invalid
	mode         byte // A = Autonomous  D = Differential  E = Estimated (dead reckoning)  M = Manual input  S = Simulator  N = Data not valid
	checksum     byte
}

// GNSS fix data
type GNS struct {
	sentence           string
	timeUTC            time.Time
	lat                float32
	latDirection       byte // N/S
	lon                float32
	lonDirection       byte   // E/W
	mode               string // A = Autonomous  D = Differential  E = Estimated (dead reckoning)  F = Float RTK  M = Manual  N = No fix  P = Precise  R = Real Time Kinematic (RTK)  S = Simulator
	satsNum            int
	hdop               float32
	altSeaLvl          float32 // meters
	geoidalSeparation  float32
	age                float32
	diffRefStationId   float32
	navStatusIndicator byte // S = Safe  C = Caution  U = Unsafe  V = Navigational status not valid
	checksum           byte
}

// GNSS range residuals
type GRS struct {
	sentence       string
	mode           byte // 0 = residuals were used to calculate position  1 = residuals were re-computed after positon was computed
	rangeResiduals []float32
	gnssSystemId   byte // 1 (GP) = GPS  2 (GL) = GLONASS  3 (GA) = GALILEO  4-F = RESERVED
	signalId       byte
	checksum       byte
}

// GNSS DOP and active satellites
type GSA struct {
	sentence     string
	mode1        byte // M = manual  A = automatic
	mode2        byte // 1 = fix not available  2 = 2D  3 = 3D
	prns         []int
	pdop         float32
	hdop         float32
	vdop         float32
	gnssSystemId byte // 1 (GP) = GPS  2 (GL) = GLONASS  3 (GA) = GALILEO  4-F = RESERVED
	checksum     byte
}

// GNSS pseudorange noise statistics
type GST struct {
	sentence string

	checksum byte
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
			Verbose(fmt.Sprintf("Checksums do not match: 0x%02X != 0x%02X    Delta of: %d [0x%X]", chksum, givenChksum[0], int(math.Abs(float64(chksum-givenChksum[0]))), int(math.Abs(float64(chksum-givenChksum[0])))))
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

	test()
}

func test() {
	file, err := os.Open("./docs/test_sentences.txt")
	Er(err)
	defer file.Close()
	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		Verbose("TEST CHECKSUM:")
		ValidateChecksum(scanner.Text())
	}
}
