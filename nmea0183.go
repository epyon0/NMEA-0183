package main

import (
	"fmt"
	"log"
	"os"
	"path"
	"runtime"
	"time"
)

var verbose bool

//////////////////////////////
///// Sentence Formaters /////
//////////////////////////////

// Waypoint arrival alarm
type AAM struct {
	sentence                 []string
	header                   string
	arrivalCircleEntered     bool
	perpendicularPassed      bool
	arrivalCircleRadius      float32
	arrivalCircleRadiusUnits byte
	waypointId               string
	checksum                 byte
}

// AIS addressed and binary broadcast acknowledgement
type ABK struct {
	sentence           []string
	header             string
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
	sentence       []string
	header         string
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
	sentence                   []string
	header                     string
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
	sentence []string
	header   string
	alarmId  uint
	checksum byte
}

// AIS channel management information source
type ACS struct {
	sentence        []string
	header          string
	seqNum          uint
	MMSI            uint
	receiptDateTime time.Time
	checksum        byte
}

// AIS interrogation request
type AIR struct {
	sentence               []string
	header                 string
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
	sentence                []string
	header                  string
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
	sentence                []string
	header                  string
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
	sentence               []string
	header                 string
	alarmChangeTime        time.Time
	alarmId                uint
	alarmThresholdExceeded bool // A = true, V = false
	alarmAcknowledged      bool // A = true, V = false
	checksum               byte
}

// Heading/track controller (autopilot) sentence B
type APB struct {
	sentence []string
	header   string
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

func main() {
	var args []string = os.Args[1:]

	for i := 0; i < len(args); i++ {
		var arg string = args[i]

		if arg == "-h" || arg == "--help" {
			fmt.Fprintf(os.Stdout, "\n\n%s -- Parse NMEA 0183 data.\n\n", os.Args[0])
			fmt.Fprintf(os.Stdout, "-h || --help     Print this help message\n")
			fmt.Fprintf(os.Stdout, "-v || --verbose  Verbose output\n")

			fmt.Fprintf(os.Stdout, "\n")
			os.Exit(0)
		}

		if arg == "-v" || arg == "--verbose" {
			verbose = true
		}
	}

	for i := 0; i < len(args); i++ {
		var arg string = args[i]

		Verbose(fmt.Sprintf("Processing argument: %s", arg))
	}
}
