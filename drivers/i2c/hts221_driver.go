package i2c

import (
	"encoding/binary"
	"errors"
	"fmt"
	"math"

	"gobot.io/x/gobot"
)

const (
	hts221address = 0x5f
)

// Averaged humidity samples configuration
const (
	hts221CfgAVGH4   = 0x00
	hts221CfgAVGH8   = 0x01
	hts221CfgAVGH16  = 0x02
	hts221CfgAVGH32  = 0x03 // default
	hts221CfgAVGH64  = 0x04
	hts221CfgAVGH128 = 0x05
	hts221CfgAVGH256 = 0x06
	hts221CfgAVGH512 = 0x07
)

// Averaged temperature samples configuration
const (
	hts221CfgAVGT2   = 0x00
	hts221CfgAVGT4   = 0x08
	hts221CfgAVGT8   = 0x10
	hts221CfgAVGT16  = 0x18 // default
	hts221CfgAVGT32  = 0x20
	hts221CfgAVGT64  = 0x28
	hts221CfgAVGT128 = 0x30
	hts221CfgAVGT256 = 0x38
)

// Control Reg1
const (
	hts221CtlPD       = 0x80 // PowerDown control
	hts221CtlBDU      = 0x04 // Block data update control
	hts221CtlODROne   = 0x00 // Output data rate: one shot
	hts221CtlODR1Hz   = 0x01 // Output data rate: 1 Hz
	hts221CtlODR7Hz   = 0x02 // Output data rate: 7 Hz
	hts221CtlODR125Hz = 0x03 // Output data rate: 12.5 Hz
)

// Status register
const (
	hts221StHDA = 0x02 // Humidity Data Available
	hts221StTDA = 0x01 // Temperature Data Available
)

// register addresses
const (
	hts221RegAVConf       = 0x10
	hts221RegCtrl1        = 0x20
	hts221RegCtrl2        = 0x21
	hts221RegCtrl3        = 0x22
	hts221RegStatus       = 0x27
	hts221RegHumidityOutL = 0x28
	hts221RegHumidityOutH = 0x29
	hts221RegTempOutL     = 0x2A
	hts221RegTempOutH     = 0x2B
	hts221RegH0_RH_X2     = 0x30
	hts221RegH1_RH_X2     = 0x31
	hts221RegT0_DEGC_X8   = 0x32
	hts221RegT1_DEGC_X8   = 0x33
	hts221RegT1_T0_MSB    = 0x35
	hts221RegH0_T0_OUT_L  = 0x36
	hts221RegH0_T0_OUT_H  = 0x37
	hts221RegH1_T0_OUT_L  = 0x3A
	hts221RegH1_T0_OUT_H  = 0x3B
	hts221RegT0_OUT_L     = 0x3C
	hts221RegT0_OUT_H     = 0x3D
	hts221RegT1_OUT_L     = 0x3E
	hts221RegT1_OUT_H     = 0x3F
)

type hts221CalibrationCoefficients struct {
	h0rh uint8
	h1rh uint8
	t0   uint16
	t1   uint16

	h0t0Out int16
	h1t0Out int16
	t0Out   int16
	t1Out   int16
}

// HTS221Driver is a driver for the HTS221 temperature/humidity sensor
type HTS221Driver struct {
	name string
	connector Connector
	connection Connection
	Config

	calib *hts221CalibrationCoefficients
}

// NewHTS221Driver creates a new driver with specified i2c interface.
// Params:
//		conn Connector - the Adaptor to use with this Driver
//
// Optional params:
//		i2c.WithBus(int):	bus to use with this driver
//		i2c.WithAddress(int):	address to use with this driver
//
func NewHTS221Driver(c Connector, options ...func(Config)) *HTS221Driver {
	h := &HTS221Driver{
		name:      gobot.DefaultName("hts221"),
		connector: c,
		Config:    NewConfig(),
		calib:       &hts221CalibrationCoefficients{},
	}

	for _, option := range options {
		option(h)
	}

	return h
}

// Name returns the name of the device.
func (d *HTS221Driver) Name() string {
	return d.name
}

// SetName sets the name of the device.
func (d *HTS221Driver) SetName(n string) {
	d.name = n
}

// Connection returns the connection of the device.
func (d *HTS221Driver) Connection() gobot.Connection {
	return d.connector.(gobot.Connection)
}

// Start initializes the HTS221, configures it, and loads the calibration coefficients.
func (d *HTS221Driver) Start() (err error) {
	bus := d.GetBusOrDefault(d.connector.GetDefaultBus())
	address := d.GetAddressOrDefault(hts221address)

	if d.connection, err = d.connector.GetConnection(address, bus); err != nil {
		return err
	}

	if err := d.connection.WriteByteData(hts221RegCtrl1, hts221CtlPD|hts221CtlODR1Hz); err != nil {
		return fmt.Errorf("hts221: power-ON error: %v", err)
	}

	if err := d.connection.WriteByteData(hts221RegAVConf, hts221CfgAVGH32|hts221CfgAVGT16); err != nil {
		return fmt.Errorf("hts221: configure error: %v", err)
	}

	// calibration
	h0rh, err := d.read(hts221RegH0_RH_X2)
	if err != nil {
		return fmt.Errorf("hts221: calibration error for H0_RH_X2: %v", err)
	}
	h1rh, err := d.read(hts221RegH1_RH_X2)
	if err != nil {
		return fmt.Errorf("hts221: calibration error for H1_RH_X2: %v", err)
	}

	raw, err := d.read(hts221RegT1_T0_MSB)
	if err != nil {
		return fmt.Errorf("hts221: calibration error for T1_T0_MSB: %v", err)
	}

	t0, err := d.read(hts221RegT0_DEGC_X8)
	if err != nil {
		return fmt.Errorf("hts221: calibration error for T0_DEGC_X8: %v", err)
	}

	t1, err := d.read(hts221RegT1_DEGC_X8)
	if err != nil {
		return fmt.Errorf("hts221: calibration error for T1_DEGC_X8: %v", err)
	}

	h0t0L, err := d.read(hts221RegH0_T0_OUT_L)
	if err != nil {
		return fmt.Errorf("hts221: calibration error for H0_T0_OUT_L: %v", err)
	}

	h0t0H, err := d.read(hts221RegH0_T0_OUT_H)
	if err != nil {
		return fmt.Errorf("hts221: calibration error for H0_T0_OUT_H: %v", err)
	}

	h1t0L, err := d.read(hts221RegH1_T0_OUT_L)
	if err != nil {
		return fmt.Errorf("hts221: calibration error for H1_T0_OUT_L: %v", err)
	}

	h1t0H, err := d.read(hts221RegH1_T0_OUT_H)
	if err != nil {
		return fmt.Errorf("hts221: calibration error for H1_T0_OUT_H: %v", err)
	}

	t0L, err := d.read(hts221RegT0_OUT_L)
	if err != nil {
		return fmt.Errorf("hts221: calibration error for T0_OUT_L: %v", err)
	}

	t0H, err := d.read(hts221RegT0_OUT_H)
	if err != nil {
		return fmt.Errorf("hts221: calibration error for T0_OUT_H: %v", err)
	}

	t1L, err := d.read(hts221RegT1_OUT_L)
	if err != nil {
		return fmt.Errorf("hts221: calibration error for T1_OUT_L: %v", err)
	}

	t1H, err := d.read(hts221RegT1_OUT_H)
	if err != nil {
		return fmt.Errorf("hts221: calibration error for T1_OUT_H: %v", err)
	}

	d.calib.h0rh = h0rh
	d.calib.h1rh = h1rh
	d.calib.t0 = (uint16(raw)&0x3)<<8 | uint16(t0)
	d.calib.t1 = (uint16(raw)&0xC)<<6 | uint16(t1)
	d.calib.h0t0Out = convI16(h0t0L, h0t0H)
	d.calib.h1t0Out = convI16(h1t0L, h1t0H)
	d.calib.t0Out = convI16(t0L, t0H)
	d.calib.t1Out = convI16(t1L, t1H)

	return nil
}

func (d *HTS221Driver) read(address byte) (byte, error) {
	if _, err := d.connection.Write([]byte{address}); err != nil {
		return 0, err
	}
	buf := make([]byte, 1)
	bytesRead, err := d.connection.Read(buf)
	if bytesRead != 1 || err != nil {
		return 0, err
	}
	return buf[0], nil
}

// Halt halts the device.
// no-op for HTS221.
func (d *HTS221Driver) Halt() (err error) {
	return nil
}

func convI16(lsb, msb uint8) int16 {
	var buf = [2]byte{lsb, msb}
	return int16(binary.LittleEndian.Uint16(buf[:]))
}

func (d *HTS221Driver) Humidity() (float64, error) {
	raw, err := d.read(hts221RegStatus)
	if err != nil {
		return 0, fmt.Errorf("hts221: error reading status register: %v", err)
	}

	if raw&hts221StHDA == 0 {
		return math.NaN(), errors.New("hts221: status: humidity data not available")
	}

	hoL, err := d.read(hts221RegHumidityOutL)
	if err != nil {
		return 0, fmt.Errorf("hts221: error reading HUMIDITY_OUT_L register: %v", err)
	}

	hoH, err := d.read(hts221RegHumidityOutH)
	if err != nil {
		return 0, fmt.Errorf("hts221: error reading HUMIDITY_OUT_H register: %v", err)
	}

	h := convI16(hoL, hoH)
	tH0rH := 0.5 * float64(d.calib.h0rh)
	tH1rH := 0.5 * float64(d.calib.h1rh)
	return tH0rH + (tH1rH-tH0rH)*float64(h-d.calib.h0t0Out)/float64(d.calib.h1t0Out-d.calib.h0t0Out), nil
}

func (d *HTS221Driver) Temperature() (float64, error) {
	raw, err := d.read(hts221RegStatus)
	if err != nil {
		return 0, fmt.Errorf("hts221: error reading status register: %v", err)
	}

	if raw&hts221StTDA == 0 {
		return math.NaN(), errors.New("hts221: status: temperature data not available")
	}

	toL, err := d.read(hts221RegTempOutL)
	if err != nil {
		return 0, fmt.Errorf("hts221: error reading TEMPERATURE_OUT_L register: %v", err)
	}

	toH, err := d.read(hts221RegTempOutH)
	if err != nil {
		return 0, fmt.Errorf("hts221: error reading TEMPERATURE_OUT_H register: %v", err)
	}

	t := convI16(toL, toH)
	t0 := 0.125 * float64(d.calib.t0)
	t1 := 0.125 * float64(d.calib.t1)
	return t0 + (t1-t0)*float64(t-d.calib.t0Out)/float64(d.calib.t1Out-d.calib.t0Out), nil
}
