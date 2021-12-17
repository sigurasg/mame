// license:BSD-3-Clause
// copyright-holders: Sigurdur Asgeirsson
/***************************************************************************

	Tektronix 2465 oscilloscope driver.

****************************************************************************/

#include "emu.h"

#include "cpu/m6800/m6800.h"
#include "machine/er1400.h"

class tek2465_state : public driver_device
{
public:
	tek2465_state(const machine_config& config, device_type type, const char* tag);

	void tek2465(machine_config& config);
	void init_tek2465();

private:
    enum IOPort {
		UNUSED,
		DAC_MSB_CLK,
		DAC_LSB_CLK,
		PORT_1_CLK,
		ROS_1_CLK,
		ROS_2_CLK,
		PORT_2_CLK,
		DMUX0_OFF,
		DMUX0_ON,
		PORT_3_IN,
		DMUX1_OFF,
		DMUX1_ON,
		LED_CLK,
		DISP_SEQ_CLK,
		ATTN_CLK,
		CH2_PA_CLK,
		CH1_PA_CLK,
		B_SWP_CLK,
		A_SWP_CLK,
		B_TRIG_CLK,
		A_TRIG_CLK,
		TRIG_STAT_STRB,
    };

	void tek2465_map(address_map& map);

	static IOPort get_io_port(offs_t offset);
	static const char* get_io_port_name(IOPort io_port);

    // IO port accesses.
	void io_write(offs_t offset, uint8_t data);
	uint8_t io_read(offs_t offset);

	required_device<cpu_device> m_maincpu;
	required_device<er1400_device> m_earom;
};

tek2465_state::tek2465_state(const machine_config& config, device_type type, const char* tag) :
	driver_device(config, type, tag),
	m_maincpu(*this, "maincpu"),
	m_earom(*this, "earom") {
}

void tek2465_state::tek2465(machine_config& config) {
	M6808(config, m_maincpu, 5_MHz_XTAL);
	ER1400(config, m_earom, 5_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &tek2465_state::tek2465_map);
}

void tek2465_state::init_tek2465() {
	// TODO(siggi): Writeme.
}

void tek2465_state::tek2465_map(address_map& map) {
	 // 2K RAM on the 2465.
	map(0x0000, 0x07FF).ram();

	// IO Registers.
	map(0x0800, 0x09FF).rw(FUNC(tek2465_state::io_read), FUNC(tek2465_state::io_write)).mirror(0x600);

	// TODO(siggi): Options from 0x1000-0x7FFF.
	map(0x1000, 0x7FFF).unmaprw();

	map(0x8000, 0xFFFF).rom().region("maincpu", 0);

	// Unmapped addresses are pinned to the NOP code.
	// Sadly MAME doesn't allow that, the only undefined
	// values allowed are 0xFF or 0x00.
}

void tek2465_state::io_write(offs_t offset, uint8_t data) {
	IOPort io_port = get_io_port(offset);
	switch (io_port) {
		case PORT_1_CLK:
			m_earom->c1_w(BIT(data, 0));
			m_earom->c2_w(BIT(data, 1));
			m_earom->c3_w(BIT(data, 2));
			m_earom->clock_w(BIT(data, 3));
			m_earom->data_w(BIT(data, 4));
			break;
		default:
			break;
	}

	logerror("Write 0x%02x to %s\n", data, get_io_port_name(io_port));
}

uint8_t tek2465_state::io_read(offs_t offset) {
	IOPort io_port = get_io_port(offset);
	uint8_t read_value = 0x01;
	switch (io_port) {
		case PORT_3_IN:
			read_value = (m_earom->data_r() << 4);
			break;
		default:
			break;
	}

	logerror("Read 0x%02X from %s\n", read_value, get_io_port_name(io_port));
	return read_value;
}

// static
const char* tek2465_state::get_io_port_name(IOPort io_port) {
	switch (io_port) {
		case UNUSED: return "UNUSED";
		case DAC_MSB_CLK: return "DAC_MSB_CLK";
		case DAC_LSB_CLK: return "DAC_LSB_CLK";
		case PORT_1_CLK: return "PORT_1_CLK";
		case ROS_1_CLK: return "ROS_1_CLK";
		case ROS_2_CLK: return "ROS_2_CLK";
		case PORT_2_CLK: return "PORT_2_CLK";
		case DMUX0_OFF: return "DMUX0_OFF";
		case DMUX0_ON: return "DMUX0_ON";
		case PORT_3_IN: return "PORT_3_IN";
		case DMUX1_OFF: return "DMUX1_OFF";
		case DMUX1_ON: return "DMUX1_ON";
		case LED_CLK: return "LED_CLK";
		case DISP_SEQ_CLK: return "DISP_SEQ_CLK";
		case ATTN_CLK: return "ATTN_CLK";
		case CH2_PA_CLK: return "CH2_PA_CLK";
		case CH1_PA_CLK: return "CH1_PA_CLK";
		case B_SWP_CLK: return "B_SWP_CLK";
		case A_SWP_CLK: return "A_SWP_CLK";
		case B_TRIG_CLK: return "B_TRIG_CLK";
		case A_TRIG_CLK: return "A_TRIG_CLK";
		case TRIG_STAT_STRB: return "TRIG_STAT_STRB";
	}
}

// static
tek2465_state::IOPort tek2465_state::get_io_port(offs_t offset) {
	switch (offset >> 6) {
		case 0: return UNUSED;
		case 1: return DAC_MSB_CLK;
		case 2: return DAC_LSB_CLK;
		case 3: return PORT_1_CLK;
		case 4: return ROS_1_CLK;
		case 5: return ROS_2_CLK;
		case 6: return PORT_2_CLK;
		case 7:
			switch (offset & 0xF) {
				case 0: return UNUSED;
				case 1: return DMUX0_OFF;
				case 2: return DMUX0_ON;
				case 3: return PORT_3_IN;
				case 4: return DMUX1_OFF;
				case 5: return DMUX1_ON;
				case 6: return LED_CLK;
				case 7: return DISP_SEQ_CLK;
				case 8: return ATTN_CLK;
				case 9: return CH2_PA_CLK;
				case 10: return CH1_PA_CLK;
				case 11: return B_SWP_CLK;
				case 12: return A_SWP_CLK;
				case 13: return B_TRIG_CLK;
				case 14: return A_TRIG_CLK;
				case 15: return TRIG_STAT_STRB;
			}
        default: return UNUSED;
	}
}

ROM_START(tek2465)
	ROM_REGION(0x10000,"maincpu",0)
	ROM_LOAD("160-1628-11.bin", 0x0000, 0x2000, CRC(0301c422))
	ROM_LOAD("160-1627-11.bin", 0x2000, 0x2000, CRC(e6707bf1))
	ROM_LOAD("160-1626-11.bin", 0x4000, 0x2000, CRC(f976700f))
	ROM_LOAD("160-1625-11.bin", 0x6000, 0x2000, CRC(187bfa89))
ROM_END

INPUT_PORTS_START(tek2465)
INPUT_PORTS_END

COMP(1984, tek2465, 0, 0, tek2465, tek2465, tek2465_state, init_tek2465, "Tektronix", "Tektronix 2465", MACHINE_NO_SOUND);
