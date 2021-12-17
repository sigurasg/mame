// license:BSD-3-Clause
// copyright-holders: Sigurdur Asgeirsson
/***************************************************************************

	Tektronix 2465 oscilloscope driver.

****************************************************************************/

#include <string>

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
	void tek2465_map(address_map& map);

	static std::string get_io_port_name(offs_t offset);
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
	logerror("Write 0x%02x to %s\n", data, get_io_port_name(offset).c_str());
}

uint8_t tek2465_state::io_read(offs_t offset) {
	logerror("Read from %s\n", get_io_port_name(offset).c_str());
	return 0x01;
}

// static
std::string tek2465_state::get_io_port_name(offs_t offset) {
	offs_t index = offset & 0x3F;
	const char* name = nullptr;
	switch (offset >> 6) {
		case 0:
			name = "UNUSED";
			break;
		case 1:
			name = "DAC_MSB_CLK";
			break;
		case 2:
			name = "DAC_LSB_CLK";
			break;
		case 3:
			name = "PORT_1_CLK";
			break;
		case 4:
			name = "ROS_1_CLK";
			break;
		case 5:
			name = "ROS_2_CLK";
			break;
		case 6:
			name = "PORT_2_CLK";
			break;
		case 7:
			switch (offset & 0xF) {
				case 0: return "UNUSED(fine)";
				case 1: return "DMUX0_OFF";
				case 2: return "DMUX0_ON";
				case 3: return "PORT_3_IN";
				case 4: return "DMUX1_OFF";
				case 5: return "DMUX1_ON";
				case 6: return "LED_CLK";
				case 7: return "DISP_SEQ_CLK";
				case 8: return "ATTN_CLK";
				case 9: return "CH2_PA_CLK";
				case 10: return "CH1_PA_CLK";
				case 11: return "B_SWP_CLK";
				case 12: return "A_SWP_CLK";
				case 13: return "B_TRIG_CLK";
				case 14: return "A_TRIG_CLK";
				case 15: return "TRIG_STAT_STRB";
			}
	}

	if (index == 0)
		return name;

	char buf[128];
	snprintf(buf, sizeof(buf), "%s+0x%02X", name, index);

	return buf;

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
