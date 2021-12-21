// license:BSD-3-Clause
// copyright-holders: Sigurdur Asgeirsson
/***************************************************************************

	Tektronix 2465 oscilloscope driver.

****************************************************************************/

#include "emu.h"

#include "cpu/m6800/m6800.h"
#include "machine/er1400.h"

#include "debug/debugcon.h"
#include "debug/debugcmd.h"
#include "debugger.h"

#include "tek2465.lh"

#define VERBOSE 1
#include "logmacro.h"


class tek2465_state : public driver_device
{
public:
	tek2465_state(const machine_config& config, device_type type, const char* tag);

	void tek2465(machine_config& config);

private:
	void debug_init();
	void debug_commands(const std::vector<std::string> &params);

	void device_start() override;

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

	enum {
		IRQ_TIMER,
	};

	void tek2465_map(address_map& map);

	static IOPort get_io_port(offs_t offset);
	static const char* get_io_port_name(IOPort io_port);

	// IO port accesses.
	// Some addresses don't care whether the MPU reads or writes them,
	// returns true if the access was handled.
	bool io_access(IOPort io_port);
	void io_write(offs_t offset, uint8_t data);
	uint8_t io_read(offs_t offset);

	uint8_t port3_r();

	// Returns a one-bit value from this multiplexer.
	uint8_t u2456_r();

	void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	required_device<cpu_device> m_maincpu;
	required_device<er1400_device> m_earom;
	emu_timer* m_irq_timer;

	// Value of the most recent write to port 1.
	uint8_t m_port_1 = 0;
	// Value of the most recent write to port 2.
	uint8_t m_port_2 = 0;
	// Current value of the LED shift register chain. 
	uint32_t m_front_panel_leds = 0;

	// Current DAC value (MSB+LSB).
	uint16_t m_dac = 0;

	// MUX disable flip flop states.
	bool m_mux_0_disable = true; 
	bool m_mux_1_disable = true; 

	//////////////////////////////////////////////////////////////////////
	// Display sequencer state.
	//////////////////////////////////////////////////////////////////////
	// The current value of the DS shift register.
	// Only the bottom 55 bits are used.
	uint64_t m_ds_shift = 0;
	// The trigger status strobe. Maybe two bits?
	// TODO(siggi): implement.
	// uint8_t m_ds_tss = 0;

	//////////////////////////////////////////////////////////////////////
	// Readout state.
	//////////////////////////////////////////////////////////////////////
	// Current value of the ROS1 shift register.
	uint16_t m_ros_1 = 0;

	// Current value of the ROS2 shift register.
	uint8_t m_ros_2_shift = 0;
	// The most recently latched value from above.
	uint8_t m_ros_2_latch = 0;

	// The readout RAM contents.
	uint8_t m_ros_ram[64] = { 0xFF };
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

void tek2465_state::debug_init() {
	if (machine().debug_flags & DEBUG_FLAG_ENABLED)
	{
		using namespace std::placeholders;
		machine().debugger().console().register_command("tek", CMDFLAG_CUSTOM_HELP, 0, 0, std::bind(&tek2465_state::debug_commands, this, _1));
	}
}

void tek2465_state::debug_commands(const std::vector<std::string> &params) {
	debugger_console &con = machine().debugger().console();

	con.printf("PORT1: 0x%02X\n", m_port_1);
	con.printf("PORT2: 0x%02X\n", m_port_2);
	con.printf("\tLED DATA: %i\n", BIT(m_port_2, 0));
	con.printf("\tOEACLK: %i\n", BIT(m_port_2, 1));
	con.printf("\tATTN STRB: %i\n", BIT(m_port_2, 2));
	con.printf("\tU2418 INH: %i\n", BIT(m_port_2, 3));
	con.printf("\tU2408 INH: %i\n", BIT(m_port_2, 4));
	con.printf("\tTRIG LED: %i\n", BIT(m_port_2, 5));
	
	con.printf("LEDs: 0x%04X\n", m_front_panel_leds);
	con.printf("DAC: 0x%02X\n", m_dac);

	con.printf("DS SHIFT: 0x%08X\n", m_ds_shift);

	con.printf("Next IRQ: %ss\n", m_irq_timer->remaining().to_string().c_str());
}

void tek2465_state::device_start() {
	debug_init();

	save_item(NAME(m_port_1));
	save_item(NAME(m_port_2));
	save_item(NAME(m_front_panel_leds));
	save_item(NAME(m_dac));

	save_item(NAME(m_ds_shift));

	save_item(NAME(m_ros_1));
	save_item(NAME(m_ros_2_shift));
	save_item(NAME(m_ros_2_latch));
	save_item(NAME(m_ros_ram));

	m_irq_timer = timer_alloc(IRQ_TIMER);
	// TODO(siggi): Does the counter start at an arbitrary count?
	m_irq_timer->adjust(attotime::from_usec(3300));
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

bool tek2465_state::io_access(IOPort io_port) {
	switch (io_port) {
		case LED_CLK:
			m_front_panel_leds <<= 1;
			m_front_panel_leds |= BIT(m_port_2, 0);
			break;
		case DISP_SEQ_CLK:
			m_ds_shift >>= 1;
			m_ds_shift |= static_cast<uint64_t>(BIT(m_port_2, 0)) << 55;
			break;
		case DMUX0_OFF:
			m_mux_0_disable = true;
			break;
		case DMUX0_ON:
			m_mux_0_disable = false;
			break;
		case DMUX1_OFF:
			m_mux_1_disable = true;
			break;
		case DMUX1_ON:
			m_mux_1_disable = false;
			break;

		default:
			return false;
	}

	return true;
}

void tek2465_state::io_write(offs_t offset, uint8_t data) {
	IOPort io_port = get_io_port(offset);
	if (io_access(io_port))
		return;

	switch (io_port) {
		case PORT_1_CLK:
			m_port_1 = data & 0x3F;
			m_earom->c1_w(BIT(data, 0));
			m_earom->c2_w(BIT(data, 1));
			m_earom->c3_w(BIT(data, 2));
			m_earom->clock_w(BIT(data, 3));
			m_earom->data_w(BIT(data, 4));
			break;

		case PORT_2_CLK:
			m_port_2 = data & 0x3F;
			break;

		case DAC_LSB_CLK:
			m_dac = (m_dac & 0xFF00) | data;
			break;

		case DAC_MSB_CLK:
			if (BIT(data, 7)) {
				// If the bit is high, the timer is reset and the IRQ is cleared.
				m_irq_timer->reset();
				m_maincpu->set_input_line(M6800_IRQ_LINE, CLEAR_LINE);
			} else if (BIT(m_dac, 15)) {
				// On toggle from high to low, set the timer.
				// We approximate the frequency to 3.3ms.
				// TODO(siggi): Make this right.
				m_irq_timer->adjust(attotime::from_usec(3300));
			}
			m_dac = (m_dac & 0x00FF) | (static_cast<uint16_t>(data) << 8);
			break;

		case ROS_1_CLK:
			m_ros_2_latch = m_ros_2_shift;
			{
				uint8_t hi = m_ros_1 >> 8;
				uint8_t lo = m_ros_1;

				if (!BIT(m_ros_2_latch, 2)) {
					hi <<= 1;
					hi |= lo >> 7;
				}
				lo <<= 1;
				lo |= BIT(data, 0);

				m_ros_1 = (static_cast<uint16_t>(hi) << 8) | lo;
			}

			// On a write with the mode set right, write through to the
			// character RAM.
			if (!BIT(m_ros_2_latch, 3)) {
				m_ros_ram[(m_ros_1 >> 1) & 0x7F] = m_ros_1 >> 8;
			}
			break;

		case ROS_2_CLK:
			m_ros_2_shift <<= 1;
			m_ros_2_shift |= BIT(data, 0);
			break;

		default:
			break;
	}

	LOG("Write 0x%02x to %s\n", data, get_io_port_name(io_port));
}

uint8_t tek2465_state::io_read(offs_t offset) {
	IOPort io_port = get_io_port(offset);
	if (io_access(io_port))
		return 0x01;

	uint8_t read_value = 0x01;
	switch (io_port) {
		case PORT_3_IN:
			read_value = port3_r();
			break;

		case ROS_1_CLK:
			m_ros_2_latch = m_ros_2_shift;
			break;
		default:
			break;
	}

	LOG("Read 0x%02X from %s\n", read_value, get_io_port_name(io_port));
	return read_value;
}

uint8_t tek2465_state::port3_r() {
	uint8_t ret = 0x01; // TODO(siggi): Implement TSO.
	ret |= m_earom->data_r() << 4;
	ret |= u2456_r() << 5;

	return ret;
}

uint8_t tek2465_state::u2456_r() {
	// This is the no-cal value.
	// TODO(siggi): Hookup the FP COL signals.
	uint8_t value = 0x80;

	// Select the bit as indicated by the current m_dac value.
	return BIT(value, BIT(m_dac, 12, 3));
}

void tek2465_state::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) {
	if (id == IRQ_TIMER)
		m_maincpu->set_input_line(M6800_IRQ_LINE, ASSERT_LINE);
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
	// Default EAROM contents.
	// TODO(siggi): Find valid EAROM contents.
	ROM_REGION16_BE(200, "earom", 0)
	ROM_LOAD16_WORD("earom.bin", 0, 200, CRC(ed086180))
ROM_END

INPUT_PORTS_START(tek2465)
INPUT_PORTS_END

GAMEL(1984, tek2465, 0, tek2465, tek2465, tek2465_state, empty_init, ROT0, "Tektronix", "Tektronix 2465", MACHINE_NO_SOUND, layout_tek2465);
