// license:BSD-3-Clause
// copyright-holders: Sigurdur Asgeirsson
/***************************************************************************

	Tektronix 2465 oscilloscope driver.

****************************************************************************/

#include "emu.h"

#include "cpu/m6800/m6800.h"
#include "machine/er1400.h"

#include "emupal.h"
#include "screen.h"

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

	// Read/write/access IO ports.
	void dac_msb_w(uint8_t data);
	void dac_lsb_w(uint8_t data);
	void port_1_w(uint8_t data);
	void ros_1_a();
	void ros_1_w(uint8_t data);
	void ros_2_w(uint8_t data);
	void dmux_0_off_a();
	void dmux_0_on_a();
	uint8_t port3_r();
	void dmux_1_off_a();
	void dmux_1_on_a();
	void led_a();
	void disp_seq_a();
	void attn_a();
	void ch2_pa_a();
	void ch1_pa_a();
	void b_swp_a();
	void a_swp_a();
	void b_trig_a();
	void a_trig_a();
	void trig_stat_strb_a();

	void dmux_0_update_dac();

	// Returns a one-bit value from this multiplexer.
	uint8_t u2456_r();

	void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// Temporarily displays the OSD only.
	u32 screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &rect);

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
	PAIR16 m_dac = {};

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
	PAIR16 m_ros_1 = {};

	// Current value of the ROS2 shift register.
	uint8_t m_ros_2_shift = 0;
	// The most recently latched value from above.
	uint8_t m_ros_2_latch = 0;

	// The readout RAM contents.
	uint8_t m_ros_ram[128] = { 0xFF };

	// The character ROM for the OSD.
	required_memory_region m_character_rom;

	// Temporary to display the OSD only.
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;

	//////////////////////////////////////////////////////////////////////
	// Front panel scanning.
	//////////////////////////////////////////////////////////////////////
	required_ioport_array<10> m_front_panel_rows;

	//////////////////////////////////////////////////////////////////////
	// A5 board sample and hold state.
	//////////////////////////////////////////////////////////////////////
	uint16_t m_neg_125_v = 0;
	uint16_t m_a_tim_ref = 0;
	uint16_t m_b_tim_ref = 0;
	uint16_t m_a_trig_lvl = 0;
	uint16_t m_b_trig_lvl = 0;
	uint16_t m_horiz_pos = 0;
	uint16_t m_dly_ref_0 = 0;
	uint16_t m_dly_ref_1 = 0;
};

tek2465_state::tek2465_state(const machine_config& config, device_type type, const char* tag) :
	driver_device(config, type, tag),
	m_maincpu(*this, "maincpu"),
	m_earom(*this, "earom"),
	m_character_rom(*this, "character_rom"),
	m_screen(*this, "screen"),
	m_palette(*this, "palette"),
	m_front_panel_rows(*this, "ROW%u", 0) {
}

void tek2465_state::tek2465(machine_config& config) {
	M6808(config, m_maincpu, 5_MHz_XTAL);
	ER1400(config, m_earom, 5_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &tek2465_state::tek2465_map);

	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	m_screen->set_refresh_hz(50);
	m_screen->set_size(32 * 8, 16 * 2);
	m_screen->set_visarea_full();
	m_screen->set_screen_update(FUNC(tek2465_state::screen_update));

	PALETTE(config, m_palette, palette_device::MONOCHROME);
	m_screen->set_palette(m_palette);
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
static const char* fp_led_names[] = {
	"CH1_AC_CPL",
	"CH1_GND1_CPL",
	"CH1_DC_CPL",
	"CH1_GND2_CPL",
	"CH1_50_CPL",

	"CH2_AC_CPL",
	"CH2_GND1_CPL",
	"CH2_DC1_CPL",
	"CH2_GND2_CPL",
	"CH2_50_CPL",

	"SLOPE_NEG",
	"READY",

	"SQL_SEQ",
	"NORM",
	"AUTO",
	"AUTO_LVL",
	"RUN",
	"TRIG",

	"TRIG_DC_CPL",
	"TRIG_NOISE_REJ_CPL",
	"TRIG_HF_REJ_CPL",
	"TRIG_LF_REJ_CPL",
	"TRIG_AC_CPL",

	"SLOPE_POS",

	"VERT",
	"CH1",
	"CH2",
	"CH3",
	"CH4",
	"LINE",

	"UNUSED1",
	"UNUSED2",
};
	static_assert(sizeof(fp_led_names)/ sizeof(fp_led_names[0]) == 32);
	for (size_t i = 0; i < 32; ++i) {
		if (BIT(m_front_panel_leds, i) == 0)
			con.printf("LED: %s\n", fp_led_names[i]);
	}

	con.printf("DAC: 0x%02X\n", m_dac.w);

	con.printf("DS SHIFT: 0x%08X\n", m_ds_shift);

	con.printf("Next IRQ: %ss\n", m_irq_timer->remaining().to_string().c_str());
}

void tek2465_state::device_start() {
	debug_init();

	save_item(NAME(m_port_1));
	save_item(NAME(m_port_2));
	save_item(NAME(m_front_panel_leds));
	save_item(NAME(m_dac.w));

	save_item(NAME(m_ds_shift));

	save_item(NAME(m_ros_1.w));
	save_item(NAME(m_ros_2_shift));
	save_item(NAME(m_ros_2_latch));
	save_item(NAME(m_ros_ram));

	save_item(NAME(m_neg_125_v));
	save_item(NAME(m_a_tim_ref));
	save_item(NAME(m_b_tim_ref));
	save_item(NAME(m_a_trig_lvl));
	save_item(NAME(m_b_trig_lvl));
	save_item(NAME(m_horiz_pos));
	save_item(NAME(m_dly_ref_0));
	save_item(NAME(m_dly_ref_1));

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
			led_a();
			break;
		case DISP_SEQ_CLK:
			disp_seq_a();
			break;
		case DMUX0_OFF:
			dmux_0_off_a();
			break;
		case DMUX0_ON:
			dmux_0_on_a();
			break;
		case DMUX1_OFF:
			dmux_1_off_a();
			break;
		case DMUX1_ON:
			dmux_1_on_a();
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
			port_1_w(data);
			break;

		case PORT_2_CLK:
			m_port_2 = data & 0x3F;
			break;

		case DAC_LSB_CLK:
			dac_lsb_w(data);
			break;

		case DAC_MSB_CLK:
			dac_msb_w(data);
			break;

		case ROS_1_CLK:
			ros_1_w(data);
			break;

		case ROS_2_CLK:
			ros_2_w(data);
			break;

		default:
			break;
	}

	LOG("Write 0x%02x to %s(0x%04X)\n", data, get_io_port_name(io_port), offset + 0x0800);
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
			ros_1_a();
			break;

		default:
			break;
	}

	LOG("Read 0x%02X from %s(0x%04X)\n", read_value, get_io_port_name(io_port), offset + 0x0800);
	return read_value;
}

void tek2465_state::dac_msb_w(uint8_t data) {
	if (BIT(data, 7)) {
		// If the bit is high, the timer is reset and the IRQ is cleared.
		m_irq_timer->reset();
		m_maincpu->set_input_line(M6800_IRQ_LINE, CLEAR_LINE);
	} else if (BIT(m_dac.w, 15)) {
		// On toggle from high to low, set the timer.
		// We approximate the frequency to 3.3ms.
		// TODO(siggi): Make this right.
		m_irq_timer->adjust(attotime::from_usec(3300));
	}

	// TODO(siggi): Assert that the timer is set anytime
	//    bit 15 is clear.

	m_dac.b.h = data;

	// Update the DMUX sample and hold.
	dmux_0_update_dac();
}

void tek2465_state::dac_lsb_w(uint8_t data) {
	m_dac.b.l = data;

	// Update the DMUX sample and hold.
	dmux_0_update_dac();
}

void tek2465_state::port_1_w(uint8_t data) {
	m_port_1 = data & 0x3F;
	m_earom->c1_w(BIT(data, 0));
	m_earom->c2_w(BIT(data, 1));
	m_earom->c3_w(BIT(data, 2));
	m_earom->clock_w(BIT(data, 3));
	m_earom->data_w(BIT(data, 4));
}

void tek2465_state::ros_1_a() {
	// A ROS1 read is used to latch ROS2 shift to output.
	ros_1_w(0x01);
}

void tek2465_state::ros_1_w(uint8_t data) {
	// Any access to the ros_1 register latches
	// the ros_2 shift register to the output.
	m_ros_2_latch = m_ros_2_shift;

	m_ros_1.w <<= 1;
	m_ros_1.w |= BIT(data, 0);
}

void tek2465_state::ros_2_w(uint8_t data) {
	m_ros_2_shift <<= 1;
	m_ros_2_shift |= BIT(data, 0);

	// The shift register address and contents are both
	// bit-swizzled to the address & data lines.
	uint8_t address = bitswap(m_ros_1.b.l, 1, 2, 3, 4, 5, 6, 7);
	if (!BIT(m_ros_2_latch, 2)) {
		// If bit 2 of the ROS2 register is set, read back the RAM
		// contents to the upper byte of the ROS1 register.
		m_ros_1.b.h = bitswap(m_ros_ram[address], 0, 1, 2, 3, 4, 5, 6, 7);

		LOG("ROS read 0x%02X from 0x%02X\n", m_ros_ram[address], address);
	} else if (!BIT(m_ros_2_latch, 3)) {
		// On a write with the mode set right, write through to the
		// character RAM.
		m_ros_ram[address] = bitswap(m_ros_1.b.h, 0, 1, 2, 3, 4, 5, 6, 7);

		LOG("ROS write 0x%02X to 0x%02X\n", m_ros_ram[address], address);
	}
}

void tek2465_state::dmux_0_off_a() {
	m_mux_0_disable = true;
}
void tek2465_state::dmux_0_on_a() {
	m_mux_0_disable = false;

	dmux_0_update_dac();
}

uint8_t tek2465_state::port3_r() {
	uint8_t ret = 0;

	ret |= 0x01 << 0; // TODO(siggi): Implement TSO.
	// ret |= comp_r();  // TODO(siggi): Implement comparator.
	ret |= BIT(m_ros_1.w, 15) << 2;
	ret |= 0x00 << 3;  // TODO(siggi): Implement RO ON.
	ret |= m_earom->data_r() << 4;
	ret |= u2456_r() << 5;

	return ret;
}

void tek2465_state::dmux_1_off_a() {
	m_mux_1_disable = true;
}
void tek2465_state::dmux_1_on_a() {
	m_mux_1_disable = false;
}

void tek2465_state::led_a() {
	m_front_panel_leds <<= 1;
	m_front_panel_leds |= BIT(m_port_2, 0);
}

void tek2465_state::disp_seq_a() {
	m_ds_shift >>= 1;
	m_ds_shift |= static_cast<uint64_t>(BIT(m_port_2, 0)) << 55;
}

void tek2465_state::attn_a() {}
void tek2465_state::ch2_pa_a() {}
void tek2465_state::ch1_pa_a() {}
void tek2465_state::b_swp_a() {}
void tek2465_state::a_swp_a() {}
void tek2465_state::b_trig_a() {}
void tek2465_state::a_trig_a() {}
void tek2465_state::trig_stat_strb_a() {}

void tek2465_state::dmux_0_update_dac() {
	if (m_mux_0_disable)
		return;

	uint16_t value = BIT(m_dac.w, 0, 12);
	switch (BIT(m_dac.b.h, 4,3)) {
		case 0: m_neg_125_v = value; break;
		case 1: m_a_tim_ref = value; break;
		case 2: m_b_tim_ref = value; break;
		case 3: m_a_trig_lvl = value; break;
		case 4: m_b_trig_lvl = value; break;
		case 5: m_horiz_pos = value; break;
		case 6: m_dly_ref_0 = value; break;
		case 7: m_dly_ref_1 = value; break;
	}
}

uint8_t tek2465_state::u2456_r() {
	// Bit 0x20 is SI, which is grounded on A1.
	uint8_t value = 0x1F;
	size_t selected_rows = 0;
	for (size_t i = 0; i < m_front_panel_rows.size(); ++i) {

		if (!BIT(m_dac.w, i)) {
			++selected_rows;
			value &= m_front_panel_rows[i]->read();
		}
	}

	// This is the no-cal value for the two MSBs.
	constexpr uint8_t kNoCal = 0x80;
	value |= kNoCal;

#if 0
	if (selected_rows != 1)
		LOG("Selected FP rows: %d\n", selected_rows);

	LOG("Output value: 0x%02X\n", value);
#endif

	// Select the bit as indicated by the current m_dac.w value.
	return BIT(value, BIT(m_dac.w, 12, 3));
}

void tek2465_state::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) {
	if (id == IRQ_TIMER)
		m_maincpu->set_input_line(M6800_IRQ_LINE, ASSERT_LINE);
}

uint32_t tek2465_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &rect) {
	// TODO(siggi): This is pretty hokey, but will do for rendering
	//    the OSD. This needs to read the ROS2 state flags to know
	//    whether to even render. Rendering in green with the set OSD
	//    brightness would be nice. Also rendering in - say - 256 gray
	//    scales, and decaying the previous contents of bitmap would be
	//    nice. The updates should probably also be timed to the ROS
	//    counter that also does the IRQs.
	bitmap.fill(0);
	for (size_t row = 0; row < 2; ++row) {
		for (size_t col = 0; col < 32; ++col) {
			uint8_t value = m_ros_ram[row * 32 + col];
			uint8_t* pixel = m_character_rom->base() + value * 16;

			while (*pixel & 0x80) {
				++pixel;
				int32_t x = col * 8 + (*pixel & 0x07);
				// Flip the image vertically.
				int32_t y = 32 - (row * 16 + ((*pixel & 0x7F) >> 3));

				bitmap.pix(y, x) = 1;
			}
		}
	}
	return 0;
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

	ROM_REGION(0x2000, "character_rom", 0)
	ROM_LOAD("160-1631-02.bin", 0, 0x1000, CRC(a3da922b))
ROM_END

INPUT_PORTS_START(tek2465)
	// The front panel is ROW/COL scanned.
	PORT_START("ROW0")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("TRIG_CPL_DN") PORT_CODE(KEYCODE_D)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("TRIG_CPL_UP") PORT_CODE(KEYCODE_U)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("CH1_CPL_DN")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("CH1_CPL_UP")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("ROW1")
	PORT_BIT( 0x1F, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_START("ROW2")
	PORT_BIT( 0x1F, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_START("ROW3")
	PORT_BIT( 0x1F, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_START("ROW4")
	PORT_BIT( 0x1F, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_START("ROW5")
	PORT_BIT( 0x1F, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_START("ROW6")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("CH_1") PORT_CODE(KEYCODE_1) PORT_TOGGLE
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("CH_2") PORT_CODE(KEYCODE_2) PORT_TOGGLE
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("ADD") PORT_CODE(KEYCODE_A) PORT_TOGGLE
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("CH_3") PORT_CODE(KEYCODE_3) PORT_TOGGLE
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("CH_4") PORT_CODE(KEYCODE_4) PORT_TOGGLE

	PORT_START("ROW7")
	PORT_BIT( 0x1F, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("ROW8")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("X10_MAG")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("TRACKING")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("DELTA_T") PORT_CODE(KEYCODE_T)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("DELTA_V") PORT_CODE(KEYCODE_V)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("SLOPE") PORT_CODE(KEYCODE_S)

	PORT_START("ROW9")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("TRIG_SRC_UP") PORT_CODE(KEYCODE_UP)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("TRIG_SRC_DN") PORT_CODE(KEYCODE_DOWN)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("TRIG_MODE_UP") PORT_CODE(KEYCODE_LEFT)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("TRIG_MODE_DN") PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("A_B_TRIG") PORT_CODE(KEYCODE_DEL)

	// Row 10 is N/C on the FP board.
	PORT_START("ROW10")
	PORT_BIT( 0x1F, IP_ACTIVE_LOW, IPT_UNUSED)


	// TODO(siggi): Model all pots.

	PORT_START("HOLDOFF")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL)

	// TODO(siggi): Model jumpers as "DIP Switches".
	// A5: P503 to enable the NOP kernel test.
	// A5: P501 CAL/NO CAL jumper.
INPUT_PORTS_END

GAMEL(1984, tek2465, 0, tek2465, tek2465, tek2465_state, empty_init, ROT0, "Tektronix", "Tektronix 2465", MACHINE_NO_SOUND, layout_tek2465);
