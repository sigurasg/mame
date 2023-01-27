// license:BSD-3-Clause
// copyright-holders: Sigurdur Asgeirsson
/***************************************************************************

    Tektronix 2465 oscilloscope driver.

****************************************************************************/

#include "emu.h"

#include "cpu/m6800/m6800.h"
#include "machine/er1400.h"

#include "screen.h"

#include "debug/debugcon.h"
#include "debug/debugcmd.h"
#include "debugger.h"

#include "tek2465.lh"

#include <cassert>

#define VERBOSE 1
#include "logmacro.h"

namespace {

class tek2465_state : public driver_device {
public:
	tek2465_state(const machine_config& config, device_type type, const char* tag);

	void tek2465(machine_config& config);

	// Fires an NMI on the CPU in simulation of what the PWRUP line does
	// in the scope.
	DECLARE_INPUT_CHANGED_MEMBER(power_pressed);

private:
	void debug_init();

	// Register debug dumpers.
	// Display sequencer shift register breakdown.
	void debug_ds(const std::vector<std::string_view> &params);
	// Port 1/2 breakdown.
	void debug_port1(const std::vector<std::string_view> &params);
	void debug_port2(const std::vector<std::string_view> &params);
	// Preamp shift register breakdown.
	void debug_pa(const std::vector<std::string_view> &params);
	// DAC sample and hold values.
	void debug_sh(const std::vector<std::string_view> &params);
	// Print or set pot values.
	void debug_pot(const std::vector<std::string_view> &params);

	void machine_start() override;
	void machine_reset() override;

	void tek2465_map(address_map& map);

	// Read/write/access IO ports.
	// Access to IO ports in this scope is a mixed bag. Only
	// a few ports actually use the data written, where others
	// are variously read or written, solely to create a strobe
	// to shift bits.
	void dac_msb_w(uint8_t data);
	void dac_lsb_w(uint8_t data);
	void port_1_w(uint8_t data);
	void port_2_w(uint8_t data);
	uint8_t ros_1_r();
	void ros_1_w(uint8_t data);
	void ros_2_w(uint8_t data);
	uint8_t dmux_0_off_r();
	void dmux_0_off_w(uint8_t data);
	uint8_t dmux_0_on_r();
	void dmux_0_on_w(uint8_t data);
	uint8_t port3_r();
	uint8_t dmux_1_off_r();
	void dmux_1_off_w(uint8_t data);
	uint8_t dmux_1_on_r();
	void dmux_1_on_w(uint8_t data);
	uint8_t led_r();
	void led_w(uint8_t data);
	uint8_t disp_seq_r();
	void disp_seq_w(uint8_t data);

	uint8_t attn_r();
	void attn_w(uint8_t data);
	uint8_t ch2_pa_r();
	void ch2_pa_w(uint8_t data);
	uint8_t ch1_pa_r();
	void ch1_pa_w(uint8_t data);
	uint8_t b_swp_r();
	void b_swp_w(uint8_t data);
	uint8_t a_swp_r();
	void a_swp_w(uint8_t data);
	uint8_t b_trig_r();
	void b_trig_w(uint8_t data);
	uint8_t a_trig_r();
	void a_trig_w(uint8_t data);
	uint8_t trig_stat_strb_r();
	void trig_stat_strb_w(uint8_t data);

	void dmux_0_update_dac();
	void dmux_1_update_dac();

	// Bit 2 of port 2 transitioned from 0 to 1.
	void attn_strobe();

	// Returns a one-bit value from this multiplexer.
	uint8_t u2456_r();

	// Returns true if selected analog value is less than DAC value.
	bool comp_r();

	TIMER_CALLBACK_MEMBER(interrupt_timer);

	// Temporarily displays the OSD only.
	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &rect);

	required_device<cpu_device> m_maincpu;
	required_device<er1400_device> m_earom;
	emu_timer* m_irq_timer;

	// Value of the most recent write to port 1.
	uint8_t m_port_1 = 0;
	// Value of the most recent write to port 2.
	uint8_t m_port_2 = 0;
	// Current value of the LED shift register chain.
	uint32_t m_front_panel_leds = 0;

	// In addition to the shifted LED state, the TRIG LED is also funneled
	// through here for convenience.
	output_finder<33> m_front_panel_led_outputs;

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
	// The trigger status register.
	// Number of bits left in the register.
	// TODO(siggi): It looks like the firmware shifts 16 bits out of this
	//    register each interrupt, though some interrupts read 32 bits.
	// uint8_t m_ds_tso_shift_num = 0;
	// uint16_t m_ds_tso_shift = 0;

	//////////////////////////////////////////////////////////////////////
	// Sweep hybrid state.
	// Each sweep hybrid conains one each
	// - 203-0231-90 die
	// - 203-0214-90 die
	// plus some discrete components. The former of these contains a 7 bit
	// shift register, which is modeled here.
	//////////////////////////////////////////////////////////////////////
	uint8_t m_swp_a_shift = 0;
	uint8_t m_swp_b_shift = 0;

	// Trigger hybrid state.
	// The trigger hybrid contains two "203-0213-90" dies, one each for the
	// A and B trigger. Each die contains a 7 bit shift register.
	uint8_t m_trig_a_shift = 0;
	uint8_t m_trig_b_shift = 0;

	// Attenuator shift register.
	// Various bits of this shift register are used for misc settings on
	// the main board A1 as well.
	uint16_t m_attn_shift = 0;

	// Shift registers for the CH1/CH2 preamps.
	// According to the service manual, each preamp can provide
	// 1/10, 1/2, 1/4, 1, 2.5 attenuation/amplification. Looking at
	// the settings applied in use, these are the codings:
	//     Code    Amplification
	//     0x5     1/10
	//     0x3     1/2
	//     0x2     1/4
	//     0x1     1
	//     0x0     2.5
	// From observation, 0x8 is the "invert" bit.
	// It appears the firmware routine shifts 8 bits to each register
	// at a time, though only 4 bits appear to be used.
	uint8_t m_pa1_shift = 0;
	uint8_t m_pa2_shift = 0;

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

	//////////////////////////////////////////////////////////////////////
	// Front panel scanning.
	//////////////////////////////////////////////////////////////////////
	required_ioport_array<10> m_front_panel_rows;
	required_ioport m_port_misc;
	required_ioport_array<16> m_analog_scanned;

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

	// Main board sample and hold state.
	uint16_t m_ch1_var = 0;
	uint16_t m_ch1_dc_bal = 0;
	uint16_t m_ch2_var = 0;
	uint16_t m_ch2_dc_bal = 0;
	uint16_t m_ch2_del_offs = 0;
	uint16_t m_holdoff = 0;
};

constexpr std::array<const char*, 16> ANALOG_SCANNED_TAGS = {
	"HOLDOFF",
	"TRIG_LEVEL",
	"HORIZ_VAR",
	"HORIZ_POS",
	"DELTA_B",
	"DELTA_A",
	"DLY_A",
	"DLY_B",
	"CH1_VAR",
	"CH2_VAR",
	"CH1_OVL",
	"CH2_OVL",
	"CH1_PRB",
	"CH2_PRB",
	"CH3_PRB",
	"CH4_PRB",
};

tek2465_state::tek2465_state(const machine_config& config, device_type type, const char* tag) :
	driver_device(config, type, tag),
	m_maincpu(*this, "maincpu"),
	m_earom(*this, "earom"),
	m_front_panel_led_outputs(*this, "FP_LED%u", 0U),
	m_character_rom(*this, "character_rom"),
	m_screen(*this, "screen"),
	m_front_panel_rows(*this, "ROW%u", 0),
	m_port_misc(*this, "MISC"),
	m_analog_scanned(*this, ANALOG_SCANNED_TAGS) {
}

void tek2465_state::tek2465(machine_config& config) {
	M6808(config, m_maincpu, 5_MHz_XTAL);
	ER1400(config, m_earom, 5_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &tek2465_state::tek2465_map);

	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	m_screen->set_refresh_hz(50);
	m_screen->set_size(320, 256);
	m_screen->set_visarea_full();
	m_screen->set_screen_update(FUNC(tek2465_state::screen_update));
}

INPUT_CHANGED_MEMBER(tek2465_state::power_pressed) {
	// Pulse the NMI when the power button is pressed.
	if (newval)
		m_maincpu->pulse_input_line(INPUT_LINE_NMI, attotime::from_msec(10));
}

void tek2465_state::debug_init() {
	if (machine().debug_flags & DEBUG_FLAG_ENABLED) {
		using namespace std::placeholders;

		debugger_console& con = machine().debugger().console();

		con.register_command("ds", CMDFLAG_NONE, 0, 0, std::bind(&tek2465_state::debug_ds, this, _1));
		con.register_command("port1", CMDFLAG_NONE, 0, 0, std::bind(&tek2465_state::debug_port1, this, _1));
		con.register_command("port2", CMDFLAG_NONE, 0, 0, std::bind(&tek2465_state::debug_port2, this, _1));
		con.register_command("pa", CMDFLAG_NONE, 0, 0, std::bind(&tek2465_state::debug_pa, this, _1));
		con.register_command("sh", CMDFLAG_NONE, 0, 0, std::bind(&tek2465_state::debug_sh, this, _1));
		con.register_command("pot", CMDFLAG_NONE, 0, 2, std::bind(&tek2465_state::debug_pot, this, _1));
	}
}

void tek2465_state::debug_ds(const std::vector<std::string_view> &params) {
	debugger_console &con = machine().debugger().console();

	con.printf("DS SHIFT: 0x%08X\n", m_ds_shift);
	con.printf("  MS10-14: 0x%02X\n", bitswap(m_ds_shift, 4, 3, 2, 1, 0));
	con.printf("  DS10-14: 0x%02X\n", bitswap(m_ds_shift, 5, 6, 7, 8, 9));
	con.printf("  TH0-4: 0x%02X\n", bitswap(m_ds_shift, 10, 11, 12, 13, 14));
	con.printf("  EXT-SWP: %X\n", BIT(m_ds_shift, 15));
	con.printf("  DL-END_MAIN: %X\n", BIT(m_ds_shift, 16));
	con.printf("  DISP-BLANK: %X\n", BIT(m_ds_shift, 17));
	con.printf("  ATS0: %X\n", BIT(m_ds_shift, 18));
	con.printf("  ATS1: %X\n", BIT(m_ds_shift, 19));
	con.printf("  ICT: %X\n", BIT(m_ds_shift, 20));
	con.printf("  ICS: %X\n", BIT(m_ds_shift, 21));
	con.printf("  Single Sequence: %X\n", BIT(m_ds_shift, 22));
	con.printf("  Include VDS1: %X\n", BIT(m_ds_shift, 23));
	con.printf("  Include VDS2: %X\n", BIT(m_ds_shift, 24));
	con.printf("  Include VDS12: %X\n", BIT(m_ds_shift, 25));
	con.printf("  Include VDS3: %X\n", BIT(m_ds_shift, 26));
	con.printf("  Include VDS4: %X\n", BIT(m_ds_shift, 27));
	con.printf("  MX: %X\n", BIT(m_ds_shift, 28));
	con.printf("  MD: %X\n", BIT(m_ds_shift, 29));
	con.printf("  MM: %X\n", BIT(m_ds_shift, 30));
	con.printf("  HDS1: %X\n", BIT(m_ds_shift, 31));
	con.printf("  HDS2: %X\n", BIT(m_ds_shift, 32));
	con.printf("  HDS3: %X\n", BIT(m_ds_shift, 33));
	con.printf("  HDS5: %X\n", BIT(m_ds_shift, 34));
	con.printf("  HDS6: %X\n", BIT(m_ds_shift, 35));
	con.printf("  HDS7: %X\n", BIT(m_ds_shift, 36));
	con.printf("  Slave delta: %X\n", BIT(m_ds_shift, 37));
	con.printf("  VT0-2: 0x%02X\n", bitswap(m_ds_shift, 40, 39, 38));
	con.printf("  CSR0-2M: 0x%02X\n", bitswap(m_ds_shift, 43, 42, 41));
	con.printf("  CSR0-2D: 0x%02X\n", bitswap(m_ds_shift, 46, 45, 44));
	con.printf("  Counter test: %X\n", BIT(m_ds_shift, 47));
	con.printf("  CP0-4: 0x%02X\n", bitswap(m_ds_shift, 52, 52, 50, 49, 48));
	con.printf("  CHL: %X\n", BIT(m_ds_shift, 53));
	con.printf("  CHM: %X\n", BIT(m_ds_shift, 54));
}

void tek2465_state::debug_port1(const std::vector<std::string_view> &params) {
	debugger_console &con = machine().debugger().console();

	con.printf("PORT1: 0x%02X\n", m_port_1);
	con.printf("  EAROM MODE: 0x%02X\n", BIT(m_port_1, 0, 3));
	con.printf("  EAROM CLK: %i\n", BIT(m_port_1, 3));
	con.printf("  EAROM DATA: %i\n", BIT(m_port_1, 4));
	con.printf("  PWR DOWN: %i\n", BIT(m_port_1, 5));
}

void tek2465_state::debug_port2(const std::vector<std::string_view> &params) {
	debugger_console &con = machine().debugger().console();

	con.printf("PORT2: 0x%02X\n", m_port_2);
	con.printf("  LED DATA: %i\n", BIT(m_port_2, 0));
	con.printf("  OEACLK: %i\n", BIT(m_port_2, 1));
	con.printf("  ATTN STRB: %i\n", BIT(m_port_2, 2));
	con.printf("  U2418 INH: %i\n", BIT(m_port_2, 3));
	con.printf("  U2408 INH: %i\n", BIT(m_port_2, 4));
	con.printf("  TRIG LED: %i\n", BIT(m_port_2, 5));
}

void tek2465_state::debug_pa(const std::vector<std::string_view> &params) {
	static const char* PA_SETTING[8] = {
		"*2.5", // 0
		"*1",   // 1
		"/2",   // 2
		"/4",   // 3
		"???",  // 4
		"/10",  // 5
		"???",  // 6
		"???"   // 7
	};
	debugger_console &con = machine().debugger().console();

	con.printf("PA1: %s%s\n", BIT(m_pa1_shift, 3) ? "INV " : "",
			PA_SETTING[BIT(m_pa1_shift, 0, 3)]);
	con.printf("PA2: %s%s\n", BIT(m_pa2_shift, 3) ? "INV " : "",
			PA_SETTING[BIT(m_pa2_shift, 0, 3)]);
}

void tek2465_state::debug_sh(const std::vector<std::string_view> &params) {
	debugger_console &con = machine().debugger().console();

	con.printf("NEG 1.25: 0x%04X\n", m_neg_125_v);
	con.printf("A TIM REF: 0x%04X\n", m_a_tim_ref);
	con.printf("B TIM REF: 0x%04X\n", m_b_tim_ref);
	con.printf("A TRIG LVL: 0x%04X\n", m_a_trig_lvl);
	con.printf("B TRIG LVL: 0x%04X\n", m_b_trig_lvl);
	con.printf("HORIZ POS: 0x%04X\n", m_horiz_pos);
	con.printf("DLY REF 0: 0x%04X\n", m_dly_ref_0);
	con.printf("DLY REF 1: 0x%04X\n", m_dly_ref_1);

	con.printf("CH1 VAR: 0x%04X\n", m_ch1_var);
	con.printf("CH1 DC BAL: 0x%04X\n", m_ch1_dc_bal);
	con.printf("CH2 VAR: 0x%04X\n", m_ch2_var);
	con.printf("CH1 DC BAL: 0x%04X\n", m_ch2_dc_bal);
	con.printf("CH2 DEL OFFS: 0x%04X\n", m_ch2_del_offs);
	con.printf("HOLDOFF: 0x%04X\n", m_holdoff);
}

void tek2465_state::debug_pot(const std::vector<std::string_view> &params) {
	debugger_console &con = machine().debugger().console();

	if (params.size() == 0) {
		for (const auto &scanned : m_analog_scanned)
			con.printf("%s: %d\n", scanned.finder_tag(), scanned->read());

		return;
	}

	// We have at least one argument.
	const std::string_view pot_name = params[0];
	auto find_pot = [this](std::string_view pot_name) -> analog_field* {
		const auto it = std::find_if(m_analog_scanned.begin(), m_analog_scanned.end(),
			[&pot_name](const auto &elem) -> bool
				{ return pot_name == elem.finder_tag(); });

		if (it == m_analog_scanned.end())
			return nullptr;

		if ((*it)->live().analoglist.size() == 0)
			return nullptr;

		return &(*it)->live().analoglist.front();
	};

	analog_field* pot_one = find_pot(pot_name);
	if (pot_one == nullptr) {
		con.printf("Couldn't find pot \"%s\"\n", std::string(pot_name).c_str());
		return;
	}

	if (params.size() == 1) {
		con.printf("%s: %d\n", std::string(pot_name), pot_one->field().port().read());
		return;
	}

	assert(params.size() >= 2);

	// Allow +- prefix for incrementally moving a pot or a pair of pots.
	std::string_view param = params[1];
	char prefix = '\0';
	if (param.size() > 0 && (param[0] == '+' || param[0] == '-')) {
		prefix = param[0];
		param = param.substr(1);
	}

	// Read the value.
	uint64_t value = 0;
	if (!con.validate_number_parameter(param, value) || value > 255) {
		con.printf("Need a numeric argument between 0x00 and 0xFF\n");
		return;
	}

	if (prefix == '\0') {
		pot_one->set_value(value);
		return;
	}

	// It's an incremental change, find the paired pot if one exists.
	analog_field* pot_two = nullptr;
	bool is_paired = pot_name.substr(pot_name.size() - 2) == "_A";
	if (is_paired) {
		std::string paired_name(pot_name);
		paired_name = paired_name.substr(0, pot_name.size() - 2) + "_B";
		pot_two = find_pot(paired_name);
		if (pot_two == nullptr) {
			con.printf("Couldn't find paired pot \"%s\"\n", paired_name.c_str());
			return;
		}
	}

	auto incremental_set = [](analog_field* pot, int64_t increment) {
		auto value = pot->field().port().read();
		pot->set_value(static_cast<uint8_t>(value + increment));
	};

	int64_t increment = static_cast<int64_t>(value);
	if (prefix == '-')
		increment = -increment;

	incremental_set(pot_one, increment);
	if (pot_two) {
		incremental_set(pot_two, increment);
	}
}

void tek2465_state::machine_start() {
	debug_init();

	m_front_panel_led_outputs.resolve();

	// TODO(siggi): More save state.
	save_item(NAME(m_port_1));
	save_item(NAME(m_port_2));
	save_item(NAME(m_front_panel_leds));
	save_item(NAME(m_dac.w));

	save_item(NAME(m_ds_shift));

	save_item(NAME(m_swp_a_shift));
	save_item(NAME(m_swp_b_shift));

	save_item(NAME(m_trig_a_shift));
	save_item(NAME(m_trig_b_shift));

	save_item(NAME(m_attn_shift));

	save_item(NAME(m_pa1_shift));
	save_item(NAME(m_pa2_shift));

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

	save_item(NAME(m_ch1_var));
	save_item(NAME(m_ch1_dc_bal));
	save_item(NAME(m_ch2_var));
	save_item(NAME(m_ch2_dc_bal));
	save_item(NAME(m_ch2_del_offs));
	save_item(NAME(m_holdoff));

	m_irq_timer = timer_alloc(FUNC(tek2465_state::interrupt_timer), this);
}

void tek2465_state::machine_reset() {
	// TODO(siggi): Does the counter start at an arbitrary count?
	m_irq_timer->adjust(attotime::from_usec(3300));

	// The PORT_1 register is cleared on RESET.
	m_port_1 = 0u;
}

void tek2465_state::tek2465_map(address_map& map) {
	// 2K RAM on the 2465.
	map(0x0000, 0x07FF).ram();

	// IO Registers.
	// UNUSED.
	map(0x0800, 0x083f).mirror(0x600).unmaprw();
	// DAC_MSB_CLK
	map(0x0840, 0x087f).mirror(0x600).w(FUNC(tek2465_state::dac_msb_w));
	// DAC_LSB_CLK
	map(0x0880, 0x08BF).mirror(0x600).w(FUNC(tek2465_state::dac_lsb_w));
	// PORT_1_CLK
	map(0x08C0, 0x08FF).mirror(0x600).w(FUNC(tek2465_state::port_1_w));
	// ROS_1_CLK
	map(0x0900, 0x0935).mirror(0x600).rw(FUNC(tek2465_state::ros_1_r), FUNC(tek2465_state::ros_1_w));
	// ROS_2_CLK
	map(0x0940, 0x097f).mirror(0x600).w(FUNC(tek2465_state::ros_2_w));
	// PORT2_CLK
	map(0x0980, 0x09BF).mirror(0x600).w(FUNC(tek2465_state::port_2_w));

	// UNUSED
	map(0x09C0, 0x09C0).mirror(0x630).unmaprw();
	// DMUX0_OFF
	map(0x09C1, 0x09C1).mirror(0x630).rw(FUNC(tek2465_state::dmux_0_off_r), FUNC(tek2465_state::dmux_0_off_w));
	// DMUX0_ON
	map(0x09C2, 0x09C2).mirror(0x630).rw(FUNC(tek2465_state::dmux_0_on_r), FUNC(tek2465_state::dmux_0_on_w));
	// PORT_3_IN
	map(0x09C3, 0x09C3).mirror(0x630).r(FUNC(tek2465_state::port3_r));
	// DMUX1_OFF
	map(0x09C4, 0x09C4).mirror(0x630).rw(FUNC(tek2465_state::dmux_1_off_r), FUNC(tek2465_state::dmux_1_off_w));
	// DMUX1_ON
	map(0x09C5, 0x09C5).mirror(0x630).rw(FUNC(tek2465_state::dmux_1_on_r), FUNC(tek2465_state::dmux_1_on_w));
	// LED_CLK
	map(0x09C6, 0x09C6).mirror(0x630).rw(FUNC(tek2465_state::led_r), FUNC(tek2465_state::led_w));
	// DISP_SEC_CLK
	map(0x09C7, 0x09C7).mirror(0x630).rw(FUNC(tek2465_state::disp_seq_r), FUNC(tek2465_state::disp_seq_w));
	// ATTN_CLK
	map(0x09C8, 0x09C8).mirror(0x630).rw(FUNC(tek2465_state::attn_r), FUNC(tek2465_state::attn_w));
	// CH_2_PA_CLK
	map(0x09C9, 0x09C9).mirror(0x630).rw(FUNC(tek2465_state::ch2_pa_r), FUNC(tek2465_state::ch2_pa_w));
	// CH_1_PA_CLK
	map(0x09CA, 0x09CA).mirror(0x630).rw(FUNC(tek2465_state::ch1_pa_r), FUNC(tek2465_state::ch1_pa_w));
	// B_SWP_CLK
	map(0x09CB, 0x09CB).mirror(0x630).rw(FUNC(tek2465_state::b_swp_r), FUNC(tek2465_state::b_swp_w));
	// A_SWP_CLK
	map(0x09CC, 0x09CC).mirror(0x630).rw(FUNC(tek2465_state::a_swp_r), FUNC(tek2465_state::a_swp_w));
	// B_TRIG_CLK
	map(0x09CD, 0x09CD).mirror(0x630).rw(FUNC(tek2465_state::b_trig_r), FUNC(tek2465_state::b_trig_w));
	// A_TRIG_CLK
	map(0x09CE, 0x09CE).mirror(0x630).rw(FUNC(tek2465_state::a_trig_r), FUNC(tek2465_state::a_trig_w));
	// TRIG_STAT_STRB
	map(0x09CF, 0x09CF).mirror(0x630).rw(FUNC(tek2465_state::trig_stat_strb_r), FUNC(tek2465_state::trig_stat_strb_w));

	// TODO(siggi): Options from 0x1000-0x7FFF.
	map(0x1000, 0x7FFF).unmaprw();

	map(0x8000, 0xFFFF).rom().region("maincpu", 0);
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
	dmux_1_update_dac();
}

void tek2465_state::dac_lsb_w(uint8_t data) {
	m_dac.b.l = data;

	// Update the DMUX sample and hold.
	dmux_0_update_dac();
	dmux_1_update_dac();
}

void tek2465_state::port_1_w(uint8_t data) {
	m_port_1 = data & 0x3F;
	m_earom->c1_w(BIT(data, 0));
	m_earom->c2_w(BIT(data, 1));
	m_earom->c3_w(BIT(data, 2));
	m_earom->clock_w(BIT(data, 3));
	m_earom->data_w(BIT(data, 4));

	// Setting bit 5 resets the processor through circuitry on the A5 board.
	if (BIT(data, 5))
		m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::from_seconds(1));
}

void tek2465_state::port_2_w(uint8_t data) {
	if (BIT(data, 2) && !BIT(m_port_2, 2))
		attn_strobe();

	m_port_2 = BIT(data, 0, 6);

	// Update the TRIG LED, the LED is lit when the bit is low.
	m_front_panel_led_outputs[32] = !BIT(data, 5);
}

uint8_t tek2465_state::ros_1_r() {
	// A ROS1 read is used to latch ROS2 shift to output.
	ros_1_w(0x01);
	return 0x01;
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
	} else if (!BIT(m_ros_2_latch, 3)) {
		// On a write with the mode set right, write through to the
		// character RAM.
		m_ros_ram[address] = bitswap(m_ros_1.b.h, 0, 1, 2, 3, 4, 5, 6, 7);
	}
}

uint8_t tek2465_state::dmux_0_off_r() {
	m_mux_0_disable = true;
	return 0x01;
}

void tek2465_state::dmux_0_off_w(uint8_t data) {
	dmux_0_off_r();
}

uint8_t tek2465_state::dmux_0_on_r() {
	m_mux_0_disable = false;

	dmux_0_update_dac();
	return 0x01;
}

void tek2465_state::dmux_0_on_w(uint8_t data) {
	dmux_0_on_r();
}

uint8_t tek2465_state::port3_r() {
	uint8_t ret = 0;

	ret |= 0x00 << 0; // TODO(siggi): Implement TSO.
	ret |= (comp_r() ? 0x02 : 0x00);
	ret |= BIT(m_ros_1.w, 15) << 2;
	ret |= 0x00 << 3;  // TODO(siggi): Implement RO ON.

	// Pin the EAROM output value while reading.
	m_earom->data_w(true);
	// It seems the EAROM read is inverted.
	ret |= (!m_earom->data_r()) << 4;
	// Restore the EAROM output value.
	m_earom->data_w(BIT(m_port_1, 4));

	ret |= u2456_r() << 5;

	return ret;
}

uint8_t tek2465_state::dmux_1_off_r() {
	m_mux_1_disable = true;
	return 0x01;
}

void tek2465_state::dmux_1_off_w(uint8_t data) {
	dmux_1_off_r();
}

uint8_t tek2465_state::dmux_1_on_r() {
	m_mux_1_disable = false;

	dmux_1_update_dac();
	return 0x01;
}

void tek2465_state::dmux_1_on_w(uint8_t data) {
	dmux_1_on_r();
}

uint8_t tek2465_state::led_r() {
	m_front_panel_leds <<= 1;
	m_front_panel_leds |= BIT(m_port_2, 0);

	// Set the LED outputs.
	for (size_t i = 0; i < 32; ++i)
		m_front_panel_led_outputs[i] = !BIT(m_front_panel_leds, i);

	return 0x01;
}

void tek2465_state::led_w(uint8_t data) {
	led_r();
}

uint8_t tek2465_state::disp_seq_r() {
	m_ds_shift = BIT((m_ds_shift << 1) | BIT(m_port_2, 0), 0, 55);
	return 0x01;
}

void tek2465_state::disp_seq_w(uint8_t data) {
	disp_seq_r();
}

uint8_t tek2465_state::attn_r() {
	m_attn_shift = (m_attn_shift << 1) | BIT(m_port_2, 0);
	return 0x01;
}
void tek2465_state::attn_w(uint8_t data) {
	attn_r();
}

uint8_t tek2465_state::ch2_pa_r() {
	m_pa2_shift = (m_pa2_shift << 1) | BIT(m_port_2, 0);
	return 0x01;
}
void tek2465_state::ch2_pa_w(uint8_t data) {
	ch2_pa_r();
}
uint8_t tek2465_state::ch1_pa_r() {
	m_pa1_shift = (m_pa1_shift << 1) | BIT(m_port_2, 0);
	return 0x01;
}
void tek2465_state::ch1_pa_w(uint8_t data) {
	ch1_pa_r();
}

uint8_t tek2465_state::b_swp_r() {
	m_swp_b_shift = BIT((m_swp_b_shift << 1) | BIT(m_port_2, 0), 0, 7);
	return 0x01;
}
void tek2465_state::b_swp_w(uint8_t data) {
	b_swp_r();
}

uint8_t tek2465_state::a_swp_r() {
	m_swp_a_shift = BIT((m_swp_a_shift << 1) | BIT(m_port_2, 0), 0, 7);
	return 0x01;
}
void tek2465_state::a_swp_w(uint8_t data) {
	a_swp_r();
}

uint8_t tek2465_state::b_trig_r() {
	m_trig_b_shift = BIT((m_trig_b_shift << 1) | BIT(m_port_2, 0), 0, 7);
	return 0x01;
}
void tek2465_state::b_trig_w(uint8_t data) {
	b_trig_r();
}

uint8_t tek2465_state::a_trig_r() {
	m_trig_a_shift = BIT((m_trig_a_shift << 1) | BIT(m_port_2, 0), 0, 7);
	return 0x01;
 }
void tek2465_state::a_trig_w(uint8_t data) {
	a_trig_r();
}

uint8_t tek2465_state::trig_stat_strb_r() {
	static uint64_t old_ds_shift = 0;

	// The firmware is checking the trigger status.
	// This is an opportune time to log any register
	// changes.
	if (m_ds_shift != old_ds_shift) {
		old_ds_shift = m_ds_shift;
		LOG("DS: 0x%08X\n", m_ds_shift);
	}

	return 0x01;
}

void tek2465_state::trig_stat_strb_w(uint8_t data) {
	trig_stat_strb_r();
}

void tek2465_state::dmux_0_update_dac() {
	if (m_mux_0_disable)
		return;

	uint16_t value = BIT(m_dac.w, 0, 12);
	switch (BIT(m_dac.b.h, 4, 3)) {
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

void tek2465_state::dmux_1_update_dac() {
	if (m_mux_1_disable)
		return;

	uint16_t value = BIT(m_dac.w, 0, 12);
	switch (BIT(m_dac.b.h, 4, 3)) {
		case 0: m_ch1_var = value; break;
		case 1: m_ch1_dc_bal = value; break;
		case 2: m_ch2_var = value; break;
		case 3: m_ch2_dc_bal = value; break;
		case 4: m_ch2_del_offs = value; break;
		case 5: break;
		case 6: m_holdoff = value; break;
		case 7: break;
	}
}

void tek2465_state::attn_strobe() {
	static const char* ATTN_BITS[16] = {
		"CH1 DC",
		"CH1 AC",
		"CH1 50Ohm",
		"CH1 1M",
		"CH1 10Xa",
		"CH1 1Xa",
		"CH1 10Xb",
		"CH1 1Xb",
		"CH2 DC",
		"CH2 AC",
		"CH2 50Ohm",
		"CH2 1M",
		"CH2 10Xa",
		"CH2 1Xa",
		"CH2 10Xb",
		"CH2 1Xb",
	};
	for (size_t i = 0; i < 16; ++i) {
		if (BIT(m_attn_shift, i))
			LOG("%s\n", ATTN_BITS[i]);
	}

	// TODO(siggi): Maintain attenuator state, play click sound on changes.
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

	// The bits from SI and J501 come from here.
	value |= m_port_misc->read();

	// Select the bit as indicated by the current m_dac.w value.
	return BIT(value, BIT(m_dac.w, 12, 3));
}

bool tek2465_state::comp_r() {
	// Bits 3/4 of port 2 inhibit the MUXes. If both is set, there's
	// no comparison, so just return a constant.
	if (BIT(m_port_2, 3, 2) == 0x3)
		return true;

	// The 3 LSBs of the port 1 register select the analog port.
	uint8_t index = BIT(m_port_1, 0, 3);

	// Both inhibit bits shouldn't be cleared at the same time,
	// so just read bit 3 here.
	if (BIT(m_port_2, 3) == 0)
		index += 8;

	constexpr float scanned_range = 1.25 + 1.36;
	constexpr float scanned_base = -1.25;
	// Scale the scanned value to the pot voltage.
	const float scanned_volts =
		(m_analog_scanned[index]->read() / 256.0) * scanned_range + scanned_base;

	constexpr float dac_base = 1.36;
	const float dac_volts = dac_base - (BIT(m_dac.w, 0, 12) / 4096.0) * 4e-3 * 681.0;

	bool out = dac_volts >= scanned_volts;
//	LOG("Scanned[%s]: %fV, DAC: %fV, out: %s\n", m_analog_scanned[index].finder_tag(),
//			scanned_volts, dac_volts, out ? "true" : "false");
	return out;
}

TIMER_CALLBACK_MEMBER(tek2465_state::interrupt_timer) {
	m_maincpu->set_input_line(M6800_IRQ_LINE, ASSERT_LINE);
}

uint32_t tek2465_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &rect) {
	// TODO(siggi): This is pretty hokey, but will do for rendering
	//    the OSD. This needs to read the ROS2 state flags to know
	//    whether to even render. Rendering in green with the set OSD
	//    brightness would be nice. Also rendering in - say - 256 gray
	//    scales, and decaying the previous contents of bitmap would be
	//    nice. The updates should probably also be timed to the ROS
	//    counter that also does the IRQs.
	bitmap.fill(0);

	const rgb_t green(0x00, 0xff, 0x00);

	// Where to start the OSD in X.
	constexpr int32_t col_offs = (320 - 256) / 2;
	// Where to start each OSD row in Y. Note that the OSD is encoded from bottom to top
	// while the bitmap is top to bottom.
	constexpr int32_t row_offs[2] = { 256, 32};
	for (size_t row = 0; row < 2; ++row) {
		for (size_t col = 0; col < 32; ++col) {
			uint8_t value = m_ros_ram[row * 32 + col];
			uint8_t* pixel = m_character_rom->base() + value * 16;

			while (*pixel & 0x80) {
				++pixel;
				int32_t x = col_offs + col * 8 + (*pixel & 0x07);
				// Flip the image vertically.
				int32_t y = row_offs[row] - (row * 16 + ((*pixel & 0x7F) >> 3));

				bitmap.pix(y, x) = green;
			}
		}
	}
	return 0;
}

const static ioport_value SEC_DIV_REMAP_TABLE[26] = {
	0x1F,   // X/Y
	0x1E,   // .5s/DIV
	0x1C,   // .2s/DIV
	0x1D,   // .1s/DIV
	0x19,   // 50ms/DIV
	0x18,   // 20ms/DIV
	0x1A,   // 10ms/DIV
	0x1B,   // 5ms/DIV
	0x13,   // 2ms/DIV
	0x12,   // 1ms/DIV
	0x10,   // .5ms/DIV
	0x11,   // .2ms/DIV
	0x15,   // .1ms/DIV
	0x14,   // 50us/DIV
	0x16,   // 20us/DIV
	0x17,   // 10us/DIV
	0x07,   // 5us/DIV
	0x06,   // 2us/DIV
	0x04,   // 1us/DIV
	0x05,   // .5us/DIV
	0x01,   // .2us/DIV
	0x00,   // .1us/DIV
	0x02,   // 50ns/DIV
	0x03,   // 20ns/DIV
	0x0B,   // 10ns/DIV
	0x0A,   // 5ns/DIV
};

const static ioport_value VOLTS_DIV_REMAP_TABLE[11] = {
	0x0F,   // 5V/DIV
	0x0E,   // 2V/DIV
	0x0C,   // 1V/DIV
	0x0D,   // .5V/DIV
	0x09,   // .2V/DIV
	0x08,   // .1V/DIV
	0x0A,   // 50mV/DIV
	0x0B,   // 20mV/DIV
	0x03,   // 10mV/DIV
	0x02,   // 5mV/DIV
	0x00,   // 2mV/DIV
};

INPUT_PORTS_START(tek2465)
	PORT_START("MISC")
	PORT_DIPNAME(0xC0, 0x80, "J501")
	PORT_DIPSETTING(0x40, "Calibrate")
	PORT_DIPSETTING(0x80, "Run")
	PORT_DIPSETTING(0xC0, "Cycle")
	PORT_DIPNAME(0x20, 0x00, "SI")
	PORT_DIPSETTING(0x00, "2465")
	PORT_DIPSETTING(0x20, "2445")
	// TODO(siggi): Model A5P503 to enable the NOP kernel test.

	PORT_START("POWER")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_BUTTON1 )
		PORT_CHANGED_MEMBER(DEVICE_SELF, tek2465_state,power_pressed, 0)

	// The front panel is ROW/COL scanned.
	PORT_START("ROW0")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("TRIG_CPL_DN")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("TRIG_CPL_UP")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("CH1_CPL_DN")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("CH1_CPL_UP")

	PORT_START("ROW1")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("CH4_VOLTS_DIV") PORT_TOGGLE
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("CH3_VOLTS_DIV") PORT_TOGGLE
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("CH2_CPL_DN")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("CH2_CPL_UP")

	PORT_START("ROW2")
	PORT_BIT( 0x0F, 0, IPT_LIGHTGUN_X ) PORT_NAME("CH1_VOLTS_DIV")
		PORT_POSITIONS(11) PORT_REMAP_TABLE(VOLTS_DIV_REMAP_TABLE) PORT_KEYDELTA(1)
		PORT_SENSITIVITY(10)
		PORT_CODE_INC(KEYCODE_Q) PORT_CODE_DEC(KEYCODE_Z)

	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("INVERT") PORT_TOGGLE

	PORT_START("ROW3")
	PORT_BIT( 0x0F, 0, IPT_LIGHTGUN_X ) PORT_NAME("CH2_VOLTS_DIV")
		PORT_POSITIONS(11) PORT_REMAP_TABLE(VOLTS_DIV_REMAP_TABLE) PORT_KEYDELTA(1)
		PORT_SENSITIVITY(10)
		PORT_CODE_INC(KEYCODE_W) PORT_CODE_DEC(KEYCODE_X)

	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("PULL_ALT_PUSH_B") PORT_TOGGLE

	PORT_START("ROW4")
	// A sweep speed. This is gray-code encoded.
	PORT_BIT( 0x1F, 0, IPT_LIGHTGUN_X ) PORT_NAME("A_SEC_DIV")
		PORT_POSITIONS(26) PORT_REMAP_TABLE(SEC_DIV_REMAP_TABLE)
		PORT_KEYDELTA(1) PORT_SENSITIVITY(10)
		PORT_CODE_INC(KEYCODE_E) PORT_CODE_DEC(KEYCODE_C)

	PORT_START("ROW5")
	// B sweep speed. This is gray-code encoded.
	PORT_BIT( 0x1F, 0, IPT_LIGHTGUN_X ) PORT_NAME("B_SEC_DIV")
		PORT_POSITIONS(26) PORT_REMAP_TABLE(SEC_DIV_REMAP_TABLE)
		PORT_KEYDELTA(1) PORT_SENSITIVITY(10)
		PORT_CODE_INC(KEYCODE_Y) PORT_CODE_DEC(KEYCODE_N)

	PORT_START("ROW6")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("CH_1") PORT_CODE(KEYCODE_1) PORT_TOGGLE
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("CH_2") PORT_CODE(KEYCODE_2) PORT_TOGGLE
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("ADD") PORT_TOGGLE
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("CH_3") PORT_CODE(KEYCODE_3) PORT_TOGGLE
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("CH_4") PORT_CODE(KEYCODE_4) PORT_TOGGLE

	PORT_START("ROW7")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("B_ENDS_A") PORT_TOGGLE
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("CHOP") PORT_TOGGLE
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("20_MHZ_BW_LIMIT") PORT_TOGGLE

	PORT_START("ROW8")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("X10_MAG") PORT_TOGGLE
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("TRACKING")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("DELTA_T") PORT_CODE(KEYCODE_T)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("DELTA_V") PORT_CODE(KEYCODE_V)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("SLOPE") PORT_CODE(KEYCODE_S)

	PORT_START("ROW9")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("TRIG_SRC_UP")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("TRIG_SRC_DN")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("TRIG_MODE_UP")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("TRIG_MODE_DN")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("A_B_TRIG")

	// Row 10 is N/C on the FP board.
	PORT_START("ROW10")
	PORT_BIT( 0x1F, IP_ACTIVE_LOW, IPT_UNUSED)

	// All the FP pots that are scanned.
	PORT_START("HORIZ_POS")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL ) PORT_SENSITIVITY(16) PORT_NAME("HORIZ_POS")

	PORT_START("DLY_A")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL ) PORT_SENSITIVITY(16) PORT_NAME("DLY_A")

	PORT_START("DLY_B")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL ) PORT_SENSITIVITY(16) PORT_NAME("DLY_B")

	PORT_START("DELTA_A")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL ) PORT_SENSITIVITY(16) PORT_NAME("DELTA_A")

	PORT_START("DELTA_B")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL ) PORT_SENSITIVITY(16) PORT_NAME("DELTA_B")

	PORT_START("TRIG_LEVEL")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL ) PORT_SENSITIVITY(16) PORT_NAME("TRIG_LEVEL")

	PORT_START("HOLDOFF")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL ) PORT_SENSITIVITY(16) PORT_NAME("HOLDOFF")

	PORT_START("CH1_VAR")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL ) PORT_SENSITIVITY(16) PORT_NAME("CH1_VAR")

	PORT_START("CH2_VAR")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL ) PORT_SENSITIVITY(16) PORT_NAME("CH2_VAR")

	PORT_START("HORIZ_VAR")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL ) PORT_SENSITIVITY(16) PORT_NAME("HORIZ_VAR")

	// All the A1 signals that are scanned.
	PORT_START("CH1_OVL")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL ) PORT_SENSITIVITY(16) PORT_NAME("CH1_OVL")

	PORT_START("CH2_OVL")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL ) PORT_SENSITIVITY(16) PORT_NAME("CH2_OVL")

	PORT_START("CH1_PRB")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL ) PORT_SENSITIVITY(16) PORT_NAME("CH1_PRB")

	PORT_START("CH2_PRB")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL ) PORT_SENSITIVITY(16) PORT_NAME("CH2_PRB")

	PORT_START("CH3_PRB")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL ) PORT_SENSITIVITY(16) PORT_NAME("CH3_PRB")

	PORT_START("CH4_PRB")
	PORT_BIT( 0xFF, 0x00, IPT_DIAL ) PORT_SENSITIVITY(16) PORT_NAME("CH4_PRB")

INPUT_PORTS_END

ROM_START(tek2465)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("160-1628-11.bin", 0x0000, 0x2000, CRC(0301c422) SHA1(93756dac882eb78960685702400b6c6fccbeacd9))
	ROM_LOAD("160-1627-11.bin", 0x2000, 0x2000, CRC(e6707bf1) SHA1(fa3c96dfdbdba61d1d98d48ba2787c3f9ca271e1))
	ROM_LOAD("160-1626-11.bin", 0x4000, 0x2000, CRC(f976700f) SHA1(ea0329824e1339831fd650f95fd1cb117d5449bc))
	ROM_LOAD("160-1625-11.bin", 0x6000, 0x2000, CRC(187bfa89) SHA1(f06e500c8add25bf7f41f0b5cefb0f303f9552eb))

	// Default EAROM contents.
	ROM_REGION16_BE(200, "earom", 0)
	ROM_LOAD16_WORD("earom.bin", 0, 200, CRC(4d8fbff7) SHA1(35c8e8157ef00f2ba2173afa6f628462440c675d))

	ROM_REGION(0x2000, "character_rom", 0)
	ROM_LOAD("160-1631-02.bin", 0, 0x1000, CRC(a3da922b))
ROM_END

}  // namespace

#define GAME_FLAGS  MACHINE_NO_SOUND|MACHINE_TYPE_OTHER|MACHINE_CLICKABLE_ARTWORK

GAMEL(1984, tek2465, 0, tek2465, tek2465, tek2465_state, empty_init, ROT0 , "Tektronix", "Tektronix 2465", GAME_FLAGS, layout_tek2465);
