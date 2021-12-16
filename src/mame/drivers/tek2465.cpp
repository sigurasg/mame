// license:BSD-3-Clause
// copyright-holders: Sigurdur Asgeirsson
/***************************************************************************

	Tektronix 2465 oscilloscope driver.

****************************************************************************/


#include "emu.h"

#include "cpu/m6800/m6800.h"
#include "machine/ram.h"

class tek2465_io_device : public device_t {
public:
	tek2465_io_device(const machine_config& mconfig, const char* tab, device_t* owner, uint32_t clock);

	// Register access
	void write(offs_t offset, uint8_t data);
	uint8_t read(offs_t offset);

	void device_start() override;

private:
	// uint32_t    m_front_panel_leds;
};

DECLARE_DEVICE_TYPE(TEK2465IO, tek2465_io_device)

class tek2465_state : public driver_device
{
public:
	tek2465_state(const machine_config& config, device_type type, const char* tag);

	void tek2465(machine_config& config);
	void init_tek2465();

private:
	void tek2465_map(address_map& map);

	required_device<cpu_device> m_maincpu;
	required_device<tek2465_io_device> m_io_device;

	// TODO(siggi): IO region device!
};

DEFINE_DEVICE_TYPE(TEK2465IO, tek2465_io_device, "tek2465_io", "Tektronix 2465 IO Device")

tek2465_io_device::tek2465_io_device(const machine_config& mconfig, const char* tag, device_t* owner, uint32_t clock) :
	device_t(mconfig, TEK2465IO, tag, owner, clock) {
}

void tek2465_io_device::write(offs_t offset, uint8_t data) {
	logerror("Write 0x%02x to 0x%02x\n", data, offset);
}

uint8_t tek2465_io_device::read(offs_t offset) {
	logerror("Read from 0x%02x\n", offset);
	return 0x01;
}

void tek2465_io_device::device_start() {
	// TODO(siggi): Writeme.
}

tek2465_state::tek2465_state(const machine_config& config, device_type type, const char* tag) :
	driver_device(config, type, tag),
	m_maincpu(*this, "maincpu"),
	m_io_device(*this, "io") {
}

void tek2465_state::tek2465(machine_config& config) {
	M6808(config, m_maincpu, 1000000);
	TEK2465IO(config, m_io_device, 100000);
	m_maincpu->set_addrmap(AS_PROGRAM, &tek2465_state::tek2465_map);
}

void tek2465_state::init_tek2465() {
	// TODO(siggi): Writeme.
}

void tek2465_state::tek2465_map(address_map& map) {
	 // 2K RAM on the 2465.
	map(0x0000, 0x07FF).ram();

	// IO Registers.
	map(0x0800, 0x09FF).rw(m_io_device, FUNC(tek2465_io_device::read), FUNC(tek2465_io_device::write)).mirror(0x600);

	// TODO(siggi): Options from 0x1000-0x7FFF.
	map(0x1000, 0x7FFF).unmaprw();

	map(0x8000, 0xFFFF).rom().region("maincpu", 0);

	// Unmapped addresses are pinned to the NOP code.
	// Sadly MAME doesn't allow that, the only undefined
	// values allowed are 0xFF or 0x00.
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
