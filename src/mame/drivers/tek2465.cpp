// license:BSD-3-Clause
// copyright-holders: Sigurdur Asgeirsson
/***************************************************************************

    Tektronix 2465 oscilloscope driver.

****************************************************************************/


#include "emu.h"

#include "cpu/m6800/m6800.h"
#include "machine/ram.h"


class tek2465_state : public driver_device
{
public:
    tek2465_state(const machine_config& config, device_type type, const char* tag);

    void tek2465(machine_config& config);
    void init_tek2465();

private:
    void tek2465_map(address_map& map);

    required_device<cpu_device> m_maincpu;

    // TODO(siggi): IO region device!
};

tek2465_state::tek2465_state(const machine_config& config, device_type type, const char* tag) :
    driver_device(config, type, tag),
    m_maincpu(*this, "maincpu") {
}

void tek2465_state::tek2465(machine_config& config) {
    M6808(config, m_maincpu, 1000000);
    m_maincpu->set_addrmap(AS_PROGRAM, &tek2465_state::tek2465_map);
}

void tek2465_state::init_tek2465() {
    // TODO(siggi): Writeme.
}

void tek2465_state::tek2465_map(address_map& map) {
     // 2K RAM on the 2465.
    map(0x0000, 0x07FF).ram();

    // TODO(siggi): IO Registers from 0x0800-0x0FFF.
    map(0x0800, 0x0FFF).unmaprw();

    // TODO(siggi): Options from 0x1000-0x7FFF.
    map(0x1000, 0x7FFF).unmaprw();

    map(0x8000, 0xFFFF).rom().region("maincpu", 0);


    // Unmapped addresses are pinned to the NOP code.

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
