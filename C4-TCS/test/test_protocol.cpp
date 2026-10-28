// Host tests for tcs_protocol.cpp
// Build/run: see README.md ("Tests")
#include "tcs_protocol.h"
#include <cstdio>

static int failures = 0;

#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

static void roundtrip(uint32_t type, uint8_t address, uint32_t payload, uint32_t serial) {
    CommandData b = buildCommand(type, address, payload, serial);
    CommandData p = parseCommand(b.command, b.is_long);
    if (b.command == 0 || p.type != type || p.address != b.address || p.is_long != b.is_long) {
        printf("FAIL roundtrip %s: cmd=%08x parsed=%s addr %02x/%02x long %d/%d\n", command_type_to_string(type),
               (unsigned)b.command, command_type_to_string(p.type), b.address, p.address, b.is_long, p.is_long);
        failures++;
    }
}

int main() {
    // Known vectors
    CHECK(buildCommand(COMMAND_TYPE_OPEN_DOOR, 3).command == 0x1103);
    CHECK(buildCommand(COMMAND_TYPE_OPEN_DOOR, 3).is_long == false);
    CHECK(buildCommand(COMMAND_TYPE_DOOR_CALL, 0, 0, 0x8c0b9).command == 0x0008c0b980);
    CHECK(buildCommand(COMMAND_TYPE_FLOOR_CALL, 0, 0, 0xC30BA).command == 0x1C30BA41);
    CHECK(buildCommand(COMMAND_TYPE_SEARCH_DOORMAN_DEVICES).command == 0x7FFF);

    // Round trips (type, address, payload, serial)
    roundtrip(COMMAND_TYPE_DOOR_CALL, 3, 0, 0x8c0b9);
    roundtrip(COMMAND_TYPE_INTERNAL_CALL, 5, 0, 0x8c0b9);
    roundtrip(COMMAND_TYPE_FLOOR_CALL, 1, 0, 0xC30BA);
    roundtrip(COMMAND_TYPE_FLOOR_CALL, 7, 0, 0xC30BA);
    roundtrip(COMMAND_TYPE_START_TALKING_DOOR_CALL, 2, 0, 0x12345);
    roundtrip(COMMAND_TYPE_START_TALKING, 2, 0, 0x12345);
    roundtrip(COMMAND_TYPE_STOP_TALKING_DOOR_CALL, 2, 0, 0);
    roundtrip(COMMAND_TYPE_STOP_TALKING, 2, 0, 0);
    roundtrip(COMMAND_TYPE_OPEN_DOOR, 9, 0, 0);
    roundtrip(COMMAND_TYPE_OPEN_DOOR_LONG, 4, 0, 0x12345);
    roundtrip(COMMAND_TYPE_LIGHT, 0, 0, 0);
    roundtrip(COMMAND_TYPE_END_OF_RINGTONE, 3, 0, 0);
    roundtrip(COMMAND_TYPE_FOUND_DEVICE, 2, 0, 0x12345);
    roundtrip(COMMAND_TYPE_FOUND_DEVICE_SUBSYSTEM, 2, 0, 0x12345);
    roundtrip(COMMAND_TYPE_DEVICE_TEST, 0, 3, 0x12345);
    roundtrip(COMMAND_TYPE_RESET, 0, 0, 0);
    roundtrip(COMMAND_TYPE_SEARCH_DEVICES, 0, 0, 0);

    // Long and short frames are told apart by the bus length, not the value
    CommandData lng = parseCommand(0x00000103, true);
    CHECK(lng.is_long);
    CHECK(!parseCommand(0x00001103, false).is_long);
    CHECK(parseCommand(0x00001103).type == COMMAND_TYPE_OPEN_DOOR); // guessed short

    // Names
    CHECK(string_to_command_type("device_test") == COMMAND_TYPE_DEVICE_TEST);
    CHECK(string_to_command_type("nonsense") == COMMAND_TYPE_UNKNOWN);

    printf(failures ? "%d FAILED\n" : "all tests passed\n", failures);
    return failures ? 1 : 0;
}
