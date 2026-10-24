#ifdef WINDOWS
#include "../new_common.h"

#include "../driver/drv_miot.h"
#include "../driver/drv_public.h"
#include "../driver/drv_uart.h"
#include "../quicktick.h"
#include "selftest_local.h"

#if ENABLE_DRIVER_MIOT
static void MIoT_TestReceive(const char *text) {
	while (*text)
		UART_AppendByteToReceiveRingBuffer((unsigned char)*text++);
	DRV_RunQuickTick();
}

static void MIoT_TestExpect(const char *text) {
	while (*text) {
		SELFTEST_ASSERT(SIM_UART_GetDataSize() > 0);
		if (!SIM_UART_GetDataSize())
			return;
		SELFTEST_ASSERT(SIM_UART_GetByte(0) == (unsigned char)*text++);
		SIM_UART_ConsumeBytes(1);
	}
	SELFTEST_ASSERT_HAS_UART_EMPTY();
}

static void MIoT_TestSetup(void) {
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(8192);
	SELFTEST_ASSERT(CMD_ExecuteCommand("startDriver MIoT", 0) == CMD_RES_OK);
	SELFTEST_ASSERT(CMD_ExecuteCommand("MIOTMAP 2 1 1 bool 0 1 1", 0) == CMD_RES_OK);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotMap 2 5 2 int 1 100 1", 0) == CMD_RES_OK);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotMap 2 2 3 int 0 255 0", 0) == CMD_RES_OK);
	MIoT_TestReceive("model xiaomi.fan.p45\r");
	MIoT_TestExpect("ok\r");
	MIoT_TestReceive("get_down\r");
	MIoT_TestExpect("down get_properties 2 1\r");
	MIoT_TestReceive("result 2 1 0 false\rget_down\r");
	MIoT_TestExpect("ok\rdown get_properties 2 5\r");
	MIoT_TestReceive("result 2 5 0 1\rget_down\r");
	MIoT_TestExpect("ok\rdown get_properties 2 2\r");
	MIoT_TestReceive("result 2 2 0 0\rget_down\r");
	MIoT_TestExpect("ok\rdown none\r");
}

static void MIoT_TestCallbacks(void) {
	MIoT_TestSetup();
	CMD_ExecuteCommand("setPinChannel 6 2", 0);
	CMD_ExecuteCommand("setPinRole 6 Rel", 0);
	CMD_ExecuteCommand("addChangeHandler Channel2 == 1 addChannel 10 1", 0);
	CHANNEL_Set(2, 0, 0);
	DRV_RunQuickTick();
	SELFTEST_ASSERT_CHANNEL(2, 1);
	SELFTEST_ASSERT_PIN_BOOLEAN(6, 1);
	SELFTEST_ASSERT_CHANNEL(10, 1);
	MIoT_TestReceive("properties_changed 2 5 1\r");
	MIoT_TestExpect("ok\r");
	SELFTEST_ASSERT_PIN_BOOLEAN(6, 1);

	MIoT_TestSetup();
	CMD_ExecuteCommand("addChangeHandler Channel1 == 1 setChannel 1 0", 0);
	MIoT_TestReceive("properties_changed 2 1 true\rget_down\r");
	MIoT_TestExpect("ok\rdown set_properties 2 1 false\r");
	SELFTEST_ASSERT_CHANNEL(1, 0);

	MIoT_TestSetup();
	CMD_ExecuteCommand("addChangeHandler Channel1 == 1 backlog uartInit 9600; uartFakeHex 6765745f646f776e0d", 0);
	MIoT_TestReceive("properties_changed 2 1 true\r");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SELFTEST_ASSERT(UART_GetDataSize() == 9);
	DRV_RunQuickTick();
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SELFTEST_ASSERT(UART_GetDataSize() == 9);

	// A stop/start callback replaces the session even though MIoT owns the new
	// UART. The old parser must not ACK or publish its remaining property tuples.
	MIoT_TestSetup();
	CMD_ExecuteCommand("addChangeHandler Channel1 == 1 backlog stopDriver MIoT; startDriver MIoT; miotMap 2 5 4 int 1 100 1", 0);
	MIoT_TestReceive("properties_changed 2 1 true 2 5 37\r");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SELFTEST_ASSERT_CHANNEL(4, 0);
	MIoT_TestReceive("get_down\r");
	MIoT_TestExpect("down get_properties 2 5\r");
}

void Test_MIoT(void) {
	int i;
	MIoT_TestCallbacks();
	MIoT_TestSetup();
	// Mapping and outgoing values have strict syntax, types and bounds.
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotMap 2 1 4 bool 0 1 1", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotMap 2 3 1 int 0 2 1", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotMap 2 3 64 int 0 2 1", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotMap 2 3 4 bool 0 2 1", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotMap 2 3 4 int 0 2 1 extra", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotSet 2 5 0", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotSet 2 5 101", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotSet 2 5 300", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotSet 2 5 999999999999999999999", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotSet 2 5 37x", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotSet 2 1 trueish", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotSet 2 1 2", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotSet 2 2 1", 0) == CMD_RES_ERROR);
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	MIoT_TestReceive("get_down\r");
	MIoT_TestExpect("down none\r");
	// Write success cannot establish state; notifications and reads do.
	SELFTEST_ASSERT(CMD_ExecuteCommand("MIOTSET 2 1 true", 0) == CMD_RES_OK);
	MIoT_TestReceive("get_down\rresult 2 1 0\r");
	MIoT_TestExpect("down set_properties 2 1 true\rok\r");
	SELFTEST_ASSERT_CHANNEL(1, 0);
	MIoT_TestReceive("properties_changed 2 1 true 2 5 37 99 1 7\r");
	MIoT_TestExpect("ok\r");
	SELFTEST_ASSERT_CHANNEL(1, 1);
	SELFTEST_ASSERT_CHANNEL(2, 37);
	MIoT_TestReceive("get_down\rresult 2 1 0 true\rget_down\r");
	MIoT_TestExpect("down get_properties 2 1\rok\rdown none\r");
	// No echo writes from received channel state.
	MIoT_TestReceive("get_down\r");
	MIoT_TestExpect("down none\r");
	// Atomic validation: malformed tuples never partly publish state.
	MIoT_TestReceive("properties_changed 2 1 false 2 5 garbage\r");
	MIoT_TestExpect("error\r");
	SELFTEST_ASSERT_CHANNEL(1, 1);
	SELFTEST_ASSERT_CHANNEL(2, 37);
	MIoT_TestReceive("properties_changed 2 1 false 2 5\r");
	MIoT_TestExpect("error\r");
	SELFTEST_ASSERT_CHANNEL(1, 1);
	MIoT_TestReceive("properties_changed 2 1 trueish\r");
	MIoT_TestExpect("error\r");
	// Unsolicited/stale results cannot change state.
	MIoT_TestReceive("result 2 1 0 false\r");
	MIoT_TestExpect("error\r");
	SELFTEST_ASSERT_CHANNEL(1, 1);
	// A silently ignored write still gets a readback at the next poll.
	CMD_ExecuteCommand("miotSet 2 5 100", 0);
	MIoT_TestReceive("get_down\rget_down\r");
	MIoT_TestExpect("down set_properties 2 5 100\rdown get_properties 2 5\r");
	MIoT_TestReceive("result 2 5 0 37\rget_down\r");
	MIoT_TestExpect("ok\rdown none\r");
	SELFTEST_ASSERT_CHANNEL(2, 37);
	// Property errors omit read values. Write errors can have side effects.
	CMD_ExecuteCommand("miotSet 2 5 100", 0);
	MIoT_TestReceive("get_down\rresult 2 5 -4005\rget_down\rresult 2 5 -4003\rget_down\r");
	MIoT_TestExpect("down set_properties 2 5 100\rok\rdown get_properties 2 5\rok\rdown none\r");
	// Missing replies time out without replaying a write.
	CMD_ExecuteCommand("miotSet 2 5 100", 0);
	MIoT_TestReceive("get_down\r");
	MIoT_TestExpect("down set_properties 2 5 100\r");
	g_timeMs += 3001;
	MIoT_RunEverySecond();
	MIoT_TestReceive("get_down\rresult 2 5 0 37\rget_down\r");
	MIoT_TestExpect("down get_properties 2 5\rok\rdown none\r");
	// Partial lines remain buffered, while incomplete timed-out frames are
	// discarded through the next CR, rather than executing a valid-looking tail.
	MIoT_TestReceive("properties_changed 2 1 fal");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	MIoT_TestReceive("se\r\n");
	MIoT_TestExpect("ok\r");
	SELFTEST_ASSERT_CHANNEL(1, 0);
	MIoT_TestReceive("properties_changed 2 1 ");
	g_timeMs += 3001;
	MIoT_TestReceive("true\rget_down\r");
	MIoT_TestExpect("error\rdown none\r");
	SELFTEST_ASSERT_CHANNEL(1, 0);
	// Oversize and non-ASCII/control data recover only at a frame boundary.
	for (i = 0; i < 600; i++)
		UART_AppendByteToReceiveRingBuffer('x');
	MIoT_TestReceive("\rget_down\r");
	MIoT_TestExpect("error\rdown none\r");
	MIoT_TestReceive("properties_changed 2 1 \001true\rget_down\r");
	MIoT_TestExpect("error\rdown none\r");
	SELFTEST_ASSERT_CHANNEL(1, 0);
	// Backpressure is bounded and preserves already queued work.
	for (i = 0; i < 16; i++)
		SELFTEST_ASSERT(CMD_ExecuteCommand("miotSet 2 5 37", 0) == CMD_RES_OK);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotSet 2 5 100", 0) == CMD_RES_ERROR);
	// MCU restart drops pre-reset writes and refreshes state.
	MIoT_TestReceive("model xiaomi.fan.p45\rget_down\r");
	MIoT_TestExpect("ok\rdown get_properties 2 1\r");
	MIoT_TestReceive("result 2 1 0 false\rget_down\rresult 2 5 0 1\rget_down\rresult 2 2 0 0\rget_down\r");
	MIoT_TestExpect("ok\rdown get_properties 2 5\rok\rdown get_properties 2 2\rok\rdown none\r");
	SELFTEST_ASSERT_CHANNEL(2, 1);
	// Lifecycle shutdown and UART ownership changes must not consume another
	// driver's bytes or emit replies on its UART.
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotMap 2 7 4 int 30 150 1 30", 0) == CMD_RES_OK);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotSet 2 7 45", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotSet 2 7 60", 0) == CMD_RES_OK);
	UART_InitUART(9600, 0, false);
	MIoT_TestReceive("get_down\r");
	SELFTEST_ASSERT_HAS_UART_EMPTY();
	SELFTEST_ASSERT(UART_GetDataSize() == 9);
	CMD_ExecuteCommand("stopDriver MIoT", 0);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotSet 2 5 37", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT_HAS_UART_EMPTY();
}
#endif
#endif
