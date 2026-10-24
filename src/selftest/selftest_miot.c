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

static void MIoT_TestSustainedPolling(void) {
	int i, piid, value, writes = 0;
	unsigned int reads = 0;
	char text[128];
	size_t length;
	SIM_ClearOBK(0);
	SIM_UART_InitReceiveRingBuffer(8192);
	CMD_ExecuteCommand("startDriver MIoT", 0);
	for (i = 1; i <= 16; i++)
		SELFTEST_ASSERT(CMD_ExecuteCommand(va("miotMap 2 %i %i int 0 100 1", i, i), 0) == CMD_RES_OK);
	MIoT_TestReceive("model test\r");
	MIoT_TestExpect("ok\r");
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotSet 2 1 37", 0) == CMD_RES_OK);
	for (i = 0; i < 40; i++) {
		g_timeMs += 1000;
		MIoT_RunEverySecond();
		MIoT_TestReceive("get_down\r");
		length = 0;
		while (SIM_UART_GetDataSize() && length < sizeof(text) - 1) {
			text[length++] = SIM_UART_GetByte(0);
			SIM_UART_ConsumeBytes(1);
		}
		text[length] = 0;
		if (sscanf(text, "down get_properties 2 %i", &piid) == 1) {
			SELFTEST_ASSERT(piid >= 1 && piid <= 16);
			if (piid < 1 || piid > 16)
				return;
			reads |= 1U << (piid - 1);
			MIoT_TestReceive(va("result 2 %i 0 %i\r", piid, piid == 1 && writes ? 37 : 0));
			MIoT_TestExpect("ok\r");
		} else if (sscanf(text, "down set_properties 2 %i %i", &piid, &value) == 2) {
			SELFTEST_ASSERT(piid == 1 && value == 37);
			writes++;
			MIoT_TestReceive("result 2 1 0\r");
			MIoT_TestExpect("ok\r");
		} else {
			SELFTEST_ASSERT_STRING(text, "down none\r");
		}
	}
	SELFTEST_ASSERT(reads == 0xFFFFU);
	SELFTEST_ASSERT(writes == 1);
	SELFTEST_ASSERT_CHANNEL(1, 37);
}

static void MIoT_TestRetainedConfig(const mainConfig_t *before) {
	mainConfig_t after = g_cfg;
	// Only the credential fields and save bookkeeping may differ.
	memcpy(after.wifi_ssid, before->wifi_ssid, sizeof(after.wifi_ssid));
	memcpy(after.wifi_pass, before->wifi_pass, sizeof(after.wifi_pass));
#if ALLOW_SSID2
	memcpy(after.wifi_ssid2, before->wifi_ssid2, sizeof(after.wifi_ssid2));
	memcpy(after.wifi_pass2, before->wifi_pass2, sizeof(after.wifi_pass2));
#endif
	after.crc = before->crc;
	after.changeCounter = before->changeCounter;
	SELFTEST_ASSERT(!memcmp(&after, before, sizeof(after)));
}

static void MIoT_TestNetworkReset(void) {
	mainConfig_t before;
	const char *startup = "startDriver MIoT\nmiotMap 2 5 2 int 1 100 1\nmiotAllowNetworkReset 1\n";
	MIoT_TestSetup();
	CMD_ExecuteCommand("lfs_format", 0);
	Test_FakeHTTPClientPacket_POST("api/lfs/autoexec.bat", startup);
	CFG_SetShortStartupCommand("echo retained-startup");
	CFG_SetWiFiSSID("miot-test-primary");
	CFG_SetWiFiPass("primary-test-password");
	CFG_SetWiFiSSID2("miot-test-alternate");
	CFG_SetWiFiPass2("alternate-test-password");
	CFG_SetMQTTHost("mqtt.example.invalid");
	CFG_Save_IfThereArePendingChanges();
	before = g_cfg;
	MIoT_TestReceive("restore\r");
	MIoT_TestExpect("error\r");
	SELFTEST_ASSERT(!memcmp(&g_cfg, &before, sizeof(before)));
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotAllowNetworkReset", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotAllowNetworkReset -1", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotAllowNetworkReset 2", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotAllowNetworkReset true", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotAllowNetworkReset 1 extra", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("MIOTALLOWNETWORKRESET 1", 0) == CMD_RES_OK);
	MIoT_TestReceive("restore extra\rRESTORE\rrestored\r");
	MIoT_TestExpect("error\rerror\rerror\r");
	SELFTEST_ASSERT(!memcmp(&g_cfg, &before, sizeof(before)));
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotAllowNetworkReset 0", 0) == CMD_RES_OK);
	MIoT_TestReceive("restore\r");
	MIoT_TestExpect("error\r");
	SELFTEST_ASSERT(!memcmp(&g_cfg, &before, sizeof(before)));
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotAllowNetworkReset 1", 0) == CMD_RES_OK);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotSet 2 5 37", 0) == CMD_RES_OK);
	MIoT_TestReceive("restore\r");
	MIoT_TestExpect("ok\r");
	SELFTEST_ASSERT_STRING(CFG_GetWiFiSSID(), "");
	SELFTEST_ASSERT_STRING(CFG_GetWiFiPass(), "");
	SELFTEST_ASSERT_STRING(CFG_GetWiFiSSID2(), "");
	SELFTEST_ASSERT_STRING(CFG_GetWiFiPass2(), "");
	MIoT_TestRetainedConfig(&before);
	// No queued or new control write may replay while a network reset is pending.
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotSet 2 5 37", 0) == CMD_RES_ERROR);
	MIoT_TestReceive("model xiaomi.fan.p45\rget_down\r");
	MIoT_TestExpect("ok\rdown get_properties 2 1\r");
	// Duplicate requests acknowledge without clearing edits or postponing reboot.
	CFG_SetWiFiSSID("edited-after-request");
	MIoT_TestReceive("restore\rrestore\r");
	MIoT_TestExpect("ok\rok\r");
	SELFTEST_ASSERT_STRING(CFG_GetWiFiSSID(), "edited-after-request");
	CFG_SetWiFiSSID("");
	Main_OnEverySecond();
	MIoT_TestReceive("restore\r");
	MIoT_TestExpect("ok\r");
	Main_OnEverySecond();
	MIoT_TestReceive("restore\r");
	MIoT_TestExpect("ok\r");
	Main_OnEverySecond();
	// Reload through the configuration CRC/storage path, not from driver state.
	memset(&g_cfg, 0, sizeof(g_cfg));
	CFG_InitAndLoad();
	SELFTEST_ASSERT_STRING(CFG_GetWiFiSSID(), "");
	SELFTEST_ASSERT_STRING(CFG_GetWiFiPass(), "");
	SELFTEST_ASSERT_STRING(CFG_GetWiFiSSID2(), "");
	SELFTEST_ASSERT_STRING(CFG_GetWiFiPass2(), "");
	MIoT_TestRetainedConfig(&before);
	Test_FakeHTTPClientPacket_GET("api/lfs/autoexec.bat");
	SELFTEST_ASSERT_HTML_REPLY(startup);
	CMD_ExecuteCommand("lfs_remove autoexec.bat", 0);
	CFG_SetShortStartupCommand("");
	CFG_Save_IfThereArePendingChanges();
	CMD_ExecuteCommand("stopDriver MIoT", 0);
	CMD_ExecuteCommand("startDriver MIoT", 0);
	MIoT_TestReceive("restore\r");
	MIoT_TestExpect("error\r");
	CMD_ExecuteCommand("stopDriver MIoT", 0);
}

static void MIoT_TestFanDiscovery(void) {
	const char *snapshot = "{\"power\":false,\"speed\":1,\"oscillation\":false,\"angle\":90}";
	char name[64], topic[128], uniqueId[75];
	SIM_ClearOBK(0);
	SIM_ClearAndPrepareForMQTTTesting("miot-test", "bekens");
	CFG_SetDeviceName("miot-fan");
	SIM_UART_InitReceiveRingBuffer(8192);
	SELFTEST_ASSERT(CMD_ExecuteCommand("startDriver MIoT", 0) == CMD_RES_OK);
	CMD_ExecuteCommand("miotMap 2 1 1 bool 0 1 1", 0);
	CMD_ExecuteCommand("miotMap 2 5 2 int 1 100 1", 0);
	CMD_ExecuteCommand("miotMap 2 6 3 bool 0 1 1", 0);
	CMD_ExecuteCommand("miotMap 2 7 4 int 30 150 1 30", 0);
	CMD_ExecuteCommand("miotMap 99 1 5 int 1 100 0", 0);
	CMD_ExecuteCommand("miotMap 99 2 6 int 0 100 1", 0);
	CMD_ExecuteCommand("miotMap 99 3 7 int 0 1000 1", 0);
	CMD_ExecuteCommand("miotMap 99 4 8 int 1 1 1", 0);
	CMD_ExecuteCommand("miotMap 99 5 9 bool 0 1 1 2", 0);
	CMD_ExecuteCommand("miotMap 99 6 10 bool 0 1 1 2", 0);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotFan", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotFan 1 2 3", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotFan 1 2 1 4", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotFan 1 2 3 8", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotFan 1 5 3 4", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotFan 1 6 3 4", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotFan 1 2 3 7", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotFan 2 1 3 4", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotFan 1 2 3 4 extra", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotFan 1 2 3 4", 0) == CMD_RES_OK);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotFan 1 2 2 4", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotFan 1 8 3 4", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotFan 9 2 3 4", 0) == CMD_RES_BAD_ARGUMENT);
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotFan 1 2 10 4", 0) == CMD_RES_BAD_ARGUMENT);
	SIM_ClearMQTTHistory();
	MIoT_OnHassDiscovery("homeassistant");
	Sim_RunSeconds(4, false);
	SELFTEST_ASSERT_HAS_MQTT_JSON_SENT("homeassistant/fan/miot-fan_fan/config", false);
	SELFTEST_ASSERT_JSON_VALUE_STRING(NULL, "stat_t", "~/miot/fan/get");
	SELFTEST_ASSERT_JSON_VALUE_STRING(NULL, "cmd_t", "cmnd/miot-test/miotSet");
	SELFTEST_ASSERT_JSON_VALUE_STRING(NULL, "avty_t", "~/connected");
	SELFTEST_ASSERT_HAS_MQTT_JSON_SENT("homeassistant/select/miot-fan_select_fan/config", false);
	SELFTEST_ASSERT_JSON_VALUE_STRING(NULL, "stat_t", "~/miot/fan/get");
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForString("miot-test/miot/fan/get", snapshot, true));
	MIoT_TestReceive("properties_changed 2 1 false 2 5 1 2 6 false 2 7 90\r");
	MIoT_TestExpect("ok\r");
	Sim_RunSeconds(2, false);
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_STR("miot-test/miot/fan/get", snapshot, true);
	SIM_ClearMQTTHistory();
	CMD_ExecuteCommand("setChannel 2 37", 0);
	CMD_ExecuteCommand("setChannel 3 1", 0);
	Sim_RunSeconds(2, false);
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForString("miot-test/miot/fan/get", "{\"power\":false,\"speed\":37,\"oscillation\":true,\"angle\":90}", true));
	MIoT_TestReceive("properties_changed 2 1 false 2 5 1 2 6 false 2 7 90\r");
	MIoT_TestExpect("ok\r");
	Sim_RunSeconds(2, false);
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_STR("miot-test/miot/fan/get", snapshot, true);
	SIM_ClearMQTTHistory();
	SELFTEST_ASSERT(CMD_ExecuteCommand("miotFan off", 0) == CMD_RES_OK);
	Sim_RunSeconds(2, false);
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_STR("homeassistant/fan/miot-fan_fan/config", "", true);
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_STR("homeassistant/select/miot-fan_select_fan/config", "", true);
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_STR("miot-test/miot/fan/get", "", true);
	// Pending discovery cannot resurrect an entity after removal.
	CMD_ExecuteCommand("miotFan 1 2 3 4", 0);
	MIoT_OnHassDiscovery("homeassistant");
	CMD_ExecuteCommand("miotFan off", 0);
	SIM_ClearMQTTHistory();
	Sim_RunSeconds(4, false);
	SELFTEST_ASSERT_HAS_NOT_MQTT_JSON_SENT_ANY("homeassistant/fan/miot-fan_fan/config", false, NULL, NULL, "cmd_t", "cmnd/miot-test/miotSet");
	// Re-enabling before tombstones drain must preserve the new config and state.
	CMD_ExecuteCommand("miotFan 1 2 3 4", 0);
	MIoT_OnHassDiscovery("homeassistant");
	CMD_ExecuteCommand("miotFan off", 0);
	CMD_ExecuteCommand("miotFan 1 2 3 4", 0);
	MIoT_OnHassDiscovery("homeassistant");
	SIM_ClearMQTTHistory();
	Sim_RunSeconds(4, false);
	SELFTEST_ASSERT_HAS_MQTT_JSON_SENT("homeassistant/fan/miot-fan_fan/config", false);
	SELFTEST_ASSERT(!SIM_CheckMQTTHistoryForString("miot-test/miot/fan/get", "", true));
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_STR("miot-test/miot/fan/get", snapshot, true);
	CMD_ExecuteCommand("miotFan off", 0);
	Sim_RunSeconds(4, false);
	// The longest valid device name must fit both identifiers, including the suffix.
	memset(name, 'x', sizeof(name) - 1);
	name[sizeof(name) - 1] = 0;
	CFG_SetDeviceName(name);
	CMD_ExecuteCommand("miotFan 1 2 3 4", 0);
	MIoT_OnHassDiscovery("homeassistant");
	SIM_ClearMQTTHistory();
	Sim_RunSeconds(4, false);
	snprintf(uniqueId, sizeof(uniqueId), "%s_select_fan", name);
	snprintf(topic, sizeof(topic), "homeassistant/select/%s/config", uniqueId);
	SELFTEST_ASSERT_HAS_MQTT_JSON_SENT(topic, false);
	SELFTEST_ASSERT_JSON_VALUE_STRING(NULL, "uniq_id", uniqueId);
	CMD_ExecuteCommand("miotFan off", 0);
	Sim_RunSeconds(4, false);
	SELFTEST_ASSERT_HAD_MQTT_PUBLISH_STR(topic, "", true);
	CMD_ExecuteCommand("stopDriver MIoT", 0);
}

void Test_MIoT(void) {
	int i;
	MIoT_TestCallbacks();
	MIoT_TestSustainedPolling();
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
	MIoT_TestNetworkReset();
	MIoT_TestFanDiscovery();
}
#endif
#endif
