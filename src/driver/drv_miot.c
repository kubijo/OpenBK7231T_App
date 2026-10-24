// Xiaomi module-side MIoT serial transport. Lines are ASCII, terminated by CR.
#include "drv_miot.h"
#include "../cmnds/cmd_public.h"
#include "../hal/hal_wifi.h"
#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_common.h"
#include "../new_pins.h"
#include "../quicktick.h"
#include "drv_ntp.h"
#include "drv_public.h"
#include "drv_uart.h"
#include <errno.h>
#include <limits.h>
#if ENABLE_HA_DISCOVERY && ENABLE_MQTT
#include "../httpserver/hass.h"
#endif

#if ENABLE_DRIVER_MIOT

#define MIOT_PROPERTIES 16
#define MIOT_QUEUE 16
#define MIOT_LINE 512
#define MIOT_TOKENS 128
#define MIOT_TIMEOUT 3000U

typedef struct {
	int siid, piid, channel, min, max, step;
	bool boolean, writable, known, read, reconcile;
	int value;
} miotProperty_t;

typedef struct {
	int property, value;
} miotWrite_t;

typedef enum { MIOT_IDLE,
			   MIOT_WRITE,
			   MIOT_READ } miotPending_t;

static miotProperty_t properties[MIOT_PROPERTIES];
static int propertyCount;
static miotWrite_t writes[MIOT_QUEUE];
static int queueHead, queueCount;
static miotPending_t pending;
static int pendingProperty;
static int readbackProperty = -1;
static int readCursor;
static bool preferWrite;
static unsigned int pendingSince, lastByte;
static int uartGeneration;
static int uartPort;
static unsigned int sessionGeneration;
static bool running, dropping;
static bool allowNetworkReset, networkResetRequested;
static int applyingChannel = -1;
static int applyingValue;
static char line[MIOT_LINE + 1];
static size_t lineLength;
static unsigned int pollSeconds;
static bool MIoT_Int(const char *text, int *value);

#if ENABLE_HA_DISCOVERY && ENABLE_MQTT
static int fanProperties[4];
static bool fanEnabled, fanDirty, fanMqttReady;
static char fanDiscoveryTopic[64] = "homeassistant";

static void MIoT_QueueFanDiscovery(const char *topic, const char *channel, const char *value) {
	if (MQTT_CancelQueuedPublish(topic, channel))
		MQTT_QueuePublish(topic, channel, value, OBK_PUBLISH_FLAG_RETAIN);
}

static void MIoT_ClearFan(void) {
	int i;
	for (i = 0; i < 2; i++) {
		HassDeviceInfo *info = hass_init_device_info(i ? HASS_SELECT : HASS_FAN, 0, NULL, NULL, 0, i ? "fan" : NULL);
		if (!info)
			continue;
		if (i)
			snprintf(info->channel, sizeof(info->channel), "select/%s/config", info->unique_id);
		MIoT_QueueFanDiscovery(fanDiscoveryTopic, info->channel, "");
		hass_free_device_info(info);
	}
	MIoT_QueueFanDiscovery(CFG_GetMQTTClientId(), "miot/fan/get", "");
}

static void MIoT_PublishFanState(void) {
	char state[128];
	int i;
	if (!fanEnabled || !fanDirty || !MQTT_IsReady())
		return;
	for (i = 0; i < 4; i++)
		if (!properties[fanProperties[i]].known)
			return;
	snprintf(state, sizeof(state), "{\"power\":%s,\"speed\":%i,\"oscillation\":%s,\"angle\":%i}",
			 properties[fanProperties[0]].value ? "true" : "false", properties[fanProperties[1]].value,
			 properties[fanProperties[2]].value ? "true" : "false", properties[fanProperties[3]].value);
	// Publish the current snapshot directly: reusable queue slots can reorder states.
	if (MQTT_Publish(CFG_GetMQTTClientId(), "miot/fan/get", state, OBK_PUBLISH_FLAG_RETAIN) == OBK_PUBLISH_OK)
		fanDirty = false;
}

static commandResult_t MIoT_FanCommand(char **tokens, int count) {
	int indices[4], i, j, channel;
	miotProperty_t *speed, *angle;
	int64_t options;
	if (count == 1 && !strcmp(tokens[0], "off")) {
		if (fanEnabled)
			MIoT_ClearFan();
		fanEnabled = false;
		return CMD_RES_OK;
	}
	if (count != 4)
		return CMD_RES_BAD_ARGUMENT;
	for (i = 0; i < 4; i++) {
		indices[i] = -1;
		if (!MIoT_Int(tokens[i], &channel))
			return CMD_RES_BAD_ARGUMENT;
		for (j = 0; j < propertyCount; j++)
			if (properties[j].channel == channel && properties[j].writable)
				indices[i] = j;
		if (indices[i] < 0)
			return CMD_RES_BAD_ARGUMENT;
		for (j = 0; j < i; j++)
			if (indices[i] == indices[j])
				return CMD_RES_BAD_ARGUMENT;
	}
	speed = &properties[indices[1]];
	angle = &properties[indices[3]];
	options = ((int64_t)angle->max - angle->min) / angle->step + 1;
	if (!properties[indices[0]].boolean || properties[indices[0]].step != 1 || !properties[indices[2]].boolean || properties[indices[2]].step != 1 || speed->boolean || speed->min < 1 || speed->max <= speed->min || speed->step != 1 || angle->boolean || options < 2 || options > 16)
		return CMD_RES_BAD_ARGUMENT;
	if (!MQTT_CancelQueuedPublish(CFG_GetMQTTClientId(), "miot/fan/get"))
		return CMD_RES_ERROR;
	memcpy(fanProperties, indices, sizeof(indices));
	fanEnabled = fanDirty = true;
	CMD_ExecuteCommand("scheduleHADiscovery 1", 0);
	return CMD_RES_OK;
}
#endif

static bool MIoT_IsCurrent(unsigned int generation) {
	return running && sessionGeneration == generation && uartPort == UART_GetSelectedPortIndex() && uartGeneration == get_g_uart_init_counter();
}

static bool MIoT_Int(const char *text, int *value) {
	char *end;
	long number;
	if (!text || !*text || (*text != '-' && !isdigit((unsigned char)*text)))
		return false;
	errno = 0;
	number = strtol(text, &end, 10);
	if (*end || errno == ERANGE || number < INT_MIN || number > INT_MAX)
		return false;
	*value = (int)number;
	return true;
}

// A bounded tokenizer is independent of the global command tokenizer, including
// when channel updates invoke scripts while a UART message is being processed.
static int MIoT_Tokenize(char *text, char **tokens, int capacity) {
	int count = 0;
	while (*text) {
		while (*text == ' ' || *text == '\t')
			text++;
		if (!*text)
			break;
		if (count == capacity)
			return -1;
		tokens[count++] = text;
		while (*text && *text != ' ' && *text != '\t')
			text++;
		if (*text)
			*text++ = 0;
	}
	return count;
}

static int MIoT_Find(int siid, int piid) {
	int i;
	for (i = 0; i < propertyCount; i++)
		if (properties[i].siid == siid && properties[i].piid == piid)
			return i;
	return -1;
}

static void MIoT_Send(const char *text) {
	unsigned int generation = sessionGeneration;
	while (*text && MIoT_IsCurrent(generation))
		UART_SendByte((byte)*text++);
	if (MIoT_IsCurrent(generation))
		UART_SendByte('\r');
}

static void MIoT_ReadAll(void) {
	int i;
	for (i = 0; i < propertyCount; i++)
		properties[i].read = true;
}

static void MIoT_ResetSession(void) {
	int i;
	queueHead = queueCount = 0;
	pending = MIOT_IDLE;
	readbackProperty = -1;
	readCursor = 0;
	preferWrite = false;
	for (i = 0; i < propertyCount; i++) {
		properties[i].known = false;
		properties[i].reconcile = false;
	}
	MIoT_ReadAll();
}

static bool MIoT_Value(int index, const char *text, int *value) {
	miotProperty_t *p = &properties[index];
	if (p->boolean) {
		if (!strcmp(text, "true"))
			*value = 1;
		else if (!strcmp(text, "false"))
			*value = 0;
		else
			return false;
	} else if (!MIoT_Int(text, value))
		return false;
	return *value >= p->min && *value <= p->max;
}

static void MIoT_Apply(int index, int value) {
	miotProperty_t *p = &properties[index];
	int previous = applyingChannel;
	int previousValue = applyingValue;
	unsigned int generation = sessionGeneration;
	p->known = true;
	p->value = value;
	p->reconcile = false;
#if ENABLE_HA_DISCOVERY && ENABLE_MQTT
	fanDirty = true;
#endif
	applyingChannel = p->channel;
	applyingValue = value;
	CHANNEL_Set(p->channel, value, 0);
	if (sessionGeneration == generation) {
		applyingChannel = previous;
		applyingValue = previousValue;
	}
}

static bool MIoT_QueueWrite(int index, int value) {
	miotProperty_t *p = &properties[index];
	int slot;
	if (networkResetRequested || !p->writable || value < p->min || value > p->max || ((int64_t)value - p->min) % p->step || queueCount == MIOT_QUEUE)
		return false;
	slot = (queueHead + queueCount) % MIOT_QUEUE;
	writes[slot].property = index;
	writes[slot].value = value;
	queueCount++;
	return true;
}

static void MIoT_FinishRequest(void) {
	if (pending == MIOT_WRITE)
		readbackProperty = pendingProperty;
	pending = MIOT_IDLE;
}

static void MIoT_RequestRead(int index) {
	char reply[96];
	properties[index].read = false;
	pending = MIOT_READ;
	pendingProperty = index;
	pendingSince = g_timeMs;
	snprintf(reply, sizeof(reply), "down get_properties %i %i", properties[index].siid, properties[index].piid);
	MIoT_Send(reply);
}

// Finish silently ignored requests without replaying writes. Readback stays
// urgent; background reads rotate and share polls with queued writes.
static void MIoT_Poll(void) {
	char reply[96];
	int i, offset, index, value;
	miotProperty_t *p;
	MIoT_FinishRequest();
	if (readbackProperty >= 0) {
		index = readbackProperty;
		readbackProperty = -1;
		MIoT_RequestRead(index);
		return;
	}
	if (!queueCount || !preferWrite) {
		for (offset = 0; offset < propertyCount; offset++) {
			i = (readCursor + offset) % propertyCount;
			if (properties[i].read) {
				readCursor = (i + 1) % propertyCount;
				preferWrite = true;
				MIoT_RequestRead(i);
				return;
			}
		}
	}
	if (queueCount) {
		index = writes[queueHead].property;
		value = writes[queueHead].value;
		queueHead = (queueHead + 1) % MIOT_QUEUE;
		queueCount--;
		preferWrite = false;
		p = &properties[index];
		pending = MIOT_WRITE;
		pendingProperty = index;
		pendingSince = g_timeMs;
		if (p->boolean)
			snprintf(reply, sizeof(reply), "down set_properties %i %i %s", p->siid, p->piid, value ? "true" : "false");
		else
			snprintf(reply, sizeof(reply), "down set_properties %i %i %i", p->siid, p->piid, value);
		MIoT_Send(reply);
		return;
	}
	MIoT_Send("down none");
}

static bool MIoT_Properties(char **tokens, int count, bool result) {
	unsigned int generation = sessionGeneration;
	int i, siid, piid, code, index, value;
	int indices[MIOT_PROPERTIES], values[MIOT_PROPERTIES], found = 0;
	int stride = 3;
	if (result) {
		// We issue one property per request. Failed reads omit the value token.
		if (pending == MIOT_IDLE || count < 4 || !MIoT_Int(tokens[3], &code))
			return false;
		stride = pending == MIOT_READ && code == 0 ? 4 : 3;
		if (count != 1 + stride)
			return false;
	}
	if (count < 1 + stride || (count - 1) % stride)
		return false;
	// Validate the whole frame before publishing any of it.
	for (i = 1; i < count; i += stride) {
		if (!MIoT_Int(tokens[i], &siid) || !MIoT_Int(tokens[i + 1], &piid) || siid <= 0 || piid <= 0)
			return false;
		index = MIoT_Find(siid, piid);
		if (result) {
			if (index != pendingProperty || !MIoT_Int(tokens[i + 2], &code))
				return false;
			if (code) {
				addLogAdv(LOG_WARN, LOG_FEATURE_DRV, "MIoT property %i.%i error %i", siid, piid, code);
				continue;
			}
			if (pending == MIOT_WRITE)
				continue; // write success is not state evidence
		}
		if (index < 0)
			continue; // unconfigured notifications are acknowledged
		if (!MIoT_Value(index, tokens[i + stride - 1], &value) || found == MIOT_PROPERTIES)
			return false;
		indices[found] = index;
		values[found++] = value;
	}
	for (i = 0; i < found; i++) {
		MIoT_Apply(indices[i], values[i]);
		if (!MIoT_IsCurrent(generation))
			return false;
	}
	return true;
}

static void MIoT_Process(void) {
	unsigned int generation = sessionGeneration;
	char *tokens[MIOT_TOKENS];
	char reply[64];
	int count = MIoT_Tokenize(line, tokens, MIOT_TOKENS);
	if (count <= 0) {
		MIoT_Send("error");
		return;
	}
	if (!strcmp(tokens[0], "get_down") && count == 1)
		MIoT_Poll();
	else if (!strcmp(tokens[0], "model") && count == 2) {
		MIoT_ResetSession();
		addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "MIoT model %s", tokens[1]);
		MIoT_Send("ok");
	} else if (!strcmp(tokens[0], "properties_changed")) {
		bool valid = MIoT_Properties(tokens, count, false);
		if (MIoT_IsCurrent(generation))
			MIoT_Send(valid ? "ok" : "error");
	} else if (!strcmp(tokens[0], "result")) {
		bool valid = MIoT_Properties(tokens, count, true);
		if (!MIoT_IsCurrent(generation))
			return;
		MIoT_FinishRequest();
		MIoT_Send(valid ? "ok" : "error");
	} else if (!strcmp(tokens[0], "net") && count == 1) {
		MIoT_Send(Main_IsConnectedToWiFi() ? "local" : "offline");
	} else if (!strcmp(tokens[0], "restore") && count == 1 && allowNetworkReset) {
		MIoT_Send("ok");
		if (!MIoT_IsCurrent(generation))
			return;
		if (!networkResetRequested) {
			networkResetRequested = true;
			MIoT_ResetSession();
			CFG_SetWiFiSSID("");
			CFG_SetWiFiPass("");
			CFG_SetWiFiSSID2("");
			CFG_SetWiFiPass2("");
			CFG_Save_SetupTimer();
			RESET_ScheduleModuleReset(3);
		}
	} else if (!strcmp(tokens[0], "mac") && count == 1) {
		char mac[32];
		MIoT_Send(HAL_GetMACStr(mac));
	} else if (!strcmp(tokens[0], "time")) {
		// A zero reply explicitly means the time source is not ready.
#if ENABLE_NTP
		if (NTP_IsTimeSynced() && count == 2 && !strcmp(tokens[1], "posix")) {
			snprintf(reply, sizeof(reply), "%u", NTP_GetCurrentTimeWithoutOffset());
			MIoT_Send(reply);
		} else
#endif
			MIoT_Send("0");
	} else if ((!strcmp(tokens[0], "mcu_version") && count == 2) || (!strcmp(tokens[0], "ble_config") && count >= 2) || (!strcmp(tokens[0], "set_mcu_auto_ota") && count == 2) || !strcmp(tokens[0], "event_occured") || !strcmp(tokens[0], "error")) {
		MIoT_Send("ok");
	} else {
		// Factory erase and OTA are not exposed by this transport. Network reset
		// requires an explicit opt-in and preserves all other configuration.
		MIoT_Send("error");
	}
}

void MIoT_RunFrame(void) {
	int c, i;
	unsigned int generation = sessionGeneration;
	if (!MIoT_IsCurrent(generation))
		return;
	// Reconcile after Channel_OnChanged has finished its GPIO and event work.
	for (i = 0; i < propertyCount; i++) {
		if (properties[i].reconcile)
			MIoT_Apply(i, properties[i].value);
		if (!MIoT_IsCurrent(generation))
			return;
	}
	if (lineLength && (unsigned int)(g_timeMs - lastByte) > MIOT_TIMEOUT) {
		lineLength = 0;
		dropping = true;
	}
	while (MIoT_IsCurrent(generation) && UART_GetDataSize()) {
		c = UART_GetByte(0);
		UART_ConsumeBytes(1);
		lastByte = g_timeMs;
		if (c == '\r') {
			line[lineLength] = 0;
			if (dropping)
				MIoT_Send("error");
			else
				MIoT_Process();
			if (!MIoT_IsCurrent(generation))
				return;
			lineLength = 0;
			dropping = false;
		} else if (c == '\n' && !lineLength && !dropping) {
			// tolerate CRLF without treating LF as another command
		} else if (c < 32 || c > 126 || lineLength == MIOT_LINE) {
			lineLength = 0;
			dropping = true;
		} else if (!dropping)
			line[lineLength++] = (char)c;
	}
}

void MIoT_OnHassDiscovery(const char *topic) {
#if ENABLE_HA_DISCOVERY && ENABLE_MQTT
	HassDeviceInfo *info;
	miotProperty_t *power, *speed, *oscillation, *angle;
	char command[CGF_MQTT_CLIENT_ID_SIZE + 24], text[256];
	int i, optionCount;
	cJSON *options;
	if (!fanEnabled || !MIoT_IsCurrent(sessionGeneration) || strlen(topic) >= sizeof(fanDiscoveryTopic))
		return;
	strcpy(fanDiscoveryTopic, topic);
	power = &properties[fanProperties[0]];
	speed = &properties[fanProperties[1]];
	oscillation = &properties[fanProperties[2]];
	angle = &properties[fanProperties[3]];
	snprintf(command, sizeof(command), "cmnd/%s/miotSet", CFG_GetMQTTClientId());
	info = hass_init_device_info(HASS_FAN, 0, "true", "false", 0, NULL);
	if (!info)
		return;
	cJSON_ReplaceItemInObject(info->root, "name", cJSON_CreateString("Fan"));
	cJSON_AddStringToObject(info->root, "cmd_t", command);
	snprintf(text, sizeof(text), "%i %i {{value}}", power->siid, power->piid);
	cJSON_AddStringToObject(info->root, "cmd_tpl", text);
	cJSON_AddStringToObject(info->root, "stat_t", "~/miot/fan/get");
	cJSON_AddStringToObject(info->root, "stat_val_tpl", "{{ 'true' if value_json.power else 'false' }}");
	cJSON_AddStringToObject(info->root, "pct_cmd_t", command);
	snprintf(text, sizeof(text), "{%% if value|int < %i %%}%i %i false{%% else %%}%i %i {{value}}{%% endif %%}", speed->min, power->siid, power->piid, speed->siid, speed->piid);
	cJSON_AddStringToObject(info->root, "pct_cmd_tpl", text);
	cJSON_AddStringToObject(info->root, "pct_stat_t", "~/miot/fan/get");
	snprintf(text, sizeof(text), "{{ value_json.speed if value_json.power else %i }}", speed->min - 1);
	cJSON_AddStringToObject(info->root, "pct_val_tpl", text);
	cJSON_AddNumberToObject(info->root, "spd_rng_min", speed->min);
	cJSON_AddNumberToObject(info->root, "spd_rng_max", speed->max);
	cJSON_AddStringToObject(info->root, "osc_cmd_t", command);
	snprintf(text, sizeof(text), "%i %i {{value}}", oscillation->siid, oscillation->piid);
	cJSON_AddStringToObject(info->root, "osc_cmd_tpl", text);
	cJSON_AddStringToObject(info->root, "osc_stat_t", "~/miot/fan/get");
	cJSON_AddStringToObject(info->root, "osc_val_tpl", "{{ 'true' if value_json.oscillation else 'false' }}");
	cJSON_AddStringToObject(info->root, "pl_osc_on", "true");
	cJSON_AddStringToObject(info->root, "pl_osc_off", "false");
	cJSON_AddBoolToObject(info->root, "ret", false);
	MIoT_QueueFanDiscovery(topic, info->channel, hass_build_discovery_json(info));
	hass_free_device_info(info);
	info = hass_init_device_info(HASS_SELECT, 0, NULL, NULL, 0, "fan");
	if (!info)
		return;
	snprintf(info->channel, sizeof(info->channel), "select/%s/config", info->unique_id);
	cJSON_ReplaceItemInObject(info->root, "name", cJSON_CreateString("Swing angle"));
	cJSON_AddStringToObject(info->root, "cmd_t", command);
	snprintf(text, sizeof(text), "%i %i {{value}}", angle->siid, angle->piid);
	cJSON_AddStringToObject(info->root, "cmd_tpl", text);
	cJSON_AddStringToObject(info->root, "stat_t", "~/miot/fan/get");
	cJSON_AddStringToObject(info->root, "val_tpl", "{{ value_json.angle }}");
	cJSON_AddBoolToObject(info->root, "ret", false);
	options = cJSON_AddArrayToObject(info->root, "ops");
	optionCount = (int)(((int64_t)angle->max - angle->min) / angle->step + 1);
	for (i = 0; i < optionCount; i++) {
		snprintf(text, sizeof(text), "%i", (int)((int64_t)angle->min + (int64_t)i * angle->step));
		cJSON_AddItemToArray(options, cJSON_CreateString(text));
	}
	MIoT_QueueFanDiscovery(topic, info->channel, hass_build_discovery_json(info));
	hass_free_device_info(info);
	fanDirty = true;
#else
	(void)topic;
#endif
}

void MIoT_RunEverySecond(void) {
	if (!MIoT_IsCurrent(sessionGeneration))
		return;
	if (pending != MIOT_IDLE && (unsigned int)(g_timeMs - pendingSince) >= MIOT_TIMEOUT) {
		if (pending == MIOT_READ)
			properties[pendingProperty].read = true;
		MIoT_FinishRequest();
	}
	if (++pollSeconds >= 10) {
		pollSeconds = 0;
		MIoT_ReadAll();
	}
#if ENABLE_HA_DISCOVERY && ENABLE_MQTT
	if (MQTT_IsReady() && !fanMqttReady && fanEnabled)
		CMD_ExecuteCommand("scheduleHADiscovery 1", 0);
	fanMqttReady = MQTT_IsReady();
	MIoT_PublishFanState();
#endif
}

void MIoT_OnChannelChanged(int channel, int value) {
	int i;
	if (!MIoT_IsCurrent(sessionGeneration))
		return;
	// Consume only the direct echo. Scripts run later in CHANNEL_Set and may
	// deliberately request a different value on this same channel.
	if (channel == applyingChannel && value == applyingValue) {
		applyingChannel = -1;
		return;
	}
	for (i = 0; i < propertyCount; i++) {
		if (properties[i].channel != channel)
			continue;
		if (!MIoT_QueueWrite(i, value)) {
			if (properties[i].known)
				properties[i].reconcile = true;
			else
				properties[i].read = true;
			addLogAdv(LOG_WARN, LOG_FEATURE_DRV, "MIoT rejected channel %i value %i", channel, value);
		}
		return;
	}
}

static commandResult_t MIoT_Command(const void *context, const char *cmd, const char *args, int flags) {
	char buffer[192], *tokens[9];
	int count, siid, piid, index, value;
	(void)context;
	(void)flags;
	if (!MIoT_IsCurrent(sessionGeneration) || strlen(args) >= sizeof(buffer))
		return CMD_RES_BAD_ARGUMENT;
	strcpy(buffer, args);
	count = MIoT_Tokenize(buffer, tokens, 9);
#if ENABLE_HA_DISCOVERY && ENABLE_MQTT
	if (!stricmp(cmd, "miotFan")) {
		commandResult_t result;
		// Reject reentrant calls rather than racing discovery, state publication or stop.
		if (!DRV_Mutex_Take(0))
			return CMD_RES_ERROR;
		result = MIoT_IsCurrent(sessionGeneration) ? MIoT_FanCommand(tokens, count) : CMD_RES_ERROR;
		DRV_Mutex_Free();
		return result;
	}
#endif
	if (!stricmp(cmd, "miotAllowNetworkReset")) {
		if (count != 1 || !MIoT_Int(tokens[0], &value) || (value != 0 && value != 1))
			return CMD_RES_BAD_ARGUMENT;
		allowNetworkReset = value != 0;
		return CMD_RES_OK;
	}
	if (!stricmp(cmd, "miotPoll")) {
		if (count)
			return CMD_RES_BAD_ARGUMENT;
		MIoT_ReadAll();
		return CMD_RES_OK;
	}
	if (count < 2 || !MIoT_Int(tokens[0], &siid) || !MIoT_Int(tokens[1], &piid) || siid <= 0 || piid <= 0)
		return CMD_RES_BAD_ARGUMENT;
	index = MIoT_Find(siid, piid);
	if (!stricmp(cmd, "miotMap")) {
		miotProperty_t p = {0};
		int i, writable;
		p.step = 1;
		if ((count != 7 && count != 8) || index >= 0 || propertyCount == MIOT_PROPERTIES || !MIoT_Int(tokens[2], &p.channel) || p.channel < 0 || p.channel >= CHANNEL_MAX || !MIoT_Int(tokens[4], &p.min) || !MIoT_Int(tokens[5], &p.max) || p.min > p.max || !MIoT_Int(tokens[6], &writable) || (writable != 0 && writable != 1) || (count == 8 && (!MIoT_Int(tokens[7], &p.step) || p.step <= 0)))
			return CMD_RES_BAD_ARGUMENT;
		if (!strcmp(tokens[3], "bool")) {
			p.boolean = true;
			if (p.min != 0 || p.max != 1)
				return CMD_RES_BAD_ARGUMENT;
		} else if (strcmp(tokens[3], "int"))
			return CMD_RES_BAD_ARGUMENT;
		for (i = 0; i < propertyCount; i++)
			if (properties[i].channel == p.channel)
				return CMD_RES_BAD_ARGUMENT;
		p.siid = siid;
		p.piid = piid;
		p.writable = writable;
		p.read = true;
		properties[propertyCount++] = p;
		return CMD_RES_OK;
	}
	if (index < 0)
		return CMD_RES_BAD_ARGUMENT;
	if (!stricmp(cmd, "miotRead") && count == 2) {
		properties[index].read = true;
		return CMD_RES_OK;
	}
	if (!stricmp(cmd, "miotSet") && count == 3 && MIoT_Value(index, tokens[2], &value)) {
		if (((int64_t)value - properties[index].min) % properties[index].step)
			return CMD_RES_BAD_ARGUMENT;
		return MIoT_QueueWrite(index, value) ? CMD_RES_OK : CMD_RES_ERROR;
	}
	return CMD_RES_BAD_ARGUMENT;
}

void MIoT_Init(void) {
	sessionGeneration++;
	propertyCount = 0;
	lineLength = 0;
	dropping = false;
	allowNetworkReset = networkResetRequested = false;
	applyingChannel = -1;
	pollSeconds = 0;
#if ENABLE_HA_DISCOVERY && ENABLE_MQTT
	fanEnabled = fanDirty = fanMqttReady = false;
	strcpy(fanDiscoveryTopic, "homeassistant");
#endif
	MIoT_ResetSession();
	UART_InitReceiveRingBuffer(2048);
	uartPort = UART_GetSelectedPortIndex();
	uartGeneration = UART_InitUART(115200, 0, false);
	running = true;
	// clang-format off
	//cmddetail:{"name":"miotMap","args":"SIID PIID Channel bool|int Min Max Writable [Step]",
	//cmddetail:"descr":"Map a MIoT property to an OpenBeken channel. Writable is 0 or 1; Step defaults to 1. Mappings are polled and refreshed after writes. See docs/MIoT.md.",
	//cmddetail:"fn":"MIoT_Command","file":"driver/drv_miot.c","requires":"MIoT","examples":"miotMap 2 5 2 int 1 100 1"}
	CMD_RegisterCommand("miotMap", MIoT_Command, NULL);
	//cmddetail:{"name":"miotSet","args":"SIID PIID Value",
	//cmddetail:"descr":"Queue a validated mapped-property write. Boolean values must be true or false. Writes are followed by readback.",
	//cmddetail:"fn":"MIoT_Command","file":"driver/drv_miot.c","requires":"MIoT","examples":"miotSet 2 1 true"}
	CMD_RegisterCommand("miotSet", MIoT_Command, NULL);
	//cmddetail:{"name":"miotRead","args":"SIID PIID",
	//cmddetail:"descr":"Schedule a mapped-property read on the next MCU poll.",
	//cmddetail:"fn":"MIoT_Command","file":"driver/drv_miot.c","requires":"MIoT","examples":"miotRead 2 5"}
	CMD_RegisterCommand("miotRead", MIoT_Command, NULL);
	//cmddetail:{"name":"miotPoll","args":"",
	//cmddetail:"descr":"Schedule fresh reads of all MIoT mappings.",
	//cmddetail:"fn":"MIoT_Command","file":"driver/drv_miot.c","requires":"MIoT","examples":"miotPoll"}
	CMD_RegisterCommand("miotPoll", MIoT_Command, NULL);
	//cmddetail:{"name":"miotAllowNetworkReset","args":"0|1",
	//cmddetail:"descr":"Allow MCU restore requests to clear both Wi-Fi networks and reboot into setup. Disabled on driver start; other configuration is preserved.",
	//cmddetail:"fn":"MIoT_Command","file":"driver/drv_miot.c","requires":"MIoT","examples":"miotAllowNetworkReset 1"}
	CMD_RegisterCommand("miotAllowNetworkReset", MIoT_Command, NULL);
#if ENABLE_HA_DISCOVERY && ENABLE_MQTT
	//cmddetail:{"name":"miotFan","args":"PowerChannel SpeedChannel OscillationChannel AngleChannel|off",
	//cmddetail:"descr":"Bind existing writable MIoT mappings to an MQTT fan and swing-angle select. Speed requires integer steps; angle permits 2 to 16 options. off removes discovery.",
	//cmddetail:"fn":"MIoT_Command","file":"driver/drv_miot.c","requires":"MIoT","examples":"miotFan 1 2 3 4"}
	CMD_RegisterCommand("miotFan", MIoT_Command, NULL);
#endif
	// clang-format on
}

void MIoT_Stop(void) {
#if ENABLE_HA_DISCOVERY && ENABLE_MQTT
	if (fanEnabled)
		MIoT_ClearFan();
	fanEnabled = false;
#endif
	running = false;
	sessionGeneration++;
	MIoT_ResetSession();
	propertyCount = 0;
	lineLength = 0;
}

#endif
