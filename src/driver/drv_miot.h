#pragma once

void MIoT_Init(void);
void MIoT_RunFrame(void);
void MIoT_RunEverySecond(void);
void MIoT_Stop(void);
void MIoT_OnChannelChanged(int channel, int value);
void MIoT_OnHassDiscovery(const char *topic);
