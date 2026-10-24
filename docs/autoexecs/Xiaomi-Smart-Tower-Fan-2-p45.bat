// Xiaomi Smart Tower Fan 2 (xiaomi.fan.p45), verified against its stock MCU image.
// Configure the UART selection for your board before starting the driver.
// Physical UART pins and electrical behavior still require board verification.
startDriver MIoT
miotMap 2 1 1 bool 0 1 1
miotMap 2 5 2 int 1 100 1
miotMap 2 6 3 bool 0 1 1
miotMap 2 7 4 int 30 150 1 30
miotFan 1 2 3 4
// Optional: allow the fan's reset-button request to clear both saved Wi-Fi networks.
// miotAllowNetworkReset 1
scheduleHADiscovery 1
