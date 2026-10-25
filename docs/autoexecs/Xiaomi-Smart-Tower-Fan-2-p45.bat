// Xiaomi Smart Tower Fan 2 (xiaomi.fan.p45), verified against its stock MCU image.
// Configure the UART selection for your board before starting the driver.
// Physical UART pins and electrical behavior still require board verification.
startDriver MIoT
miotMap 2 1 1 bool 0 1 1
miotMap 2 5 2 int 1 100 1
miotMap 2 6 3 bool 0 1 1
miotMap 2 7 4 int 30 150 1 30
miotMap 2 3 5 int 0 2 1
miotMap 2 4 6 int 1 4 1
miotMap 12 2 7 int 0 480 1
miotMap 12 3 8 int 0 480 0
miotFan 1 2 3 4
miotFanEnum preset 5 Direct Natural Sleep
miotFanEnum profile 6 Lakeside Campsite Orchard Terraced_Field
miotFanTimer 7 8
// Optional: allow the fan's reset-button request to clear both saved Wi-Fi networks.
// miotAllowNetworkReset 1
scheduleHADiscovery 1
