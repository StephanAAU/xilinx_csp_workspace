# Usage with Vitis IDE:
# In Vitis IDE create a Single Application Debug launch configuration,
# change the debug type to 'Attach to running target' and provide this 
# tcl script in 'Execute Script' option.
# Path of this script: C:\School\Vivado\ZynqDemoRemake\workspace_csp_cam_test\test_system\_ide\scripts\jtag.tcl
# 
# 
# Usage with xsct:
# To debug using xsct, launch xsct and run below command
# source C:\School\Vivado\ZynqDemoRemake\workspace_csp_cam_test\test_system\_ide\scripts\jtag.tcl
# 
connect -url tcp:127.0.0.1:3121
targets -set -nocase -filter {name =~"APU*"}
rst -system
after 3000
targets -set -filter {jtag_cable_name =~ "JTAG-ONB4 2516330075ADA" && level==0 && jtag_device_ctx=="jsn-JTAG-ONB4-2516330075ADA-13722093-0"}
fpga -file C:/School/Vivado/ZynqDemoRemake/workspace_csp_cam_test/test/_ide/bitstream/zsys_wrapper.bit
targets -set -nocase -filter {name =~"APU*"}
loadhw -hw C:/School/Vivado/ZynqDemoRemake/workspace_csp_test/csptest/export/csptest/hw/zsys_wrapper.xsa -mem-ranges [list {0x40000000 0xbfffffff}] -regs
configparams force-mem-access 1
targets -set -nocase -filter {name =~"APU*"}
source C:/School/Vivado/ZynqDemoRemake/workspace_csp_cam_test/test/_ide/psinit/ps7_init.tcl
ps7_init
ps7_post_config
targets -set -nocase -filter {name =~ "*A9*#0"}
dow C:/School/Vivado/ZynqDemoRemake/workspace_csp_cam_test/test/Debug/test.elf
configparams force-mem-access 0
targets -set -nocase -filter {name =~ "*A9*#0"}
con
