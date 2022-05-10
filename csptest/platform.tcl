# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct C:\School\Vivado\ZynqDemoRemake\workspace_csp_test\csptest\platform.tcl
# 
# OR launch xsct and run below command.
# source C:\School\Vivado\ZynqDemoRemake\workspace_csp_test\csptest\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {csptest}\
-hw {C:\School\Vivado\ZynqDemoRemake\vivado\zsys_wrapper.xsa}\
-proc {ps7_cortexa9_0} -os {freertos10_xilinx} -out {C:/School/Vivado/ZynqDemoRemake/workspace_csp_test}

platform write
platform generate -domains 
platform active {csptest}
platform generate
platform generate -domains freertos10_xilinx_domain 
platform generate
platform generate
platform clean
platform generate
platform generate
platform generate -domains freertos10_xilinx_domain 
platform generate
platform generate
platform generate
platform generate
