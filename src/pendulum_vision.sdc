create_clock -name sys_clk -period 37.037 -waveform {0 18.5185} [get_ports {clk}]
# D6 samples asynchronous DVP pins through system-clock input synchronizers.
# These exceptions end at their first sampling registers; subsequent receiver,
# RAM and UART paths remain timed by sys_clk. There is no raw PCLK clock domain.
# STA cannot validate the external signal voltage or oversampling aperture.
set_false_path -from [get_ports {cam_pclk cam_vs cam_hs cam_d[*]}]
