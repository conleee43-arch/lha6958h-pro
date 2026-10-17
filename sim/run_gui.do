# ModelSim GUI Waveform script for lha6958h_reg_read
vlib work
vlog -work work ../rtl/lha6958h_reg_read.v tb_lha6958h_reg_read.v
vsim -voptargs="+acc" work.tb_lha6958h_reg_read

# Configure Wave window
add wave -noupdate -divider {System Control}
add wave -noupdate -color {Cyan} /tb_lha6958h_reg_read/clk
add wave -noupdate -color {Gray} /tb_lha6958h_reg_read/rst_n
add wave -noupdate -color {Yellow} /tb_lha6958h_reg_read/start_i
add wave -noupdate -color {Orange} -radix hex /tb_lha6958h_reg_read/addr_i
add wave -noupdate -color {Yellow} /tb_lha6958h_reg_read/busy_o

add wave -noupdate -divider {LHA6958H Parallel Bus}
add wave -noupdate -color {Cyan} /tb_lha6958h_reg_read/reg_cs_n
add wave -noupdate -color {Red} /tb_lha6958h_reg_read/reg_wr_n
add wave -noupdate -color {Orange} /tb_lha6958h_reg_read/reg_rd_n
add wave -noupdate -color {Blue} /tb_lha6958h_reg_read/reg_db_oe
add wave -noupdate -color {Green} -radix hex /tb_lha6958h_reg_read/db_bus

add wave -noupdate -divider {Readback Output Result}
add wave -noupdate -color {Magenta} -radix hex /tb_lha6958h_reg_read/data_o
add wave -noupdate -color {Yellow} /tb_lha6958h_reg_read/done_o
add wave -noupdate -color {Red} /tb_lha6958h_reg_read/err_o

run -all
wave zoom full

