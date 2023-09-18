module FPGA_Top_Wrapper (
input         Resetn_pin , // Reset Signal from push button
input         Clock_pin  , // Clock Signal from FPGA 50 MHz Oscillator
input  [ 4:0] SW_pin     , // 4 user switches and one pushbutton
output [ 7:0] Display_pin  // 8 LEDs on the FPGA
);
wire c0;
wire c1;
wire c2;

ast_621RISC_SIMD_v #(.rom_init("ast_test.mif"), .ram_init("null.mif")) risc_inst_0(
	.Resetn_pin      ( Resetn_pin  ), 
	.c0              ( c0          ), 
   .c1              ( c1          ),
	.c2              ( c2          ),
	.SW_pin          ( SW_pin      ), 
	.Display_pin     ( Display_pin ), 
	.ICis            ( /*NC*/      )
);
//defparam risc_inst_0.my_ram.my_ram.altsyncram_component.ram_init = "ast_led_test.mif";

ast_pll_3_v my_pll (
	.inclk0 ( Clock_pin ),
	.c0     ( c0        ),
	.c1     ( c1        ),
	.c2     ( c2        )
);


endmodule
