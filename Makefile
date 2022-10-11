PRJ=adc blinky blinky2 dac lcd pwr uart usbmouse uart_at ts draw
all:
	$(foreach n,$(PRJ), make -C $(n);)
clean:
	$(foreach n,$(PRJ), make -C $(n) clean;)
