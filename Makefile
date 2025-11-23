OBJDIR = obj

${OBJDIR}/%.o : %.s
	@mkdir -p $(@D)
	arm-none-eabi-as -a=$@.lst $< -o $@ -mcpu=cortex-m33

main_blink.elf : ${OBJDIR}/main_blink.o ${OBJDIR}/mcxn947_startup.o
	arm-none-eabi-ld -Ttext 0 -Tdata 0x20000000 -nostdlib ${OBJDIR}/mcxn947_startup.o ${OBJDIR}/main_blink.o -o $@ --print-memory-usage -Map $@.map
	#arm-none-eabi-objcopy $@ -O ihex main_blink.hex

${OBJDIR}/main_blink.o : main_blink.s
${OBJDIR}/mcxn947_startup.o : mcxn947_startup.s


clean :
	rm -rf ${OBJDIR}

clobber : clean
	rm -rf *.lst *.out *.elf *.hex *.map
