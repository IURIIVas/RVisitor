-include varsmake.mk

GCC := riscv64-unknown-elf-gcc
EXEC := RVisitor
OUTPUT := $(EXEC).elf

INC_FLAGS := $(addprefix -I,$(INC_DIRS))
CFLAGS := $(INC_FLAGS) -Wall -g -Og -march=rv32imac_zicsr -mabi=ilp32 -nostartfiles
LINK_FLAGS := -T $(LINKER_SCRIPT)
LIBS := -lm

all: clean $(OUTPUT) $(EXEC).lst

$(OUTPUT): $(OBJ) $(OBJ_S)
	$(GCC) $(CFLAGS) $(LINK_FLAGS) -o $@ $(wildcard $(OBJDIR)/*.o) $(INC_FLAGS) $(LIBS)

$(EXEC).lst: $(OUTPUT)
	riscv64-unknown-elf-objdump --all-headers --demangle --disassemble "$<" > "$@"

$(OBJ): %.o: %.c $(OBJDIR)
	$(GCC) $(CFLAGS) -c $< -o $(OBJDIR)/$(notdir $@) 
	
$(OBJ_S): %.o: %.S $(OBJDIR)
	$(GCC) $(CFLAGS) -c $< -o $(OBJDIR)/$(notdir $@) 
	
$(OBJDIR): 
	mkdir -p $(OBJDIR)

clean:
	-rm -f $(OBJDIR)/* $(OUTPUT) $(EXEC).lst
	