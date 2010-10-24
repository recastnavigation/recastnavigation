BIN = RecastDemo/Bin
BUILD = RecastDemo/Build/GNUMake
OBJ = $(BUILD)/Objects

# Add new targets here and add a corresponding
# .mk file to RecastDemo/Build/GNUMake
TARGETS = DebugUtils Detour Recast RecastDemo

# Dependencies
RecastDemo: $(OBJ) DebugUtils Detour Recast

$(OBJ):
	mkdir -p $(OBJ)

all: $(TARGETS)
$(TARGETS):
	make BIN=$(BIN) BUILD=$(BUILD) OBJ=$(OBJ) -f $(BUILD)/$(@).mk

CLEAN_TARGETS = $(foreach target,$(TARGETS),$(target)-clean)
clean: $(CLEAN_TARGETS)
	rm -rf $(OBJ)
$(CLEAN_TARGETS): %-clean:
	make BIN=$(BIN) BUILD=$(BUILD) OBJ=$(OBJ) -f $(BUILD)/$(*).mk clean

.PHONY: all clean $(TARGETS) $(CLEAN_TARGETS)