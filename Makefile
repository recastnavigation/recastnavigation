BIN = RecastDemo/Bin
BUILD = RecastDemo/Build/GNUMake

# Add new targets here and add a corresponding
# .mk file to RecastDemo/Build/GNUMake
TARGETS = DebugUtils Detour Recast RecastDemo

# Dependencies
RecastDemo: DebugUtils Detour Recast

all: $(TARGETS)
$(TARGETS):
	make BIN=$(BIN) BUILD=$(BUILD) -f $(BUILD)/$(@).mk

CLEAN_TARGETS = $(foreach target,$(TARGETS),$(target)-clean)
clean: $(CLEAN_TARGETS)
$(CLEAN_TARGETS): %-clean:
	make BIN=$(BIN) BUILD=$(BUILD) -f $(BUILD)/$(*).mk clean

.PHONY: all clean $(TARGETS) $(CLEAN_TARGETS)