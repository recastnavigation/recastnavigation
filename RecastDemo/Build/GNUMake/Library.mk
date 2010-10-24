include $(BUILD)/Common.mk

CPPFLAGS += -fPIC
LDFLAGS += -shared

$(BIN)/lib$(NAME).so: $(OBJECTS)
	c++ $(LDFLAGS) -o $@ $(OBJECTS) $(LIBS)

clean:
	rm -f $(BIN)/lib$(NAME).so $(OBJECTS)