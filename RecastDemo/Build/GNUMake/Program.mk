include $(BUILD)/Common.mk

$(BIN)/$(NAME): $(OBJECTS)
	c++ $(LDFLAGS) -o $@ $(OBJECTS) $(LIBS)

clean:
	rm -f $(BIN)/$(NAME).a $(OBJECTS)