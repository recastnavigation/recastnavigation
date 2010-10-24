include $(BUILD)/Common.mk

$(BIN)/$(NAME).a: $(OBJECTS)
	ar -q $@ $(OBJECTS)

clean:
	rm -f $(BIN)/$(NAME).a $(OBJECTS)