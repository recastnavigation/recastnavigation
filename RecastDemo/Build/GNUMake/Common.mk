OBJECTS = $(patsubst $(NAME)/Source/%.cpp,$(BIN)/%.o,$(wildcard $(NAME)/Source/*.cpp))
CPPFLAGS += -I$(NAME)/Include

$(BIN)/%.o: $(NAME)/Source/%.cpp
	c++ $(CPPFLAGS) -c -o $@ $<

.PHONY: clean