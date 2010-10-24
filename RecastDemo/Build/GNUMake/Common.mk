OBJECTS = $(patsubst $(NAME)/Source/%.cpp,$(OBJ)/%.o,$(wildcard $(NAME)/Source/*.cpp))
CPPFLAGS += -I$(NAME)/Include

$(OBJ)/%.o: $(NAME)/Source/%.cpp
	c++ $(CPPFLAGS) -c -o $@ $<

.PHONY: clean