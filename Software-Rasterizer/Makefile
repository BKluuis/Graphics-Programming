all: main

CXX = clang++
override CXXFLAGS += -Wno-everything

SRCS = $(shell find . -name '.ccls-cache' -type d -prune -o -type f -name '*.cpp' -print | sed -e 's/ /\\ /g')
HEADERS = $(shell find . -name '.ccls-cache' -type d -prune -o -type f -name '*.h' -print)


remake: clean m

main: $(SRCS) $(HEADERS)
	$(CXX) $(CXXFLAGS) -O3 $(SRCS) -o "$@"

main-debug: $(SRCS) $(HEADERS)
	$(CXX) $(CXXFLAGS) -O0 -g $(SRCS) -o "$@"

clean:
	rm -f main main-debug