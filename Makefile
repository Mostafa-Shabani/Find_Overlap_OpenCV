APP := imageProc

SRCS := \
	test_overlap.cpp\
	Overlap.cpp\

#OBJS := $(SRCS:.cpp=.o)
OBJS := $(patsubst %.cpp, ./build/%.o,$(SRCS))

CPP := g++ -std=c++11 -g
LDFLAGS:=`pkg-config --libs opencv` 
INCLUDE:= -I./myHeaderFiles \
		  

CPPFLAGS:=

#all: $(APP)

./build/%.o: %.cpp
	$(CPP) $(INCLUDE) $(LDFLAGS) $(CPPFLAGS) -c $< -o $@

$(APP): $(OBJS)
	$(CPP) -o ./build/$@ $(OBJS)  $(CPPFLAGS) $(INCLUDE) $(LDFLAGS)

clean:
	rm -rf ./build/* 
