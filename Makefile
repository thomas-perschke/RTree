.PHONY: all build test clean

CXXFLAGS=-Wall -Wextra -Wpedantic -std=c++11

all: build

build: out out/Test out/TestBadData out/MemoryTest

out:
	mkdir -p out

out/Test: Test.cpp RTree.h
	$(CXX) -o $@ ${CXXFLAGS} $<

out/TestBadData: TestBadData.cpp RTree.h
	$(CXX) -o $@ ${CXXFLAGS} $<

out/MemoryTest: MemoryTest.cpp RTree.h
	$(CXX) -o $@ ${CXXFLAGS} $<

test: build
	./out/Test
	./out/TestBadData baddata.txt
	./out/MemoryTest

clean:
	rm -rf out
