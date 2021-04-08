.PHONY: all build test clean

CXX=g++-10
CXXFLAGS= -Wall -Wextra -Wpedantic -std=c++2a 

all: build

build: out out/Test out/TestBadData out/MemoryTest out/TreeTest

out:
	mkdir -p out

out/TreeTest: TreeTest.cpp RTree.h
	$(CXX) -o $@ ${CXXFLAGS} $<

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
