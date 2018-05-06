all:
	g++ -I./ -o slam ./*cpp -std=c++11

clean:
	rm slam