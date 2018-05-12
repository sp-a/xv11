all:
	g++ -I./ -I./libicp/src/ -o slam ./*cpp ./libicp/src/*.cpp -std=c++11 -lopencv_core -lopencv_imgproc -lopencv_highgui        

clean:
	rm slam