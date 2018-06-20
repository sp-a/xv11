all:
	g++ -I./ -o slam ./*cpp -std=c++11 -lopencv_core -lopencv_imgproc -lopencv_highgui        

clean:
	rm slam