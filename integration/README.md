docker build -t charvak-integration .

docker run --rm -v %cd%:/app charvak-integration

run command for windows

docker run --rm -v C:\Users\rohit\Documents\codes\github\Charvak\integration:/app 


docker run --rm -v $(pwd):/app charvak-integration


charvak-integration