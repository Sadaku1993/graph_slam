echo "Configureing and building Thirdparty/g20 ..."

cd Thirdparty/g2o

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
