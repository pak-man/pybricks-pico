git submodule update --init --recursive
git clone https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
pushd ~/pico-sdk
git submodule update --init --recursive
popd
export PICO_SDK_PATH=~/pico-sdk
git clone https://github.com/raspberrypi/picotool.git ~/picotool
pushd ~/picotool
mkdir build
cd build
cmake ..
make
popd