export PICO_SDK_PATH=~/pico-sdk
export PICOTOOL_FETCH_FROM_GIT_PATH=~/picotool
rm -Rf build/*
pushd build
cmake ..
make -j4
popd
