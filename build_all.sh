#! /bin/bash
echo ""
echo "------------------------------------ Building ------------------------------------"
echo ""


ARCH=$(uname -m)
export ARCH=$ARCH
if [ "$ARCH" == "aarch64" ]
then
    CMAKE_OPTIONS="-DARCH_DIR=$ARCH -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ -DOpenCV_DIR=/usr/local/share/OpenCV"
else
    CMAKE_OPTIONS="-DARCH_DIR=$ARCH -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ -DOpenCV_DIR=/usr/local/share/OpenCV"
fi

if [ "$ARCH" == "armv7l" ]
then
    export WIRINGPI_GPIOMEM=1
    sudo ./mouse_droid_dependencies.sh
fi


catkin_make --cmake-args $CMAKE_OPTIONS

if [ $? -eq 0 ]; then
    echo ""
    echo "--------------------------- Building And Running Tests ---------------------------"
    echo ""

	catkin_make run_tests --cmake-args $CMAKE_OPTIONS

	if [ $? -eq 0 ]; then
		echo ""
		echo "--------------------------------- Build Succeeded --------------------------------"
		echo ""
		echo ""
		echo "------------------------------------ Results -----------------------------------"
		echo ""
		catkin_test_results
		exit $?
	else
		echo ""
		echo "----------------------------- Building Tests Failed ----------------------------"
		echo ""
		exit 1
	fi
else
	echo ""
	echo "--------------------------------- Build Failed ---------------------------------"
	echo ""
	exit 1
fi
. ./devel/setup.sh

