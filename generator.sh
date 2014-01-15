#!/bin/bash

# Define here the installation path for your platform
case "$HOSTNAME" in
	"Arcteryx") echo "You dont have what it takes to compile me !!!"
		install_path="$PWD/install"
		options =		"-DCMAKE_VERBOSE_MAKEFILE:BOOL=OFF"
		;;
		
	*) echo "Your platform is not defined in $0. Proceding with default installation."
		install_path="$PWD/install"
		options = 	"-DPCL_DIR=/Shared/PCL-1.7/share/pcl-1.7/"
    options +=  "-DOpenCV_DIR=/Shared/OpenCV-2.4.8/share/OpenCV/"
		options +=	"-DCMAKE_VERBOSE_MAKEFILE:BOOL=OFF"

esac

# Functions
function build
{
  mkdir -p build
  cd build
  cmake -DCMAKE_INSTALL_PREFIX:PATH=$install_path         \
        $options                                          \
        .. &&
  make &&
  make install &&
  cd ..
}

function remove
{
  rm -rfv $1
}

function help
{
  echo "Project generator"
  echo "Options are:"
  echo " -h, --help         show brief help"
  echo " -b, --build        build the project"
  echo " -c, --clean        clean the project"
  echo " --clear            clear the project (remove binaries)"
}

############################################
#                   MAIN                   #
############################################

if test $# -gt 0; then
  case "$1" in
    -h|--help)
      help
      exit 0
      ;;
    
    -b|--build)
      build
      exit 0
      ;;
      
    -c|--clean)
      remove "build"
      exit 0
      ;;
    
    --clear)
      remove "build install"
      exit 0
      ;;
      
    *)
      echo "invalid option -- '$1'"
      help
      exit 0
      ;;
  esac
fi

build
#help
