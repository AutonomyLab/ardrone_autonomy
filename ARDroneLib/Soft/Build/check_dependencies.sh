#!/bin/sh

## Author : stephane.piskorski@parrot.com
## Date   : 19th,Oct. 2010

# Parameters :
# $1 : target : can be 'iphone (cross compile static libs for iPhone)', 'static (static library for the host)', 'host (shared library for the host)'
# $2 : RELEASE_BUILD=xxx
# $3 : 'clean' if we want to clean, or nothing


check()
{
	if [ `cat $TEMPFILE | grep $1 | wc -l` -eq 0 ] ; then 
		echo " $1";
	fi
}

verify()
{
		p="$(check $1)";

		if [ "$p" != "" ]; then
			packages="$packages $p";
			if [ "$2" != "" ]; 
				then messages="$messages\n $p : $2"; 
			  else messages="$messages\n $p"; 
			fi;
		fi;
}

TEMPFILE=`mktemp`

if [ "$ALL_TARGETS" != "" ] ; then
	TARGET_DIR=$ALL_TARGETS
else
	TARGET_DIR=./targets_versions
fi

SDK_PATH=`echo $0 | sed "s:/Soft/Build/check_dependencies.sh::"`

if [ "$1" != "iphone" ] ; then
    if [ "$3" = "cleanAll" ] ; then
	make -C $SDK_PATH/FFMPEG clean
	exit 0
    elif [ "$3" = "clean" ] ; then
	# do nothing here
	echo "" > /dev/null
    else
	make -C $SDK_PATH/FFMPEG $1 $2
    fi
fi

packages="";
messages="";

if [ `which apt-get` ] ; then 
	if [ "`apt-get -v | grep -o ubuntu`" != "" ] ; then

		echo "\033[31mChecking required Ubuntu packages ...\033[0m";  

			dpkg -l > $TEMPFILE;
			
			# To compile the AR.Drone project
			verify "daemontools" "Mandatory to build the AR.Drone project on Ubuntu";
		
			# To use the Wiimote in Navigation
			#verify "libcwiid1-dev";
			#verify "libbluetooth-dev";
		
			# To compile Navigation
			verify "libsdl1.2-dev";
			verify "libgtk2.0-dev";
			verify "libxml2-dev";
			verify "libudev-dev";
			verify "libiw-dev";

			if [ "$packages" != "" ] ; then
				echo "You should install the following packages to compile the AR.Drone SDK with Ubuntu:\n $messages \n";
				echo "Do you want to install them now [y/n] ?";
				read user_answer ;
				if [ "$user_answer" = "y" ]; then
					sudo apt-get install $packages;
				fi
			else
				echo "ok.";
			fi
	fi
fi


