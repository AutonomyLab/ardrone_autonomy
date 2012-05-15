#!/bin/sh

## Author : stephane.piskorski@parrot.com
## Date   : 19th,Oct. 2010


check()
{
	if [ `cat ./temporary_file | grep $1 | wc -l` -eq 0 ] ; then 
		echo " $1";
	fi
}

verify()
{
		packages="$packages$(check $1)";
}


packages="";

if [ `which uname` ] ; then 
	if [ `uname -a | grep Ubuntu | wc -l` ] ; then

		echo "\033[31mChecking required Ubuntu packages ...\033[0m";  

		if [ ! -e ./temporary_file ] ; then   # check that the temp file does not exist
		
			dpkg -l > ./temporary_file;
		
			#To use FFMPEG recording
			verify "libavformat-dev";
			verify "libavcodec-dev";
			verify "libavutil-dev";
			verify "libswscale-dev";
			verify "libavfilter-dev";

			#To use the Wiimote in Navigation
			verify "libcwiid1-dev";
			verify "libbluetooth-dev";
		
			#To compile Navigation
			verify "libsdl1.2-dev";
			verify "libgtk2.0-dev";
      		verify "libxml2-dev";
      		verify "libudev-dev";
      		verify "libiw-dev";

			if [ "$packages" != "" ] ; then
				echo "You should install the following packages to compile the Mykonos project with Ubuntu:\n $packages";
				echo "Do you want to install them now [y/n] ?";
				read user_answer ;
				if [ "$user_answer" = "y" ]; then
					sudo apt-get install $packages;
				fi
			else
				echo "ok.";
			fi
		
			rm 	./temporary_file;
		fi
	fi
fi

