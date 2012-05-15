#! /bin/sh

#####################
# Parameters
#####################

QUIET_BUILD=no

USE_SDK=yes
USE_APP=no
NO_EXAMPLES=yes

RELEASE_BUILD=yes
SDK_VERSION=dev
IPHONE_SDK_VERSION=3.0
USE_FFMPEG=no
USE_MJPEG=no
USE_JPEG=no
USE_JPEG_P6=no
USE_SWING=no
USE_VLIB=yes
SWING_VERSION=head

# Check validity of script usage.
if [ $1 ] && [ $1 = ecos ] ; then
  if [ ! $QUIET_BUILD = yes ] ; then
    echo ; echo "BUILD FOR ECOS" ; echo
  fi
  USE_ECOS=yes
  USE_LINUX=no
  USE_ELINUX=no
  USE_NDS=no
  USE_IPHONE=no
  # head ecos-stable-2006-11-21 ecos-stable-2007-07-23 CK5300_Version_20070928_Beta5
  ECOS_VERSION=Mykonos_Version_20090212
  PROJECT=mykonos_p5p
  MODE_TARGET=rls_ram_wifi_ap
  CUSTOM_PRIORITIES=$ALL_SOURCES/video/$SDK_VERSION/Video/VP_SDK/VP_Os/ecos/task_priorities.h
  # gnutools_2008_03_28 gnutools_2007_02_07 gnutools_2005_05_20
  GNUTOOLS_VERSION=gnutools_2008_03_28
  USE_ECOS_RELEASE=no
  NO_COM=no
  USE_BONJOUR=no
  USE_BLUES32=no
  # Stable_20070307 Stable_20060922 Stable_20061129_Blues_3_6_4 Version_KEA_1_00_RC5_200701131
  BLUES32_VERSION=Stable_20060922
  COMMONSOFT_VERSION=head
  USE_PVSP=yes
  USE_SOUL=no
  SOUL_VERSION=Stable_20070307
  USE_TANGO=no
  # Stable_20061222 Stable_20070509
  TANGO_VERSION=Stable_20070509
  DONT_USE_TTS=yes
  USE_CK5050=no
  CK5050_VERSION=head
  USE_BLUEZ=no
  # P5 Intel smdk2412
  FF_ARCH=P5
  USE_PARROTOS_CORE=no
elif [ $1 ] && [ $1 = elinux ] ; then
  if [ ! $QUIET_BUILD = yes ] ; then
    echo ; echo "BUILD FOR EMBEDDED LINUX" ; echo
  fi
  USE_ECOS=no
  USE_LINUX=no
  USE_ELINUX=yes
  USE_NDS=no
  USE_IPHONE=no
  ELINUX_VERSION=head
  TOOLCHAIN_VERSION=arm-eglibc
  NO_COM=no
  USE_BLUES32=no
  USE_BLUEZ=no
  USE_BONJOUR=no
  FF_ARCH=Intel
  USE_BROADCOM=no
  USE_IWLIB=yes
  USE_PARROTOS_CORE=no
  COMMONSOFT_VERSION=head
  CONFIG_PARROTOS=$ALL_SOURCES/video/$SDK_VERSION/Video/VP_SDK/VP_Os/parrotos/default_config.h
elif [ $1 ] && [ $1 = linux ] ; then
  if [ ! $QUIET_BUILD = yes ] ; then
    echo ; echo "BUILD FOR LINUX" ; echo
  fi
  USE_ECOS=no
  USE_LINUX=yes
  USE_ELINUX=no
  USE_NDS=no
  USE_IPHONE=no
  NO_COM=no
  USE_BLUES32=no
  USE_BLUEZ=no
  USE_WIFI=no
  USE_BONJOUR=no
  FF_ARCH=Intel
  USE_PARROTOS_CORE=no
elif [ $1 ] && [ $1 = nds ] ; then
  if [ ! $QUIET_BUILD = yes ] ; then
    echo ; echo "BUILD FOR NINTENDO DS" ; echo
  fi
  USE_ECOS=no
  USE_LINUX=no
  USE_ELINUX=no
  USE_NDS=yes
  NDS_CPU=ARM9
  USE_IPHONE=no
  NO_COM=no
  USE_BLUES32=no
  USE_BLUEZ=no
  USE_BONJOUR=no
  FF_ARCH=Intel
  USE_PARROTOS_CORE=no
elif [ $1 ] && [ ${1:0:6} = iphone ] ; then
	if [ ! $QUIET_BUILD = yes ] ; then
		echo ; echo "BUILD FOR IPHONE with platform $1 $IPHONE_SDK_VERSION" ; echo
	fi
 	USE_ECOS=no
	USE_LINUX=no
	USE_ELINUX=no
	USE_NDS=no
	USE_IPHONE=yes
	IPHONE_PLATFORM=$1
	NO_COM=no
	USE_BLUES32=no
	USE_BLUEZ=no
	USE_WIFI=no
	USE_BONJOUR=no
	FF_ARCH=Intel
	USE_PARROTOS_CORE=no
else
  echo ; echo "UNDEFINED BUILD" ; echo ;
fi

#####################
# Update flags
#####################
FLAGS="USE_APP=$USE_APP"
FLAGS="IPHONE_SDK_VERSION=$IPHONE_SDK_VERSION $FLAGS"
FLAGS="NO_EXAMPLES=$NO_EXAMPLES $FLAGS"
FLAGS="GNUTOOLS_VERSION=$GNUTOOLS_VERSION $FLAGS"
FLAGS="USE_ECOS=$USE_ECOS $FLAGS"
FLAGS="USE_LINUX=$USE_LINUX $FLAGS"
FLAGS="USE_ELINUX=$USE_ELINUX $FLAGS"
FLAGS="USE_NDS=$USE_NDS $FLAGS"
FLAGS="NDS_CPU=$NDS_CPU $FLAGS"
FLAGS="USE_IPHONE=$USE_IPHONE $FLAGS"
FLAGS="IPHONE_PLATFORM=$IPHONE_PLATFORM $FLAGS"
FLAGS="USE_SDK=$USE_SDK $FLAGS"
FLAGS="NO_COM=$NO_COM $FLAGS"
FLAGS="USE_BLUES32=$USE_BLUES32 $FLAGS"
FLAGS="USE_SOUL=$USE_SOUL $FLAGS"
FLAGS="USE_TANGO=$USE_TANGO $FLAGS"
FLAGS="USE_CK5050=$USE_CK5050 $FLAGS"
FLAGS="USE_BLUEZ=$USE_BLUEZ $FLAGS"
FLAGS="USE_FFMPEG=$USE_FFMPEG $FLAGS"
FLAGS="USE_MJPEG=$USE_MJPEG $FLAGS"
FLAGS="USE_JPEG=$USE_JPEG $FLAGS"
FLAGS="USE_JPEG_P6=$USE_JPEG_P6 $FLAGS"
FLAGS="USE_SWING=$USE_SWING $FLAGS"
FLAGS="USE_VLIB=$USE_VLIB $FLAGS"
FLAGS="SWING_VERSION=$SWING_VERSION $FLAGS"
FLAGS="USE_BONJOUR=$USE_BONJOUR $FLAGS"
FLAGS="USE_WIFI=$USE_WIFI $FLAGS"

FLAGS="QUIET_BUILD=$QUIET_BUILD $FLAGS"
FLAGS="RELEASE_BUILD=$RELEASE_BUILD $FLAGS"
FLAGS="SDK_VERSION=$SDK_VERSION $FLAGS"

FLAGS="USE_ECOS_RELEASE=$USE_ECOS_RELEASE $FLAGS"
FLAGS="ECOS_VERSION=$ECOS_VERSION $FLAGS"
FLAGS="ELINUX_VERSION=$ELINUX_VERSION $FLAGS"
FLAGS="PROJECT=$PROJECT $FLAGS"
FLAGS="MODE_TARGET=$MODE_TARGET $FLAGS"

FLAGS="TOOLCHAIN_VERSION=$TOOLCHAIN_VERSION $FLAGS"

FLAGS="BLUES32_VERSION=$BLUES32_VERSION $FLAGS"
FLAGS="SOUL_VERSION=$SOUL_VERSION $FLAGS"
FLAGS="TANGO_VERSION=$TANGO_VERSION $FLAGS"
FLAGS="CK5050_VERSION=$CK5050_VERSION $FLAGS"
FLAGS="DONT_USE_TTS=$DONT_USE_TTS $FLAGS"

FLAGS="FF_ARCH=$FF_ARCH $FLAGS"

FLAGS="USE_BROADCOM=$USE_BROADCOM $FLAGS"
FLAGS="USE_IWLIB=$USE_IWLIB $FLAGS"
FLAGS="USE_PARROTOS_CORE=$USE_PARROTOS_CORE $FLAGS"
FLAGS="COMMONSOFT_VERSION=$COMMONSOFT_VERSION $FLAGS"

if [ $USE_ECOS = yes ] ; then
  FLAGS="CUSTOM_PRIORITIES=$CUSTOM_PRIORITIES $FLAGS"
fi

if [ $USE_ELINUX = yes ] ; then
  FLAGS="CONFIG_PARROTOS=$CONFIG_PARROTOS $FLAGS"
fi

if [ $USE_ECOS = yes ] && [ $2 ] && ! [ $2 = check ] && ! [ $2 = clean ] || [ $USE_ECOS = yes ] && ! [ $2 ] ; then
  CHOOSE=yes
fi
if [ $2 ] && [ $2 = check ] ; then
  CHECK=yes
fi
shift

#####################
# Build
#####################

size=`stty size`
if [ $QUIET_BUILD = yes ] ; then
  make_theme=darkq
else
  make_theme=dark
fi
if [ $CHECK ] && [ $CHECK = yes ] ; then
  make -f Makefile $FLAGS $* 2>&1 | grep -v "^$"
else
  make -f Makefile $FLAGS $* 2>&1
fi

#####################
# Choose example ?
#####################

if [ $CHOOSE ] && [ $CHOOSE = yes ] ; then
  j=1 ; for i in $( find . -type d -name "*Examples" | grep arm ) ; do j=$( expr $j + 1 ) ; done

  if [ $j = 1 ] ; then
    echo No example directory found
    exit
  fi

  if ! [ $j = 2 ] ; then
    j=1 ; for i in $( find . -type d -name "*Examples" | grep arm ) ; do echo -e "\t$j\t$i" ; j=$( expr $j + 1 ) ; done
    echo -n -e "\nChoose directory : "
    read var
    j=1 ; for i in $( find . -type d -name "*Examples" | grep arm ) ; do if [ $j = $var ] ; then ex_dir=$i ; fi ; j=$( expr $j + 1 ) ; done
  else
    ex_dir=$( find . -type d -name "*Examples" | grep arm )
  fi

  echo -e "\nDirectory $ex_dir :"

  j=1 ; for i in $( find $ex_dir -type f -exec basename \{\} \; ) ; do echo -e "\t$j\t$i" ; j=$( expr $j + 1 ) ; done
  echo -n -e "\nChoose example : "
  read var
  j=1 ; for i in $( find $ex_dir -type f ) ; do if [ $j = $var ] ; then ex_file=$i ; fi ; j=$( expr $j + 1 ) ; done

  if [ ! -z $ex_file ] && [ -f $ex_file ] ; then
    cp $ex_file /srv/tftp/program.elf && echo -e "\n$ex_file copied to /srv/tftp/program.elf"
  else
    echo -e "\nBad choice !"
  fi
fi
