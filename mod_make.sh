#! /bin/bash

if [[ -f "kernel_path.txt" ]] ; then
	KERNELDIR=`sed -n '1p' kernel_path.txt`
fi

if [[ "${KERNELDIR}" == "" ]] ; then
	KERNELDIR="/usr/src/linux"
fi

SETTINGS="agt_version.h"
# VERSION=`date +%Y.%m.%d-%H:%M:%S`

VERSION=`git log | grep commit | sed -n '1p' | awk '{print $2}'`

if [[ "$VERSION" == "" ]]; then
	VERSION=`cat "$SETTINGS" | grep define | grep VERSION | awk '{print $3}' | awk '{gsub(/[^0-9\.]/,""); print}' `
	echo -n "Version? ($VERSION) "
	read r
	r=`echo "${r}" | awk '{gsub(/[^0-9\.]/, ""); print}'`
	
	if [ "${r}" != "" ] ; then VERSION="${r}"; fi
fi

CLEAN=`echo "$1" | grep "clean"`
INSTALL=`echo "$1" | grep "install"`

r=""


if [ "$CLEAN" == "" ] && [ "$INSTALL" == "" ]
then
	rm -rf "${SETTINGS}"
	echo "#ifndef AGT_VERSION_H" > "${SETTINGS}"
	echo "#define AGT_VERSION_H" >> "${SETTINGS}"
	echo >> "${SETTINGS}"
	echo "#define AGT__MODULE_VERSION \"${VERSION}\"" >> "${SETTINGS}"
	echo >> "${SETTINGS}"
	echo "#endif //AGT_VERSION_H" >> "${SETTINGS}"
fi

make -C "${KERNELDIR}" SUBDIRS=$PWD $1
