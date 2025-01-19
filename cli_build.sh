#!/bin/bash
currentDir=$(
  cd $(dirname "$0")
  pwd
)
install_direct="$currentDir/compile_dir"
if [ -d "$currentDir/builddir" ] 
then
    rm -r "$currentDir/builddir"
    mkdir "$currentDir/builddir"
else
    mkdir "$currentDir/builddir"
fi;

if [ -d "$install_direct" ] 
then
    mkdir "$install_direct"
fi;

make -j8 DEBUG=1

