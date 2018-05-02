#!/bin/bash
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

cd -- "$(dirname "$BASH_SOURCE")"

if [[ $1 == '' ]];then
  dir="/Volumes/Untitled"
elif [[ $1 == /Volumes/* ]]
then
  dir=$1
else
  dir="/Volumes/$1"
fi

if [ ! -d "$dir" ];then
  echo "error: SD card is not found in path $dir"
  echo "Usage: ./backup_gopro_sdcard.command <SD Card Name>"
  echo 'e.g., ./backup_gopro_sdcard.command "GoPRO Storage"'
  echo
  exit
fi

echo
echo -e "${GREEN}Copying videos from $dir ...${NC}"
echo
for file in "$dir"/DCIM/*/*.MP4; do
  echo "$file"
  mv -i "$file" ./Video/
done

rfiles=`ls "$dir"/DCIM/*/*.MP4`

if [[ $rfiles == '' ]]; then

  echo 
  echo -e "${GREEN}Emptying SD Card...${NC}"
  rm -rf "$dir"/DCIM/* 
else
  echo
  echo -e "${RED}SD Card not empty. List of remaining files in SD card:${NC}"
  echo "$rfiles"
  echo
  read -p "Do you want to force delete? (y/n [n])" del
  if [[ $del == 'y' ]]; then
    echo 
    echo -e "${GREEN}Emptying SD Card...${NC}"
    rm -rf "$dir"/DCIM/*
  else
    echo -e "${GREEN}Check files and rerun script.${NC}"
  fi
fi

echo




