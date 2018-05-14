#!/bin/bash
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

cd -- "$(dirname "$BASH_SOURCE")"

dest_dir="./Videos/"

if [ ! -d "$dest_dir" ]; then
  echo
  echo -e "${GREEN}Creating destination folder "$dest_dir" ${NC}"
  mkdir "$dest_dir"
  echo
fi

for dir in /Volumes/*; do
  if [ -d "$dir/DCIM" ]; then 
    echo
    echo -e "${GREEN}Copying videos from "$dir" ...${NC}"
    echo

    for file in "$dir"/DCIM/*/*.MP4; do
      echo "copying file "$file""
      mv -i "$file" "$dest_dir"
    done
    echo
  
    rfiles=`ls "$dir"/DCIM/*/*.MP4`

    if [[ $rfiles == '' ]]; then

      echo 
      echo -e "${GREEN}Emptying SD Card "$dir"...${NC}"
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
  fi
done





