#!/bin/bash
#list='1 2 3 4 5 6 7 8 mid end'
#if ! [[ $list =~ (^| )$1($| ) ]]; then
if [[ $1 == '' ]];then
  echo "error: missing directory"
  echo "Usage: ./rename_gopro_vids.sh <dir-path>"
  echo "./rename_gopro_vids.sh Session1"
  exit
fi

if [ ! -d $1 ];then
  echo "error: directory [$1] does not exist"
  echo "Usage: ./rename_gopro_vids.sh <dir-path>"
  echo "./rename_gopro_vids.sh Session1"
  exit
fi

for filename in $1/*.MP4; do

  dd=`mediainfo $filename | grep Encoded`
  dt=`echo ${dd//:/-} | awk -F "UTC" '{print $2}' | awk '{print $1"-"$2}'`

  bn=${filename##*/}

  if [[ $bn == G* ]]
  then
    seq=`echo ${bn:4:4}`
    nn=`echo $1'/x'$seq'_s'${1:7:1}`
  elif [[ $bn == p* ]]
  then
    seq=`echo $bn | cut -d_ -f1`
    nn=`echo $1'/'$seq'_s'${1:7:1}`
  else
    nn=`echo ${bn%.*}`
  fi

  new_fn=`echo $nn'_'$dt.MP4`

  if [ "$filename" != "$new_fn" ]
  then
    if [ "$dt" != "-" ]
    then
      echo $filename" -> "$new_fn
      mv -i $filename $new_fn
    else
      echo "$filename: no date information. skipping"
    fi
  else
    echo "$filename: already in correct format. skipping"
  fi
done


