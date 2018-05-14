#!/bin/bash
#list='1 2 3 4 5 6 7 8 mid end'
#if ! [[ $list =~ (^| )$1($| ) ]]; then
cd -- "$(dirname "$BASH_SOURCE")"
if [[ $1 == '' ]];then
  dir="./"
else
  dir=$1
fi

if [ ! -d $dir ];then
  echo "error: directory [$dir] does not exist"
  echo "Usage: ./rename_gopro_vids.sh <dir-path>"
  echo "./rename_gopro_vids.sh Session1"
  exit
fi

shopt -s nullglob
for filename in $dir/*; do
  

  if [[ $filename == *.MP4 ]] || [[ $filename == *.mp4 ]] || [[ $filename == *.MOV ]] || [[ $filename == *.mov ]]
  then
    bn=${filename##*/}
    x=${bn//[!-]/}
  
    if [ ${#x} -eq 5 ]
    then 
      echo "$filename: already in correct format. skipping"
      continue
    fi

    dd=`mediainfo "$filename" | grep Encoded`
    dt=`echo ${dd//:/-} | awk -F "UTC" '{print $2}' | awk '{print $1"-"$2}'`

    if [[ "$bn" == G* ]]
    then
      seq=`echo ${bn:4:4}`
      nn=`echo $dir'/x'$seq'_s'${1:7:1}`
    elif [[ "$bn" == p* ]] || [[ "$bn" == x* ]]
    then
      seq=`echo "$bn" | cut -d_ -f1`
      nn=`echo $dir'/'$seq'_s'${1:7:1}`
    else
      nn=${bn%.*}
    fi

    new_fn=`echo $nn'_'$dt.MP4`

    if [ "$filename" != "$new_fn" ]
    then
      if [ "$dt" != "-" ]
      then
        echo $filename" -> "$new_fn
        mv -i "$filename" "$new_fn"
      else
        echo "$filename: no date information. skipping"
      fi
    else
      echo "$filename: already in correct format. skipping"
    fi
  fi
done


