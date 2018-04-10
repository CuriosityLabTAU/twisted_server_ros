echo $1, $2, $3

#check $1
list="p000 p001 p002 p003 p004 p005 p006 p007 p008 p009 p010 p011 p012 p013 p014 p015 p016 p017 p018 p019 p020 p021 p022 p023 p024 p025 p026 p027 p028 p029 p030 p031 p032 p033 p034 p035 p036 p037 p038 p039 p040 p041 p042 p043 p044 p045 p046 p047 p048 p049 p050 p051 p052 p053 p054 p055 p056 p057 p058 p101 p102 p103 p104 p105 p106 p107 p108 p109 p110 p111 p112 p113 p114 p115 p116 p117 p118 p119 p120 p121 p122 p123 p124 p125 p126 p127 p128 p129 p130 p131 p132 p133 p134 p135 p136 p137 p138 p139" 

if ! [[ $list =~ (^| )$1($| ) ]]; then
  echo "error: participant_id [$1] does not exist"
  echo "Usage: ./startStudy.sh <participant_id> <session_#>  continue(optional)"
  echo "./startStudy.sh p000 1"
  exit
fi

#check $2
list='1 2 3 4 5 6 7 8'
if ! [[ $list =~ (^| )$2($| ) ]]; then
  echo "error: session [$2] does not exist"
  echo "Usage: ./startStudy.sh <participant_id> <session_#>  continue(optional)"
  echo "./startStudy.sh p000 1"
  exit
fi

#check $3
if [ "$3" != "" ] && [ "$3" != "continue" ]; then
  echo "error: [$3] is not the right format. If you wish to continue from previous session, input \"continue\""
  echo "Usage: ./startStudy.sh <participant_id> <session_#>  continue(optional)"
  echo "./startStudy.sh p000 7 continue"
  exit
fi

entry=$3

if [ "$3" == "" ]; then
  entry="start"
fi

mkdir -p log
mkdir -p rosbag


gnome-terminal --geometry 24x12+0+0 --title ">>>Mindset Study MAIN<<<" -e "scripts/runStudy.sh $1 $2 $entry"

#gnome-terminal --geometry 24x12+0+400 --title ">>>Front Camera View<<<" -e "roslaunch cv_camera cv_camera.launch"

scripts/rosbag_record.sh $1 $2 $entry

