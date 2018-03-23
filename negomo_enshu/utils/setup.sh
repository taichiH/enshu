#!/bin/bash

# check path

if [[ $(pwd | awk -F/ '{print $(NF-1)"/"$NF}') != "negomo_enshu/utils" ]]
then
    echo "setup must be conducted under negomo_enshu/utils."
    exit
fi

# create CMakeLists.txt

cp cmake.setup ../CMakeLists.txt

# check settings

for arg in `seq 1 ${#}`
do
    if [[ $1 == "--e" ]]
    then
        cp cmake.setup ../CMakeLists.txt
    fi

    shift
done

# first time build warnings
if [[ $(rospack find negomo_enshu 2>&1 | grep 'not found') != "" ]]
then
    echo "please build package. once build has completed, source ~/.bashrc and re-run this script"
    exit
fi

# find hmmlib
# if multiple installed, specify by changing grep -m number

# hmmlib_dir=$(find / -name "StochHMMlib.h" 2>/dev/null)
# hmmlib_dir=$(locate StochHMMlib.h | grep -m 1 "StochHMMlib.h")
hmmlib_dir="$(rospack find negomo_enshu)/StochHMM/src"

if [[ $hmmlib_dir == "" ]]
then
    echo "requires StochHMM but not found"
    exit
fi

# hmmlib_dir=$(echo $hmmlib_dir | sed -e "s@src/StochHMMlib.h@src@g")

sed -i "s@set(FOUND_HMMLIB 0)@set(FOUND_HMMLIB 1)@g" ../CMakeLists.txt
sed -i "s@include_directories() #hmmlib@include_directories($hmmlib_dir)@g" ../CMakeLists.txt
sed -i "s@link_directories() #hmmlib@link_directories($hmmlib_dir)@g" ../CMakeLists.txt

# find static hmm object files

files=$(ls $hmmlib_dir | grep "\.o")
num_of_hmm_files=$(ls $hmmlib_dir | grep "\.o" | wc -l)

# add static hmm object files

write_to_line=$(grep -n "add_library(hmmlib" ../CMakeLists.txt | cut -d: -f1)
write_to_line=$(($write_to_line + 2))

write=''
j=0
while [[ $j -lt $num_of_hmm_files ]]
do
    file=$(echo $files | awk '{print $'$(($j + 1))'}')
    write="${write}${hmmlib_dir}/${file}\n"
    j=$(($j + 1))
done

echo $write | xargs -0 -I{} sed -i "${write_to_line}i\{}" ../CMakeLists.txt

# add static hmm object file settings

write_to_line=$(grep -n "SET_TARGET_PROPERTIES" ../CMakeLists.txt | cut -d: -f1)

j=0
while [[ $j -lt $num_of_hmm_files ]]
do
    file=$(echo $files | awk '{print $'$(($j + 1))'}')
    write="SET_SOURCE_FILES_PROPERTIES(${hmmlib_dir}/${file}\nPROPERTIES\nEXTERNAL_OBJECT true\nGENERATED true\n)"
    echo $write | xargs -0 -I{} sed -i "${write_to_line}i\{}" ../CMakeLists.txt
    j=$(($j + 1))
done
