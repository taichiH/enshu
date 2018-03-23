#!/bin/bash

# check path

if [[ $(rospack find negomo | grep 'not found') != "" ]]
then
    echo "error"
    exit
fi

echo "creating header ......"

# create data header

template_file="$(rospack find negomo)/utils/template.hh"
data_file="$(rospack find negomo)/src/data.hh"

cp $template_file $data_file

# init parameter names and filenames

parameter_names="\t"
filenames="\t"

# write to header

for dat in `seq 1 ${#}`
do
    rosrun negomo create_negomo.sh $1
    parameter_names="${parameter_names}\"/negotiation_model/k_${1}\", "
    filenames="${filenames}\"${1}.hmm\", "
    shift
done

parameter_names=${parameter_names::-2}
write_to_line=$(grep -n "write parameter names" $data_file | cut -d: -f1)
echo $parameter_names | xargs -0 -I{} sed -i "${write_to_line}i\{}" $data_file
filenames=${filenames::-2}
write_to_line=$(grep -n "write filenames" $data_file | cut -d: -f1)
echo $filenames | xargs -0 -I{} sed -i "${write_to_line}i\{}" $data_file

echo "finished"
