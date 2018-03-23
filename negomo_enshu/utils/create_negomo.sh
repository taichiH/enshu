#!/bin/bash

# check path

if [[ $(rospack find negomo | grep 'not found') != "" ]]
then
    echo "error"
    exit
fi


# set variables

parameter_file="$(rospack find negomo)/data/$1"
write_file="$(rospack find negomo)/data/$1.hmm"
template_file="$(rospack find negomo)/utils/template.hmm"

echo $parameter_file
echo $write_file
echo $template_file

tab=$'  '

# define action probability

action_count=$(grep "action" $parameter_file | grep -v "#" | wc -l)
action_probability=$(echo "" | awk '{x=1.0/'$action_count'; printf"%0.3f\n", x}')

get_action_probability() {
    echo $action_probability
}


# define transition probability table

get_transition_probability() {
    next_state_name=$1
    next_action_name=$2
    state_name=$3

    transition="${next_state_name:0:1}|${state_name:0:1}"
    action=$(grep "action" $parameter_file | grep -v "#" | grep -n $next_action_name | cut -d: -f1)
    p=$(grep $transition $parameter_file | cut -d: -f2 | awk '{print $'$action'}')

    ap=$(get_action_probability $next_action_name)
    p=$(echo "" | awk '{x='$ap'*'$p'; printf"%0.3f\n", x}')
    echo $p
}


# define emission probability table

get_emission_probability() {
    observation_name=$1
    observation_action_name=$2
    state_name=$3
    state_action_name=$4
    p=0.0
    if [[ $observation_action_name == $state_action_name ]]
    then
        emission="${observation_name}|${state_name:0:1}"
        action=$(grep "action" $parameter_file | grep -v "#" | grep -n $state_action_name | cut -d: -f1)
        p=$(grep $emission $parameter_file | grep -v "#" | cut -d: -f2 | awk '{print $'$action'}')
    fi
    echo $p
}


# define prior probability table

get_prior_probability() {
    state_name=$1
    action_name=$2

    action=$(grep "action" $parameter_file | grep -v "#" | grep -n $action_name | cut -d: -f1)
    p=$(grep "${state_name:0:1}:" $parameter_file | grep -v "|" | cut -d: -f2 | awk '{print $'$action'}')
    echo $p
}


# create state template file

echo "creating model file ......"

state_template_file="/tmp/negomostatetemplate"
begin_line=$(grep -n "STATE" $template_file | awk '//{i++}i==3{print; exit}' | cut -d: -f1)
end_line=$(($begin_line + 6))
begin_line=$(($begin_line - 1))
awk "NR>=$begin_line && NR<=$end_line" $template_file > $state_template_file


# create hmm model file

cp $template_file $write_file
sed -i "${begin_line},${end_line}d" $write_file


# get list of state names

echo "getting state names ......"

states=()

state_count=$(grep "state" $parameter_file | grep -v "#" | wc -l)
i=0
j=0

while [[ $i -lt $state_count ]]
do
    while [[ $j -lt $action_count ]]
    do
        st_line=$(awk '/state_/{k++}k=='$(($i + 1))'{print; exit}' $parameter_file)
        ac_line=$(awk '/action_/{k++}k=='$(($j + 1))'{print; exit}' $parameter_file)
        st_name=$(echo ${st_line} | cut -d_ -f2)
        ac_name=$(echo ${ac_line} | cut -d_ -f2)
        states+=("${st_name}_${ac_name}")
        j=$(($j + 1))
    done
    i=$(($i + 1))
    j=0
done


# get list of observation names

echo "getting observation names ......"

observations=()

observation_count=$(grep "observation" $parameter_file | grep -v "#" | wc -l)
i=0
j=0

while [[ $i -lt $observation_count ]]
do
    while [[ $j -lt $action_count ]]
    do
        ob_line=$(awk '/observation_/{k++}k=='$(($i + 1))'{print; exit}' $parameter_file)
        ac_line=$(awk '/action_/{k++}k=='$(($j + 1))'{print; exit}' $parameter_file)
        ob_name=$(echo ${ob_line} | cut -d: -f1 | cut -d_ -f2)
        ac_name=$(echo ${ac_line} | cut -d_ -f2)
        observations+=("${ob_name}_${ac_name}")
        j=$(($j + 1))
    done
    i=$(($i + 1))
    j=0
done


# write observation information to hmm model file

echo "writing observations ......"

write_to_line=$(grep -n "OBSERVATION" $template_file | awk '//{i++}i==1{print; exit}' | cut -d: -f1)
sed -i "${write_to_line},$((${write_to_line} + 1))d" $write_file
write="OBSERVATION: "
for observation_name in "${observations[@]}"
do
    write="${write}${observation_name},"
done
write=${write::-1}
echo "${write}\n" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $write_file


# write initial state information to hmm model file

echo "writing prior probabilities ......"

write_to_line=$(grep -n "STATE" $template_file | awk '//{i++}i==2{print; exit}' | cut -d: -f1)
write_to_line=$(($write_to_line + 3))
for state_name in "${states[@]}"
do
    x_name=$(echo $state_name | cut -d_ -f1)
    a_name=$(echo $state_name | cut -d_ -f2)    
    p=$(get_prior_probability $x_name $a_name)
    write="${tab}${state_name}: ${p}"
    echo "${write}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $write_file
    write_to_line=$(($write_to_line + 1))
done


# write state information to hmm model file

echo "writing transition probabilities ......"

IFS=''
for state_name in "${states[@]}"
do
    write_to_line=$(grep -n "//END" $write_file | cut -d: -f1)
    write_to_line=$(($write_to_line - 2))
    while read line
    do
        write="$line"
        if [[ $(echo $line | grep "NAME") != "" ]]
        then
            # fill in name
            write="${line} ${state_name}"
            echo "${write}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $write_file
            write_to_line=$(($write_to_line + 1))
        elif [[ $(echo $line | grep "PATH_LABEL") != "" ]]
        then
            # fill in label
            label=$(echo ${state_name} | awk '{print substr($0,1,1)}')
            write="${line} ${label}"
            echo "${write}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $write_file
            write_to_line=$(($write_to_line + 1))
        elif [[ $(echo $line | grep "TRANSITION") != "" ]]
        then
            # fill in state transition probabilities
            echo "${write}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $write_file
            write_to_line=$(($write_to_line + 1))
            for next_state_name in "${states[@]}"
            do
                x_name=$(echo $state_name | cut -d_ -f1)
                na_name=$(echo $next_state_name | cut -d_ -f2)
                nx_name=$(echo $next_state_name | cut -d_ -f1)
                p=$(get_transition_probability $nx_name $na_name $x_name)
                write="${tab}${next_state_name}: ${p}"
                echo "${write}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $write_file
                write_to_line=$(($write_to_line + 1))
            done
        elif [[ $(echo $line | grep "EMISSION") != "" ]]
        then
            # fill in emission probabilities
            echo "${write}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $write_file
            write_to_line=$(($write_to_line + 1))
            back_to_line=$write_to_line
            write=''
            for observation_name in "${observations[@]}"
            do
                z_name=$(echo $observation_name | cut -d_ -f1)
                za_name=$(echo $observation_name | cut -d_ -f2)
                x_name=$(echo $state_name | cut -d_ -f1)
                a_name=$(echo $state_name | cut -d_ -f2)
                p=$(get_emission_probability $z_name $za_name $x_name $a_name)
                write="${write}${p} "
            done
            echo "${write}\n" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $write_file
            write_to_line=$back_to_line
        else
            echo "${write}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $write_file
            write_to_line=$(($write_to_line + 1))
        fi
    done < $state_template_file
done

echo "finished"
